"""Build the SAU campus graph from the Measure-Map GPS exports.

Layers (see README "Movement model"):

    layer      walk  bike(2W)  car(4W)   source file
    -------    ----  --------  -------   ------------------------------
    lane        yes    yes       no      walkways.geojson  (SAU_Project_1)
    road        yes    yes       yes     roads.geojson     (SAU_Vehicleonly_Route)
    connector   yes    yes       no      auto: lane <-> road junctions
    entrance    yes    no        no      auto: building door -> nearest path
    parking     *      *         *       auto: parking spot -> lane / road
    gate        *      *         *       auto: campus gate -> lane / road
    (* = inherits the modes of the layer it attaches to)

The survey traces were clicked by hand, so the builder repairs them:
  1. vertices closer than MERGE_M are merged (the walkway trace doubles back
     over itself many times),
  2. a vertex lying within SNAP_M of another segment of the same layer is
     spliced into it (T-junctions the surveyor clicked "near" but not "on"),
  3. lane and road layers are joined where they come within LINK_M.
"""
from __future__ import annotations

import csv
import json
import math
import os
from dataclasses import dataclass, field
from pathlib import Path

from .geo import haversine, project_point_on_segment, to_latlon, to_xy


def get_data_dir() -> Path:
    env = os.environ.get("SAU_NAVIGATOR_DATA_DIR")
    if env:
        return Path(env).resolve()
    return Path(__file__).resolve().parent.parent / "data"


DATA_DIR = get_data_dir()

MERGE_M = 2.0     # merge vertices closer than this
SNAP_M = 4.0      # splice a vertex into a nearby segment of the same layer
LINK_M = 7.0      # join lane and road layers when this close
ENTRANCE_MAX_M = 45.0
PARKING_MAX_M = 45.0

ALL_MODES = frozenset({"walk", "bike", "car"})
LAYER_MODES = {
    "lane": frozenset({"walk", "bike"}),
    "road": ALL_MODES,
    "connector": frozenset({"walk", "bike"}),
    "entrance": frozenset({"walk"}),
}


@dataclass
class Node:
    id: int
    lat: float
    lon: float
    kind: str                  # lane | road | entrance | parking | gate
    name: str | None = None
    alt: float | None = None   # metres above sea level (survey), if known

    @property
    def xy(self):
        return to_xy(self.lat, self.lon)


@dataclass
class Edge:
    id: int
    u: int
    v: int
    length: float              # metres
    layer: str
    modes: frozenset
    grade: float = 0.0         # |rise| / run, from survey altitudes (lane only)
    stairs: bool = False       # set from data/overrides.json

    def other(self, n: int) -> int:
        return self.v if n == self.u else self.u


@dataclass
class CampusGraph:
    nodes: dict[int, Node] = field(default_factory=dict)
    edges: dict[int, Edge] = field(default_factory=dict)
    adj: dict[int, list[int]] = field(default_factory=dict)   # node -> edge ids
    places: dict[str, dict] = field(default_factory=dict)     # key -> {label, nodes}
    parking: dict[str, int] = field(default_factory=dict)     # name -> node id
    warnings: list[str] = field(default_factory=list)

    # ------------------------------------------------------------- mutation
    def add_node(self, lat, lon, kind, name=None, alt=None) -> int:
        nid = len(self.nodes)
        while nid in self.nodes:
            nid += 1
        self.nodes[nid] = Node(nid, lat, lon, kind, name, alt)
        self.adj[nid] = []
        return nid

    def add_edge(self, u, v, layer, modes=None) -> int | None:
        if u == v:
            return None
        for eid in self.adj[u]:                      # no parallel duplicates
            e = self.edges[eid]
            if e.other(u) == v and e.layer == layer:
                return eid
        a, b = self.nodes[u], self.nodes[v]
        eid = max(self.edges, default=-1) + 1
        self.edges[eid] = Edge(eid, u, v, haversine(a.lat, a.lon, b.lat, b.lon),
                               layer, modes if modes is not None else LAYER_MODES[layer])
        self.adj[u].append(eid)
        self.adj[v].append(eid)
        return eid

    def remove_edge(self, eid):
        e = self.edges.pop(eid)
        self.adj[e.u].remove(eid)
        self.adj[e.v].remove(eid)

    def split_edge(self, eid, lat, lon, kind=None) -> int:
        """Insert a node on edge `eid` at (lat, lon); returns the node id."""
        e = self.edges[eid]
        for end in (e.u, e.v):                       # reuse an endpoint if close
            n = self.nodes[end]
            if haversine(n.lat, n.lon, lat, lon) < MERGE_M:
                return end
        a, b = self.nodes[e.u], self.nodes[e.v]
        alt = None
        if a.alt is not None and b.alt is not None:
            t = haversine(a.lat, a.lon, lat, lon) / max(e.length, 1e-9)
            alt = a.alt + t * (b.alt - a.alt)
        mid = self.add_node(lat, lon, kind or a.kind, alt=alt)
        layer, modes = e.layer, e.modes
        self.remove_edge(eid)
        self.add_edge(e.u, mid, layer, modes)
        self.add_edge(mid, e.v, layer, modes)
        return mid

    # ------------------------------------------------------------- queries
    def nearest_edge(self, lat, lon, layers, exclude_node=None):
        """Closest edge (in given layers) to a point -> (eid, (lat, lon), dist_m)."""
        p = to_xy(lat, lon)
        best = (None, None, math.inf)
        for eid, e in self.edges.items():
            if e.layer not in layers:
                continue
            if exclude_node is not None and exclude_node in (e.u, e.v):
                continue
            q, _, d = project_point_on_segment(p, self.nodes[e.u].xy, self.nodes[e.v].xy)
            if d < best[2]:
                best = (eid, to_latlon(*q), d)
        return best

    def neighbors(self, n):
        for eid in self.adj[n]:
            e = self.edges[eid]
            yield e.other(n), e

    def components(self, mode="walk"):
        seen, comps = set(), []
        for s in self.nodes:
            if s in seen:
                continue
            stack, comp = [s], []
            seen.add(s)
            while stack:
                n = stack.pop()
                comp.append(n)
                for m, e in self.neighbors(n):
                    if mode in e.modes and m not in seen:
                        seen.add(m)
                        stack.append(m)
            comps.append(comp)
        return sorted(comps, key=len, reverse=True)

    def place_of(self, node_id):
        for k, p in self.places.items():
            if node_id in p["nodes"]:
                return k
        return None

    def summary(self) -> str:
        by_layer = {}
        for e in self.edges.values():
            s = by_layer.setdefault(e.layer, [0, 0.0])
            s[0] += 1
            s[1] += e.length
        lines = [f"{len(self.nodes)} nodes, {len(self.edges)} edges, "
                 f"{len(self.places)} places, {len(self.parking)} parking spots"]
        for k, (c, L) in sorted(by_layer.items()):
            lines.append(f"  {k:<10} {c:>4} edges {L:8.0f} m")
        return "\n".join(lines)


# ---------------------------------------------------------------- loaders
def _read_geojson(name):
    with open(DATA_DIR / name, encoding="utf-8") as f:
        return json.load(f)["features"]


def _read_points_csv(name):
    """Altitudes from the 'Points' table of a Measure-Map CSV, in trace order."""
    rows, in_pts = [], False
    with open(DATA_DIR / name, encoding="utf-8-sig") as f:
        for row in csv.reader(f):
            if row[:2] == ["#", "Latitude"]:
                in_pts = True
                continue
            if in_pts:
                if not row or not row[0].strip().isdigit():
                    break
                alt = float(row[3]) if row[3] else None
                rows.append((float(row[1]), float(row[2]), alt))
    return rows


def _read_spots_csv(name):
    out, in_spots = {}, False
    with open(DATA_DIR / name, encoding="utf-8-sig") as f:
        for row in csv.reader(f):
            if row[:2] == ["Name", "Latitude"]:
                in_spots = True
                continue
            if in_spots and len(row) >= 4 and row[0]:
                out[row[0]] = float(row[3]) if row[3] else None
    return out


class _Merger:
    """Grid hash that returns an existing node when a vertex is within MERGE_M."""

    def __init__(self, g: CampusGraph, kind: str):
        self.g, self.kind, self.cells = g, kind, {}

    def get(self, lat, lon, alt=None):
        x, y = to_xy(lat, lon)
        cx, cy = int(x // MERGE_M), int(y // MERGE_M)
        for i in (-1, 0, 1):
            for j in (-1, 0, 1):
                for nid in self.cells.get((cx + i, cy + j), []):
                    n = self.g.nodes[nid]
                    if haversine(n.lat, n.lon, lat, lon) < MERGE_M:
                        if n.alt is None:
                            n.alt = alt
                        return nid
        nid = self.g.add_node(lat, lon, self.kind, alt=alt)
        self.cells.setdefault((cx, cy), []).append(nid)
        return nid


def _snap_layer(g: CampusGraph, layer: str):
    """Splice vertices of `layer` into nearby non-incident segments of the same layer."""
    changed = True
    while changed:
        changed = False
        for nid in list(g.nodes):
            n = g.nodes[nid]
            if not any(g.edges[e].layer == layer for e in g.adj[nid]):
                continue
            eid, (lat, lon), d = g.nearest_edge(n.lat, n.lon, {layer}, exclude_node=nid)
            if eid is None or d > SNAP_M:
                continue
            e = g.edges[eid]
            if any(g.edges[x].other(nid) in (e.u, e.v) for x in g.adj[nid]) and d < 0.5:
                continue
            m = g.split_edge(eid, lat, lon)
            if m != nid and not any(g.edges[x].other(nid) == m for x in g.adj[nid]):
                g.add_edge(nid, m, layer)
                changed = True


def _attach(g, nid, layers, max_d, kind_label):
    """Connect point-node `nid` to the nearest edge among `layers`."""
    n = g.nodes[nid]
    eid, (lat, lon), d = g.nearest_edge(n.lat, n.lon, layers)
    if eid is None or d > max_d:
        return None, d
    layer = g.edges[eid].layer
    m = g.split_edge(eid, lat, lon)
    return (m, layer), d


def load_overrides():
    p = DATA_DIR / "overrides.json"
    if p.exists():
        with open(p, encoding="utf-8") as f:
            return json.load(f)
    return {}


def build_graph() -> CampusGraph:
    g = CampusGraph()

    # 1. lane layer (2-wheeler + walking) -------------------------------
    lane_alts = _read_points_csv("walkways.csv")
    lm = _Merger(g, "lane")
    for feat in _read_geojson("walkways.geojson"):
        coords = feat["geometry"]["coordinates"]
        prev = None
        for i, (lon, lat) in enumerate(coords):
            alt = lane_alts[i][2] if i < len(lane_alts) else None
            cur = lm.get(lat, lon, alt)
            if prev is not None:
                g.add_edge(prev, cur, "lane")
            prev = cur
    _snap_layer(g, "lane")

    # 2. road layer (all vehicles + walking) ----------------------------
    rm = _Merger(g, "road")
    for feat in _read_geojson("roads.geojson"):
        prev = None
        for lon, lat in feat["geometry"]["coordinates"]:
            cur = rm.get(lat, lon)
            if prev is not None:
                g.add_edge(prev, cur, "road")
            prev = cur
    # the short road stub that leads to the Boys' Hostel parking
    for feat in _read_geojson("parking.geojson"):
        if feat["geometry"]["type"] == "LineString":
            prev = None
            for lon, lat in feat["geometry"]["coordinates"]:
                cur = rm.get(lat, lon)
                if prev is not None:
                    g.add_edge(prev, cur, "road")
                prev = cur
    _snap_layer(g, "road")

    # 3. join lane <-> road where they meet -----------------------------
    for nid in list(g.nodes):
        n = g.nodes[nid]
        if n.kind not in ("lane", "road"):
            continue
        other = "road" if n.kind == "lane" else "lane"
        eid, (lat, lon), d = g.nearest_edge(n.lat, n.lon, {other})
        if eid is not None and d <= LINK_M:
            m = g.split_edge(eid, lat, lon)
            g.add_edge(nid, m, "connector")

    # 4. altitude for nodes without survey data: inverse-distance weighting
    known = [(n.lat, n.lon, n.alt) for n in g.nodes.values() if n.alt is not None]
    for n in g.nodes.values():
        if n.alt is None and known:
            ws = [(1 / max(haversine(n.lat, n.lon, a, b), 1.0) ** 2, z) for a, b, z in known]
            n.alt = sum(w * z for w, z in ws) / sum(w for w, _ in ws)

    # 5. building entrances & gates --------------------------------------
    spot_alt = _read_spots_csv("entrances.csv")
    name_to_node = {}
    for feat in _read_geojson("entrances.geojson"):
        name = feat["properties"]["name"].strip()
        lon, lat = feat["geometry"]["coordinates"][:2]
        is_gate = name.lower().startswith("gate")
        nid = g.add_node(lat, lon, "gate" if is_gate else "entrance", name, spot_alt.get(name))
        name_to_node[name] = nid
        if is_gate:                     # gates admit every mode on every layer
            hit_any = False
            for layer in ("lane", "road"):
                hit, d = _attach(g, nid, {layer}, 30.0, "gate")
                if hit:
                    m, lay = hit
                    g.add_edge(nid, m, "gate", LAYER_MODES[lay])
                    hit_any = True
            if not hit_any:
                g.warnings.append(f"gate '{name}' is not near any path")
        else:
            hit, d = _attach(g, nid, {"lane", "road"}, ENTRANCE_MAX_M, "entrance")
            if hit:
                g.add_edge(nid, hit[0], "entrance")
            else:
                g.warnings.append(f"entrance '{name}' is {d:.0f} m from any path; left unconnected")

    # 6. parking spots ---------------------------------------------------
    for feat in _read_geojson("parking.geojson"):
        if feat["geometry"]["type"] != "Point":
            continue
        name = feat["properties"]["name"].strip()
        lon, lat = feat["geometry"]["coordinates"][:2]
        nid = g.add_node(lat, lon, "parking", name)
        g.parking[name] = nid
        linked = []
        for layer in ("road", "lane"):
            hit, d = _attach(g, nid, {layer}, PARKING_MAX_M, "parking")
            if hit:
                g.add_edge(nid, hit[0], "parking", LAYER_MODES[layer])
                linked.append(layer)
        if "road" not in linked:
            g.warnings.append(f"parking '{name}' has no road access (2-wheelers/walk only)")

    # 7. grades on lane edges (survey altitudes on both ends) -----------
    for e in g.edges.values():
        a, b = g.nodes[e.u], g.nodes[e.v]
        if e.layer == "lane" and a.alt is not None and b.alt is not None:
            # very short edges give noisy grades; measure over >= 10 m
            e.grade = abs(a.alt - b.alt) / max(e.length, 10.0)

    # 8. places (buildings -> set of entrance nodes) ---------------------
    with open(DATA_DIR / "places.json", encoding="utf-8") as f:
        pl = json.load(f)["places"]
    for key, p in pl.items():
        nodes = [name_to_node[n] for n in p["entrances"] if n in name_to_node]
        missing = [n for n in p["entrances"] if n not in name_to_node]
        if missing:
            g.warnings.append(f"place {key}: unknown entrances {missing}")
        g.places[key] = {"label": p["label"], "nodes": nodes}

    # 9. manual overrides (stairs, permanently restricted edges) --------
    ov = load_overrides()
    for s in ov.get("stairs", []):
        for e in edges_near(g, s["lat"], s["lon"], s.get("radius_m", 6)):
            g.edges[e].stairs = True
    return g


def edges_near(g: CampusGraph, lat, lon, radius_m, layers=None):
    """Edge ids whose closest point lies within radius of (lat, lon)."""
    p = to_xy(lat, lon)
    out = []
    for eid, e in g.edges.items():
        if layers and e.layer not in layers:
            continue
        _, _, d = project_point_on_segment(p, g.nodes[e.u].xy, g.nodes[e.v].xy)
        if d <= radius_m:
            out.append(eid)
    return out


_CACHE: CampusGraph | None = None


def campus() -> CampusGraph:
    """Cached graph (building takes ~1 s)."""
    global _CACHE
    if _CACHE is None:
        _CACHE = build_graph()
    return _CACHE
