"""The navigator agent: turns a user query into a search problem, solves it,
and explains the result.

PEAS
    Performance  shortest expected time (or distance / safest), deadlines met,
                 accessibility respected, reliability under uncertainty
    Environment  SAU campus graph; closures; congestion by time of day
    Actuators    route + turn list, visiting order, replanned route
    Sensors      user query (start, destination(s), vehicle, role, needs,
                 time), closure reports
"""
from __future__ import annotations

from dataclasses import dataclass

from .graph import CampusGraph, campus, edges_near
from .problem import Closures, Profile, RouteProblem
from .search import ALGORITHMS, Landmarks, astar
from .uncertainty import TravelModel, arrival_stats


def resolve(g: CampusGraph, where) -> list[int]:
    """Place key ('LIBRARY'), entrance/parking name, or node id -> node ids."""
    if isinstance(where, int):
        return [where]
    key = where.strip()
    if key.upper() in g.places:
        return list(g.places[key.upper()]["nodes"])
    for n in g.nodes.values():
        if n.name and n.name.lower() == key.lower():
            return [n.id]
    low = key.lower()
    for k, p in g.places.items():
        if low in p["label"].lower():
            return list(p["nodes"])
    raise KeyError(f"unknown place '{where}'. Try one of: {', '.join(g.places)}")


def label(g, node_id):
    n = g.nodes[node_id]
    if n.name:
        return n.name
    p = g.place_of(node_id)
    return g.places[p]["label"] if p else f"node {node_id}"


@dataclass
class Route:
    problem: RouteProblem
    result: object

    @property
    def g(self):
        return self.problem.g

    def legs(self):
        """Group the step list into legs by mode: [(mode, [edge ids], metres)]."""
        out, cur = [], None
        for i, (kind, val) in enumerate(self.result.actions):
            if kind in ("park", "unpark"):
                out.append((kind, val, 0.0))
                cur = None
                continue
            mode = self.result.states[i + 1][1]
            e = self.g.edges[val]
            if cur is None or cur[0] != mode:
                cur = [mode, [], 0.0]
                out.append(cur)
            cur[1].append(val)
            cur[2] += e.length
        return [tuple(x) for x in out]

    def distance_m(self):
        return sum(self.g.edges[v].length for k, v in self.result.actions if k == "move")

    def time_s(self):
        p = self.problem
        if p.metric == "time":
            return self.result.cost
        t = RouteProblem(p.g, [s for s, _ in p.initial], p.goals, p.profile, "time",
                         p.closures, p.depart, p.travel)
        from .search import _path_cost
        return _path_cost(t, self.result.states, self.result.actions)

    def parking_used(self):
        for k, v in self.result.actions:
            if k == "park":
                return v
        return None

    def describe(self):
        if not self.result.found:
            return "No route: every path is blocked for this traveller (closures / accessibility)."
        g, r = self.g, self.result
        start, end = r.states[0][0], r.states[-1][0]
        verb = {"walk": "Walk", "fetch": "Walk", "bike": "Ride", "car": "Drive"}
        lines = [f"{label(g, start)}  ->  {label(g, end)}"]
        for leg in self.legs():
            if leg[0] == "park":
                lines.append(f"  * Park at {leg[1]}")
            elif leg[0] == "unpark":
                lines.append(f"  * Collect your {self.problem.profile.vehicle} at {leg[1]}")
            else:
                mode, eids, m = leg
                layers = sorted({g.edges[e].layer for e in eids} - {"connector", "entrance", "parking", "gate"})
                lines.append(f"  * {verb[mode]} {m:.0f} m" + (f" via {'/'.join(layers)}" if layers else ""))
        lines.append(f"  Total {self.distance_m():.0f} m, about {self.time_s() / 60:.1f} min "
                     f"({r.algorithm}, {r.expanded} nodes expanded)")
        return "\n".join(lines)


class NavigatorAgent:
    def __init__(self, g: CampusGraph | None = None):
        self.g = g or campus()
        self.closures = Closures()
        self.travel = TravelModel(self.g)
        self._alt = None

    # ------------------------------------------------------------ sensing
    def report_closure(self, lat, lon, radius_m=8, modes=("walk", "bike", "car"), layers=None):
        eids = edges_near(self.g, lat, lon, radius_m, layers)
        self.closures.close(eids, modes)
        return eids

    def close_place_link(self, a, b, modes=("walk", "bike", "car")):
        """Close the stretch of path between two named points (midpoint, 15 m)."""
        na, nb = self.g.nodes[resolve(self.g, a)[0]], self.g.nodes[resolve(self.g, b)[0]]
        return self.report_closure((na.lat + nb.lat) / 2, (na.lon + nb.lon) / 2, 15, modes)

    def clear_closures(self):
        self.closures = Closures()

    # ------------------------------------------------------------ acting
    def problem(self, start, goal, profile=None, metric="time", depart="10:00", closures=None):
        return RouteProblem(self.g, resolve(self.g, start), resolve(self.g, goal),
                            profile or Profile(), metric,
                            closures if closures is not None else self.closures,
                            depart, self.travel)

    def route(self, start, goal, profile=None, algorithm="astar", metric="time",
              depart="10:00", heuristic="haversine", closures=None, **kw) -> Route:
        p = self.problem(start, goal, profile, metric, depart, closures)
        if heuristic == "alt":
            if self._alt is None:
                self._alt = Landmarks(self.g)
            self._alt.attach(p)
        res = ALGORITHMS[algorithm](p, **kw)
        return Route(p, res)

    def compare(self, start, goal, profile=None, metric="time", depart="10:00"):
        rows = []
        for name in ("bfs", "dfs", "iddfs", "ucs", "greedy", "astar"):
            rows.append(self.route(start, goal, profile, name, metric, depart).result.row())
        rows.append(self.route(start, goal, profile, "astar", metric, depart, w=2.0).result.row())
        if (profile or Profile()).vehicle == "walk":
            r = self.route(start, goal, profile, "astar", metric, depart, heuristic="alt").result
            r.algorithm = "A* + ALT"
            rows.append(r.row())
        return rows

    def reliability(self, route: Route, deadline_s=None):
        return arrival_stats(route.problem, route.result, deadline_s)
