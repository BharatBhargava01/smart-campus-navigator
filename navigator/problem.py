"""Route-finding as a state-space search problem (AIMA style).

    State        (node, mode)      mode in {fetch, car, bike, walk}
                 fetch = walking to where your vehicle is parked
    Initial      at a gate/parking with a vehicle: (start, car|bike)
                 at a building with a vehicle:     (start, fetch)
                 on foot:                          (start, walk)
    Actions      MOVE   along an open edge whose layer allows the mode
                 UNPARK at your vehicle's spot: fetch -> car|bike
                 PARK   at an allowed parking node: car|bike -> walk
    Transition   MOVE e: (u, m) -> (v, m);  UNPARK/PARK change only the mode
    Goal test    on foot (walk, or fetch = never used the vehicle) at an
                 entrance of the destination
    Path cost    sum of action costs; the metric is one of
                   distance : metres
                   time     : expected seconds (congestion-aware)
                   safe     : expected seconds + k * std   (risk-averse)
    Heuristic    straight-line (haversine) distance to the nearest goal,
                 divided by the fastest speed still available in that state.
                 Never overestimates -> A* returns optimal routes.
"""
from __future__ import annotations

from dataclasses import dataclass, field

from .geo import haversine
from .graph import CampusGraph, Edge, load_overrides
from .uncertainty import TravelModel

_OV = load_overrides()
SPEEDS = _OV.get("speeds_mps", {"walk": 1.3, "wheelchair": 0.9, "bike": 4.2, "car": 5.6})
PARK_TIME = _OV.get("park_time_s", {"bike": 60, "car": 180})
PARKING_RULES = {k: v for k, v in _OV.get("parking_rules", {}).items() if not k.startswith("_")}


@dataclass
class Profile:
    """Who is travelling and how."""
    vehicle: str = "walk"           # walk | bike | car
    role: str = "student"           # student | faculty | visitor | resident
    wheelchair: bool = False
    max_grade: float = 0.0833       # ADA ramp limit 1:12
    avoid_roads: float = 1.0        # >1 penalises walking on vehicle roads
    risk_k: float = 1.5             # for metric == "safe"
    vehicle_at: str | None = None   # parking spot where the vehicle is now

    def speed(self, mode):
        if mode == "fetch":
            mode = "walk"
        if mode == "walk" and self.wheelchair:
            return SPEEDS["wheelchair"]
        return SPEEDS[mode]

    def can_park(self, name):
        allowed = PARKING_RULES.get(name)
        return allowed is None or self.role in allowed


@dataclass
class Closures:
    """Edges that are blocked, optionally only for some modes."""
    blocked: dict[int, frozenset] = field(default_factory=dict)   # eid -> modes blocked

    def close(self, eids, modes=("walk", "bike", "car")):
        for e in eids:
            self.blocked[e] = frozenset(modes) | self.blocked.get(e, frozenset())
        return self

    def is_blocked(self, eid, mode):
        return mode in self.blocked.get(eid, ())

    def copy(self):
        return Closures(dict(self.blocked))


class RouteProblem:
    def __init__(self, g: CampusGraph, starts, goals, profile: Profile | None = None,
                 metric="time", closures: Closures | None = None, depart="10:00",
                 travel: TravelModel | None = None):
        self.g = g
        self.profile = profile or Profile()
        self.metric = metric
        self.closures = closures or Closures()
        self.travel = travel or TravelModel(g)
        self.depart = depart
        starts = [starts] if isinstance(starts, int) else list(starts)
        veh = self.profile.vehicle
        self.vehicle_node = None
        if veh == "walk" or all(g.nodes[s].kind in ("gate", "parking") for s in starts):
            self.initial = [(s, veh) for s in starts]
        else:
            self.vehicle_node = self._locate_vehicle(starts)
            self.initial = [(s, "fetch") for s in starts]
        self.goals = set([goals] if isinstance(goals, int) else goals)
        self._goal_ll = [(g.nodes[n].lat, g.nodes[n].lon) for n in self.goals]
        self._hcache = {}

    def _locate_vehicle(self, starts):
        """Parking node holding the user's vehicle: given, else nearest on foot."""
        if self.profile.vehicle_at:
            return self.g.parking[self.profile.vehicle_at]
        import heapq
        dist = {s: 0.0 for s in starts}
        pq = [(0.0, s) for s in starts]
        while pq:
            d, n = heapq.heappop(pq)
            if d > dist[n]:
                continue
            node = self.g.nodes[n]
            if node.kind == "parking" and any(self.profile.vehicle in e.modes for _, e in self.g.neighbors(n)):
                return n
            for m, e in self.g.neighbors(n):
                if "walk" in e.modes and d + e.length < dist.get(m, float("inf")):
                    dist[m] = d + e.length
                    heapq.heappush(pq, (dist[m], m))
        raise ValueError("no parking spot reachable on foot")

    # ---------------------------------------------------------------- API
    def is_goal(self, state):
        n, mode = state
        return mode in ("walk", "fetch") and n in self.goals

    def edge_allowed(self, e: Edge, mode):
        if mode == "fetch":
            mode = "walk"
        if mode not in e.modes or self.closures.is_blocked(e.id, mode):
            return False
        if mode == "walk" and self.profile.wheelchair:
            if e.stairs or e.grade > self.profile.max_grade:
                return False
        return True

    def edge_cost(self, e: Edge, mode):
        if mode == "fetch":
            mode = "walk"
        if self.metric == "distance":
            return e.length
        speed = self.profile.speed(mode)
        mean, std = self.travel.edge_time(e, mode, speed, self.depart)
        if mode == "walk":
            if e.layer == "road":
                mean *= self.profile.avoid_roads
            if self.profile.wheelchair and e.grade > 0.05:   # gentle slopes still slow you
                mean *= 1 + 4 * (e.grade - 0.05)
        if self.metric == "safe":
            return mean + self.profile.risk_k * std
        return mean

    def actions(self, state):
        """Yields (next_state, cost, action) triples."""
        n, mode = state
        for m, e in self.g.neighbors(n):
            if self.edge_allowed(e, mode):
                yield (m, mode), self.edge_cost(e, mode), ("move", e.id)
        node = self.g.nodes[n]
        if mode == "fetch" and n == self.vehicle_node:
            veh = self.profile.vehicle
            yield (n, veh), (0.0 if self.metric == "distance" else PARK_TIME[veh]), ("unpark", node.name)
        if mode in ("car", "bike") and node.kind == "parking" and self.profile.can_park(node.name):
            cost = 0.0 if self.metric == "distance" else PARK_TIME[mode]
            yield (n, "walk"), cost, ("park", node.name)

    def h(self, state):
        """Admissible straight-line heuristic."""
        n, mode = state
        key = n
        if key not in self._hcache:
            a = self.g.nodes[n]
            self._hcache[key] = min(haversine(a.lat, a.lon, la, lo) for la, lo in self._goal_ll)
        d = self._hcache[key]
        if self.metric == "distance":
            return d
        # fastest speed this state can still use; congestion factor >= 1 so this is a lower bound
        if mode == "fetch":
            return d / max(self.profile.speed("walk"), self.profile.speed(self.profile.vehicle))
        return d / self.profile.speed(mode)
