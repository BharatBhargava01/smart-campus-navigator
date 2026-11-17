"""Travel-time uncertainty.

Each edge's traversal time is modelled as a log-normal random variable:

    mean = length / speed * congestion_factor(edge, time of day)
    std  = cv(edge, mode, time of day) * mean

Congestion comes from hotspot zones in data/overrides.json (mess at lunch,
academic blocks at class change, main gate at rush hour). Trips on campus last
minutes, so the factor is evaluated at the departure time for the whole trip.

Edge times are assumed independent. Under that assumption a path's mean is the
sum of edge means and its variance the sum of edge variances; Monte Carlo
sampling gives the full distribution and P(arrive before deadline).
"""
from __future__ import annotations

import math
import random

from .geo import haversine
from .graph import CampusGraph, Edge, load_overrides


def hhmm(t) -> float:
    """'09:30' -> minutes after midnight (also accepts numbers)."""
    if isinstance(t, (int, float)):
        return float(t)
    h, m = t.split(":")
    return int(h) * 60 + int(m)


def fmt_min(m: float) -> str:
    if m == math.inf or m != m:
        return "--:--"
    m = round(m)
    return f"{int(m // 60) % 24:02d}:{int(m % 60):02d}"


class TravelModel:
    def __init__(self, g: CampusGraph, enabled=True):
        cfg = load_overrides().get("congestion", {})
        self.g = g
        self.enabled = enabled
        self.zones = cfg.get("zones", [])
        self.base_cv = cfg.get("base_cv", {"walk": 0.1, "bike": 0.2, "car": 0.25})
        self._edge_zones = {}
        for eid, e in g.edges.items():
            a, b = g.nodes[e.u], g.nodes[e.v]
            mlat, mlon = (a.lat + b.lat) / 2, (a.lon + b.lon) / 2
            self._edge_zones[eid] = [z for z in self.zones
                                     if haversine(mlat, mlon, z["lat"], z["lon"]) <= z["radius_m"]]

    def zone_active(self, z, t):
        t = hhmm(t)
        return any(hhmm(a) <= t <= hhmm(b) for a, b in z["windows"])

    def factors(self, e: Edge, mode, t):
        if mode == "fetch":
            mode = "walk"
        factor, cv = 1.0, self.base_cv.get(mode, 0.15)
        if self.enabled:
            for z in self._edge_zones.get(e.id, []):
                if self.zone_active(z, t):
                    factor = max(factor, z["factor"])
                    cv = max(cv, z["cv"])
        return factor, cv

    def edge_time(self, e: Edge, mode, speed, t):
        factor, cv = self.factors(e, mode, t)
        mean = e.length / speed * factor
        return mean, cv * mean

    # ------------------------------------------------------------ sampling
    @staticmethod
    def _lognormal(rng, mean, std):
        if mean <= 0:
            return 0.0
        s2 = math.log(1 + (std / mean) ** 2)
        return rng.lognormvariate(math.log(mean) - s2 / 2, math.sqrt(s2))

    def sample_path(self, legs, n=5000, seed=0):
        """legs: list of (mean, std) per step. Returns sorted sample totals (s)."""
        rng = random.Random(seed)
        out = [sum(self._lognormal(rng, m, s) for m, s in legs) for _ in range(n)]
        out.sort()
        return out


def path_legs(problem, result):
    """(mean, std) of each step on a found route, for Monte Carlo."""
    from .problem import PARK_TIME
    legs = []
    for i, (kind, val) in enumerate(result.actions):
        if kind in ("park", "unpark"):
            veh = problem.profile.vehicle
            legs.append((PARK_TIME[veh], PARK_TIME[veh] * 0.3))
            continue
        e = problem.g.edges[val]
        mode = result.states[i + 1][1]
        mean, std = problem.travel.edge_time(e, mode, problem.profile.speed(mode), problem.depart)
        legs.append((mean, std))
    return legs


def arrival_stats(problem, result, deadline_s=None, n=5000, seed=0):
    samples = problem.travel.sample_path(path_legs(problem, result), n, seed)
    mean = sum(samples) / len(samples)
    stats = {"mean_s": mean, "p50_s": samples[len(samples) // 2],
             "p90_s": samples[int(0.9 * len(samples))], "p95_s": samples[int(0.95 * len(samples))]}
    if deadline_s is not None:
        stats["p_on_time"] = sum(x <= deadline_s for x in samples) / len(samples)
    return stats
