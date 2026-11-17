"""Command-line interface for the SAU Smart Campus Navigator.

Usage examples:
    sau-navigator places
    python -m navigator places
    python cli.py places

    python -m navigator route GATE1 LIBRARY --vehicle car
    python -m navigator route BOYS_HOSTEL AB4 --algo bfs
    python -m navigator route AMPHITHEATRE KITCHEN --wheelchair
    python -m navigator route LIBRARY AB4 --depart 09:55 --metric safe --deadline 8.1
    python -m navigator route BOYS_HOSTEL AB4 --close 28.4846,77.2002
    python -m navigator compare BOYS_HOSTEL AB4
    python -m navigator tour BOYS_HOSTEL --depart 09:00 \
        --stop HEALTH@-09:15 --stop ADMIN@09:10-09:20/10 --stop LIBRARY/8 \
        --stop AB3@-10:00 --before LIBRARY:AB3 --end AB4@10:10
"""
from __future__ import annotations

import argparse
import sys

from .agent import NavigatorAgent
from .multistop import Stop, TourPlanner
from .problem import Profile


def parse_stop(spec: str) -> Stop:
    """Parse stop specification: PLACE[@OPEN-DEADLINE][/SERVICE_MIN].
    e.g. ADMIN@09:10-09:20/10, AB3@-10:00, LIBRARY/8
    """
    service = 5.0
    if "/" in spec:
        spec, s = spec.split("/")
        service = float(s)
    opens = deadline = None
    if "@" in spec:
        spec, win = spec.split("@")
        o, _, d = win.partition("-")
        opens, deadline = o or None, d or None
    return Stop(spec, deadline=deadline, opens=opens, service_min=service)


def build_parser() -> argparse.ArgumentParser:
    ap = argparse.ArgumentParser(
        prog="sau-navigator",
        description="SAU Smart Campus Navigator - Multi-modal AI Route Planner",
    )
    sub = ap.add_subparsers(dest="cmd", required=True)
    sub.add_parser("places", help="List all campus destinations and parking locations")

    def common(p):
        p.add_argument("--vehicle", choices=["walk", "bike", "car"], default="walk", help="Mode of transport")
        p.add_argument("--role", default="student", choices=["student", "faculty", "resident", "visitor"], help="User role")
        p.add_argument("--wheelchair", action="store_true", help="Avoid stairs and steep gradients (> 1:12)")
        p.add_argument("--avoid-roads", type=float, default=1.0, help="Penalty factor for pedestrians on vehicle roads")
        p.add_argument("--metric", choices=["time", "distance", "safe"], default="time", help="Optimization objective")
        p.add_argument("--depart", default="10:00", help="Departure time in HH:MM format")
        p.add_argument("--close", action="append", default=[], help="Report closed zone: LAT,LON[,RADIUS_M]")

    r = sub.add_parser("route", help="Find route between two locations")
    r.add_argument("start", help="Start place or node id")
    r.add_argument("goal", help="Destination place or node id")
    r.add_argument("--algo", default="astar", choices=["bfs", "dfs", "iddfs", "ucs", "greedy", "astar"], help="Search algorithm")
    r.add_argument("--heuristic", default="haversine", choices=["haversine", "alt"], help="Heuristic function")
    r.add_argument("--weight", type=float, default=1.0, help="Weighted A* factor w (>= 1.0)")
    r.add_argument("--deadline", type=float, help="Arrival deadline in minutes; computes P(on time)")
    common(r)

    c = sub.add_parser("compare", help="Compare all search algorithms on a route")
    c.add_argument("start", help="Start place")
    c.add_argument("goal", help="Destination place")
    common(c)

    t = sub.add_parser("tour", help="Plan multi-stop tour under deadlines and constraints")
    t.add_argument("start", help="Starting place")
    t.add_argument("--stop", action="append", required=True, help="Stop spec: PLACE[@OPEN-DEADLINE][/SERVICE_MIN]")
    t.add_argument("--before", action="append", default=[], help="Precedence constraint: A:B (visit A before B)")
    t.add_argument("--end", help="Final destination: PLACE[@DEADLINE]")
    common(t)

    return ap


def main(args: list[str] | None = None):
    parser = build_parser()
    a = parser.parse_args(args)
    agent = NavigatorAgent()

    if a.cmd == "places":
        for k, p in agent.g.places.items():
            print(f"{k:<14} {p['label']}")
        print("\nParking:", ", ".join(agent.g.parking))
        return 0

    prof = Profile(a.vehicle, a.role, a.wheelchair, avoid_roads=a.avoid_roads)
    for spec in a.close:
        parts = [float(x) for x in spec.split(",")]
        eids = agent.report_closure(parts[0], parts[1], parts[2] if len(parts) > 2 else 8)
        print(f"closed {len(eids)} edge(s) near {parts[0]:.5f},{parts[1]:.5f}")

    if a.cmd == "route":
        kw = {"w": a.weight} if a.algo == "astar" and a.weight != 1 else {}
        route = agent.route(a.start, a.goal, prof, a.algo, a.metric, a.depart, a.heuristic, **kw)
        print(route.describe())
        if route.result.found and a.metric != "distance":
            st = agent.reliability(route, a.deadline * 60 if a.deadline else None)
            line = f"  Uncertainty: median {st['p50_s'] / 60:.1f} min, 95th pct {st['p95_s'] / 60:.1f} min"
            if a.deadline:
                line += f", P(on time within {a.deadline} min) = {st['p_on_time']:.2f}"
            print(line)
    elif a.cmd == "compare":
        rows = agent.compare(a.start, a.goal, prof, a.metric, a.depart)
        cols = list(rows[0])
        print("  ".join(f"{c:>12}" for c in cols))
        for row in rows:
            print("  ".join(f"{str(row[c]):>12}" for c in cols))
    elif a.cmd == "tour":
        end, end_dl = None, None
        if a.end:
            end, _, end_dl = a.end.partition("@")
            end_dl = end_dl or None
        tp = TourPlanner(
            agent,
            a.start,
            [parse_stop(s) for s in a.stop],
            depart=a.depart,
            end=end,
            end_deadline=end_dl,
            precedence=[tuple(x.split(":")) for x in a.before],
            profile=prof,
        )
        csp = tp.solve_csp()
        print(csp.describe())
        if csp.feasible:
            print(f"   P(all deadlines met under uncertainty) = {tp.on_time_probability(csp):.2f}")
        else:
            print(tp.solve_anneal().describe())

    return 0


if __name__ == "__main__":
    sys.exit(main())
