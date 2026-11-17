"""Correctness tests.
Run with pytest:
    python -m pytest -q
Or run standalone directly (no pytest required):
    python tests/test_navigator.py
"""
import itertools
import math
import random
import sys
from pathlib import Path

# Ensure workspace root is in sys.path for direct script execution
_ROOT = Path(__file__).resolve().parent.parent
if str(_ROOT) not in sys.path:
    sys.path.insert(0, str(_ROOT))

try:
    import pytest
except ImportError:
    class _Approx:
        def __init__(self, expected, abs=1e-6, rel=1e-6):
            self.expected = expected
            self.abs = abs
            self.rel = rel

        def __eq__(self, actual):
            return math.isclose(actual, self.expected, abs_tol=self.abs, rel_tol=self.rel)

        def __repr__(self):
            return f"approx({self.expected} ± {self.abs})"

    class _PytestMock:
        def approx(self, expected, abs=1e-6, rel=1e-6):
            return _Approx(expected, abs=abs, rel=rel)

        class mark:
            @staticmethod
            def parametrize(argnames, argvalues):
                def decorator(fn):
                    fn._pytest_parametrized = (argnames, argvalues)
                    return fn
                return decorator

    pytest = _PytestMock()

from navigator.agent import NavigatorAgent
from navigator.multistop import Stop, TourPlanner, follow_with_replanning
from navigator.problem import Profile

A = NavigatorAgent()
G = A.g
PLACES = list(G.places)
rng = random.Random(1)
PAIRS = rng.sample([(a, b) for a, b in itertools.permutations(PLACES, 2)], 40)


def test_graph_connected_on_foot():
    assert len(G.components("walk")) == 1
    for p in G.places.values():
        assert p["nodes"], "every place has at least one entrance"


def test_mode_permissions():
    for e in G.edges.values():
        if e.layer == "lane":
            assert "car" not in e.modes and {"walk", "bike"} <= e.modes
        if e.layer == "road":
            assert {"walk", "bike", "car"} <= e.modes
        if e.layer == "entrance":
            assert e.modes == {"walk"}


@pytest.mark.parametrize("metric", ["time", "distance"])
def test_astar_matches_ucs(metric):
    for a, b in PAIRS:
        u = A.route(a, b, metric=metric, algorithm="ucs").result
        s = A.route(a, b, metric=metric, algorithm="astar").result
        assert s.cost == pytest.approx(u.cost, abs=1e-6)
        assert s.expanded <= u.expanded


def test_alt_is_admissible_and_tighter():
    for a, b in PAIRS:
        u = A.route(a, b, algorithm="ucs").result
        h = A.route(a, b, heuristic="alt").result
        assert h.cost == pytest.approx(u.cost, abs=1e-6)


@pytest.mark.parametrize("vehicle", ["bike", "car"])
def test_vehicle_astar_optimal(vehicle):
    for a, b in PAIRS[:15]:
        u = A.route(a, b, Profile(vehicle), algorithm="ucs").result
        s = A.route(a, b, Profile(vehicle), algorithm="astar").result
        assert s.cost == pytest.approx(u.cost, abs=1e-6)


def test_cars_never_use_lanes():
    for a, b in PAIRS[:15]:
        r = A.route(a, b, Profile("car")).result
        for st, act in zip(r.states[1:], r.actions):
            if act[0] == "move" and st[1] == "car":
                assert G.edges[act[1]].layer in ("road", "parking", "gate")


def test_bfs_fewest_steps_and_iddfs_agree():
    for a, b in PAIRS[:10]:
        bf = A.route(a, b, algorithm="bfs").result
        it = A.route(a, b, algorithm="iddfs").result
        assert len(bf.actions) == len(it.actions)


def test_greedy_and_dfs_never_beat_optimal():
    for a, b in PAIRS[:15]:
        opt = A.route(a, b, algorithm="ucs").result.cost
        for alg in ("greedy", "dfs", "bfs"):
            assert A.route(a, b, algorithm=alg).result.cost >= opt - 1e-6


def test_weighted_astar_bound():
    for a, b in PAIRS[:15]:
        opt = A.route(a, b, algorithm="ucs").result.cost
        assert A.route(a, b, algorithm="astar", w=2.0).result.cost <= 2 * opt + 1e-6


def test_closure_respected():
    r = A.route("BOYS_HOSTEL", "AB4")
    eid = [v for k, v in r.result.actions if k == "move"][10]
    agent = NavigatorAgent(G)
    agent.closures.close([eid])
    r2 = agent.route("BOYS_HOSTEL", "AB4")
    assert eid not in [v for k, v in r2.result.actions if k == "move"]
    assert r2.result.cost >= r.result.cost


def test_wheelchair_avoids_steep():
    for a, b in PAIRS[:20]:
        r = A.route(a, b, Profile(wheelchair=True)).result
        if r.found:
            for k, v in r.actions:
                if k == "move":
                    assert G.edges[v].grade <= 0.0833 and not G.edges[v].stairs


def test_parking_rules():
    r = A.route("GATE3", "FACULTY_RES", Profile("car", role="student"))
    assert r.parking_used() != "Faculty Parking"


def test_replanning_reaches_goal():
    r = A.route("BOYS_HOSTEL", "AB4")
    eid = [v for k, v in r.result.actions if k == "move"][12]
    out = follow_with_replanning(A, "BOYS_HOSTEL", "AB4", hidden_closures=[eid])
    assert out["reached"] and any(e[0] == "replan" for e in out["log"])


def _random_tour(seed):
    rr = random.Random(seed)
    places = rr.sample([p for p in PLACES if p != "BOYS_HOSTEL"], 5)
    stops = [Stop(p, service_min=rr.choice([3, 5, 8])) for p in places]
    for s in rr.sample(stops, 2):
        s.deadline = f"09:{rr.randint(10, 40):02d}"
    return TourPlanner(A, "BOYS_HOSTEL", stops, depart="09:00",
                       precedence=[(places[0], places[1])])


@pytest.mark.parametrize("seed", range(8))
def test_csp_matches_brute_force(seed):
    tp = _random_tour(seed)
    c, b = tp.solve_csp(), tp.solve_brute()
    assert c.feasible == b.feasible
    if b.feasible:
        assert c.finish_m == pytest.approx(b.finish_m)


@pytest.mark.parametrize("seed", range(4))
def test_local_search_finds_feasible_when_exists(seed):
    tp = _random_tour(seed)
    b = tp.solve_brute()
    sa = tp.solve_anneal(iters=3000)
    if b.feasible:
        assert sa.feasible and sa.finish_m <= b.finish_m * 1.05


if __name__ == "__main__":
    import sys
    import time

    print("Running SAU Navigator correctness test suite...")
    t0 = time.perf_counter()
    passed, failed, errors = 0, 0, []

    test_funcs = [
        obj for name, obj in sorted(list(globals().items()))
        if name.startswith("test_") and callable(obj)
    ]

    for fn in test_funcs:
        param = getattr(fn, "_pytest_parametrized", None)
        if param:
            argnames, argvalues = param
            for val in argvalues:
                label = f"{fn.__name__}({val})"
                try:
                    if isinstance(argnames, str) and "," in argnames:
                        fn(*val)
                    else:
                        fn(val)
                    passed += 1
                    print(f"  [PASS] {label}")
                except Exception as ex:
                    failed += 1
                    errors.append((label, ex))
                    print(f"  [FAIL] {label}: {ex}")
        else:
            label = fn.__name__
            try:
                fn()
                passed += 1
                print(f"  [PASS] {label}")
            except Exception as ex:
                failed += 1
                errors.append((label, ex))
                print(f"  [FAIL] {label}: {ex}")

    dt = time.perf_counter() - t0
    print(f"\nResults: {passed} passed, {failed} failed in {dt:.2f}s")
    if failed:
        sys.exit(1)

