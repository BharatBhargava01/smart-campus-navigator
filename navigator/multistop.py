"""Multi-stop routing under deadlines.

Given a start, a set of stops (each with optional opening time, deadline,
service time) and precedence rules, find a visiting order.

CSP formulation
    Variables    S1..Sn   (Si = the i-th stop visited)
                 Ti       (arrival time at Si, derived)
    Domains      Si in {requested stops}
    Constraints  AllDifferent(S1..Sn)
                 Ti = depart(S_{i-1}) + travel(S_{i-1}, Si)        (A* times)
                 depart(Si) = max(Ti, open(Si)) + service(Si)     (wait if early)
                 Ti <= deadline(Si)
                 a before b  for each precedence pair (a, b)
                 optional: arrive at `end` by end_deadline
    Objective    minimise finish time (constraint optimisation)

Solved two ways:
    solve_csp()       backtracking + forward checking + branch & bound
                      (exact; reports nodes explored and prunes)
    solve_anneal()    simulated annealing over permutations with lateness
                      penalties (local search; scales, not guaranteed optimal)
    solve_brute()     enumerates all orders (ground truth for tests)
"""
from __future__ import annotations

import itertools
import math
import random
import time
from dataclasses import dataclass, field

from .agent import NavigatorAgent
from .problem import Profile
from .search import astar
from .uncertainty import fmt_min, hhmm


@dataclass
class Stop:
    place: str
    deadline: str | None = None     # latest arrival 'HH:MM'
    opens: str | None = None        # earliest service start
    service_min: float = 5.0        # time spent there

    @property
    def deadline_m(self):
        return hhmm(self.deadline) if self.deadline else math.inf

    @property
    def opens_m(self):
        return hhmm(self.opens) if self.opens else -math.inf


@dataclass
class TourResult:
    method: str
    order: list[str]
    feasible: bool
    finish_m: float
    schedule: list[dict] = field(default_factory=list)
    lateness_m: float = 0.0
    stats: dict = field(default_factory=dict)
    explanation: list = field(default_factory=list)

    def describe(self):
        lines = [f"[{self.method}] order: {' -> '.join(self.order)}  "
                 f"({'feasible' if self.feasible else 'VIOLATES DEADLINES'}; finish {fmt_min(self.finish_m)})"]
        for s in self.schedule:
            flag = "" if s["ok"] else f"   LATE by {s['arrive'] - s['deadline']:.1f} min"
            wait = f", wait {s['wait']:.1f} min" if s["wait"] > 0.05 else ""
            dl = f" (deadline {fmt_min(s['deadline'])})" if s["deadline"] != math.inf else ""
            lines.append(f"   arrive {s['place']:<14} {fmt_min(s['arrive'])}{dl}{wait}{flag}")
        for why in self.explanation:
            lines.append(f"   why: {why}")
        if self.stats:
            lines.append("   " + ", ".join(f"{k}={v}" for k, v in self.stats.items()))
        return "\n".join(lines)


class TourPlanner:
    def __init__(self, agent: NavigatorAgent, start: str, stops: list[Stop], depart="09:00",
                 end: str | None = None, end_deadline: str | None = None,
                 precedence: list[tuple[str, str]] = (), profile: Profile | None = None,
                 metric="time"):
        self.agent, self.start, self.stops = agent, start, stops
        self.depart_m = hhmm(depart)
        self.depart = depart
        self.end, self.end_deadline = end, (hhmm(end_deadline) if end_deadline else math.inf)
        self.precedence = list(precedence)
        self.profile = profile or Profile("walk")
        self.metric = metric
        self.idx = {s.place: i for i, s in enumerate(stops)}
        self.matrix_stats = {}
        self._build_matrix()

    # -------------------------------------------------- travel-time matrix
    def _build_matrix(self):
        """Pairwise travel minutes between all points of interest, via A*."""
        pts = [self.start] + [s.place for s in self.stops] + ([self.end] if self.end else [])
        self.T = {}
        t0, expanded = time.perf_counter(), 0
        for a in pts:
            for b in pts:
                if a == b or (a, b) in self.T:
                    continue
                r = self.agent.route(a, b, self.profile, "astar", "time", self.depart)
                expanded += r.result.expanded
                self.T[(a, b)] = r.result.cost / 60 if r.result.found else math.inf
        self.matrix_stats = {"pairs": len(self.T), "astar_expanded": expanded,
                             "ms": round((time.perf_counter() - t0) * 1000, 1)}

    def travel(self, a, b):
        return 0.0 if a == b else self.T[(a, b)]

    # -------------------------------------------------- evaluation
    def simulate(self, order):
        """Schedule for a full order -> (finish, lateness, schedule, precedence_violations)."""
        t, here, sched, late = self.depart_m, self.start, [], 0.0
        for place in order:
            s = self.stops[self.idx[place]]
            arrive = t + self.travel(here, place)
            wait = max(0.0, s.opens_m - arrive)
            ok = arrive <= s.deadline_m + 1e-9
            late += max(0.0, arrive - s.deadline_m)
            sched.append({"place": place, "arrive": arrive, "wait": wait, "deadline": s.deadline_m, "ok": ok})
            t = arrive + wait + s.service_min
            here = place
        if self.end:
            arrive = t + self.travel(here, self.end)
            ok = arrive <= self.end_deadline + 1e-9
            late += max(0.0, arrive - self.end_deadline)
            sched.append({"place": self.end + " (end)", "arrive": arrive, "wait": 0.0,
                          "deadline": self.end_deadline, "ok": ok})
            t = arrive
        pos = {p: i for i, p in enumerate(order)}
        prec_viol = sum(1 for a, b in self.precedence if pos[a] > pos[b])
        return t, late, sched, prec_viol

    def _result(self, method, order, stats):
        finish, late, sched, pv = self.simulate(order)
        return TourResult(method, list(order), late == 0 and pv == 0, finish, sched, late, stats)

    # -------------------------------------------------- CSP: backtracking
    def solve_csp(self, forward_checking=True, branch_and_bound=True, value_order="deadline"):
        t0 = time.perf_counter()
        stats = {"nodes": 0, "pruned_deadline": 0, "pruned_forward_check": 0,
                 "pruned_bound": 0, "pruned_precedence": 0}
        best = {"finish": math.inf, "order": None}
        must_before = {}
        for a, b in self.precedence:
            must_before.setdefault(b, set()).add(a)
        places = [s.place for s in self.stops]

        def lower_bound(t, here, remaining):
            # must still reach the farthest remaining stop and do all service
            if not remaining:
                return t + (self.travel(here, self.end) if self.end else 0.0)
            far = max(self.travel(here, p) for p in remaining)
            svc = sum(self.stops[self.idx[p]].service_min for p in remaining)
            return t + far + svc

        def backtrack(order, t, here, remaining):
            stats["nodes"] += 1
            if not remaining:
                finish = t + (self.travel(here, self.end) if self.end else 0.0)
                if finish <= self.end_deadline + 1e-9 and finish < best["finish"]:
                    best["finish"], best["order"] = finish, list(order)
                return
            cand = list(remaining)
            if value_order == "deadline":           # earliest-deadline-first, then nearest
                cand.sort(key=lambda p: (self.stops[self.idx[p]].deadline_m, self.travel(here, p)))
            for p in cand:
                if must_before.get(p, set()) & remaining:
                    stats["pruned_precedence"] += 1
                    continue
                s = self.stops[self.idx[p]]
                arrive = t + self.travel(here, p)
                if arrive > s.deadline_m + 1e-9:     # constraint check
                    stats["pruned_deadline"] += 1
                    continue
                t2 = max(arrive, s.opens_m) + s.service_min
                rem2 = remaining - {p}
                if forward_checking:
                    # every unvisited stop must still be reachable before its deadline
                    # (triangle inequality on shortest paths makes "go directly" a lower bound)
                    if any(t2 + self.travel(p, q) > self.stops[self.idx[q]].deadline_m + 1e-9 for q in rem2):
                        stats["pruned_forward_check"] += 1
                        continue
                    if self.end and t2 + self.travel(p, self.end) > self.end_deadline + 1e-9:
                        stats["pruned_forward_check"] += 1
                        continue
                if branch_and_bound and lower_bound(t2, p, rem2) >= best["finish"]:
                    stats["pruned_bound"] += 1
                    continue
                order.append(p)
                backtrack(order, t2, p, rem2)
                order.pop()

        backtrack([], self.depart_m, self.start, frozenset(places))
        stats["ms"] = round((time.perf_counter() - t0) * 1000, 2)
        if best["order"] is None:
            res = TourResult("CSP backtracking", [], False, math.inf, stats=stats)
            res.explanation = self.explain_infeasible()
            return res
        return self._result("CSP backtracking", best["order"], stats)

    def explain_infeasible(self):
        """Why no order works: stops unreachable in time even if visited first."""
        reasons = []
        for s in self.stops:
            direct = self.depart_m + self.travel(self.start, s.place)
            if direct > s.deadline_m:
                reasons.append(f"{s.place}: even going there first you arrive {fmt_min(direct)}, "
                               f"after its deadline {s.deadline}")
        if not reasons:
            reasons.append("each stop is reachable alone, but no order meets all deadlines together; "
                           "relax a deadline or drop a stop (see local-search result for the least-late order)")
        return reasons

    # -------------------------------------------------- local search
    def cost(self, order, late_w=100.0, prec_w=1000.0):
        finish, late, _, pv = self.simulate(order)
        return finish + late_w * late + prec_w * pv

    def solve_anneal(self, iters=4000, t_start=30.0, t_end=0.05, seed=0, restarts=3):
        """Simulated annealing; moves = swap / relocate / 2-opt reversal."""
        t0 = time.perf_counter()
        rng = random.Random(seed)
        places = [s.place for s in self.stops]
        best_order, best_cost, accepted, evals = None, math.inf, 0, 0
        trace = []
        for r in range(restarts):
            cur = places[:]
            rng.shuffle(cur)
            cur_c = self.cost(cur)
            evals += 1
            for i in range(iters):
                T = t_start * (t_end / t_start) ** (i / max(iters - 1, 1))
                nxt = self._neighbour(cur, rng)
                c = self.cost(nxt)
                evals += 1
                if c < cur_c or rng.random() < math.exp((cur_c - c) / T):
                    cur, cur_c = nxt, c
                    accepted += 1
                if cur_c < best_cost:
                    best_order, best_cost = cur[:], cur_c
                if i % 50 == 0:
                    trace.append(best_cost)
        stats = {"evaluations": evals, "accepted": accepted, "restarts": restarts,
                 "ms": round((time.perf_counter() - t0) * 1000, 1)}
        res = self._result("Simulated annealing", best_order, stats)
        res.trace = trace
        return res

    def solve_hill_climb(self, restarts=10, seed=0):
        """Steepest-ascent hill climbing with random restarts."""
        t0 = time.perf_counter()
        rng = random.Random(seed)
        places = [s.place for s in self.stops]
        best, best_c, evals = None, math.inf, 0
        for _ in range(restarts):
            cur = places[:]
            rng.shuffle(cur)
            cur_c = self.cost(cur)
            while True:
                nbrs = list(self._all_neighbours(cur))
                evals += len(nbrs)
                cand = min(nbrs, key=self.cost) if nbrs else cur
                c = self.cost(cand)
                if c >= cur_c - 1e-9:
                    break
                cur, cur_c = cand, c
            if cur_c < best_c:
                best, best_c = cur, cur_c
        return self._result("Hill climbing (restarts)", best,
                            {"evaluations": evals, "restarts": restarts,
                             "ms": round((time.perf_counter() - t0) * 1000, 1)})

    @staticmethod
    def _neighbour(order, rng):
        o = order[:]
        n = len(o)
        if n < 2:
            return o
        i, j = sorted(rng.sample(range(n), 2))
        move = rng.random()
        if move < 0.33:
            o[i], o[j] = o[j], o[i]
        elif move < 0.66:
            o.insert(j, o.pop(i))
        else:
            o[i:j + 1] = reversed(o[i:j + 1])
        return o

    @staticmethod
    def _all_neighbours(order):
        n = len(order)
        for i in range(n):
            for j in range(i + 1, n):
                o = order[:]
                o[i], o[j] = o[j], o[i]
                yield o
                o = order[:]
                o[i:j + 1] = reversed(o[i:j + 1])
                yield o

    # -------------------------------------------------- ground truth
    def solve_brute(self):
        t0 = time.perf_counter()
        best, best_f, n = None, math.inf, 0
        for perm in itertools.permutations([s.place for s in self.stops]):
            n += 1
            f, late, _, pv = self.simulate(perm)
            if late == 0 and pv == 0 and f < best_f:
                best, best_f = perm, f
        stats = {"orders": n, "ms": round((time.perf_counter() - t0) * 1000, 1)}
        if best is None:
            return TourResult("Brute force", [], False, math.inf, stats=stats)
        return self._result("Brute force", best, stats)

    # -------------------------------------------------- risk
    def on_time_probability(self, tour: TourResult, n=3000, seed=0):
        """Monte Carlo: P(every deadline met) when leg times are uncertain."""
        if not tour.order:
            return 0.0
        from .uncertainty import arrival_stats  # noqa: F401  (same lognormal model)
        rng = random.Random(seed)
        legs = []
        pts = [self.start] + tour.order + ([self.end] if self.end else [])
        for a, b in zip(pts, pts[1:]):
            r = self.agent.route(a, b, self.profile, "astar", "time", self.depart)
            from .uncertainty import path_legs
            legs.append(path_legs(r.problem, r.result))
        tm = self.agent.travel
        ok = 0
        for _ in range(n):
            t, good = self.depart_m, True
            for k, place in enumerate(tour.order + ([None] if self.end else [])):
                leg = sum(tm._lognormal(rng, m, s) for m, s in legs[k]) / 60
                t += leg
                if place is None:
                    good &= t <= self.end_deadline
                else:
                    st = self.stops[self.idx[place]]
                    good &= t <= st.deadline_m
                    t = max(t, st.opens_m) + st.service_min
            ok += good
        return ok / n


# ------------------------------------------------------- dynamic replanning
def follow_with_replanning(agent: NavigatorAgent, start, goal, profile=None, hidden_closures=(),
                           sense_radius_steps=1, depart="10:00"):
    """Simulate an agent walking its plan while closures are discovered en route.

    `hidden_closures` are edge ids blocked in reality but unknown at planning
    time. The agent senses a closure when it reaches a node adjacent to it,
    adds it to its belief, and re-runs A* from where it stands.
    Returns a log of events and the full path actually travelled.
    """
    from .problem import Closures
    belief = agent.closures.copy()
    hidden = set(hidden_closures)
    route = agent.route(start, goal, profile, "astar", "time", depart, closures=belief)
    log = [("plan", route.result.nodes[:], route.result.cost)]
    travelled = [route.result.states[0]]
    i = 0
    states, actions = route.result.states, route.result.actions
    while not route.problem.is_goal(travelled[-1]):
        here = travelled[-1]
        # sense: blocked edges touching the current node
        seen = [eid for eid in agent.g.adj[here[0]] if eid in hidden and eid not in belief.blocked]
        if seen:
            belief.close(seen)
            # is the remaining plan affected?
            upcoming = {a[1] for a in actions[i:] if a[0] == "move"}
            if upcoming & set(seen):
                p = agent.problem(here[0], goal, profile, "time", depart, belief)
                p.initial = [here]
                res = astar(p)
                log.append(("replan", here[0], seen, res.nodes[:], res.cost))
                if not res.found:
                    log.append(("stuck", here[0]))
                    break
                states, actions, i = res.states, res.actions, 0
                continue
        if i >= len(actions):
            break
        travelled.append(states[i + 1])
        i += 1
    return {"log": log, "travelled": [s for s in travelled], "reached": route.problem.is_goal(travelled[-1])}
