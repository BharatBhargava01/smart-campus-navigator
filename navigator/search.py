"""Search algorithms, written from scratch over the generic RouteProblem API:

    problem.initial        list of start states
    problem.is_goal(s)
    problem.actions(s)     -> (next_state, step_cost, action)
    problem.h(s)           heuristic (informed search only)

Uninformed:  BFS, DFS, IDDFS, UCS (= Dijkstra)
Informed:    Greedy best-first, A*, weighted A*, A* with landmark (ALT) heuristic
"""
from __future__ import annotations

import heapq
import itertools
import time
from collections import deque
from dataclasses import dataclass, field


@dataclass
class SearchResult:
    algorithm: str
    found: bool
    states: list = field(default_factory=list)
    actions: list = field(default_factory=list)
    cost: float = float("inf")          # in the problem's metric
    expanded: int = 0                   # nodes popped & expanded
    generated: int = 0                  # successors created
    max_frontier: int = 0
    runtime_ms: float = 0.0
    explored: list = field(default_factory=list)   # expansion order (for plots)

    @property
    def nodes(self):
        out = []
        for n, _ in self.states:
            if not out or out[-1] != n:
                out.append(n)
        return out

    def row(self):
        return {"algorithm": self.algorithm, "found": self.found, "cost": round(self.cost, 1),
                "steps": len(self.actions), "expanded": self.expanded,
                "generated": self.generated, "max_frontier": self.max_frontier,
                "runtime_ms": round(self.runtime_ms, 2)}


def _reconstruct(parent, goal):
    states, actions = [goal], []
    while parent[states[-1]] is not None:
        prev, act = parent[states[-1]]
        actions.append(act)
        states.append(prev)
    return states[::-1], actions[::-1]


def _path_cost(problem, states, actions):
    total = 0.0
    for s, a in zip(states, actions):
        for nxt, c, act in problem.actions(s):
            if act == a:
                total += c
                break
    return total


def _finish(name, problem, parent, goal, stats, t0, explored):
    r = SearchResult(name, goal is not None, explored=explored, **stats)
    if goal is not None:
        r.states, r.actions = _reconstruct(parent, goal)
        r.cost = _path_cost(problem, r.states, r.actions)
    r.runtime_ms = (time.perf_counter() - t0) * 1000
    return r


# ------------------------------------------------------------- uninformed
def bfs(problem):
    """Breadth-first graph search: fewest steps, ignores edge lengths."""
    t0 = time.perf_counter()
    stats = dict(expanded=0, generated=0, max_frontier=0)
    parent = {s: None for s in problem.initial}
    for s in problem.initial:
        if problem.is_goal(s):
            return _finish("BFS", problem, parent, s, stats, t0, [])
    frontier, explored = deque(problem.initial), []
    while frontier:
        s = frontier.popleft()
        stats["expanded"] += 1
        explored.append(s[0])
        for nxt, _, act in problem.actions(s):
            stats["generated"] += 1
            if nxt not in parent:
                parent[nxt] = (s, act)
                if problem.is_goal(nxt):          # early goal test is safe for BFS
                    return _finish("BFS", problem, parent, nxt, stats, t0, explored)
                frontier.append(nxt)
        stats["max_frontier"] = max(stats["max_frontier"], len(frontier))
    return _finish("BFS", problem, parent, None, stats, t0, explored)


def dfs(problem):
    """Depth-first graph search: finds *a* route, usually a poor one."""
    t0 = time.perf_counter()
    stats = dict(expanded=0, generated=0, max_frontier=0)
    parent = {s: None for s in problem.initial}
    stack, visited, explored = list(problem.initial), set(), []
    while stack:
        s = stack.pop()
        if s in visited:
            continue
        visited.add(s)
        stats["expanded"] += 1
        explored.append(s[0])
        if problem.is_goal(s):
            return _finish("DFS", problem, parent, s, stats, t0, explored)
        for nxt, _, act in problem.actions(s):
            stats["generated"] += 1
            if nxt not in visited:
                parent[nxt] = (s, act)
                stack.append(nxt)
        stats["max_frontier"] = max(stats["max_frontier"], len(stack))
    return _finish("DFS", problem, parent, None, stats, t0, explored)


def iddfs(problem, max_depth=500):
    """Iterative deepening DFS. Memory O(depth); finds the fewest-step route.

    Uses a per-iteration 'best depth seen' table so the search does not
    enumerate every simple path of a cyclic graph.
    """
    t0 = time.perf_counter()
    stats = dict(expanded=0, generated=0, max_frontier=0)
    explored = []
    for limit in range(max_depth + 1):
        best_depth = {}
        cutoff = False
        for s0 in problem.initial:
            # each stack entry carries its own path as a linked list (prev, state, action)
            stack = [(s0, 0, (None, s0, None))]
            while stack:
                s, d, link = stack.pop()
                if best_depth.get(s, 1 << 30) <= d:
                    continue
                best_depth[s] = d
                stats["expanded"] += 1
                explored.append(s[0])
                if problem.is_goal(s):
                    parent, node = {}, link
                    while node is not None:
                        prev, st, act = node
                        parent[st] = (prev[1], act) if prev is not None else None
                        node = prev
                    r = _finish("IDDFS", problem, parent, s, stats, t0, explored)
                    r.depth_limit = limit
                    return r
                if d == limit:
                    cutoff = True
                    continue
                for nxt, _, act in problem.actions(s):
                    stats["generated"] += 1
                    if best_depth.get(nxt, 1 << 30) > d + 1:
                        stack.append((nxt, d + 1, (link, nxt, act)))
                stats["max_frontier"] = max(stats["max_frontier"], len(stack))
        if not cutoff:
            break
    return _finish("IDDFS", problem, {}, None, stats, t0, explored)


def _best_first(problem, name, priority, reopen=True):
    """Shared engine for UCS / Greedy / A*.  priority(g, h) -> key."""
    t0 = time.perf_counter()
    stats = dict(expanded=0, generated=0, max_frontier=0)
    tie = itertools.count()
    g_best = {s: 0.0 for s in problem.initial}
    parent = {s: None for s in problem.initial}
    frontier = [(priority(0.0, problem.h(s)), next(tie), 0.0, s) for s in problem.initial]
    heapq.heapify(frontier)
    closed, explored = set(), []
    while frontier:
        _, _, g, s = heapq.heappop(frontier)
        if g > g_best.get(s, float("inf")) + 1e-9:
            continue                                  # stale queue entry
        if s in closed and not reopen:
            continue
        if problem.is_goal(s):                        # goal test on expansion -> optimal
            return _finish(name, problem, parent, s, stats, t0, explored)
        closed.add(s)
        stats["expanded"] += 1
        explored.append(s[0])
        for nxt, c, act in problem.actions(s):
            stats["generated"] += 1
            g2 = g + c
            if g2 + 1e-9 < g_best.get(nxt, float("inf")):
                if nxt in closed and not reopen:
                    continue
                g_best[nxt] = g2
                parent[nxt] = (s, act)
                closed.discard(nxt)
                heapq.heappush(frontier, (priority(g2, problem.h(nxt)), next(tie), g2, nxt))
        stats["max_frontier"] = max(stats["max_frontier"], len(frontier))
    return _finish(name, problem, parent, None, stats, t0, explored)


def ucs(problem):
    """Uniform-cost search (Dijkstra): optimal, no heuristic."""
    return _best_first(problem, "UCS", lambda g, h: g)


def greedy(problem):
    """Greedy best-first: follows the heuristic only. Fast, not optimal."""
    return _best_first(problem, "Greedy", lambda g, h: h, reopen=False)


def astar(problem, w=1.0):
    """A*: f = g + w*h. w = 1 is optimal with an admissible h;
    w > 1 (weighted A*) trades optimality (bounded by factor w) for speed."""
    name = "A*" if w == 1.0 else f"WA*(w={w:g})"
    return _best_first(problem, name, lambda g, h: g + w * h)


# ------------------------------------------------------ landmark heuristic
class Landmarks:
    """ALT heuristic (A*, Landmarks, Triangle inequality).

    Pre-computes shortest walking distances d(L, .) from k landmarks chosen by
    farthest-point selection. For any node n and goal t:
        d(n, t) >= |d(L, t) - d(L, n)|
    so h(n) = max over L of that bound is admissible, and usually much tighter
    than straight-line distance on a campus where paths wind around buildings.
    Closures only make true distances longer, so the bound stays valid.
    """

    def __init__(self, g, k=6, seed_node=None):
        self.g = g
        self.dist = []
        nodes = list(g.nodes)
        cur = seed_node if seed_node is not None else nodes[0]
        far = self._dijkstra(cur)
        self.landmarks = []
        for i in range(k):
            cur = max(far, key=far.get)               # farthest from chosen set
            d = self._dijkstra(cur)
            self.landmarks.append(cur)
            self.dist.append(d)
            if i == 0:
                far = d
            else:
                far = {n: min(far[n], d.get(n, float("inf"))) for n in far}

    def _dijkstra(self, src):
        dist, pq = {src: 0.0}, [(0.0, src)]
        while pq:
            d, n = heapq.heappop(pq)
            if d > dist[n]:
                continue
            for m, e in self.g.neighbors(n):
                if "walk" in e.modes and d + e.length < dist.get(m, float("inf")):
                    dist[m] = d + e.length
                    heapq.heappush(pq, (dist[m], m))
        return dist

    def bound(self, n, goals):
        best = 0.0
        for d in self.dist:
            dn = d.get(n)
            if dn is None:
                continue
            lb = min(abs(d[t] - dn) for t in goals if t in d)
            best = max(best, lb)
        return best

    def attach(self, problem):
        """Replace problem.h with max(straight-line, landmark) bound (walk only)."""
        if problem.profile.vehicle != "walk":
            raise ValueError("ALT heuristic is built on the walking graph; use it for walking queries")
        base_h, goals = problem.h, list(problem.goals)
        cache = {}

        def h(state):
            n = state[0]
            if n not in cache:
                lb = self.bound(n, goals)
                if problem.metric != "distance":
                    lb /= problem.profile.speed("walk")
                cache[n] = max(base_h(state), lb)
            return cache[n]

        problem.h = h
        return problem


ALGORITHMS = {"bfs": bfs, "dfs": dfs, "iddfs": iddfs, "ucs": ucs,
              "greedy": greedy, "astar": astar}
