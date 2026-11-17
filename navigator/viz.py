"""Static figures (matplotlib) of the campus graph, routes and search effort."""
from __future__ import annotations

import matplotlib

matplotlib.use("Agg")
import matplotlib.pyplot as plt  # noqa: E402

LAYER_STYLE = {
    "lane": dict(color="#8aa4c8", lw=1.6, zorder=2),
    "road": dict(color="#4a4a4a", lw=3.0, zorder=1),
    "connector": dict(color="#b9c6d8", lw=0.8, zorder=1),
    "entrance": dict(color="#c9a46a", lw=0.8, ls=":", zorder=1),
    "parking": dict(color="#7a5fb0", lw=0.8, ls=":", zorder=1),
    "gate": dict(color="#2e7d32", lw=1.0, zorder=1),
}
MODE_COLOR = {"walk": "#1b9e77", "fetch": "#66c2a5", "bike": "#d95f02", "car": "#c2185b"}


def draw_base(ax, g, labels=True, closures=None):
    for e in g.edges.values():
        a, b = g.nodes[e.u].xy, g.nodes[e.v].xy
        ax.plot([a[0], b[0]], [a[1], b[1]], solid_capstyle="round", **LAYER_STYLE[e.layer])
        if e.stairs or e.grade > 0.0833:
            ax.plot([a[0], b[0]], [a[1], b[1]], color="#e6ab02", lw=3.5, alpha=0.55, zorder=2)
    if closures:
        for eid in closures.blocked:
            e = g.edges[eid]
            a, b = g.nodes[e.u].xy, g.nodes[e.v].xy
            ax.plot([a[0], b[0]], [a[1], b[1]], color="#d32f2f", lw=4, zorder=6)
            ax.plot((a[0] + b[0]) / 2, (a[1] + b[1]) / 2, marker="X", color="#d32f2f", ms=10, zorder=7)
    for n in g.nodes.values():
        x, y = n.xy
        if n.kind == "entrance":
            ax.plot(x, y, "o", color="#c9a46a", ms=3.5, zorder=3)
        elif n.kind == "parking":
            ax.plot(x, y, "s", color="#7a5fb0", ms=6, zorder=4)
            ax.text(x, y, "P", color="white", fontsize=5, ha="center", va="center", zorder=5, weight="bold")
        elif n.kind == "gate":
            ax.plot(x, y, "D", color="#2e7d32", ms=6, zorder=4)
    if labels:
        for k, p in g.places.items():
            if not p["nodes"]:
                continue
            xs = [g.nodes[i].xy[0] for i in p["nodes"]]
            ys = [g.nodes[i].xy[1] for i in p["nodes"]]
            ax.text(sum(xs) / len(xs), sum(ys) / len(ys) + 6, k.replace("_", " "), fontsize=5.5,
                    ha="center", color="#333", zorder=8,
                    bbox=dict(boxstyle="round,pad=0.15", fc="white", ec="none", alpha=0.7))
    ax.set_aspect("equal")
    ax.set_xticks([])
    ax.set_yticks([])
    for s in ax.spines.values():
        s.set_visible(False)


def draw_route(ax, g, result, color=None, lw=3.2, label=None, alpha=0.95):
    first = True
    for i, (kind, val) in enumerate(result.actions):
        if kind != "move":
            x, y = g.nodes[result.states[i][0]].xy
            ax.plot(x, y, "*", color="black", ms=12, zorder=11)
            continue
        e = g.edges[val]
        mode = result.states[i + 1][1]
        a, b = g.nodes[e.u].xy, g.nodes[e.v].xy
        ax.plot([a[0], b[0]], [a[1], b[1]], color=color or MODE_COLOR[mode], lw=lw, zorder=9,
                alpha=alpha, solid_capstyle="round", label=label if first else None)
        first = False
    if result.states:
        s, t = g.nodes[result.states[0][0]].xy, g.nodes[result.states[-1][0]].xy
        ax.plot(*s, "o", color="white", mec="black", ms=9, zorder=12)
        ax.plot(*t, "o", color="black", ms=9, zorder=12)


def draw_explored(ax, g, result, color="#e41a1c"):
    xs = [g.nodes[n].xy[0] for n in result.explored]
    ys = [g.nodes[n].xy[1] for n in result.explored]
    ax.scatter(xs, ys, s=10, color=color, alpha=0.45, zorder=8, linewidths=0)


def campus_figure(g, path, title="SAU campus graph"):
    fig, ax = plt.subplots(figsize=(9, 8))
    draw_base(ax, g)
    from matplotlib.lines import Line2D
    handles = [Line2D([], [], **{k: v for k, v in LAYER_STYLE["lane"].items() if k != "zorder"}, label="lane (walk + 2-wheeler)"),
               Line2D([], [], **{k: v for k, v in LAYER_STYLE["road"].items() if k != "zorder"}, label="road (all modes)"),
               Line2D([], [], color="#e6ab02", lw=3.5, alpha=0.55, label="steep (> 1:12) or stairs"),
               Line2D([], [], marker="o", ls="", color="#c9a46a", label="building entrance"),
               Line2D([], [], marker="s", ls="", color="#7a5fb0", label="parking"),
               Line2D([], [], marker="D", ls="", color="#2e7d32", label="gate")]
    ax.legend(handles=handles, loc="lower left", fontsize=7, frameon=False)
    ax.set_title(title, fontsize=11)
    fig.tight_layout()
    fig.savefig(path, dpi=180)
    plt.close(fig)


def routes_figure(g, routes, path, title, closures=None):
    """routes: list of (label, Route)."""
    fig, ax = plt.subplots(figsize=(9, 8))
    draw_base(ax, g, closures=closures)
    palette = ["#1b9e77", "#d95f02", "#c2185b", "#7570b3", "#e6ab02"]
    for i, (lab, r) in enumerate(routes):
        if r.result.found:
            per_mode = len(routes) == 1
            draw_route(ax, g, r.result, None if per_mode else palette[i % len(palette)],
                       lw=4 - i * 0.6, label=lab, alpha=0.85)
    if len(routes) > 1:
        ax.legend(loc="lower left", fontsize=8, frameon=False)
    ax.set_title(title, fontsize=11)
    fig.tight_layout()
    fig.savefig(path, dpi=180)
    plt.close(fig)


def explored_figure(g, results, path, title):
    """Grid of panels showing which nodes each algorithm expanded."""
    n = len(results)
    cols = min(n, 3)
    rows = (n + cols - 1) // cols
    fig, axes = plt.subplots(rows, cols, figsize=(5.2 * cols, 4.8 * rows))
    axes = [axes] if n == 1 else list(getattr(axes, "flat", axes))
    for ax, r in zip(axes, results):
        draw_base(ax, g, labels=False)
        draw_explored(ax, g, r)
        if r.found:
            draw_route(ax, g, r, color="#111", lw=2.4)
        ax.set_title(f"{r.algorithm}: {r.expanded} expanded, cost {r.cost:.0f}", fontsize=9)
    for ax in axes[n:]:
        ax.axis("off")
    fig.suptitle(title, fontsize=11)
    fig.tight_layout()
    fig.savefig(path, dpi=160)
    plt.close(fig)


def bar_figure(rows, path, title, key="expanded"):
    fig, ax = plt.subplots(figsize=(7, 3.6))
    names = [r["algorithm"] for r in rows]
    vals = [r[key] for r in rows]
    ax.barh(names[::-1], vals[::-1], color="#5b7fb5")
    for i, v in enumerate(vals[::-1]):
        ax.text(v, i, f" {v}", va="center", fontsize=8)
    ax.set_xlabel(key.replace("_", " "))
    ax.set_title(title, fontsize=10)
    ax.spines[["top", "right"]].set_visible(False)
    fig.tight_layout()
    fig.savefig(path, dpi=160)
    plt.close(fig)


def anneal_figure(trace, optimum, path):
    fig, ax = plt.subplots(figsize=(6, 3.2))
    ax.plot([i * 50 for i in range(len(trace))], trace, color="#d95f02", lw=1.5, label="SA best-so-far")
    ax.axhline(optimum, color="#1b9e77", ls="--", lw=1.2, label="CSP optimum")
    ax.set_xlabel("iteration (all restarts)")
    ax.set_ylabel("cost (finish min + penalties)")
    ax.set_yscale("log")
    ax.legend(fontsize=8, frameon=False)
    ax.spines[["top", "right"]].set_visible(False)
    fig.tight_layout()
    fig.savefig(path, dpi=160)
    plt.close(fig)
