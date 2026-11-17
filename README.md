# SAU Smart Campus Navigator Agent

A route-planning agent for the South Asian University campus (Maidan Garhi, New Delhi), built on a
graph made from our own GPS survey. It plans single and multi-stop routes for people walking, on a
2-wheeler or in a 4-wheeler, and handles:

- closures, including ones discovered mid-trip with online replanning;
- wheelchair accessibility;
- parking permissions;
- time-of-day congestion with uncertain travel times;
- deadlines and opening hours across several stops.

Every algorithm is written from scratch in plain Python; the only dependency is matplotlib, for figures.

```bash
# 1. Run all scenarios, benchmarks and figure generation
python demo.py

# 2. Run the test suite (standalone or with pytest)
python tests/test_navigator.py   # runs standalone (no extra dependencies)
python -m pytest -q              # runs via pytest (if installed)

# 3. CLI usage (works via module or direct script)
python -m navigator places
python -m navigator route GATE1 LIBRARY --vehicle car
python cli.py route BOYS_HOSTEL AB4 --algo bfs

# 4. Interactive Web Apps
python -m http.server -d web 8000   # open http://localhost:8000
```

---

## 1. Data: the campus graph

| File (in `data/`) | Source zip | Content |
|---|---|---|
| `walkways.geojson` / `.csv` | SAU_Project_1 | **Lane** network, 7.6 km trace with altitudes. Usable by walkers and 2-wheelers. |
| `roads.geojson` | SAU_Vehicleonly_Route | 13 **road** polylines. Usable by every mode, walking included. |
| `entrances.geojson` / `.csv` | SAU_Project | 42 named points: building entrances and Gates 1–3. |
| `parking.geojson` | SAU_Project_2 | 10 parking spots plus the Boys' Hostel parking road. |
| `places.json` | hand-made | Groups entrances into 27 places (AB1…AB8, LIBRARY, ADMIN, …). |
| `overrides.json` | hand-made | Stairs, parking rules, speeds, congestion zones. Edit freely. |

`navigator/graph.py` cleans up the hand-clicked traces:

1. It merges vertices closer than 2 m. The walkway trace doubles back on itself many times.
2. It splices T-junctions that were clicked within 4 m of a path but not on it.
3. It joins lanes to roads wherever they come within 7 m.
4. It attaches every entrance, gate and parking spot to the nearest path.

**Result:** 288 nodes and 408 edges. Every place is reachable on foot, and all 3 gates and all 10
parking spots are reachable by car and by 2-wheeler.

### Movement model

| Edge layer | Walk | 2-wheeler | 4-wheeler |
|---|---|---|---|
| lane | ✅ | ✅ | ❌ |
| road | ✅ | ✅ | ✅ |
| entrance link (door → path) | ✅ | ❌ | ❌ |
| parking / gate link | inherits from the layer it attaches to | | |

Vehicles may only be left at one of the 10 parking spots. The final leg is always on foot.

---

## 2. The agent (PEAS)

| | |
|---|---|
| **Performance** | Least expected time (or distance, or the risk-averse cost); deadlines and opening hours met; accessibility respected; high probability of arriving on time |
| **Environment** | The SAU campus graph, closures, congestion by time of day |
| **Actuators** | A route with turn-by-turn legs, a visiting order with a schedule, replanned routes |
| **Sensors** | The user query (start, destinations, vehicle, role, wheelchair, departure time, deadlines) and closure reports |

**Environment properties:**
- partially observable (closures are discovered en route);
- stochastic (travel times vary);
- sequential;
- dynamic (closures can appear while the agent is travelling);
- discrete (it is a graph);
- single-agent.

---

## 3. Point-to-point routing as state-space search (`problem.py`, `search.py`)

| Component | Definition |
|---|---|
| **State** | `(node, mode)` with mode ∈ {`fetch`, `car`, `bike`, `walk`}. `fetch` means walking to where your vehicle is parked. |
| **Initial state** | `(start, car/bike)` at a gate or parking spot; `(start, fetch)` at a building with a vehicle; `(start, walk)` on foot |
| **Actions** | **MOVE** along an open edge the mode may use. **UNPARK** at your vehicle's spot (fetch → vehicle). **PARK** at a permitted parking spot (vehicle → walk). |
| **Goal test** | On foot at *any* entrance of the destination place (a multi-goal search) |
| **Path cost** | `distance` (m), `time` (expected s, congestion-aware) or `safe` (mean + k·σ per edge) |
| **Heuristic** | Haversine distance to the nearest goal entrance ÷ the fastest speed still available in that state |

**Why the heuristic is admissible.** No path is shorter than the straight line. Every edge's time is
length ÷ speed × a congestion factor ≥ 1. Wheelchair and avoid-roads penalties only increase costs.
So *h* never overestimates. A mode switch can make *h* drop by more than the switch cost, which makes
it inconsistent there, so the A* implementation re-opens closed states whenever it finds a cheaper
path. This keeps A* optimal, and the tests check it against UCS.

**Landmark (ALT) heuristic.** Six landmarks are chosen by farthest-point selection, with exact walking
distances precomputed from each. By the triangle inequality, *d(n,t) ≥ |d(L,t) − d(L,n)|*. This bound
is admissible and much tighter than the straight line on a campus where paths wind around buildings.
Closures only lengthen real distances, so the bound stays valid.

**Algorithms implemented:** BFS, DFS, IDDFS, UCS (Dijkstra), Greedy best-first, A*, weighted A*, and
A* + ALT.

On a typical query (Boys' Hostel → AB4, on foot):

| Algorithm | Cost (s) | Expanded |
|---|---|---|
| BFS | 585 | 227 |
| DFS | 1621 | 139 |
| IDDFS | 624 | 13606 |
| UCS | **565** | 235 |
| Greedy | 596 | 29 |
| A* | **565** | 168 |
| A* + ALT | **565** | 80 |

Averaged over all 351 place pairs: UCS 152 nodes, A* 67, A* + ALT 42, with all three optimal. Greedy expands only 21 but is 11 % worse on cost. Full tables are
in `outputs/results.md`.

**Vehicle decision.** A* does not loop over parking spots; it chooses one inside the search, because
"where to PARK" is just another action. If you start at a building with a vehicle, the agent may
decide *not* to use it. Boys' Hostel → AB4 by car is slower than walking once you count 3 min to
unpark and 3 min to park.

**Constraints:**
- **Closures:** blocking an edge for some or all modes; see `Closures` and `NavigatorAgent.report_closure`.
- **Wheelchair:** edges with stairs or a grade above 1:12 are forbidden, grades above 5 % are slowed, and travel speed is 0.9 m/s.
- **Parking permissions:** set per role in `overrides.json`.

**Online replanning** (`multistop.follow_with_replanning`). The agent follows its plan, senses closed
edges next to its current node, updates what it believes is closed, and re-runs A* from where it
stands.

---

## 4. Travel-time uncertainty (`uncertainty.py`)

Each edge's time is log-normal:
- **mean** = length ÷ speed × congestion factor;
- **standard deviation** = CV × mean.

Three hotspots raise both the factor and the CV during set time windows: the mess at lunch, the
academic blocks at class change, and the main gate at rush hour. Monte Carlo sampling (5000 draws)
gives the median, the 95th percentile and P(arrival ≤ deadline).

The `safe` metric (mean + 1.5σ per edge) prefers predictable routes. Library → AB4 at 09:55 with an
8.1-minute deadline shows why this matters:

| Objective | Mean (s) | P(on time) |
|---|---|---|
| Fastest on average | 463 | **0.79** |
| Risk-averse | longer | **0.90** |

The risk-averse route is slower on average but much more predictable, so it arrives on time more often.

---

## 5. Multi-stop routing under deadlines (`multistop.py`)

**Travel-time matrix.** The agent first runs A* between every pair of stops. The resulting times
become the CSP's constraint data.

### CSP / constraint optimisation

| | |
|---|---|
| **Variables** | S₁…Sₙ (the stop visited i-th); arrival times Tᵢ, derived |
| **Domains** | Sᵢ ∈ {the requested places} |
| **Constraints** | AllDifferent(S); Tᵢ = depart(Sᵢ₋₁) + travel(Sᵢ₋₁, Sᵢ); wait until the place opens; Tᵢ ≤ deadline(Sᵢ); precedence (A before B); optional end place with its own deadline |
| **Objective** | Minimise the finish time |

**Solver:** backtracking with:
- **Value ordering:** earliest deadline first.
- **Forward checking:** after each assignment, every unvisited stop must still be reachable directly
  before its deadline. Going there directly is a lower bound because of the triangle inequality on
  shortest paths.
- **Branch and bound:** prune when the current time + the farthest remaining stop + the remaining
  service time ≥ the best finish found so far.

**Infeasible requests.** The planner reports *why* no order works.

**Local search:**
- **Simulated annealing:** swap, relocate and 2-opt moves; geometric cooling; restarts.
- **Steepest-ascent hill climbing** with random restarts.
- **Objective:** finish time + 100 × minutes late + 1000 × precedence violations.

Both run to completion even on infeasible requests and return the least-late order.

**Verification.** On random 5-stop tours, the CSP matches brute force in all 8 tested cases. On
9 stops (362,880 possible orders), the CSP explores 700 nodes.

**Risk.** Monte Carlo gives P(every deadline met) for the chosen plan.

---

## 6. Interactive web applications

The project provides two interactive web interfaces:

1. **Vector Campus Navigator App (`web/index.html` & `outputs/navigator_app.html`):**
   - Self-contained, responsive single-page application with dark/light mode and zero external dependencies.
   - **Directions:** Multi-modal routing (walk, 2-wheeler, car) with parking selection and turn-by-turn legs.
   - **Algorithms:** Live visual side-by-side comparison of BFS, DFS, UCS, Greedy, and A*, highlighting explored nodes.
   - **Multi-stop:** Full TSPTW schedule optimizer with opening windows, deadlines, and precedence.
   - **Closures & Replanning:** Interactive edge closures with simulated en-route replanning.

2. **Satellite Campus Map (`outputs/campus_map.html`):**
   - Interactive Leaflet map overlaying the surveyed network onto ESRI high-resolution satellite imagery.
   - Click any path segment to toggle road closures and re-route in real-time.

Serve locally with:
```bash
python -m http.server -d web 8000
# or open web/index.html directly in any browser
```

---

## 7. Project layout

```
sau_navigator/
├── pyproject.toml       # Standard PEP 517/621 package metadata & scripts
├── .gitignore           # Git ignore patterns for Python, builds & caches
├── requirements.txt     # Python requirements (matplotlib, pytest)
├── cli.py               # Root CLI convenience wrapper
├── demo.py              # End-to-end benchmark & scenario generator
├── data/                # GPS survey exports & domain configuration
│   ├── walkways.*       # Pedestrian & 2-wheeler lane network (GeoJSON + CSV with altitudes)
│   ├── roads.geojson    # Road polylines for all vehicle modes
│   ├── entrances.*      # 42 building entrances & gates (GeoJSON + CSV)
│   ├── parking.geojson  # 10 parking spots & access roads
│   ├── places.json      # Grouping of entrance nodes into 27 destinations
│   └── overrides.json   # Speeds, parking permissions, stairs & congestion zones
├── navigator/           # Core Python engine (pure Python algorithms)
│   ├── __init__.py      # Package public API exports & version
│   ├── __main__.py      # Module entry point (python -m navigator)
│   ├── cli.py           # Command-line interface implementation
│   ├── agent.py         # NavigatorAgent controller (PEAS framework)
│   ├── geo.py           # Haversine distance & local 2D projection
│   ├── graph.py         # CampusGraph builder, topology snapping & cleaning
│   ├── multistop.py     # Multi-stop CSP, simulated annealing & replanning
│   ├── problem.py       # RouteProblem state-space search formulation
│   ├── search.py        # 8 graph search algorithms & ALT landmark heuristic
│   ├── uncertainty.py   # Log-normal congestion model & Monte Carlo simulation
│   ├── viz.py           # Matplotlib publication figures
│   └── webmap.py        # Interactive Leaflet satellite map generator
├── web/                 # Web application assets
│   └── index.html       # Self-contained modern interactive vector navigator app
├── outputs/             # Generated benchmark artifacts & figures
│   ├── results.md       # Benchmark results from demo.py
│   ├── campus_map.html  # Generated satellite map
│   ├── navigator_app.html
│   └── *.png            # 9 generated evaluation figures
└── tests/
    ├── __init__.py
    └── test_navigator.py # 27 unit tests (runs with pytest or standalone)
```

## 8. Assumptions and limitations

- **Placeholder values to replace with real ones:**
  - Speeds: walk 1.3 m/s, wheelchair 0.9 m/s, 2-wheeler 15 km/h, car 20 km/h.
  - Park and unpark times: 1 min for a 2-wheeler, 3 min for a car.
  - Congestion windows.
  - Parking rules: Faculty Parking for faculty; hostel parking for residents and faculty.
  - The stairs entry near the Amphitheatre.

  All of these live in `data/overrides.json`.
- **Slopes:** these come from the survey's altitude values, which are noisy over short distances, so
  grades are measured over at least 10 m. Treat the "steep" flags as estimates and confirm them on
  the ground.
- **Inside buildings:** travel between two entrances of the same building is treated as free. A
  multi-stop leg can leave by any entrance.
- **No one-way roads** are modelled. Add a direction to an edge in `graph.py` if needed.
- **Edge times are independent** in the uncertainty model, and congestion is evaluated at the
  departure time for the whole trip. Trips last only minutes, so this is a reasonable simplification.
#   s m a r t - c a m p u s - n a v i g a t o r  
 