# ConvIO
### A Computational Framework for Zonal Convergence of Sensors and Actuators in Automotive E/E Architecture


---

## Motivation

The automotive industry is navigating a pivotal transition toward **Software-Defined Vehicles (SDVs)**, driven by the *Connected, Automated, Shared, and Electrified* (C.A.S.E.) megatrends. Yet this evolution faces a critical physical bottleneck: the **wiring harness**.

Traditional domain-based E/E architectures group ECUs by function, such as powertrain, chassis, ADAS, body, etc., and wire every sensor and actuator back to its domain controller, regardless of physical proximity. In modern commercial vehicles, this results in:

- Wiring harnesses exceeding **15 km** in total length
- Harness weights that represent a significant share of vehicle mass
- Assembly complexity that is difficult to scale as autonomy requirements grow
- Hundreds of ECUs are causing gateway bottlenecks and interface fragmentation

**Zone-based E/E architecture** resolves this by grouping components according to their **physical location** in the vehicle rather than their function. Each zone is served by a local **I/O Aggregator** that consolidates connections from nearby sensors and actuators before linking to a central High-Performance Computer (HPC) via an in-vehicle bus backbone. This dramatically reduces total cable length, weight, and system complexity.

Despite widespread industry recognition of this approach, a significant methodological gap existed: no validated, algorithmic framework automated the *design* of these zonal architectures. Engineers still relied on manual, heuristic-based processes, inadequate for the complexity of next-generation vehicle platforms with hundreds of I/O devices.

**ConvIO** (*Convergence of I/O*) was developed to close this gap.

---

## What ConvIO Does

ConvIO is an **infrastructure-aware optimization framework** that automates the transition from a domain-based E/E architecture to a zonal one. Unlike naive approaches that cluster I/O nodes using straight-line (Euclidean) distances, ConvIO models the vehicle chassis as a **weighted graph** and uses true shortest-path distances for all clustering and placement decisions. This ensures that zone boundaries and I/O Aggregator positions respect the physical routing constraints of the actual chassis, not abstract geometric space.


---

## Why This Problem is Hard

The **Zonal Wiring Harness Optimization Problem (ZWHOP)** couples three NP-hard subproblems simultaneously:

1. **Zone assignment** — which I/O belongs to which zone (combinatorial clustering)
2. **I/O Aggregator placement** — where to physically position the zone hub (facility location on a graph, including edge-interpolated positions)
3. **Backbone routing** — how to connect all Aggregators to the HPC (network design)

For a realistic vehicle with n=400 I/O devices, |V|=150 chassis nodes, and k=3 zones, exhaustive enumeration requires approximately **10²⁰⁰ operations**  exceeding the age of the universe by ~10¹⁷⁴ orders of magnitude on modern hardware.

ConvIO addresses this by decomposing ZWHOP into a **five-stage heuristic pipeline**, each stage solving a tractable subproblem in polynomial time. The dominant complexity term is **O(n² log n)** from the agglomerative clustering stage, requiring approximately 1.38×10⁶ operations for n=400, and executing in under 0.3 seconds.

---

## The Five-Stage Pipeline

### Stage 1 — Data Loading & Graph Construction
The chassis topology is loaded from a JSON file into a NetworkX weighted graph. I/O device coordinates from a CSV are then integrated using a **KD-tree** for efficient nearest-neighbor queries (O(n log |V|)).

For each I/O point, the framework determines the optimal chassis attachment:
- If the I/O is within a configurable threshold (default: 2000 mm) of a chassis node → **direct connection**
- Otherwise → **edge projection**: the I/O is projected perpendicularly onto the nearest chassis edge, which is split into two sub-edges at the projection point

This produces the unified **network graph** G_network, which serves as the foundation for all subsequent stages.

### Stage 2 — Optimal Zone Count Determination (Elbow Method)
KMeans clustering is applied to the 2D coordinates of I/O nodes for k ∈ [k_min, k_max]. The **Within-Cluster Sum of Squares (WCSS)** is recorded for each k. The elbow point, the k where additional zones yield diminishing returns, is automatically detected by finding the point of maximum perpendicular distance from the line connecting the first and last points of the normalized WCSS curve.

The result is a data-driven recommendation for the zone count, which the engineer can accept or override, and which is cross-checked against hardware and cost constraints.

### Stage 3 — Graph-Constrained Clustering
The final high-fidelity zone assignment uses **Agglomerative Hierarchical Clustering** operating on a precomputed **Dijkstra-derived shortest-path distance matrix**. Unlike KMeans (used only for the elbow sweep), agglomerative clustering with graph distances ensures that zone assignments reflect actual physical wiring paths along the chassis, not straight-line proximity. All-pairs shortest paths are precomputed in O(|V|² log |V|), and the resulting distance matrix is passed directly to scikit-learn's `AgglomerativeClustering` with `metric="precomputed"`.

### Stage 4 — Centroid Optimization (I/O Aggregator Placement)
For each zone, the framework evaluates all candidate I/O Aggregator locations by:
- Sampling along all chassis edges at a configurable step (default: Δt = 0.25, ~4 samples/edge)
- Including all chassis junction nodes as candidates
- Optionally, computing the geometric median of the cluster as an additional candidate

For each candidate, the total routed wiring cost to all I/O nodes in the cluster is evaluated using precomputed path lengths. The lowest-cost candidate becomes the new I/O Aggregator position. I/O nodes are then reassigned to the nearest centroid by graph distance, and the process iterates until convergence. In the validation study, convergence was achieved in **3 iterations**.

### Stage 5 — CAN Bus Backbone Routing
A **greedy nearest-neighbor heuristic** (O(k² · |V| log |V|)) constructs the communication backbone, connecting the HPC to all I/O Aggregators in a bus topology by iteratively extending the path to the nearest unvisited Aggregator via Dijkstra's shortest path. This is a fast, near-optimal approximation that avoids the O(3^k · |V|) cost of exact Steiner tree computation.

---

## Monte Carlo Validation

To validate the routing correctness of ConvIO, a **Monte Carlo Self-Avoiding Random Walk** was run for 10,000 iterations on the truck chassis graph, stochastically exploring all topological path variations between source and target nodes.

Results:
- **Three topologically distinct paths** sharing the same optimal distance of **7,487 mm** were discovered by Monte Carlo, all matching ConvIO's computed minimum
- ConvIO (Dijkstra) consistently selected one of these optimal paths
- Across 10,000 random attempts, **no path shorter than the ConvIO baseline was found**
- Monte Carlo occasionally produced sub-optimal paths with loops, adding ~22.7% cable length, inefficiencies that ConvIO's deterministic logic inherently eliminates

This cross-validation confirms that ConvIO's routing logic correctly identifies the shortest path in the graph.

---

## Project Structure

```
convio/
├── main.py                      # GUI entry point (PyQt5, MVC architecture)
├── config.yaml                  # Central configuration — all settings live here
├── requirements.txt             # Python dependencies
│
├── modules/
│   ├── graph_loader.py          # Stage 1: Chassis + I/O graph construction
│   ├── elbow_method.py          # Stage 2: WCSS sweep & elbow detection
│   ├── clustering_dijkstra.py   # Stages 3 & 4: Agglomerative clustering & centroid opt.
│   ├── hpc_connector.py         # Baseline: Direct point-to-point HPC wiring calculation
│   └── report_generator.py      # PDF report generation (ReportLab)
│
└── data/
    ├── Truck.json               # Example truck chassis graph
    └── exemplary_IO_coordinates.csv   # Example I/O node positions
```

The application follows a **Model-View-Controller (MVC)** pattern. Heavy computation runs in a `QThread` worker (`OptimizationWorker`) to keep the GUI responsive during analysis.

---

## Installation

**Requirements:** Python 3.9+

```bash
git clone https://github.com/your-org/convio.git
cd convio
pip install -r requirements.txt
python main.py
```

### Dependencies

| Category | Library | Purpose |
|---|---|---|
| Computation | `networkx` | Graph data structures, Dijkstra's algorithm |
| Computation | `numpy`, `scipy` | Vectorized algebra, KDTree spatial indexing |
| Computation | `scikit-learn` | KMeans & Agglomerative Clustering |
| GUI | `PyQt5` | Application framework and threading |
| Visualization | `pyqtgraph` | Interactive graph topology rendering |
| Visualization | `matplotlib` | Elbow plots and static report figures |
| Data | `pyyaml` | Configuration file parsing |
| Export | `reportlab` | PDF report generation |

---

## Input File Formats

### Chassis Graph (JSON)

Defines the vehicle chassis as a weighted undirected graph. Node coordinates are in millimeters (x, y, z). Edge weights represent physical routing distances in mm.

```json
{
  "nodes": ["J1", "J2", "J3", "H1"],
  "coordinates": {
    "J1": [0, 0, 0],
    "J2": [0, 675, 0],
    "J3": [0, 1525, 0],
    "H1": [320, 1720, 1260]
  },
  "edges": {
    "J1": [["J2", 675]],
    "J2": [["J1", 675], ["J3", 850]],
    "J3": [["J2", 850], ["H1", 225]],
    "H1": [["J3", 225]]
  }
}
```

Node naming follows a prefix convention used by ConvIO for type detection and visual styling:

| Prefix | Type | Description |
|---|---|---|
| `J` | Junction | Chassis junction / routing waypoints |
| `W` | Wire Junction | Wire routing junctions |
| `H` | HPC | High-Performance Computer (central compute node) |
| `I` | I/O | Input/Output nodes (injected from CSV at runtime) |
| `E` | I/O Extender | Optimally placed zone Aggregator nodes |
| `S` | Sensor | Sensor nodes |
| `A` | Actuator | Actuator nodes |
| `P` | Power | Power distribution nodes |

### I/O Coordinates (CSV)

A CSV file with `X` and `Y` column headers, one row per I/O device. Units are millimeters.

```csv
X,Y
832,400
1200,1100
3000,750
```

---

## Configuration

All application behavior is controlled through `config.yaml`. The file is validated at startup; a restart is required for changes to take effect.

### `graph_loader`
- `min_direct_node_distance_mm` (default: `2000.0`): Distance threshold below which an I/O connects directly to the nearest chassis node; above it, edge projection is used.
- `allow_projection_on_edge`: Set `false` to disable edge splitting and fall back to nearest-node only (legacy mode).

### `elbow_method`
- `k_min` / `k_max`: Search range for zone count. `k_max` is automatically capped to `io_count - 1`.
- `elbow_detection_method`: `"knee"` uses the max-perpendicular-distance heuristic automatically. Set `manual_k_override` to an integer to bypass auto-detection entirely.

### `clustering`
- `clustering_method`: `"agglomerative"` (recommended, graph-aware) or `"kmeans"` (Euclidean-only, faster but less accurate for complex chassis topologies).
- `distance_metric`: `"graph"` uses Dijkstra-derived distances; `"euclidean"` uses straight-line distances.
- `edge_sample_step`: Sampling granularity for centroid candidates along edges (0.1 = fine, 0.25 = default, 0.5 = coarse).
- `max_cluster_x_range` / `max_cluster_y_range`: Spatial bounds enforced on each zone in mm.

### `cost`
- `wire_price_per_m`, `CAN_bus_price_per_m`, `wire_weight_per_m_kg`: Used to compute cost and weight comparisons displayed in the GUI's Comparison tab.

### `profiling`
Set `enabled: true` and select a backend (`cprofile`, `pyinstrument`, or `tracemalloc`) to profile individual pipeline stages. Alternatively, set `ENABLE_PROFILING=true` as an environment variable to activate per-function profiling via the `@profile_function` decorator used across all modules.

---

## Application Workflow

The GUI guides the user through the following sequential steps:

1. **Load files** — select the chassis JSON and I/O CSV, or auto-load defaults from `config.yaml`
2. **Run analysis** — the `OptimizationWorker` thread executes all five pipeline stages
3. **Review elbow plot** — inspect the WCSS curve and accept or override the recommended zone count k
4. **Inspect cluster visualization** — view the KMeans initial assignment and the refined agglomerative result side-by-side
5. **Review comparison** — the Comparison tab displays baseline vs. zonal architecture metrics for length, cost, and weight
6. **Export** — save results as JSON or generate a PDF engineering report

---

## Output & Comparison Metrics

After analysis, the GUI's **Comparison** tab reports three scenarios:

| Scenario | Description |
|---|---|
| **Domain-based (Baseline)** | Every I/O wired directly to the HPC via Dijkstra's shortest path — the traditional star topology |
| **Zonal EEA (wire only)** | I/Os connected to their zone's I/O Aggregator; Aggregator-to-HPC backbone excluded |
| **Zonal EEA (with CAN bus)** | Full zonal architecture including the CAN bus backbone connecting each Aggregator to the HPC |

Percentage savings in total length, estimated cost, and estimated weight are computed and displayed for each scenario.

---

## Algorithmic Complexity Summary

| Pipeline Stage | Algorithm | Complexity |
|---|---|---|
| I/O Graph Integration | KD-Tree nearest neighbor | O(n log \|V\|) |
| Elbow Method (k-sweep) | KMeans × k_range | O(n · i · n · k²_range) |
| All-Pairs Shortest Path | Dijkstra from each node | O(\|V\|² log \|V\|) |
| Zone Clustering | Agglomerative (precomputed) | **O(n² log n)** ← dominant |
| Centroid Optimization | Iterative edge sampling | O(t · c · n) |
| CAN Bus Routing | Greedy nearest-neighbor | O(k² · \|V\| log \|V\|) |
| **Total (core workflow)** | | **O(n² log n)** |

For n=400 I/O devices and \|V\|=150 chassis nodes: approximately **1.38×10⁶ operations**, executing in **~0.3 seconds**.

---

## Reproducibility

Set `reproducibility.set_global_seeds: true` in `config.yaml` to fix `numpy_seed` and `sklearn_seed`. The `random_state` parameters in `elbow_method` and `clustering` mirror these values. This ensures that repeated runs on identical inputs produce identical results.

---

## Future Work

The following extensions are identified as directions for future research:

- **3D chassis modeling** — extend from 2D graph projection to a 3D environment incorporating height constraints, cable bending radii, and bundle stiffness
- **Redundant topologies** — optimize double-ring or mesh CAN backbones for ISO 26262 functional safety compliance
- **HV harness integration** — extend routing logic to High Voltage wiring with distinct safety spacing and shielding constraints
- **Multi-objective optimization** — simultaneously minimize wiring length, BOM cost, and installation weight using Pareto-front methods (e.g., NSGA-II)
- **KMeans++ initialization** — improve convergence stability for very large I/O counts
- **QoS constraints** — integrate latency and bandwidth requirements directly into the cost function for ADAS-critical zones

---

## Academic Context

ConvIO was developed as part of a Master's thesis in Autonomous Driving at **Hochschule Coburg**, conducted in collaboration with **Daimler Truck AG**. The research methodology combined a structured Rapid Review of 19 peer-reviewed publications with expert interviews from six automotive E/E architects and low-voltage harness specialists with 3–25 years of industry experience.

The work directly addresses an identified gap in the literature: the absence of validated, AI-driven, end-to-end frameworks for zonal wiring harness optimization that couple infrastructure-aware clustering, graph-based routing, and empirical industrial validation on real vehicle data.

---

## License

This project is provided for research and internal engineering use. See `LICENSE` for details.
