# Robot Pathfinding Lab

A small, reproducible lab for comparing A* and Dijkstra on occupancy grids.
Python 3.10+, standard library only. Simulation only; not a robot controller.

## Run

```sh
python3 planner.py demo.map
python3 planner.py demo.map --algorithm dijkstra
python3 -m unittest -v
```

`S` is the start, `G` the goal, `#` an obstacle, `.` free space. Output overlays
`*` on the route and reports path cost and expanded nodes. Exit status 2 means
no route was found; 0 means a route exists.

## Design decisions

- Four-connected motion and unit edge costs; no diagonal corner cutting.
- Manhattan distance is an admissible, consistent heuristic under these assumptions.
- The priority queue uses insertion order to break ties reproducibly.
- Stale queue entries are skipped. Parents reconstruct the route without copying
  the whole path into every queue entry.
- Dijkstra uses the same engine with a zero heuristic.

The implementation keeps at most O(V) distances and parents; heap operations give
O((V + E) log V) time for this bounded-degree grid. The returned cost counts moves,
not visited cells. Expanded nodes include the goal.

## Validation

Tests compare both algorithms against an independent breadth-first-search oracle
on 150 seeded random maps, and check route adjacency, obstacles, unreachable goals,
invalid input, and a start equal to the goal. Run the tests to reproduce results;
no hardware performance or real-world navigation claims are made.

## Limits and next steps

Static maps, point robot, no dynamics or localization error. For a physical robot,
first add obstacle inflation for its footprint, sensor uncertainty and a local
collision-avoidance controller. Useful extensions: weighted terrain, D* Lite,
and measured search benchmarks on a documented map corpus.

## Development

See source and reproducible tests for the current scope. MIT licensed.
