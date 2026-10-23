"""Reproducible occupancy-grid workloads and independently checked measurements.

The corpus is synthetic. Timings are observations of this Python implementation,
not robot latency estimates or guarantees of one algorithm's superiority.
"""
from __future__ import annotations

import argparse
import csv
import hashlib
import json
import platform
import random
import statistics
import sys
from collections import deque
from dataclasses import dataclass
from datetime import datetime, timezone
from pathlib import Path
from time import perf_counter_ns

from planner import Result, search


DEFAULT_SEEDS = (0, 7, 42, 2026)
Point = tuple[int, int]


@dataclass(frozen=True)
class Case:
    name: str
    scenario: str
    seed: int | None
    rows: tuple[str, ...]
    start: Point
    goal: Point

    @property
    def map_text(self) -> str:
        return '\n'.join(
            ''.join('S' if (x, y) == self.start else
                    'G' if (x, y) == self.goal else cell
                    for x, cell in enumerate(row))
            for y, row in enumerate(self.rows)
        ) + '\n'

    @property
    def sha256(self) -> str:
        return hashlib.sha256(self.map_text.encode('ascii')).hexdigest()


def make_case(scenario: str, width: int = 80, height: int = 50,
              seed: int | None = None) -> Case:
    """Create bounded rooms, random occupancy, shelves, or a disconnected room.

    All maps have solid exterior walls. Random maps clear a radius-three square
    around each endpoint (clipped to the interior) to model access areas. They
    do not carve a connecting route or reject disconnected samples.
    Warehouse racks leave vertical aisles and regular horizontal cross aisles.
    """
    if width < 16 or height < 16:
        raise ValueError('Benchmark dimensions must both be at least 16')
    if scenario not in ('open-room', 'random-sparse', 'random-dense',
                         'warehouse', 'unreachable'):
        raise ValueError(f'Unknown scenario: {scenario}')
    rng = random.Random(seed)
    cells = [['#' if x in (0, width - 1) or y in (0, height - 1) else '.'
              for x in range(width)] for y in range(height)]
    start, goal = (1, 1), (width - 2, height // 2)
    if scenario.startswith('random-'):
        if seed is None:
            raise ValueError('Random scenarios require an explicit seed')
        density = 0.18 if scenario == 'random-sparse' else 0.32
        for y in range(1, height - 1):
            for x in range(1, width - 1):
                cells[y][x] = '#' if rng.random() < density else '.'
        for cx, cy in (start, goal):
            for y in range(max(cy - 3, 1), min(cy + 4, height - 1)):
                for x in range(max(cx - 3, 1), min(cx + 4, width - 1)):
                    cells[y][x] = '.'
    elif scenario == 'warehouse':
        if seed is None:
            raise ValueError('Warehouse scenarios require an explicit seed')
        for left in range(4, width - 5, 7):
            for top in range(4, height - 4, 10):
                # Differing rack lengths model several layouts without changing
                # the fixed three-cell cross aisles and four-cell side aisles.
                length = rng.randint(4, 7)
                for y in range(top, min(top + length, height - 3)):
                    for x in range(left, min(left + 3, width - 3)):
                        cells[y][x] = '#'
    elif scenario == 'unreachable':
        for y in range(height):
            cells[y][width // 2] = '#'
    for x, y in (start, goal):
        cells[y][x] = '.'
    suffix = '' if seed is None else f'-seed-{seed}'
    return Case(scenario + suffix, scenario, seed,
                tuple(''.join(row) for row in cells), start, goal)


def build_corpus(width: int = 80, height: int = 50,
                 seeds: tuple[int, ...] = DEFAULT_SEEDS) -> tuple[Case, ...]:
    if not seeds or len(set(seeds)) != len(seeds):
        raise ValueError('Provide at least one unique seed; repeated seeds are not allowed')
    cases = [make_case('open-room', width, height)]
    for scenario in ('random-sparse', 'random-dense', 'warehouse'):
        cases.extend(make_case(scenario, width, height, seed) for seed in seeds)
    cases.append(make_case('unreachable', width, height))
    return tuple(cases)


def bfs_cost(rows: tuple[str, ...], start: Point, goal: Point) -> int | None:
    """Independent unit-cost oracle: FIFO layers, no heap or heuristic."""
    frontier = deque([(start[0], start[1], 0)])
    visited = {start}
    while frontier:
        x, y, distance = frontier.popleft()
        if (x, y) == goal:
            return distance
        for dx, dy in ((0, -1), (-1, 0), (0, 1), (1, 0)):
            nx, ny = x + dx, y + dy
            point = (nx, ny)
            if (0 <= ny < len(rows) and 0 <= nx < len(rows[0])
                    and rows[ny][nx] != '#' and point not in visited):
                visited.add(point)
                frontier.append((nx, ny, distance + 1))
    return None


def check_result(case: Case, result: Result, expected_cost: int | None) -> None:
    """Check optimal cost and every returned step, not just equal path lengths."""
    if result.cost != expected_cost:
        raise AssertionError(f'{case.name}: expected cost {expected_cost}, got {result.cost}')
    if not result.path:
        return
    if result.path[0] != case.start or result.path[-1] != case.goal:
        raise AssertionError(f'{case.name}: route endpoints do not match')
    for x, y in result.path:
        if not (0 <= y < len(case.rows) and 0 <= x < len(case.rows[0])):
            raise AssertionError(f'{case.name}: route leaves grid')
        if case.rows[y][x] == '#':
            raise AssertionError(f'{case.name}: route crosses an obstacle')
    for (x, y), (nx, ny) in zip(result.path, result.path[1:]):
        if abs(x - nx) + abs(y - ny) != 1:
            raise AssertionError(f'{case.name}: route contains a non-adjacent step')


def measure_case(case: Case, repeats: int = 9) -> list[dict]:
    if repeats < 1:
        raise ValueError('repeats must be positive')
    expected_cost = bfs_cost(case.rows, case.start, case.goal)
    records = []
    for algorithm in ('astar', 'dijkstra'):
        # Validate one warm-up before timing; oracle and path checks are outside
        # the measured search interval. Validate every timed result afterward.
        warmup = search(case.rows, case.start, case.goal, algorithm)
        check_result(case, warmup, expected_cost)
        elapsed = []
        for _ in range(repeats):
            started = perf_counter_ns()
            result = search(case.rows, case.start, case.goal, algorithm)
            elapsed.append((perf_counter_ns() - started) / 1_000_000)
            check_result(case, result, expected_cost)
            if result != warmup:
                raise AssertionError(f'{case.name}: non-deterministic {algorithm} output')
        obstacles = sum(row.count('#') for row in case.rows)
        records.append({
            'case': case.name,
            'scenario': case.scenario,
            'seed': '' if case.seed is None else case.seed,
            'width': len(case.rows[0]),
            'height': len(case.rows),
            'obstacles': obstacles,
            'map_sha256': case.sha256,
            'algorithm': algorithm,
            'reachable': expected_cost is not None,
            'cost': '' if result.cost is None else result.cost,
            'bfs_cost': '' if expected_cost is None else expected_cost,
            'expanded': result.expanded,
            'median_ms': round(statistics.median(elapsed), 6),
            'min_ms': round(min(elapsed), 6),
            'max_ms': round(max(elapsed), 6),
            'repeats': repeats,
        })
    return records


def run_benchmark(output: Path, width: int = 80, height: int = 50,
                  seeds: tuple[int, ...] = DEFAULT_SEEDS,
                  repeats: int = 9) -> tuple[list[dict], dict]:
    corpus = build_corpus(width, height, seeds)
    records = [record for case in corpus for record in measure_case(case, repeats)]
    metadata = {
        'schema_version': 1,
        'generated_at_utc': datetime.now(timezone.utc).isoformat(),
        'python': sys.version,
        'implementation': platform.python_implementation(),
        'system': platform.system(),
        'system_release': platform.release(),
        'machine': platform.machine(),
        'timer': 'time.perf_counter_ns',
        'width': width,
        'height': height,
        'seeds': list(seeds),
        'cases': len(corpus),
        'algorithms': ['astar', 'dijkstra'],
        'warmup_runs_per_algorithm_case': 1,
        'measured_runs_per_algorithm_case': repeats,
        'oracle': 'Independent breadth-first search for unit-cost grids',
        'validation': 'Optimal cost, endpoints, traversability, adjacency, deterministic output',
        'timing_scope': 'search only; excludes corpus generation, BFS, validation and file IO',
        'execution_order': 'corpus order, then astar before dijkstra; no process isolation',
        'limitations': 'Synthetic maps and single-machine microbenchmarks; timing varies with environment',
        'maps': {case.name: case.sha256 for case in corpus},
    }
    output.parent.mkdir(parents=True, exist_ok=True)
    with output.open('w', newline='', encoding='utf-8') as stream:
        writer = csv.DictWriter(stream, fieldnames=list(records[0]))
        writer.writeheader()
        writer.writerows(records)
    output.with_suffix('.meta.json').write_text(json.dumps(metadata, indent=2) + '\n')
    return records, metadata


def main() -> None:
    parser = argparse.ArgumentParser(description=__doc__)
    parser.add_argument('--output', type=Path, default=Path('benchmarks.csv'))
    parser.add_argument('--width', type=int, default=80)
    parser.add_argument('--height', type=int, default=50)
    parser.add_argument('--seeds', type=int, nargs='+', default=list(DEFAULT_SEEDS))
    parser.add_argument('--repeats', type=int, default=9)
    args = parser.parse_args()
    try:
        records, metadata = run_benchmark(args.output, args.width, args.height,
                                          tuple(args.seeds), args.repeats)
    except ValueError as error:
        parser.error(str(error))
    print(f"Validated {len(records)} algorithm/case pairs against BFS across {metadata['cases']} maps.")
    print(f'CSV: {args.output}; environment: {args.output.with_suffix(".meta.json")}')


if __name__ == '__main__':
    main()
