import csv
import json
from pathlib import Path
from tempfile import TemporaryDirectory
import unittest
from xml.etree import ElementTree

from benchmark import Case, bfs_cost, build_corpus, check_result, make_case, run_benchmark
from planner import Result, search
from visualize import render_svg


class CorpusTests(unittest.TestCase):
    def test_seeded_corpus_is_deterministic_and_diverse(self):
        first, second = build_corpus(), build_corpus()
        self.assertEqual(first, second)
        self.assertEqual(len(first), 14)
        self.assertEqual(len({case.sha256 for case in first}), 14)
        for case in first:
            self.assertEqual(len(case.rows), 50)
            self.assertTrue(all(len(row) == 80 for row in case.rows))
            self.assertTrue(all(cell == '#' for cell in case.rows[0] + case.rows[-1]))
            self.assertTrue(all(row[0] == row[-1] == '#' for row in case.rows))

    def test_warehouse_cross_aisles_remain_traversable(self):
        for seed in range(10):
            case = make_case('warehouse', seed=seed)
            self.assertGreater(sum(row.count('#') for row in case.rows), 256)
            self.assertIsNotNone(bfs_cost(case.rows, case.start, case.goal))
            self.assertTrue(all(row[1] == '.' for row in case.rows[1:-1]))

    def test_unreachable_cannot_cross_partition(self):
        case = make_case('unreachable')
        self.assertIsNone(bfs_cost(case.rows, case.start, case.goal))
        for algorithm in ('astar', 'dijkstra'):
            result = search(case.rows, case.start, case.goal, algorithm)
            check_result(case, result, None)
            self.assertEqual(result.expanded, 39 * 48)

    def test_oracle_known_cost(self):
        self.assertEqual(bfs_cost(('...#', '.#..', '....'), (0, 0), (3, 2)), 5)
        self.assertIsNone(bfs_cost(('.#.',), (0, 0), (2, 0)))

    def test_invalid_configuration(self):
        for parameters in ({'width': 1}, {'height': 0}, {'seeds': ()}, {'seeds': (7, 7)}):
            with self.assertRaises(ValueError):
                build_corpus(**parameters)
        with self.assertRaises(ValueError):
            make_case('warehouse')
        with self.assertRaises(ValueError):
            make_case('random-sparse')
        with self.assertRaises(ValueError):
            make_case('not-a-scenario')

    def test_illegal_route_cannot_pass_using_correct_cost_only(self):
        case = Case('known', 'fixture', None, ('..#', '...'), (0, 0), (2, 1))
        for route in (
            ((0, 0), (0, 0), (1, 0), (2, 1)),  # stationary/diagonal steps
            ((0, 0), (1, 0), (2, 0), (2, 1)),  # obstacle
            ((0, 0), (1, 0), (1, 1), (0, 1)),  # wrong goal
            ((0, 0), (-1, 0), (1, 1), (2, 1)),  # outside map
        ):
            with self.subTest(route=route), self.assertRaises(AssertionError):
                check_result(case, Result(route, 4), 3)
        with self.assertRaises(AssertionError):
            check_result(case, Result((), 0), 3)

    def test_csv_export_matches_oracle_and_records_environment(self):
        with TemporaryDirectory() as directory:
            output = Path(directory) / 'nested' / 'results.csv'
            records, metadata = run_benchmark(output, width=20, height=16, seeds=(42,), repeats=1)
            self.assertEqual(len(records), 10)
            with output.open(newline='') as stream:
                saved = list(csv.DictReader(stream))
            self.assertEqual(len(saved), 10)
            self.assertTrue(all(row['cost'] == row['bfs_cost'] for row in saved))
            self.assertTrue(all(float(row['median_ms']) >= 0 for row in saved))
            self.assertEqual(json.loads(output.with_suffix('.meta.json').read_text()), metadata)
            self.assertEqual(metadata['seeds'], [42])
            self.assertEqual(metadata['cases'], 5)
            self.assertTrue(metadata['python'])


class VisualizationTests(unittest.TestCase):
    def test_svg_contains_correct_route_and_accessible_description(self):
        svg = render_svg('S..\n.#G')
        root = ElementTree.fromstring(svg)
        namespace = {'svg': 'http://www.w3.org/2000/svg'}
        self.assertEqual(root.get('role'), 'img')
        self.assertIn('3 moves', root.find('svg:desc', namespace).text)
        points = root.find('.//svg:polyline', namespace).get('points').split()
        self.assertEqual(len(points), 4)
        self.assertEqual(svg, render_svg('S..\n.#G'))

    def test_unreachable_svg_omits_route(self):
        svg = render_svg('S#G')
        self.assertIn('No route', svg)
        self.assertNotIn('<polyline', svg)


if __name__ == '__main__':
    unittest.main()
