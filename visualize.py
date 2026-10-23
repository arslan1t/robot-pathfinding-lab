"""Render a grid and its route as a self-contained, deterministic SVG."""
import argparse
from html import escape
from pathlib import Path

from planner import parse_grid, search


def render_svg(text: str, algorithm: str = 'astar') -> str:
    rows, start, goal = parse_grid(text)
    result = search(rows, start, goal, algorithm)
    width, height = len(rows[0]), len(rows)
    cell = min(42, 760 / width, 430 / height)
    left, top = 48, 138
    canvas_width = max(1040, left + width * cell + 264)
    canvas_height = max(610, top + height * cell + 128)
    panel_x = left + width * cell + 40
    label = 'A*' if algorithm == 'astar' else 'Dijkstra'
    route_label = 'No route' if result.cost is None else f'{result.cost} moves'
    pieces = [
        f'<svg xmlns="http://www.w3.org/2000/svg" width="{canvas_width:g}" height="{canvas_height:g}" viewBox="0 0 {canvas_width:g} {canvas_height:g}" role="img" aria-labelledby="title description">',
        '<title id="title">Robot Pathfinding Lab: occupancy grid route</title>',
        f'<desc id="description">{escape(label)} on a {width} by {height} grid. {route_label}; {result.expanded} expanded nodes. Green line is the returned route, dark cells are obstacles.</desc>',
        f'<rect width="{canvas_width:g}" height="{canvas_height:g}" rx="18" fill="#f8fafc"/>',
        '<g font-family="ui-sans-serif, system-ui, -apple-system, sans-serif">',
        '<text x="48" y="54" font-size="16" font-weight="700" letter-spacing="2" fill="#047857">ROBOT PATHFINDING LAB</text>',
        '<text x="48" y="96" font-size="30" font-weight="700" fill="#0f172a">A route you can inspect.</text>',
    ]
    for y, row in enumerate(rows):
        for x, marker in enumerate(row):
            color = '#334155' if marker == '#' else '#e2e8f0'
            pieces.append(f'<rect x="{left + x * cell + 1:g}" y="{top + y * cell + 1:g}" width="{cell - 2:g}" height="{cell - 2:g}" rx="3" fill="{color}"/>')
    if result.path:
        points = ' '.join(f'{left + (x + .5) * cell:g},{top + (y + .5) * cell:g}' for x, y in result.path)
        pieces.append(f'<polyline points="{points}" fill="none" stroke="#059669" stroke-width="5" stroke-linecap="round" stroke-linejoin="round"/>')
    for (x, y), mark, fill in ((start, 'S', '#047857'), (goal, 'G', '#b45309')):
        cx, cy = left + (x + .5) * cell, top + (y + .5) * cell
        pieces.append(f'<circle cx="{cx:g}" cy="{cy:g}" r="{cell * .36:g}" fill="{fill}"/>')
        pieces.append(f'<text x="{cx:g}" y="{cy + cell * .14:g}" text-anchor="middle" font-size="{cell * .42:g}" font-weight="700" fill="white">{mark}</text>')
    for dy, value, size, color, weight in (
        (22, label, 30, '#0f172a', 700),
        (66, route_label, 24, '#047857', 700),
        (98, f'{result.expanded} expanded nodes', 16, '#475569', 400),
        (156, '4-connected motion', 17, '#0f172a', 600),
        (186, 'Unit edge costs', 17, '#475569', 400),
        (216, 'Manhattan heuristic' if algorithm == 'astar' else 'Zero heuristic', 17, '#475569', 400),
        (268, f'{width} × {height} cells', 16, '#475569', 400),
    ):
        pieces.append(f'<text x="{panel_x:g}" y="{top + dy:g}" font-size="{size}" font-weight="{weight}" fill="{color}">{escape(value)}</text>')
    legend_y = top + height * cell + 42
    for x, color, title in ((left, '#334155', 'Obstacle'), (left + 170, '#e2e8f0', 'Free cell'), (left + 340, '#059669', 'Route')):
        pieces.append(f'<rect x="{x:g}" y="{legend_y - 14:g}" width="16" height="16" rx="3" fill="{color}"/>')
        pieces.append(f'<text x="{x + 26:g}" y="{legend_y:g}" font-size="16" fill="#475569">{title}</text>')
    pieces.append(f'<text x="48" y="{canvas_height - 28:g}" font-size="14" fill="#64748b">Static-grid simulation · deterministic tie-breaking · Python standard library</text>')
    pieces.extend(['</g>', '</svg>'])
    return '\n'.join(pieces) + '\n'


def main() -> None:
    parser = argparse.ArgumentParser(description=__doc__)
    parser.add_argument('map', type=Path)
    parser.add_argument('--algorithm', choices=('astar', 'dijkstra'), default='astar')
    parser.add_argument('--output', type=Path, default=Path('demo-route.svg'))
    args = parser.parse_args()
    svg = render_svg(args.map.read_text(), args.algorithm)
    args.output.parent.mkdir(parents=True, exist_ok=True)
    args.output.write_text(svg, encoding='utf-8')
    print(args.output)


if __name__ == '__main__':
    main()
