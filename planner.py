"""Deterministic A* and Dijkstra on rectangular, four-connected occupancy grids."""
from dataclasses import dataclass
from heapq import heappop, heappush
from itertools import count
from math import inf

@dataclass(frozen=True)
class Result:
    path: tuple[tuple[int, int], ...]
    expanded: int
    @property
    def cost(self):
        return len(self.path) - 1 if self.path else None

def parse_grid(text):
    rows = text.strip().splitlines()
    if not rows or not rows[0] or any(len(r) != len(rows[0]) for r in rows):
        raise ValueError('Grid must be a nonempty rectangle')
    if any(c not in '.#SG' for r in rows for c in r):
        raise ValueError('Allowed cells: . # S G')
    starts = [(x,y) for y,r in enumerate(rows) for x,c in enumerate(r) if c == 'S']
    goals = [(x,y) for y,r in enumerate(rows) for x,c in enumerate(r) if c == 'G']
    if len(starts) != 1 or len(goals) != 1:
        raise ValueError('Exactly one start and goal required')
    return rows, starts[0], goals[0]

def search(rows, start, goal, algorithm='astar'):
    if algorithm not in ('astar', 'dijkstra'):
        raise ValueError('Unknown algorithm')
    if not rows or not rows[0] or any(len(r) != len(rows[0]) for r in rows):
        raise ValueError('Grid must be a nonempty rectangle')
    width,height=len(rows[0]),len(rows)
    def valid(p):
        x,y=p
        return 0 <= x < width and 0 <= y < height and rows[y][x] != '#'
    if not valid(start) or not valid(goal):
        raise ValueError('Endpoints must be traversable and inside grid')
    def heuristic(p):
        return abs(p[0]-goal[0])+abs(p[1]-goal[1]) if algorithm == 'astar' else 0
    serial=count(); frontier=[(heuristic(start),next(serial),0,start)]
    distance={start:0}; parents={}; expanded=0
    while frontier:
        _,_,cost,current=heappop(frontier)
        if cost != distance[current]:
            continue
        expanded += 1
        if current == goal:
            path=[current]
            while current in parents:
                current=parents[current]; path.append(current)
            return Result(tuple(reversed(path)),expanded)
        x,y=current
        for neighbor in ((x+1,y),(x,y+1),(x-1,y),(x,y-1)):
            candidate=cost+1
            if valid(neighbor) and candidate < distance.get(neighbor,inf):
                distance[neighbor]=candidate; parents[neighbor]=current
                heappush(frontier,(candidate+heuristic(neighbor),next(serial),candidate,neighbor))
    return Result((),expanded)

if __name__ == '__main__':
    import argparse,json
    from pathlib import Path
    parser=argparse.ArgumentParser(description=__doc__)
    parser.add_argument('map',type=Path)
    parser.add_argument('--algorithm',choices=['astar','dijkstra'],default='astar')
    args=parser.parse_args()
    rows,start,goal=parse_grid(args.map.read_text())
    result=search(rows,start,goal,args.algorithm)
    cells=set(result.path)-{start,goal}
    print('\n'.join(''.join('*' if (x,y) in cells else c for x,c in enumerate(row)) for y,row in enumerate(rows)))
    print(json.dumps({'algorithm':args.algorithm,'cost':result.cost,'expanded':result.expanded}))
    raise SystemExit(0 if result.path else 2)
