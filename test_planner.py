import random,unittest
from collections import deque
from planner import parse_grid,search

def bfs(rows,start,goal):
    queue=deque([(start,0)]);seen={start}
    while queue:
        (x,y),cost=queue.popleft()
        if (x,y)==goal:return cost
        for a,b in ((x+1,y),(x-1,y),(x,y+1),(x,y-1)):
            if 0<=b<len(rows) and 0<=a<len(rows[0]) and rows[b][a]!='#' and (a,b) not in seen:
                seen.add((a,b));queue.append(((a,b),cost+1))
    return None

class PlannerTests(unittest.TestCase):
    def test_random_maps_against_independent_bfs(self):
        rng=random.Random(42)
        for _ in range(150):
            rows=[''.join('#' if rng.random()<.3 else '.' for _ in range(12)) for _ in range(10)]
            rows[0]='.'+rows[0][1:];rows[-1]=rows[-1][:-1]+'.'
            for algorithm in ('astar','dijkstra'):
                result=search(rows,(0,0),(11,9),algorithm)
                self.assertEqual(result.cost,bfs(rows,(0,0),(11,9)))
                for a,b in zip(result.path,result.path[1:]):
                    self.assertEqual(abs(a[0]-b[0])+abs(a[1]-b[1]),1)
                    self.assertNotEqual(rows[b[1]][b[0]],'#')
    def test_unreachable(self):
        rows,start,goal=parse_grid('S#G');self.assertEqual(search(rows,start,goal).path,())
    def test_same_endpoint(self):
        self.assertEqual(search(['.'],(0,0),(0,0)).cost,0)
    def test_invalid_maps(self):
        for text in ('','S..','SSG','S?G','S.\nG'):
            with self.assertRaises(ValueError):parse_grid(text)
    def test_blocked_endpoint(self):
        with self.assertRaises(ValueError):search(['.#'],(0,0),(1,0))

if __name__=='__main__':unittest.main()
