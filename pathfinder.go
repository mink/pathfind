// Copyright 2023 Frederik Zipp. All rights reserved.
// Use of this source code is governed by a BSD-style
// license that can be found in the LICENSE file.

// Package pathfind finds the shortest path between two points
// constrained by a set of polygons.
package pathfind

import (
	"math"
	"runtime"
	"sync"

	"github.com/fzipp/astar"
	"github.com/fzipp/geom"
	"github.com/fzipp/pathfind/internal/poly"
)

// A Pathfinder is created and initialized with a set of polygons via
// NewPathfinder. Its Path method finds the shortest path between two points
// in this polygon set.
type Pathfinder struct {
	polygons        [][]Point
	polygonSet      poly.PolygonSet
	concaveVertices []Point
	cachedGraph     graph[Point]
	visibilityGraph graph[Point]
}

// NewPathfinder creates a Pathfinder instance and initializes it with a set of
// polygons.
//
// A polygon is represented by a slice of points, i.e. []Point, describing
// the vertices of the polygon. Thus [][]Point is a slice of polygons,
// i.e. the set of polygons.
//
// Each polygon in the polygon set designates either an area that is accessible
// for path finding or a hole inside such an area, i.e. an obstacle. Nested
// polygons alternate between accessible area and inaccessible hole:
//   - Polygons at the first level are area polygons.
//   - Polygons contained inside an area polygon are holes.
//   - Polygons contained inside a hole are area polygons again.
func NewPathfinder(polygons [][]Point) *Pathfinder {
	polygonSet := convert(polygons, func(ps []Point) poly.Polygon {
		return ps2vs(ps)
	})
	concave := concaveVertices(polygonSet)
	return &Pathfinder{
		polygons:        polygons,
		polygonSet:      polygonSet,
		concaveVertices: concave,
		cachedGraph:     visibilityGraph(polygonSet, concave),
	}
}

// VisibilityGraph returns the calculated visibility graph from the last Path
// call. It is only available after Path was called, otherwise nil.
func (p *Pathfinder) VisibilityGraph() map[Point][]Point {
	return p.visibilityGraph
}

// Path finds the shortest path from start to dest within the bounds of the
// polygons the Pathfinder was initialized with.
// If dest is outside the polygon set it will be clamped to the nearest
// polygon edge.
// The function returns nil if no path exists because start is outside
// the polygon set.
func (p *Pathfinder) Path(start, dest Point) []Point {
	d := p2v(dest)
	if !p.polygonSet.Contains(d) {
		dest = ensureInside(p.polygonSet, v2p(p.polygonSet.ClosestPt(d)))
	}
	p.visibilityGraph = p.prepareVisibilityGraph(start, dest)
	return astar.FindPath[Point](p.visibilityGraph, start, dest, nodeDist, nodeDist)
}

func ensureInside(ps poly.PolygonSet, pt Point) Point {
	if ps.Contains(p2v(pt)) {
		return pt
	}
adjustment:
	for dx := -1; dx <= 1; dx++ {
		for dy := -1; dy <= 1; dy++ {
			if dx == 0 && dy == 0 {
				continue
			}
			npt := pt.Add(Point{X: float64(dx), Y: float64(dy)})
			if ps.Contains(p2v(npt)) {
				pt = npt
				break adjustment
			}
		}
	}
	return pt
}

func concaveVertices(ps poly.PolygonSet) []Point {
	var vs []Point
	for i, p := range ps {
		t := concave
		if isHole(ps, i) {
			t = convex
		}
		vs = append(vs, verticesOfType(p, t)...)
	}
	return vs
}

func isHole(ps poly.PolygonSet, i int) bool {
	hole := false
	for j, p := range ps {
		if i != j && p.Contains(ps[i][0], false) {
			hole = !hole
		}
	}
	return hole
}

type vertexType int

const (
	concave = vertexType(iota)
	convex
)

func verticesOfType(p poly.Polygon, t vertexType) []Point {
	var vs []Point
	for i, v := range p {
		isConcave := p.IsConcaveAt(i)
		if (t == concave && isConcave) || (t == convex && !isConcave) {
			vs = append(vs, v2p(v))
		}
	}
	return vs
}

func visibilityGraph(ps poly.PolygonSet, points []Point) graph[Point] {
	type edge struct {
		from, to Point
	}

	numWorkers := runtime.NumCPU()
	jobs := make(chan int, len(points))
	results := make(chan edge, len(points)*len(points))

	var wg sync.WaitGroup
	wg.Add(numWorkers)

	for w := 0; w < numWorkers; w++ {
		go func() {
			defer wg.Done()
			for i := range jobs {
				a := points[i]
				for j, b := range points {
					if i == j {
						continue
					}
					if inLineOfSight(ps, p2v(a), p2v(b)) {
						results <- edge{from: a, to: b}
					}
				}
			}
		}()
	}

	for i := range points {
		jobs <- i
	}
	close(jobs)

	go func() {
		wg.Wait()
		close(results)
	}()

	vis := make(graph[Point])
	for e := range results {
		vis.link(e.from, e.to)
	}

	return vis
}

func inLineOfSight(ps poly.PolygonSet, start, end geom.Vec2) bool {
	lineOfSight := poly.LineSeg{A: start, B: end}
	for _, p := range ps {
		if p.IsCrossedBy(lineOfSight) {
			return false
		}
	}
	return ps.Contains(lineOfSight.Middle())
}

// nodeDist is the cost function for the A* algorithm. The visibility graph has
// 2d points as nodes, so we calculate the Euclidean distance.
func nodeDist(a, b Point) float64 {
	c := a.Sub(b)
	return math.Sqrt(c.X*c.X + c.Y*c.Y)
}

func (p *Pathfinder) prepareVisibilityGraph(start, dest Point) graph[Point] {
	vis := copyGraph(p.cachedGraph)
	vis[start] = vis[start]
	vis[dest] = vis[dest]

	points := append([]Point(nil), p.concaveVertices...)
	points = append(points, dest)
	for _, b := range points {
		if b != start && inLineOfSight(p.polygonSet, p2v(start), p2v(b)) {
			vis.link(start, b)
		}
		if b != start && inLineOfSight(p.polygonSet, p2v(b), p2v(start)) {
			vis.link(b, start)
		}
	}

	points = append(p.concaveVertices, start)
	for _, b := range points {
		if b != dest && inLineOfSight(p.polygonSet, p2v(dest), p2v(b)) {
			vis.link(dest, b)
		}
		if b != dest && inLineOfSight(p.polygonSet, p2v(b), p2v(dest)) {
			vis.link(b, dest)
		}
	}

	if inLineOfSight(p.polygonSet, p2v(start), p2v(dest)) {
		vis.link(start, dest)
		vis.link(dest, start)
	}

	return vis
}

func copyGraph(src graph[Point]) graph[Point] {
	dst := make(graph[Point], len(src))
	for n, adj := range src {
		if len(adj) > 0 {
			dst[n] = append([]Point(nil), adj...)
		}
	}
	return dst
}
