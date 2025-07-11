// Copyright 2023 Frederik Zipp. All rights reserved.
// Use of this source code is governed by a BSD-style
// license that can be found in the LICENSE file.

// Package pathfind finds the shortest path between two points
// constrained by a set of polygons.
package pathfind

import (
	"math"

	"github.com/fzipp/astar"
	"github.com/fzipp/geom"
	"github.com/fzipp/pathfind/internal/poly"
)

const margin = 0.002

// A Pathfinder is created and initialized with a set of polygons via
// NewPathfinder. Its Path method finds the shortest path between two points
// in this polygon set.
type Pathfinder struct {
	polygons        [][]Point
	polygonSet      poly.PolygonSet
	concaveVertices []Point
	cachedGraph     graph[Point]
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

// Path finds the shortest path from start to dest within the bounds of the
// polygons the Pathfinder was initialized with.
// If dest is outside the polygon set it will be clamped to the nearest
// polygon edge.
// The function returns nil if no path exists because start is outside
// the polygon set.
func (p *Pathfinder) Path(start, dest Point) []Point {
	if containmentLevel(p.polygonSet, start) != containmentLevel(p.polygonSet, dest) {
		return nil
	}
	d := p2v(dest)
	if !p.polygonSet.Contains(d) {
		dest = ensureInside(p.polygonSet, v2p(p.polygonSet.ClosestPt(d)))
	}
	visibilityGraph := p.prepareVisibilityGraph(start, dest)
	path := astar.FindPath[Point](visibilityGraph, start, dest, nodeDist, nodeDist)
	for i := 1; i < len(path)-1; i++ {
		path[i] = offsetFromBoundary(p.polygonSet, path[i])
	}
	return path
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
			npt := pt.Add(Point{X: float64(dx) * margin, Y: float64(dy) * margin})
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

func containmentLevel(ps poly.PolygonSet, pt Point) int {
	level := 0
	v := p2v(pt)
	for _, p := range ps {
		if p.Contains(v, true) {
			level++
		}
	}
	return level
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
	vis := make(graph[Point])
	for i, a := range points {
		for j, b := range points {
			if i == j {
				continue
			}
			if inLineOfSight(ps, p2v(a), p2v(b)) {
				vis.link(a, b)
			}
		}
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

func offsetFromBoundary(ps poly.PolygonSet, pt Point) Point {
	v := p2v(pt)
	for pi, p := range ps {
		orient := p.Orientation()
		hole := isHole(ps, pi)
		for i, pv := range p {
			if pv.NearEq(v) {
				prev := p[p.WrapIndex(i-1)]
				next := p[p.WrapIndex(i+1)]
				e1 := pv.Sub(prev).Norm()
				e2 := next.Sub(pv).Norm()
				var n1, n2 geom.Vec2
				if orient > 0 { // ccw
					n1 = geom.Vec2{X: e1.Y, Y: -e1.X}
					n2 = geom.Vec2{X: e2.Y, Y: -e2.X}
				} else { // cw
					n1 = geom.Vec2{X: -e1.Y, Y: e1.X}
					n2 = geom.Vec2{X: -e2.Y, Y: e2.X}
				}

				bis := n1.Add(n2)
				if bis.Len() == 0 {
					bis = n1
				}
				bis = bis.Norm().Mul(float32(margin))
				if hole {
					moved := pv.Add(bis)
					if ps.Contains(moved) {
						return v2p(moved)
					}
				} else {
					movedIn := pv.Sub(bis)
					if ps.Contains(movedIn) {
						return v2p(movedIn)
					}
				}
			}
		}
	}
	return pt
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
