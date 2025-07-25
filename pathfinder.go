// Copyright 2023 Frederik Zipp. All rights reserved.
// Use of this source code is governed by a BSD-style
// license that can be found in the LICENSE file.

// Package pathfind finds the shortest path between two points
// constrained by a set of polygons.
package pathfind

import (
	"math"

	"github.com/fzipp/astar"
	"github.com/fzipp/pathfind/internal/poly"
)

// A Pathfinder is created and initialized with a set of polygons via
// NewPathfinder. Its Path method finds the shortest path between two points
// in this polygon set.
type Pathfinder struct {
	polygons []poly.Polygon
	graph    graph[int]
	portals  map[[2]int]Point
	centers  []Point
}

// NewPathfinder creates a Pathfinder initialized with a navmesh.
// The polygons slice contains convex polygons that make up the navmesh.
func NewPathfinder(polygons [][]Point) *Pathfinder {
	ps := make([]poly.Polygon, len(polygons))
	centers := make([]Point, len(polygons))
	for i, p := range polygons {
		ps[i] = ps2vs(p)
		centers[i] = centroid(ps[i])
	}
	g, portals := navGraph(polygons)
	return &Pathfinder{polygons: ps, graph: g, portals: portals, centers: centers}
}

// Path returns a path from start to dest as a sequence of points crossing
// neighbouring polygons of the navmesh. If no path exists nil is returned.
func (p *Pathfinder) Path(start, dest Point) []Point {
	sIdx := p.polyIndex(start)
	dIdx := p.polyIndex(dest)
	if sIdx < 0 || dIdx < 0 {
		return nil
	}
	polyPath := astar.FindPath[int](p.graph, sIdx, dIdx, p.polyDist, p.polyDist)
	if polyPath == nil {
		return nil
	}
	pts := []Point{start}
	for i := 0; i < len(polyPath)-1; i++ {
		key := [2]int{polyPath[i], polyPath[i+1]}
		if mid, ok := p.portals[key]; ok {
			pts = append(pts, mid)
		}
	}
	pts = append(pts, dest)
	return pts
}

func (p *Pathfinder) polyIndex(pt Point) int {
	v := p2v(pt)
	for i, poly := range p.polygons {
		if poly.Contains(v, false) {
			return i
		}
	}
	return -1
}

func (p *Pathfinder) polyDist(a, b int) float64 {
	return nodeDist(p.centers[a], p.centers[b])
}

func centroid(polygon poly.Polygon) Point {
	var x, y float64
	for _, v := range polygon {
		x += float64(v.X)
		y += float64(v.Y)
	}
	n := float64(len(polygon))
	return Pt(x/n, y/n)
}

// navGraph builds a graph of polygon adjacency and the portal midpoint for each
// connection.
func navGraph(polygons [][]Point) (graph[int], map[[2]int]Point) {
	g := make(graph[int])
	portals := make(map[[2]int]Point)
	for i := range polygons {
		for j := i + 1; j < len(polygons); j++ {
			if a1, a2, ok := sharedEdge(polygons[i], polygons[j]); ok {
				g.link(i, j)
				g.link(j, i)
				mid := Pt((a1.X+a2.X)/2, (a1.Y+a2.Y)/2)
				portals[[2]int{i, j}] = mid
				portals[[2]int{j, i}] = mid
			}
		}
	}
	return g, portals
}

func sharedEdge(a, b []Point) (Point, Point, bool) {
	for i := range a {
		a1 := a[i]
		a2 := a[(i+1)%len(a)]
		for j := range b {
			b1 := b[j]
			b2 := b[(j+1)%len(b)]
			if (a1 == b2 && a2 == b1) || (a1 == b1 && a2 == b2) {
				return a1, a2, true
			}
		}
	}
	return Point{}, Point{}, false
}

func nodeDist(a, b Point) float64 {
	dx := a.X - b.X
	dy := a.Y - b.Y
	return math.Hypot(dx, dy)
}
