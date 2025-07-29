// Copyright 2023 Frederik Zipp. All rights reserved.
// Use of this source code is governed by a BSD-style
// license that can be found in the LICENSE file.

// Package pathfind finds the shortest path between two points
// constrained by a set of polygons.
package pathfind

import (
	"math"
	"sort"

	"github.com/fzipp/astar"
	"github.com/fzipp/pathfind/internal/poly"
)

// A Pathfinder is created and initialized with a set of polygons via
// NewPathfinder. Its Path method finds the shortest path between two points
// in this polygon set.
type Pathfinder struct {
	polygons []poly.Polygon
	graph    graph[int]
	portals  map[[2]int][2]Point
	centers  []Point

	boxes []polyBox // minimal spatial index
}

type AABB struct {
	minX, minY float64
	maxX, maxY float64
}

func (b AABB) Contains(pt Point) bool {
	return pt.X >= b.minX && pt.X <= b.maxX &&
		pt.Y >= b.minY && pt.Y <= b.maxY
}

type polyBox struct {
	index int
	box   AABB
}

// NewPathfinder creates a Pathfinder initialized with a navmesh.
// The polygons slice contains convex polygons that make up the navmesh.
func NewPathfinder(polygons [][]Point) *Pathfinder {
	ps := make([]poly.Polygon, len(polygons))
	centers := make([]Point, len(polygons))
	var boxes []polyBox

	for i, p := range polygons {
		ps[i] = ps2vs(p)
		centers[i] = centroid(ps[i])

		// bounding box
		minX, minY := math.MaxFloat64, math.MaxFloat64
		maxX, maxY := -math.MaxFloat64, -math.MaxFloat64
		for _, pt := range p {
			if pt.X < minX {
				minX = pt.X
			}
			if pt.Y < minY {
				minY = pt.Y
			}
			if pt.X > maxX {
				maxX = pt.X
			}
			if pt.Y > maxY {
				maxY = pt.Y
			}
		}
		boxes = append(boxes, polyBox{
			index: i,
			box:   AABB{minX, minY, maxX, maxY},
		})
	}

	// sort for binary search
	sort.Slice(boxes, func(i, j int) bool {
		return boxes[i].box.minX < boxes[j].box.minX
	})

	g, portals := navGraph(polygons)
	return &Pathfinder{polygons: ps, graph: g, portals: portals, centers: centers, boxes: boxes}
}

// Path returns a path from start to dest as a sequence of points crossing
// neighbouring polygons of the navmesh. If no path exists nil is returned.
func (p *Pathfinder) Path(start, dest Point) []Point {
	sIdx := p.polyIndexTol(start)
	dIdx := p.polyIndexTol(dest)
	if sIdx < 0 || dIdx < 0 {
		return nil
	}
	if sIdx == dIdx {
		// start and dest are in the same polygon, no need for pathfinding
		return []Point{start, dest}
	}
	if p.hasNavmeshLineOfSight(start, dest) {
		return []Point{start, dest}
	}
	polyPath := astar.FindPath[int](p.graph, sIdx, dIdx, p.polyDist, p.polyDist)
	if polyPath == nil {
		return nil
	}
	var portals [][2]Point
	for i := 0; i < len(polyPath)-1; i++ {
		key := [2]int{polyPath[i], polyPath[i+1]}
		if edge, ok := p.portals[key]; ok {
			a1, a2 := edge[0], edge[1]
			c := p.centers[polyPath[i]]
			if triArea2(a1, a2, c) < 0 {
				a1, a2 = a2, a1
			}
			portals = append(portals, [2]Point{a1, a2})
		}
	}
	pts := funnel(start, dest, portals)
	pts = cleanPath(pts)
	return pts
}

// if a point lies on the boundary it is moved a small distance towards the center of its polygon.
func (p *Pathfinder) PathWithMargin(start, dest Point) []Point {
	const margin = 0.000001
	pts := p.Path(start, dest)
	if pts == nil {
		return nil
	}
	for i, pt := range pts {
		idx := p.polyIndexTol(pt)
		if idx < 0 {
			continue
		}
		poly := p.polygons[idx]
		v := p2v(pt)
		if poly.Contains(v, false) {
			continue
		}
		c := p.centers[idx]
		dir := c.Sub(pt)
		dist := math.Hypot(dir.X, dir.Y)
		if dist == 0 {
			continue
		}
		pts[i] = Pt(pt.X+dir.X/dist*margin, pt.Y+dir.Y/dist*margin)
	}
	return pts
}

func (p *Pathfinder) hasNavmeshLineOfSight(start, end Point) bool {
	const samples = 32
	for i := 0; i <= samples; i++ {
		t := float64(i) / float64(samples)
		x := start.X + t*(end.X-start.X)
		y := start.Y + t*(end.Y-start.Y)
		pt := Pt(x, y)
		if p.polyIndexTol(pt) == -1 {
			return false
		}
	}
	return true
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

// like polyIndex but treats points on polygon edges as being inside
func (p *Pathfinder) polyIndexTol(pt Point) int {
	v := p2v(pt)

	// binary search for starting index
	low, high := 0, len(p.boxes)
	for low < high {
		mid := (low + high) / 2
		if pt.X < p.boxes[mid].box.minX {
			high = mid
		} else {
			low = mid + 1
		}
	}

	for i := low - 1; i >= 0; i-- {
		box := p.boxes[i].box
		if pt.X < box.minX {
			break
		}
		if box.Contains(pt) && p.polygons[p.boxes[i].index].Contains(v, true) {
			return p.boxes[i].index
		}
	}

	for i := low; i < len(p.boxes); i++ {
		box := p.boxes[i].box
		if pt.X > box.maxX {
			break
		}
		if box.Contains(pt) && p.polygons[p.boxes[i].index].Contains(v, true) {
			return p.boxes[i].index
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

// navGraph builds a graph of polygon adjacency and the portal edge for each
// connection.
func navGraph(polygons [][]Point) (graph[int], map[[2]int][2]Point) {
	g := make(graph[int])
	portals := make(map[[2]int][2]Point)
	for i := range polygons {
		for j := i + 1; j < len(polygons); j++ {
			if a1, a2, ok := sharedEdge(polygons[i], polygons[j]); ok {
				g.link(i, j)
				g.link(j, i)
				portals[[2]int{i, j}] = [2]Point{a1, a2}
				portals[[2]int{j, i}] = [2]Point{a2, a1}
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

func triArea2(a, b, c Point) float64 {
	return (b.X-a.X)*(c.Y-a.Y) - (b.Y-a.Y)*(c.X-a.X)
}

func funnel(start, end Point, portals [][2]Point) []Point {
	if len(portals) == 0 {
		return []Point{start, end}
	}
	portals = append(portals, [2]Point{end, end})

	apex := start
	left := portals[0][0]
	right := portals[0][1]
	apexIndex, leftIndex, rightIndex := 0, 0, 0
	path := []Point{start}

	for i := 1; i < len(portals); i++ {
		pLeft := portals[i][0]
		pRight := portals[i][1]

		if triArea2(apex, right, pRight) <= 0 {
			if apex == right || triArea2(apex, left, pRight) > 0 {
				right = pRight
				rightIndex = i
			} else {
				path = append(path, left)
				apex = left
				apexIndex = leftIndex
				left = apex
				right = apex
				leftIndex = apexIndex
				rightIndex = apexIndex
				i = apexIndex
				continue
			}
		}

		if triArea2(apex, left, pLeft) >= 0 {
			if apex == left || triArea2(apex, right, pLeft) < 0 {
				left = pLeft
				leftIndex = i
			} else {
				path = append(path, right)
				apex = right
				apexIndex = rightIndex
				left = apex
				right = apex
				leftIndex = apexIndex
				rightIndex = apexIndex
				i = apexIndex
				continue
			}
		}
	}

	if path[len(path)-1] != end {
		path = append(path, end)
	}
	return path
}

func cleanPath(path []Point) []Point {
	if len(path) < 3 {
		return path
	}
	res := []Point{path[0]}
	for i := 1; i < len(path)-1; i++ {
		prev := res[len(res)-1]
		next := path[i+1]
		cur := path[i]
		if collinear(prev, cur, next) {
			continue
		}
		res = append(res, cur)
	}
	res = append(res, path[len(path)-1])
	return res
}

func collinear(a, b, c Point) bool {
	area := triArea2(a, b, c)
	if math.Abs(area) > 1e-6 {
		return false
	}
	abx := b.X - a.X
	aby := b.Y - a.Y
	cbx := c.X - b.X
	cby := c.Y - b.Y
	return (abx*cbx >= 0) && (aby*cby >= 0)
}
