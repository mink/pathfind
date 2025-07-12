package pathfind

import "math"

type rect struct {
	min, max Point
}

func (r rect) contains(p Point) bool {
	return p.X >= r.min.X && p.X <= r.max.X && p.Y >= r.min.Y && p.Y <= r.max.Y
}

func (r rect) intersects(o rect) bool {
	return !(o.min.X > r.max.X || o.max.X < r.min.X || o.min.Y > r.max.Y || o.max.Y < r.min.Y)
}

func boundingRect(polygons [][]Point) rect {
	minX, minY := math.Inf(1), math.Inf(1)
	maxX, maxY := math.Inf(-1), math.Inf(-1)
	for _, poly := range polygons {
		for _, p := range poly {
			if p.X < minX {
				minX = p.X
			}
			if p.Y < minY {
				minY = p.Y
			}
			if p.X > maxX {
				maxX = p.X
			}
			if p.Y > maxY {
				maxY = p.Y
			}
		}
	}
	if minX == math.Inf(1) {
		minX, minY, maxX, maxY = 0, 0, 0, 0
	}
	return rect{min: Point{minX, minY}, max: Point{maxX, maxY}}
}

func queryRect(a, b Point, radius float64) rect {
	minX := math.Min(a.X, b.X) - radius
	minY := math.Min(a.Y, b.Y) - radius
	maxX := math.Max(a.X, b.X) + radius
	maxY := math.Max(a.Y, b.Y) + radius
	return rect{min: Point{minX, minY}, max: Point{maxX, maxY}}
}

type quadTree struct {
	boundary       rect
	capacity       int
	points         []Point
	divided        bool
	nw, ne, sw, se *quadTree
}

func newQuadTree(b rect, capacity int) *quadTree {
	return &quadTree{boundary: b, capacity: capacity}
}

func (qt *quadTree) insert(p Point) bool {
	if !qt.boundary.contains(p) {
		return false
	}
	if len(qt.points) < qt.capacity && !qt.divided {
		qt.points = append(qt.points, p)
		return true
	}
	if !qt.divided {
		qt.subdivide()
	}
	if qt.nw.insert(p) || qt.ne.insert(p) || qt.sw.insert(p) || qt.se.insert(p) {
		return true
	}
	return false
}

func (qt *quadTree) subdivide() {
	b := qt.boundary
	midX := (b.min.X + b.max.X) / 2
	midY := (b.min.Y + b.max.Y) / 2
	qt.nw = newQuadTree(rect{b.min, Point{midX, midY}}, qt.capacity)
	qt.ne = newQuadTree(rect{Point{midX, b.min.Y}, Point{b.max.X, midY}}, qt.capacity)
	qt.sw = newQuadTree(rect{Point{b.min.X, midY}, Point{midX, b.max.Y}}, qt.capacity)
	qt.se = newQuadTree(rect{Point{midX, midY}, b.max}, qt.capacity)
	qt.divided = true
	for _, p := range qt.points {
		qt.nw.insert(p)
		qt.ne.insert(p)
		qt.sw.insert(p)
		qt.se.insert(p)
	}
	qt.points = nil
}

func (qt *quadTree) query(r rect, found *[]Point) {
	if !qt.boundary.intersects(r) {
		return
	}
	if qt.divided {
		qt.nw.query(r, found)
		qt.ne.query(r, found)
		qt.sw.query(r, found)
		qt.se.query(r, found)
		return
	}
	for _, p := range qt.points {
		if r.contains(p) {
			*found = append(*found, p)
		}
	}
}

func (qt *quadTree) rangeSearch(r rect) []Point {
	var pts []Point
	qt.query(r, &pts)
	return pts
}
