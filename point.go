package pathfind

import "fmt"

type Point struct {
	X, Y float64
}

func Pt(x, y float64) Point {
	return Point{X: x, Y: y}
}

func (p Point) Add(q Point) Point {
	return Point{X: p.X + q.X, Y: p.Y + q.Y}
}

func (p Point) Sub(q Point) Point {
	return Point{X: p.X - q.X, Y: p.Y - q.Y}
}

func (p Point) String() string {
	return fmt.Sprintf("(%g,%g)", p.X, p.Y)
}
