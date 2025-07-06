// Copyright 2023 Frederik Zipp. All rights reserved.
// Use of this source code is governed by a BSD-style
// license that can be found in the LICENSE file.

package pathfind_test

import (
	"reflect"
	"testing"

	"github.com/fzipp/pathfind"
)

// A U-shaped polygon. Origin is at the top-left corner.
//
//	 0,0 >---+   +---+ 30,0
//	     |   |   |   |
//	     |   +---+   |
//	     |           |
//	0,20 +-----------+ 30,20
var polygonU = [][]pathfind.Point{
	{
		pathfind.Pt(0, 0),
		pathfind.Pt(10, 0),
		pathfind.Pt(10, 10),
		pathfind.Pt(20, 10),
		pathfind.Pt(20, 0),
		pathfind.Pt(30, 0),
		pathfind.Pt(30, 20),
		pathfind.Pt(0, 20),
	},
}

// A square with a diamond shaped hole inside. Origin is at the top-left corner.
//
//	 0,0 >-----------+ 40,0
//	     |     >     |
//	     |    / \    |
//	     |   +   +   |
//	     |    \ /    |
//	     |     +     |
//	0,40 +-----------+ 40,40
var polygonO = [][]pathfind.Point{
	{
		// Outer rectangle
		pathfind.Pt(0, 0),
		pathfind.Pt(40, 0),
		pathfind.Pt(40, 40),
		pathfind.Pt(0, 40),
	},
	{
		// Inner diamond
		pathfind.Pt(20, 10),
		pathfind.Pt(30, 20),
		pathfind.Pt(20, 30),
		pathfind.Pt(10, 20),
	},
}

func TestPathfinderPath(t *testing.T) {
	tests := []struct {
		name     string
		polygons [][]pathfind.Point
		start    pathfind.Point
		dest     pathfind.Point
		want     []pathfind.Point
	}{
		{
			// +---+   +---+
			// | s |   |   |
			// |   +---+   |
			// | d         |
			// +-----------+
			name:     "Direct connection",
			polygons: polygonU,
			start:    pathfind.Pt(5, 5),
			dest:     pathfind.Pt(5, 15),
			want: []pathfind.Point{
				pathfind.Pt(5, 5),
				pathfind.Pt(5, 15),
			},
		},
		{
			// +---+   +---+
			// | s |   |   |
			// |   +---+   |
			// |         d |
			// +-----------+
			name:     "One corner",
			polygons: polygonU,
			start:    pathfind.Pt(5, 5),
			dest:     pathfind.Pt(25, 15),
			want: []pathfind.Point{
				pathfind.Pt(5, 5),
				pathfind.Pt(10, 10),
				pathfind.Pt(25, 15),
			},
		},
		{
			// >---+   +---+
			// | s |   | d |
			// |   +---+   |
			// |           |
			// +-----------+
			name:     "Two corners",
			polygons: polygonU,
			start:    pathfind.Pt(5, 5),
			dest:     pathfind.Pt(25, 5),
			want: []pathfind.Point{
				pathfind.Pt(5, 5),
				pathfind.Pt(10, 10),
				pathfind.Pt(20, 10),
				pathfind.Pt(25, 5),
			},
		},
		{
			// +---+   +---+
			// | s | d |   |
			// |   +---+   |
			// |           |
			// +-----------+
			name:     "No path through wall: dest clamped to polygons",
			polygons: polygonU,
			start:    pathfind.Pt(5, 5),
			dest:     pathfind.Pt(15, 5),
			want: []pathfind.Point{
				pathfind.Pt(5, 5),
				pathfind.Pt(10, 5),
			},
		},
		{
			// +---+ s +---+
			// |   | d |   |
			// |   +---+   |
			// |           |
			// +-----------+
			name:     "No path outside polygon",
			polygons: polygonU,
			start:    pathfind.Pt(15, 0),
			dest:     pathfind.Pt(15, 5),
			want:     nil,
		},
		{
			// >-----------+
			// | s   >     |
			// |    / \    |
			// |   +   +   |
			// |    \ /    |
			// |     + d   |
			// +-----------+
			name:     "Path around inner polygon",
			polygons: polygonO,
			start:    pathfind.Pt(15, 10),
			dest:     pathfind.Pt(30, 30),
			want: []pathfind.Point{
				pathfind.Pt(15, 10),
				pathfind.Pt(20, 10),
				pathfind.Pt(30, 20),
				pathfind.Pt(30, 30),
			},
		},
		{
			// >
			// | \
			// | s \ d
			// |     +-----+
			// |           |
			// +-----+     |
			//         \   |
			//           \ |
			//             +
			name: "No path out of thunderbolt shape: dest clamped to polygons",
			polygons: [][]pathfind.Point{
				{
					pathfind.Pt(0, 0),
					pathfind.Pt(100, 100),
					pathfind.Pt(200, 100),
					pathfind.Pt(200, 300),
					pathfind.Pt(100, 200),
					pathfind.Pt(0, 200),
				},
			},
			start: pathfind.Pt(30, 70),
			dest:  pathfind.Pt(100, 70),
			want: []pathfind.Point{
				pathfind.Pt(30, 70),
				pathfind.Pt(85, 85),
			},
		},
		{
			name: "ensure clamped dest inside 1",
			polygons: [][]pathfind.Point{
				{
					pathfind.Pt(70, 55),
					pathfind.Pt(250, 54),
					pathfind.Pt(300, 100),
				},
			},
			start: pathfind.Pt(180, 60),
			dest:  pathfind.Pt(181, 54),
			want: []pathfind.Point{
				pathfind.Pt(180, 60),
				pathfind.Pt(180, 55),
			},
		},
		{
			name: "ensure clamped dest inside 2",
			polygons: [][]pathfind.Point{
				{
					pathfind.Pt(73, 55),
					pathfind.Pt(100, 100),
					pathfind.Pt(76, 168),
				},
			},
			start: pathfind.Pt(90, 100),
			dest:  pathfind.Pt(74, 98),
			want: []pathfind.Point{
				pathfind.Pt(90, 100),
				pathfind.Pt(75, 97),
			},
		},
	}
	for _, tt := range tests {
		t.Run(tt.name, func(t *testing.T) {
			pathfinder := pathfind.NewPathfinder(tt.polygons)
			got := pathfinder.Path(tt.start, tt.dest)
			if !reflect.DeepEqual(got, tt.want) {
				t.Errorf(`%s
polygons: %v
Path(%v, %v)
 got: %v
want: %v`,
					tt.name, tt.polygons, tt.start, tt.dest, got, tt.want)
			}
		})
	}
}

func TestPathfinderVisibilityGraph(t *testing.T) {
	tests := []struct {
		name     string
		polygons [][]pathfind.Point
		start    pathfind.Point
		dest     pathfind.Point
		want     map[pathfind.Point][]pathfind.Point
	}{
		{
			// >---+   +---+
			// | s |   | d |
			// |   +---+   |
			// |           |
			// +-----------+
			name:     "Two corners",
			polygons: polygonU,
			start:    pathfind.Pt(5, 5),
			dest:     pathfind.Pt(25, 5),
			want: map[pathfind.Point][]pathfind.Point{
				pathfind.Pt(5, 5):   {pathfind.Pt(10, 10)},
				pathfind.Pt(10, 10): {pathfind.Pt(20, 10), pathfind.Pt(5, 5)},
				pathfind.Pt(20, 10): {pathfind.Pt(10, 10), pathfind.Pt(25, 5)},
				pathfind.Pt(25, 5):  {pathfind.Pt(20, 10)},
			},
		},
	}
	for _, tt := range tests {
		t.Run(tt.name, func(t *testing.T) {
			pathfinder := pathfind.NewPathfinder(tt.polygons)
			pathfinder.Path(tt.start, tt.dest)
			got := pathfinder.VisibilityGraph()
			if !reflect.DeepEqual(got, tt.want) {
				t.Errorf(`%s
polygons: %v
Path(%v, %v)
VisibilityGraph()
 got: %v
want: %v`,
					tt.name, tt.polygons, tt.start, tt.dest, got, tt.want)
			}
		})
	}
}
