// Copyright 2023 Frederik Zipp. All rights reserved.
// Use of this source code is governed by a BSD-style
// license that can be found in the LICENSE file.

package pathfind_test

import (
	"fmt"

	"github.com/fzipp/pathfind"
)

func ExamplePathfinder_Path() {
	//  (0,0) >---+   +-----------+ (50,0)
	//        | s |   |   >---+   |
	//        |   +---+   |   | d |
	//        |           +---+   |
	// (0,20) +-------------------+ (50,20)
	//
	// s = start, d = destination
	polygons := [][]pathfind.Point{
		// Outer shape
		{
			pathfind.Pt(0, 0),
			pathfind.Pt(10, 0),
			pathfind.Pt(10, 10),
			pathfind.Pt(20, 10),
			pathfind.Pt(20, 0),
			pathfind.Pt(50, 0),
			pathfind.Pt(50, 20),
			pathfind.Pt(0, 20),
		},
		// Inner rectangle ("hole")
		{
			pathfind.Pt(30, 5),
			pathfind.Pt(40, 5),
			pathfind.Pt(40, 15),
			pathfind.Pt(30, 15),
		},
	}
	start := pathfind.Pt(5, 5)
	destination := pathfind.Pt(45, 10)

	pathfinder := pathfind.NewPathfinder(polygons)
	path := pathfinder.Path(start, destination)
	fmt.Println(path)
	// Output:
	// [(5,5) (10,10) (30,15) (40,15) (45,10)]
}
