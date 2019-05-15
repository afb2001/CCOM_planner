package parse

import (
	"bufio"
	"fmt"
	"github.com/afb2001/CCOM_planner/common"
	. "github.com/afb2001/CCOM_planner/util"
	"math"
	"strconv"
	"strings"
)

// Anything that does parsing goes here

/**
Read a line.
I feel like I'm fighting the input stuff here...
*/
func GetLine(reader *bufio.Reader) string {
	l, _ := reader.ReadString('\n')
	return l
}

/**
Read the map from stdin and build the corresponding grid.
*/
func BuildGridOld(reader *bufio.Reader) *common.Grid {
	PrintLog("Reading map dimensions")
	var width, height, resolution int
	_, err := fmt.Sscanf(GetLine(reader), "map %d %d %d", &resolution, &width, &height)
	HandleError(err, ParseErr)
	PrintLog("Building grid")
	grid := common.NewGrid(width*resolution, height*resolution)
	for y := height - 1; y >= 0; y-- {
		var line string
		line = GetLine(reader)
		for x, c := range line {
			if c == '#' {
				grid.BlockRange(x*resolution, y*resolution, resolution)
			}
		}
	}
	return &grid
}

func BuildGrid(reader *bufio.Reader) *common.Grid {
	PrintLog("Reading map dimensions")
	var width, height, resolution int
	_, err := fmt.Sscanf(GetLine(reader), "map %d %d %d", &resolution, &width, &height)
	HandleError(err, ParseErr)
	PrintLog("Building grid")
	grid := common.NewGrid(width*resolution, height*resolution)
	for y := height - 1; y >= 0; y-- {
		line := strings.Fields(GetLine(reader))
		if len(line) == 0 { // hit an error with this once
			continue
		}
		block := line[0] == "#"
		line = line[1:]
		var x int
		for _, s := range line {
			col, _ := strconv.Atoi(s)
			if block {
				for ; x < col; x++ {
					grid.BlockRange(x*resolution, y*resolution, resolution)
				}
			} else {
				x = col
			}
			block = !block
		}
		if block {
			for ; x < width; x++ {
				grid.BlockRange(x*resolution, y*resolution, resolution)
			}
		}

	}
	return &grid
}

/**
Parse a state from a string in the format: x y heading speed time.
Turns heading into angle.
*/
func ParseState(line string) *common.State {
	var x, y, heading, speed, t float64
	_, err := fmt.Sscanf(line, "%f %f %f %f %f", &x, &y, &heading, &speed, &t)
	HandleError(err, ParseErr)
	return &common.State{X: x, Y: y, Heading: (heading * -1) + math.Pi/2, Speed: speed, Time: t}
}

func ReadPath(reader *bufio.Reader, maxSpeed float64) *common.Path {
	p := new(common.Path)
	var pathLength int
	_, err := fmt.Sscanf(GetLine(reader), "path to cover %d", &pathLength)
	PrintLog("Reading path to cover of length", pathLength)
	HandleError(err, ParseErr)
	var x, y float64
	for l := 0; l < pathLength; l++ {
		_, err := fmt.Sscanf(GetLine(reader), "%f %f", &x, &y)
		HandleError(err, ParseErr)
		s := common.State{X: x, Y: y, Speed: maxSpeed}
		*p = append(*p, s)
		if l > 0 {
			sPrev := (*p)[l-1]
			sPrev.Heading = sPrev.HeadingTo(&s)
		}
	}
	return p
}

func UpdatePath(reader *bufio.Reader, path *common.Path) {
	//PrintLog("Reading newly covered path")
	var covered int
	_, err := fmt.Sscanf(GetLine(reader), "newly covered %d", &covered)
	HandleError(err, ParseErr)
	var x, y float64
	for i := 0; i < covered; i++ {
		_, err := fmt.Sscanf(GetLine(reader), "%f %f", &x, &y)
		HandleError(err, ParseErr)
		*path = path.Without(common.State{X: x, Y: y})
	}
}

/**
Update the given obstacle collection to account for n
updated obstacles coming from stdin.
*/
func ReadObstacles(reader *bufio.Reader, o common.Obstacles, n int) {
	for i := 0; i < n; i++ {
		var id int
		var x, y, heading, speed, t float64
		var line string = GetLine(reader)
		_, err := fmt.Sscanf(line, "%d %f %f %f %f %f", &id, &x, &y, &heading, &speed, &t)
		HandleError(err, ParseErr)
		s := &common.State{X: x, Y: y, Heading: (heading * -1) + math.Pi/2, Speed: speed, Time: t}
		// this is a change from when o was a map
		o[i] = s
	}
}
