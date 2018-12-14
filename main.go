package main

import (
	"bufio"
	"fmt"
	"github.com/afb2001/CCOM_planner/bitStar"
	"github.com/afb2001/CCOM_planner/common"
	. "github.com/afb2001/CCOM_planner/parse"
	"github.com/afb2001/CCOM_planner/tsp"
	. "github.com/afb2001/CCOM_planner/util"
	"math/rand"
	"os"
	"strings"
)

const (
	timeToPlan float64 = 0.09 // make parameter (why is it off by a factor of 10??)
)

// globals
var maxSpeed, maxTurningRadius float64
var reader = bufio.NewReader(os.Stdin)

func main() {
	rand.Seed(4) // set seed for now

	var line string
	GetLine(reader) // start

	_, err := fmt.Sscanf(GetLine(reader), "max speed %f", &maxSpeed)
	HandleError(err, ParseErr)
	_, err = fmt.Sscanf(GetLine(reader), "max turning radius %f", &maxTurningRadius)
	HandleError(err, ParseErr)

	var grid = BuildGrid(reader)

	// PrintLog(grid.Dump())

	var path = ReadPath(reader, maxSpeed)

	solver := tsp.NewSolver(*path)

	bitStar.InitGlobals(*grid, maxSpeed, maxTurningRadius, solver)

	fmt.Println("ready")

	// planning loop
	PrintLog("ready to plan")
	for line = GetLine(reader); line != "done\n"; line = GetLine(reader) {

		if line != "plan\n" {
			continue
		}

		UpdatePath(reader, path)
		// bitStar.UpdatePath(path)
		line = GetLine(reader)
		line = strings.TrimPrefix(line, "start state ")
		start := ParseState(line)

		if grid.IsBlocked(start.X, start.Y) {
			PrintLog("Start location is blocked. Returning default plan.")
			fmt.Println(common.DefaultPlan(start))
			continue
		}

		var nObstacles int
		PrintLog("Reading dynamic obstacles")
		_, err := fmt.Sscanf(GetLine(reader), "dynamic obs %d", &nObstacles)
		o := make(common.Obstacles, nObstacles)
		HandleError(err, LogErr)
		ReadObstacles(reader, o, nObstacles)

		PrintLog("Planning...")
		//plan := bitStar.BitStar(*start, path, timeToPlan, o)
		//plan := bitStar.FindAStarPlan(*start, path, timeToPlan, o)
		plan := bitStar.PointToPointPlan(*start, path, timeToPlan, o)
		if plan == nil {
			PrintLog("Couldn't find a plan.")
			fmt.Println(common.DefaultPlan(start))
		} else {
			fmt.Println(plan.String())
		}

		PrintLog("ready to plan")
	}
}
