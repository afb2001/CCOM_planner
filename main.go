package main

import (
	"bufio"
	"fmt"
	"github.com/afb2001/CCOM_planner/common"
	"github.com/afb2001/CCOM_planner/globals"
	. "github.com/afb2001/CCOM_planner/parse"
	"github.com/afb2001/CCOM_planner/rhrsaStar"
	"github.com/afb2001/CCOM_planner/tsp"
	. "github.com/afb2001/CCOM_planner/util"
	"math/rand"
	"os"
	"strings"
	"time"
)

const (
	timeToPlan float64 = 0.09 // make parameter (why is it off by a factor of 10??)
)

// globals
var maxSpeed, maxTurningRadius float64
var reader = bufio.NewReader(os.Stdin)

func main() {
	//rand.Seed(4) // set seed for now

	var line string
	GetLine(reader) // start

	PrintLog("Starting planner")

	_, err := fmt.Sscanf(GetLine(reader), "max speed %f", &maxSpeed)
	HandleError(err, ParseErr)
	_, err = fmt.Sscanf(GetLine(reader), "max turning radius %f", &maxTurningRadius)
	HandleError(err, ParseErr)

	var grid = BuildGrid(reader)

	// PrintLog(grid.Dump())

	var path = ReadPath(reader, maxSpeed)

	solver := tsp.NewSolver(*path)

	globals.InitGlobals(*grid, maxSpeed, maxTurningRadius, solver)
	//rrt.SetBoatConstants(maxSpeed, maxTurningRadius)

	SetupDebugWriter()
	defer CleanupDebugWriter()

	seed := time.Now().UnixNano()
	rand.Seed(seed)
	PrintLog("Seed:", seed)

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
		//PrintLog("Reading dynamic obstacles")
		_, err := fmt.Sscanf(GetLine(reader), "dynamic obs %d", &nObstacles)
		o := make(common.Obstacles, nObstacles)
		HandleError(err, LogErr)
		ReadObstacles(reader, o, nObstacles)

		PrintLog("Planning...")
		//plan := bitStar.BitStar(*start, path, timeToPlan, o)
		plan := rhrsaStar.FindAStarPlan(*start, path, timeToPlan, o)
		//plan := bitStar.PointToPointPlan(*start, path, timeToPlan, o)
		//plan := rrt.MakePlan(grid, start, *path, &o, timeToPlan)
		if plan == nil || len(plan.States) == 0 {
			PrintLog("Couldn't find a plan.")
			fmt.Println(common.DefaultPlan(start))
		} else {
			fmt.Println(plan.String())
			PrintLog("Found a plan of length", len(plan.States))
			//PrintLog(plan.String())
		}

		PrintLog("ready to plan")
	}
}
