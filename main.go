package main

import (
	"bufio"
	"fmt"
	"github.com/afb2001/CCOM_planner/bitStar"
	"github.com/afb2001/CCOM_planner/common"
	"log"
	"math"
	"math/rand"
	"os"
	"strings"
)

// region Constants
const (
	verbose               = true
	rrtInc        float64 = 0.5
	timeToPlan    float64 = 0.095 // make parameter (why is it off by a factor of 10??)
	dubinsDensity float64 = 1     // factor of dubinsInc

)

//endregion

//region Util
/**
Print a fatal error and die.
*/
func printError(v interface{}) {
	log.Fatal("Planner error:", v)
}

/**
Log a message to stderr.
*/
func printLog(v interface{}) {
	log.Println("Planner message:", v)
}

/**
Read a line.
I feel like I'm fighting the input stuff here...
*/
func getLine() string {
	l, _ := reader.ReadString('\n')
	return l
}

//endregion

//region Parse

// Anything that does parsing goes here

/**
Read the map from stdin and build the corresponding grid.
*/
func buildGrid() *common.Grid {
	printLog("Reading map dimensions")
	var width, height, resolution int
	fmt.Sscanf(getLine(), "map %d %d %d", &resolution, &width, &height)
	printLog("Building grid")
	grid := common.NewGrid(width*resolution, height*resolution)
	for y := height - 1; y >= 0; y-- {
		var line string
		line = getLine()
		for x, c := range line {
			if c == '#' {
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
func parseState(line string) *common.State {
	//fmt.Println("parsing line", line)
	var x, y, heading, speed, t float64
	fmt.Sscanf(line, "%f %f %f %f %f", &x, &y, &heading, &speed, &t)
	return &common.State{X: x, Y: y, Heading: (heading * -1) + math.Pi/2, Speed: speed, Time: t}
}

func readPath() *common.Path {
	p := new(common.Path)
	var pathLength int
	printLog("Reading path to cover")
	fmt.Sscanf(getLine(), "path to cover %d", &pathLength)
	var x, y float64
	for l := 0; l < pathLength; l++ {
		fmt.Sscanf(getLine(), "%f %f", &x, &y)
		s := common.State{X: x, Y: y, Speed: maxSpeed}
		*p = append(*p, s)
		if l > 0 {
			sPrev := (*p)[l-1]
			sPrev.Heading = sPrev.HeadingTo(&s)
		}
	}
	return p
}

/**
Update the given obstacle collection to account for n
updated obstacles coming from stdin.
*/
func updateObstacles(o common.Obstacles, n int) {
	for i := 0; i < n; i++ {
		var id int
		var x, y, heading, speed, t float64
		var line string = getLine()
		fmt.Sscanf(line, "%d %f %f %f %f %f", &id, &x, &y, &heading, &speed, &t)
		s := &common.State{X: x, Y: y, Heading: (heading * -1) + math.Pi/2, Speed: speed, Time: t}
		o[id] = s
	}
}

//endregion

//region main

// globals
var maxSpeed, maxTurningRadius float64
var reader = bufio.NewReader(os.Stdin)

func main() {
	rand.Seed(3) // set seed for now

	var line string
	getLine() // start

	fmt.Sscanf(getLine(), "max speed %f", &maxSpeed)
	fmt.Sscanf(getLine(), "max turning radius %f", &maxTurningRadius)

	var grid = buildGrid()

	// printLog(grid.Dump())

	var path = readPath()

	bitStar.InitGlobals(*grid, path, maxSpeed, maxTurningRadius)

	fmt.Println("ready")

	// planning loop
	printLog("ready to plan")
	for line = getLine(); line != "done\n"; line = getLine() {

		if line != "plan\n" {
			continue
		}

		printLog("Reading newly covered path")
		var covered int
		fmt.Sscanf(getLine(), "newly covered %d", &covered)
		var x, y int
		for i := 0; i < covered; i++ {
			fmt.Sscanf(getLine(), "%d %d", &x, &y)
			path = path.Without(common.State{X: float64(x), Y: float64(y)})
		}
		bitStar.UpdatePath(path)
		line = getLine()
		line = strings.TrimPrefix(line, "start state ")
		start := parseState(line)

		if grid.IsBlocked(start.X, start.Y) {
			printLog("Start location is blocked. Returning default plan.")
			fmt.Println(common.DefaultPlan(start))
			continue
		}

		var nObstacles int
		o := make(common.Obstacles)
		printLog("Reading dynamic obstacles")
		fmt.Sscanf(getLine(), "dynamic obs %d", &nObstacles)
		updateObstacles(o, nObstacles)

		// plan := makePlan(grid, start, *path, o)
		// plan := bitStar.BitStar(*start, timeToPlan, o)
		printLog("Planning...")
		plan := bitStar.FindAStarPlan(*start, timeToPlan, &o)
		if plan == nil {
			printLog("Couldn't find a plan.")
			fmt.Println(common.DefaultPlan(start))
		} else {
			fmt.Println(plan.String())
		}

		printLog("ready to plan")
	}
}

//endregion
