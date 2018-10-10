package main

import (
	"fmt"
	"log"
	"math"
)

//region Constants

//endregion

//region Grid
/**
Cell struct to make up grid
*/
type cell struct {
	x, y            int // location
	distanceToShore int // in cells
}

/**
Cell constructor.
*/
func newCell(x int, y int) cell {
	return cell{x: x, y: y, distanceToShore: math.MaxInt32}
}

/**
Convenience function to tell if cell is a static obstacle
*/
func (c *cell) isBlocked() bool {
	return c.distanceToShore == 0
}

/**
Grid struct for holding distances to static obstacles (shore)
*/
type grid struct {
	cells         [][]cell
	width, height int
}

/**
Create a new grid of given dimensions
*/
func newGrid(width int, height int) grid {
	cells := new([][]cell)
	for x := 0; x < width; x++ {
		col := new([]cell)
		for y := 0; y < height; y++ {
			*col = append(*col, newCell(x, y))
		}
		*cells = append(*cells, *col)
	}
	return grid{cells: *cells, width: width, height: height}
}

/**
Get the cell at x, y.
*/
func (g *grid) get(x int, y int) *cell {
	return &(g.cells[x][y])
}

/**
Block the cell at x, y. For initialization only (probably).
*/
func (g *grid) block(x int, y int) {
	g.get(x, y).distanceToShore = 0
}

//endregion

//region Obstacles

/**
Type alias for (dynamic) obstacle collection.
*/
type obstacles map[int]*state

/**
Add or update the obstacle collection with the new obstacle.
*/
func (o *obstacles) update(id int, newState *state) {
	(*o)[id] = newState
}

func (o *obstacles) remove(id int) {
	delete(*o, id)
}

/**
Check if any of the obstacles collide with the given state.

Returns a float64 probability of collision.
*/
func (o *obstacles) collisionExists(state *state) float64 {
	for _, s := range *o {
		if s.collides(s.project(s.timeUntil(state))) {
			return 1.0
		}
	}
	return 0
}

//endregion

//region State

/**
Represents a singular state in the world.
*/
type state struct {
	x, y, heading, speed, time float64
}

/**
Parse a state from a string in the format: x y heading speed time
*/
func parseState(line string) *state {
	var x, y, heading, speed, t float64
	fmt.Sscanf(line, "%f %f %f %f %f", &x, &y, &heading, &speed, &t)
	return &state{x, y, heading, speed, t}
}

/**
Returns the time in seconds until state other.
*/
func (s *state) timeUntil(other *state) float64 {
	return other.time - s.time
}

/**
True iff other is within 1m in the x and y directions and
within 1 second in time.
*/
func (s *state) collides(other *state) bool {
	return (math.Abs(s.time-other.time) < 1) &&
		(math.Abs(s.x-other.x) < 1) &&
		(math.Abs(s.y-other.y) < 1)
}

func (s *state) String() string {
	return fmt.Sprintf("%f %f %f %f %f", s.x, s.y, s.heading, s.speed, s.time)
}

/**
Projects a state to a specified time assuming constant speed and heading.
Creates a new state.

Meant to be used with future times but should work either way.
*/
func (s *state) project(time float64) *state {
	deltaT := time - s.time
	magnitude := deltaT * s.speed
	deltaX := math.Cos(s.heading+math.Pi/2) * magnitude
	deltaY := math.Sin(s.heading+math.Pi/2) * magnitude
	return &state{x: s.x + deltaX, y: s.y + deltaY, heading: s.heading, speed: s.speed, time: time}
}

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

//endregion

type plan struct {
	states []state
}

func (p *plan) String() string {
	s := fmt.Sprintf("plan %d", len(p.states))
	for _, state := range p.states {
		s += "\n" + state.String()
	}
	return s
}

type path []state

/**
Remove the given state from the path. Modifies the original path.
*/
func (p *path) remove(s state) {
	b := (*p)[0:]
	for _, x := range *p {
		if s == x {
			b = append(b, x)
		}
	}
}

func buildGrid() *grid {
	printLog("Reading map dimensions")
	var width, height int
	fmt.Scanf("map %d %d", &width, &height)
	printLog("Building grid")
	grid := newGrid(width, height)
	for y := 0; y < height; y++ {
		var line string
		fmt.Scanln(&line)
		for x, c := range line {
			if c == '#' {
				grid.block(x, y)
			}
		}
	}
	return &grid
}

func readPath() *path {
	p := new(path)
	var pathLength int
	printLog("Reading path to cover")
	fmt.Scanf("path to cover %d", &pathLength)
	var x, y float64
	for l := 0; l < pathLength; l++ {
		fmt.Scanf("%f %f", &x, &y)
		*p = append(*p, state{x: x, y: y})
	}
	return p
}

func makePlan(grid *grid, start *state, path *path) *plan {
	printLog("Starting to plan")
	var plan = new(plan)

	// go north 50m over 20 seconds
	s := state{x: start.x, y: start.y + 50, heading: 0, speed: 2.5, time: start.time + 20}
	plan.states = append(plan.states, s)

	printLog("Done planning")
	return plan
}

func updateObstacles(o *obstacles, n int) {
	for i := 0; i < n; i++ {
		var id int
		var line string
		fmt.Scanf("%d %s", &id, &line)
		state := parseState(line)
		o.update(id, state)
	}
}

var maxSpeed, maxTurningRadius float64

func main() {

	//var startTime = float64(time.Now().UnixNano()) / 10e9
	//printLog(fmt.Sprintf("Planner starting at %f", startTime))

	var line string
	fmt.Scanf("%s", &line) // "start"

	fmt.Scanf("max speed %f", &maxSpeed)
	fmt.Scanf("max turning radius %f", &maxTurningRadius)

	var grid = buildGrid()

	var path = readPath()

	fmt.Println("ready")
	printLog("ready to plan")

	//printLog(fmt.Sprint(grid, path)) // to make Go stop complaining about unused variables

	// planning loop
	for {
		fmt.Scanf("%s", &line)
		if line != "plan" {
			break
		}

		printLog("Reading newly covered path")
		var covered int
		fmt.Scanf("newly covered %d", &covered)
		var x, y int
		for i := 0; i < covered; i++ {
			fmt.Scanf("%d %d", &x, &y)
			path.remove(state{x: float64(x), y: float64(y)})
		}
		fmt.Scanf("start state %s", &line)
		start := parseState(line)

		var nObstacles int
		o := new(obstacles)
		fmt.Scanf("dynamic obs %d", nObstacles)
		updateObstacles(o, nObstacles)

		plan := makePlan(grid, start, path)
		fmt.Println(plan.String())
	}
}
