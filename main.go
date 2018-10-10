package main

import (
	"fmt"
	"log"
	"math"
	"strconv"
	"strings"
	"time"
)

//region Constants
const maxSpeed = 2.5

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
	cells [][]cell
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
	return grid{cells: *cells}
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
	split := strings.Split(line, " ")
	x, _ = strconv.ParseFloat(split[0], 64)
	y, _ = strconv.ParseFloat(split[1], 64)
	heading, _ = strconv.ParseFloat(split[2], 64)
	speed, _ = strconv.ParseFloat(split[3], 64)
	t, _ = strconv.ParseFloat(split[4], 64)

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

type path struct {
	states []state
}

func buildGrid() *grid {
	log.Println("Reading map dimensions")
	var maxX, maxY, minX, minY int
	_, err := fmt.Scanf("%d %d %d %d", &maxX, &maxY, &minX, &minY)
	if err != nil {
		log.Fatal(err)
	}
	grid := newGrid(maxX, maxY)
	//for y := 0; y < maxY; y++ {
	//	var line string
	//	_, err = fmt.Scanln(&line)
	//if(len(line) != maxX) { log.Fatal("Error: wrong map dimension") }
	//for x := 0; x < maxX; x++ {
	//	fmt.Println(line[x])
	//}
	//for x, c := range line {
	//	fmt.Println(x, c)
	//}
	//}
	return &grid
}

func readPath() *path {
	return new(path)
}

func makePlan(start *state) *plan {
	var plan = new(plan)

	// go north 50m over 20 seconds
	s := state{x: start.x, y: start.y + 50, heading: 0, speed: 2.5, time: start.time + 20}
	plan.states = append(plan.states, s)

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

func main() {
	var startTime = float64(time.Now().UnixNano()) / 10e9
	fmt.Println("Planner starting at", startTime)
	//var grid = buildGrid()
	//var path = readPath()

}
