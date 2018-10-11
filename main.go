package main

import (
	"bufio"
	"fmt"
	"log"
	"math"
	"math/rand"
	"os"
	"strings"
	"time"
)

//region Constants
const (
	rrtInc       float64 = 0.5
	timeToPlan   float64 = 0.5 // TOOO! -- make parameter
	goalBias     float64 = 0.1
	maxSpeedBias float64 = 0.5
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

/**
Determine if a given point is within a static obstacle.
*/
func (g *grid) isBlocked(x float64, y float64) bool {
	return g.get(int(x), int(y)).isBlocked()
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
Returns the time in seconds until state other.
*/
func (s *state) timeUntil(other *state) float64 {
	return other.time - s.time
}

/**
Returns the Euclidean distance in two dimensions (x,y).
*/
func (s *state) distanceTo(other *state) float64 {
	return math.Sqrt(math.Pow(s.x-other.x, 2) + math.Pow(s.y-other.y, 2))
}

func (s *state) headingTo(other *state) float64 {
	dx := s.x - other.x
	dy := s.y - other.y
	h := math.Atan2(dy, dx)
	if h < 0 {
		return h + (2 * math.Pi) // may not need this? I don't remember how tangents work
	}
	return h
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
Create a string representation of the state.
Angle is turned back into heading.
*/
func (s *state) String() string {
	return fmt.Sprintf("%f %f %f %f %f", s.x, s.y, (-1*s.heading)+math.Pi/2, s.speed, s.time)
}

/**
Projects a state to a specified time assuming constant speed and heading.
Creates a new state.

Meant to be used with future times but should work either way.
*/
func (s *state) project(time float64) *state {
	deltaT := time - s.time
	magnitude := deltaT * s.speed
	deltaX := math.Cos(s.heading) * magnitude
	deltaY := math.Sin(s.heading) * magnitude
	return &state{x: s.x + deltaX, y: s.y + deltaY, heading: s.heading, speed: s.speed, time: time}
}

/**
Push a state at a given angle for a given distance.
Mutates the current state.

Written for ray-casting of sorts during collision checking.
Probably should not use past version 0.
*/
func (s *state) push(heading float64, distance float64) {
	dx := distance * math.Cos(heading)
	dy := distance * math.Sin(heading)
	s.x += dx
	s.y += dy
}

//endregion

//region Path

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

//endregion

//region Plan
type plan struct {
	states []*state
}

func (p *plan) String() string {
	s := fmt.Sprintf("plan %d", len(p.states))
	for _, state := range p.states {
		s += "\n" + state.String()
	}
	return s
}

/**
Default do-nothing plan.
*/
func defaultPlan(start *state) *plan {
	plan := new(plan)
	s := state{x: start.x, y: start.y, heading: start.heading, speed: start.speed, time: start.time + 1.0}
	plan.states = append(plan.states, &s)
	return plan
}

func makePlan(grid *grid, start *state, path path) *plan {
	printLog("Starting to plan")
	//var plan = new(plan)
	//
	//// go north 50m over 20 seconds
	//s := state{x: start.x, y: start.y + 50, heading: 0, speed: 2.5, time: start.time + 20}
	//plan.states = append(plan.states, s)

	if len(path) == 0 {
		return defaultPlan(start)
	}
	// for now...
	goal := path[0]

	p := rrt(grid, start, &goal)

	printLog("Done planning")
	return p
}

//endregion

//region RRT

type rrtNode struct {
	state  *state
	parent *rrtNode
}

/**
Create a new state with random values.
Time is unset (zero).
*/
func randomState(bounds *grid) *state {
	s := new(state)
	s.x = rand.Float64() * float64(bounds.width)
	s.y = rand.Float64() * float64(bounds.height)
	s.heading = rand.Float64() * math.Pi * 2
	s.speed = rand.Float64() * maxSpeed
	return s
}

/**
Create a random sample using the biasing constants.
*/
func biasedRandomState(bounds *grid, goal *state) *state {
	s := randomState(bounds)
	if r := rand.Float64(); r < maxSpeedBias {
		s.speed = maxSpeed
	}
	if r := rand.Float64(); r < goalBias {
		return goal
	}
	return s
}

func randomNode(bounds *grid, goal *state) *rrtNode {
	n := rrtNode{state: biasedRandomState(bounds, goal)}
	return &n
}

/**
Find the closest node in a list of nodes.
O(n) time.
*/
func getClosest(nodes []*rrtNode, n *rrtNode) (*rrtNode, float64) {
	var closest *rrtNode
	minDistance := math.MaxFloat64
	for _, node := range nodes {
		if d := n.state.distanceTo(node.state); d < minDistance {
			minDistance = d
			closest = node
		}
	}
	return closest, minDistance
}

func rrt(g *grid, start *state, goal *state) *plan {
	startTime := float64(time.Now().UnixNano()) / 10e9
	printLog(fmt.Sprintf("Start state is %s", start.String()))
	p := new(plan)
	root := &rrtNode{state: start}
	nodes := []*rrtNode{root}
	var n *rrtNode
	// RRT loop
	for startTime+timeToPlan > float64(time.Now().UnixNano())/10e9 {
		n = randomNode(g, goal)
		//printLog(fmt.Sprintf("Sampled state %s", n.state.String()))
		closest, distance := getClosest(nodes, n)
		angle := n.state.headingTo(closest.state)

		// there should be a better way to do this
		dummy := *(closest.state)
		dummyPointer := &dummy
		var collision bool
		for l := 0.0; l < distance; l += rrtInc {
			if l+rrtInc > distance {
				dummyPointer.push(angle, distance-l)
			} else {
				dummyPointer.push(angle, rrtInc)
			}
			if g.isBlocked(dummyPointer.x, dummyPointer.y) {
				collision = true
				break
			}
		}
		if collision {
			continue
		}
		n.parent = closest
		nodes = append(nodes, n)
		if n.state == goal {
			break
		}
	}
	// if we run out of time return a do-nothing plan (for now)
	if !(startTime+timeToPlan > float64(time.Now().UnixNano())/10e9) {
		return defaultPlan(start)
	}

	for cur := n; cur.state != start; cur = cur.parent {
		p.states = append(p.states, cur.state)
	}

	// reverse the plan order
	s := p.states
	for i, j := 0, len(s)-1; i < j; i, j = i+1, j-1 {
		s[i], s[j] = s[j], s[i]
	}

	// hack to put times on the states at 1s intervals
	for i, j := range s {
		j.time = float64(i + 1)
	}

	p.states = s

	return p
}

//endregion

//region Parse

// Anything that does parsing goes here

/**
Read the map from stdin and build the corresponding grid.
*/
func buildGrid() *grid {
	printLog("Reading map dimensions")
	var width, height int
	fmt.Sscanf(getLine(), "map %d %d", &width, &height)
	printLog("Building grid")
	grid := newGrid(width, height)
	for y := 0; y < height; y++ {
		var line string
		line = getLine()
		for x, c := range line {
			if c == '#' {
				grid.block(x, y)
			}
		}
	}
	return &grid
}

/**
Parse a state from a string in the format: x y heading speed time.
Turns heading into angle.
*/
func parseState(line string) *state {
	//fmt.Println("parsing line", line)
	var x, y, heading, speed, t float64
	fmt.Sscanf(line, "%f %f %f %f %f", &x, &y, &heading, &speed, &t)
	return &state{x, y, (heading * -1) + math.Pi/2, speed, t}
}

func readPath() *path {
	p := new(path)
	var pathLength int
	printLog("Reading path to cover")
	fmt.Sscanf(getLine(), "path to cover %d", &pathLength)
	var x, y float64
	for l := 0; l < pathLength; l++ {
		fmt.Sscanf(getLine(), "%f %f", &x, &y)
		*p = append(*p, state{x: x, y: y})
	}
	return p
}

/**
Update the given obstacle collection to account for n
updated obstacles coming from stdin.
*/
func updateObstacles(o *obstacles, n int) {
	for i := 0; i < n; i++ {
		var id int
		var x, y, heading, speed, t float64
		var line string = getLine()
		fmt.Sscanf(line, "%d %f %f %f %f %f", &id, &x, &y, &heading, &speed, &t)
		s := &state{x, y, (heading * -1) + math.Pi/2, speed, t}
		o.update(id, s)
	}
}

//endregion

//region main
var maxSpeed, maxTurningRadius float64
var reader = bufio.NewReader(os.Stdin)

func main() {

	//var startTime = float64(time.Now().UnixNano()) / 10e9
	//printLog(fmt.Sprintf("Planner starting at %f", startTime))

	rand.Seed(2) // set seed for now

	// redoing the parsing stuff
	var line string
	getLine() // start

	fmt.Sscanf(getLine(), "max speed %f", &maxSpeed)
	fmt.Sscanf(getLine(), "max turning radius %f", &maxTurningRadius)

	var grid = buildGrid()

	var path = readPath()

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
			path.remove(state{x: float64(x), y: float64(y)})
		}
		line = getLine()
		line = strings.TrimPrefix(line, "start state ")
		start := parseState(line)

		var nObstacles int
		o := new(obstacles)
		fmt.Sscanf(getLine(), "dynamic obs %d", nObstacles)
		updateObstacles(o, nObstacles)

		plan := makePlan(grid, start, *path)
		fmt.Println(plan.String())

		printLog("ready to plan")
	}
}

//endregion
