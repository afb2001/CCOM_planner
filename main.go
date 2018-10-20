package main

import (
	"./dubins"
	"bufio"
	"container/heap"
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
	rrtInc        float64 = 0.5
	dubinsInc     float64 = 0.1   // this might be low
	dubinsDensity float64 = 1     // factor of dubinsInc
	timeToPlan    float64 = 0.095 // TOOO! -- make parameter (why is it off by a factor of 10??)
	goalBias      float64 = 0.1
	maxSpeedBias  float64 = 1.0
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

/**
Returns the current time in seconds as a float
*/
func now() float64 {
	return float64(time.Now().UnixNano()) / 10e9
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
	for y := 0; y < height; y++ {
		col := new([]cell)
		for x := 0; x < width; x++ {
			*col = append(*col, newCell(y, x))
		}
		*cells = append(*cells, *col)
	}
	return grid{cells: *cells, width: width, height: height}
}

/**
Get the cell at x, y.
*/
func (g *grid) get(x int, y int) *cell {
	return &(g.cells[y][x])
}

/**
Block the cell at x, y. For initialization only (probably).
*/
func (g *grid) block(x int, y int) {
	g.get(x, y).distanceToShore = 0
}

/**
Block cells at the specified resolution
*/
func (g *grid) blockRange(x int, y int, r int) {
	for i := 0; i < r; i++ {
		for j := 0; j < r; j++ {
			g.block(x+i, y+j)
		}
	}
}

func (g *grid) dump() string {
	var s = "\n"
	for y := g.height - 1; y >= 0; y-- {
		for x := 0; x < g.width; x++ {
			if g.isBlocked(float64(x), float64(y)) {
				s += "#"
			} else {
				s += "_"
			}
		}
		s += "\n"
	}
	return s
}

/**
Determine if a given point is within a static obstacle.
*/
func (g *grid) isBlocked(x float64, y float64) bool {
	if x < 0 || x > float64(g.width) || y < 0 || y > float64(g.height) {
		return true
	}
	return g.get(int(x), int(y)).isBlocked()
}

//endregion

//region Obstacles

/**
Type alias for (dynamic) obstacle collection.
*/
type obstacles map[int]*State

/**
Add or update the obstacle collection with the new obstacle.
*/
func (o *obstacles) update(id int, newState *State) {
	(*o)[id] = newState
}

func (o *obstacles) remove(id int) {
	delete(*o, id)
}

/**
Check if any of the obstacles collide with the given state.

Returns a float64 probability of collision.
*/
func (o *obstacles) collisionExists(state *State) float64 {
	for _, s := range *o {
		if s.Collides(s.Project(s.TimeUntil(state))) {
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
type State struct {
	x, y, heading, speed, time float64
	collisionProbability       float64
}

/**
Returns the time in seconds until state other.
*/
func (s *State) TimeUntil(other *State) float64 {
	return other.time - s.time
}

/**
Returns the Euclidean distance in two dimensions (x,y).
*/
func (s *State) DistanceTo(other *State) float64 {
	return math.Sqrt(math.Pow(s.x-other.x, 2) + math.Pow(s.y-other.y, 2))
}

func (s *State) HeadingTo(other *State) float64 {
	dx := s.x - other.x
	dy := s.y - other.y
	h := math.Atan2(dy, dx)
	if h < 0 {
		return h + (2 * math.Pi) // may not need this? I don't remember how tangents work
	}
	return h
}

/**
True iff other is within 1.5m in the x and y directions.
*/
func (s *State) Collides(other *State) bool {
	return s.time == other.time &&
		(math.Abs(s.x-other.x) < 1.5) &&
		(math.Abs(s.y-other.y) < 1.5)
}

/**
Tests whether the states have same (x, y)
*/
func (s *State) IsSamePosition(other *State) bool {
	return s.x == other.x && s.y == other.y
}

/**
Convert this state to a 3D vector for Dubins functions.
*/
func (s *State) ToArrayPointer() *[3]float64 {
	return &[3]float64{s.x, s.y, s.heading}
}

/**
Create a string representation of the state.
Angle is turned back into heading.
*/
func (s *State) String() string {
	return fmt.Sprintf("%f %f %f %f %f", s.x, s.y, (-1*s.heading)+math.Pi/2, s.speed, s.time)
}

/**
Projects a state to a specified time assuming constant speed and heading.
Creates a new state.

Meant to be used with future times but should work either way.
*/
func (s *State) Project(time float64) *State {
	deltaT := time - s.time
	magnitude := deltaT * s.speed
	deltaX := math.Cos(s.heading) * magnitude
	deltaY := math.Sin(s.heading) * magnitude
	return &State{x: s.x + deltaX, y: s.y + deltaY, heading: s.heading, speed: s.speed, time: time}
}

/**
Push a state at a given angle for a given distance.
Mutates the current state.

Written for ray-casting of sorts during collision checking.
Probably should not use past version 0.
*/
func (s *State) Push(heading float64, distance float64) {
	dx := distance * math.Cos(heading)
	dy := distance * math.Sin(heading)
	s.x += dx
	s.y += dy
}

//endregion

//region Path

type path []State

/**
Remove the given state from the path. Modifies the original path.
*/
func (p *path) remove(s State) {
	b := (*p)[0:0]
	for _, x := range *p {
		if s != x {
			b = append(b, x)
		}
	}
	p = &b
}

//endregion

//region Plan
type Plan struct {
	states []*State
}

func (p *Plan) String() string {
	s := fmt.Sprintf("plan %d", len(p.states))
	for _, state := range p.states {
		s += "\n" + state.String()
	}
	return s
}

/**
Append a state to the plan.
*/
func (p *Plan) appendState(s *State) {
	p.states = append(p.states, s)
}

/**
Concatenate two plans.
*/
func (p *Plan) appendPlan(other *Plan) {
	for _, s := range other.states {
		p.appendState(s)
	}
}

/**
Default do-nothing plan.
*/
func defaultPlan(start *State) *Plan {
	plan := new(Plan)
	s := State{x: start.x, y: start.y, heading: start.heading, speed: start.speed, time: start.time + 1.0}
	plan.states = append(plan.states, &s)
	return plan
}

// TODO! -- fix duplicate states when planning for multiple goals
func makePlan(grid *grid, start *State, path path, o *obstacles) *Plan {
	printLog("Starting to plan")

	if len(path) == 0 {
		return defaultPlan(start)
	}
	// set goal as first item in path
	goalCount := 0
	goal := &path[goalCount]

	startTime := float64(time.Now().UnixNano()) / 10e9
	var p = new(Plan)
	for startTime+timeToPlan > now() {
		newPlan := rrt(grid, start, goal, o, (startTime+timeToPlan)-now())
		if newPlan == nil {
			if len(p.states) == 0 {
				printLog("Done planning")
				return defaultPlan(start)
			}
		} else {
			p.appendPlan(newPlan)
			if goalCount++; len(path) > goalCount {
				start = goal
				goal = &path[goalCount]
			} else {
				return p
			}
		}
	}

	printLog("Done planning")
	return p
}

//endregion

//region RRT

type rrtNode struct {
	state        *State
	parent       *rrtNode
	pathToParent *dubins.Path
	trajectory   *Plan // trajectory to parent
}

/**
Create a new state with random values.
Time is unset (zero).
*/
func randomState(bounds *grid) *State {
	s := new(State)
	s.x = rand.Float64() * float64(bounds.width)
	s.y = rand.Float64() * float64(bounds.height)
	s.heading = rand.Float64() * math.Pi * 2
	s.speed = rand.Float64() * maxSpeed
	return s
}

/**
Create a random sample using the biasing constants.
*/
func biasedRandomState(bounds *grid, goal *State) *State {
	//verbose = false
	s := randomState(bounds)
	if r := rand.Float64(); r < maxSpeedBias {
		s.speed = maxSpeed
	}
	if r := rand.Float64(); r < goalBias {
		//verbose = true
		return goal
	}
	return s
}

func randomNode(bounds *grid, goal *State) *rrtNode {
	n := rrtNode{state: biasedRandomState(bounds, goal)}
	return &n
}

/**
Find the closest node in a list of nodes.
O(n) time.
*/
func getClosest(nodes []*rrtNode, n *rrtNode) (*rrtNode, float64, *dubins.Path) {
	return getClosestNoHeap(nodes, n)
	//return getClosestWithHeap(nodes, n)
}

func getClosestWithHeap(nodes []*rrtNode, n *rrtNode) (*rrtNode, float64, *dubins.Path) {
	var closest *rrtNode
	minDistance := math.MaxFloat64 // dubins distance
	var bestPath dubins.Path
	nodeHeap := heapify(nodes, n.state)
	for heapNode := nodeHeap.Pop().(*rrtHeapNode);
	// euclidean distance (2d) vs dubins distance
	heapNode.node.state.DistanceTo(n.state) < minDistance; {
		dPath, err := shortestPath(heapNode.node.state, n.state)
		if err != dubins.EDUBOK {
			if verbose {
				printLog("Couldn't make dubins path")
			}
			continue
		}
		if d := dPath.Length(); d < minDistance {
			minDistance = d
			closest = heapNode.node
			bestPath = *dPath
		}
		if nodeHeap.Len() > 0 {
			heapNode = nodeHeap.Pop().(*rrtHeapNode)
		} else {
			break
		}
	}
	return closest, minDistance, &bestPath
}

func getClosestNoHeap(nodes []*rrtNode, n *rrtNode) (*rrtNode, float64, *dubins.Path) {
	var closest *rrtNode
	minDistance := math.MaxFloat64
	var bestPath dubins.Path
	for _, node := range nodes {
		dPath, err := shortestPath(node.state, n.state)
		if err != dubins.EDUBOK {
			if verbose {
				printLog("Couldn't make dubins path")
			}
			continue
		}
		if d := dPath.Length(); d < minDistance {
			minDistance = d
			closest = node
			bestPath = *dPath
		}
	}
	return closest, minDistance, &bestPath
}

var verbose = false

func rrt(g *grid, start *State, goal *State, o *obstacles, timeRemaining float64) *Plan {
	if *start == *goal {
		return defaultPlan(start)
	}
	startTime := float64(time.Now().UnixNano()) / 10e9
	printLog(fmt.Sprintf("Start state is %s", start.String()))
	p := new(Plan)
	root := &rrtNode{state: start}
	nodes := []*rrtNode{root}
	var n *rrtNode
	var sampleCount, blockedCount int
	// RRT loop
	for startTime+timeRemaining > now() {
		//printLog("Starting RRT loop")
		n = randomNode(g, goal)
		sampleCount++
		//printLog(fmt.Sprintf("Sampled state %s", n.state.String()))
		closest, distance, dPath := getClosest(nodes, n)
		if distance == math.MaxFloat64 {
			blockedCount++
			//printLog("Could not find state to connect to")
			continue
		}
		n.trajectory = getSamples(dPath, g, o)
		if n.trajectory == nil {
			blockedCount++
			//printLog("Could not find state to connect to")
			continue // no path so continue
		}
		//printLog(fmt.Sprintf("Found nearest state %s at distance %f", closest.state.String(), distance))

		n.parent = closest
		n.pathToParent = dPath
		nodes = append(nodes, n)
		if n.state == goal {
			break
		}
		//printLog((fmt.Sprintf("Current node count is now %d", len(nodes))))
	}
	printLog(fmt.Sprintf("Samples: %d, blocked: %d, ratio: %f", sampleCount, blockedCount, float64(blockedCount)/float64(sampleCount)))
	//printLog("Samples on map:" + showSamples(nodes, g, start, goal))

	if !(startTime+timeToPlan > now()) {
		printLog("Time elapsed before we found a plan")
		return nil // calling function handles default plan
	}

	// if we couldn't even start
	if n == nil {
		return nil
	}

	// collect all nodes into slice
	var branch []*rrtNode

	for cur := n; cur.state != start && cur.parent != nil; cur = cur.parent {
		branch = append(branch, cur)
	}

	// reverse the plan order (this might look dumb)
	s := branch
	for i, j := 0, len(s)-1; i < j; i, j = i+1, j-1 {
		s[i], s[j] = s[j], s[i]
	}
	branch = s

	// add states to plan and compute times
	t := start.time
	var prev *State = nil
	var headingDelta float64
	for _, n := range branch {
		//printLog(fmt.Sprintf("Trajectory from %s has type %d", n.state.String(), n.pathToParent.GetPathType()))
		t += dubinsInc / maxSpeed
		cur := n.state
		traj := n.trajectory // shorthand, I guess?
		for _, s := range traj.states {
			t += dubinsInc / maxSpeed
			s.time += t
			// filter which states to include in output plan
			if prev == nil ||
				(headingDelta != 0 &&
					!prev.IsSamePosition(s) &&
					// heading delta changes sign, i.e we moved to new dubins segment
					headingDelta*(prev.heading-s.heading) <= 0) {
				// so we want to include this state
				p.appendState(s)
			}
			if prev != nil {
				headingDelta = prev.heading - s.heading
			}
			prev = s
		}
		t += dubinsInc / maxSpeed
		cur.time = t
		p.appendState(cur)
		prev = cur
	}
	//printLog(fmt.Sprintf("Found a plan for goal %s", goal.String()))
	//printLog(p.String())

	return p
}

func showSamples(nodes []*rrtNode, g *grid, start *State, goal *State) string {
	var bytes = []byte(g.dump())
	var arrays [][]byte
	for i := g.height - 1; i >= 0; i-- {
		arrays = append(arrays, bytes[1+(i*(g.width+1)):1+(i+1)*(g.width+1)])
	}
	//printLog("All nodes sampled:")
	for _, n := range nodes {
		//printLog(n.state.String())
		arrays[int(n.state.y)][int(n.state.x)] = 'o'
	}
	arrays[int(start.y)][int(start.x)] = '@'
	arrays[int(goal.y)][int(goal.x)] = '*'
	//printLog("Map:")
	return string(bytes)
}

//endregion

//region NodeHeap

type rrtHeapNode struct {
	node       *rrtNode
	otherState *State
}

type NodeHeap []*rrtHeapNode

func (h NodeHeap) Len() int { return len(h) }
func (h NodeHeap) Less(i, j int) bool {
	return h[i].node.state.DistanceTo(h[i].otherState) <
		h[j].node.state.DistanceTo(h[j].otherState)
}
func (h NodeHeap) Swap(i, j int) { h[i], h[j] = h[j], h[i] }

func (h *NodeHeap) Push(x interface{}) {
	*h = append(*h, x.(*rrtHeapNode))
}

func (h *NodeHeap) Pop() interface{} {
	old := *h
	n := len(old)
	x := old[n-1]
	*h = old[0 : n-1]
	return x
}

func heapify(nodes []*rrtNode, otherState *State) *NodeHeap {
	var nodeHeap = make(NodeHeap, len(nodes))
	for i, n := range nodes {
		//printLog(n.state.String())
		nodeHeap[i] = &rrtHeapNode{n, otherState}
	}
	heap.Init(&nodeHeap)
	return &nodeHeap
}

//endregion

//region Dubins integration

/**
Find the shortest Dubins path between two states.
*/
func shortestPath(s1 *State, s2 *State) (path *dubins.Path, err int) {
	if verbose {
		printLog(fmt.Sprintf("Computing dubins path between %s, %s", s1.String(), s2.String()))
	}
	path = new(dubins.Path)
	err = dubins.ShortestPath(path, s1.ToArrayPointer(), s2.ToArrayPointer(), maxTurningRadius)
	return path, err
}

/**
Convert the given path into a feasible plan.
*/
func getSamples(path *dubins.Path, g *grid, o *obstacles) (plan *Plan) {
	plan = new(Plan)
	var t float64
	callback := func(q *[3]float64, inc float64) int {
		//printLog(q)
		//t += inc / maxSpeed
		s := &State{x: q[0], y: q[1], heading: q[2], speed: maxSpeed, time: t} // t = 0 now
		s.collisionProbability = o.collisionExists(s)
		// collision probability is 0 or 1 for now
		if g.isBlocked(s.x, s.y) || s.collisionProbability > 0 {
			if verbose {
				printLog(fmt.Sprintf("Blocked path: %f %f %f", s.x, s.y, s.collisionProbability))
			}
			return dubins.EDUBNOPATH
		}
		plan.appendState(s)
		return 0
	}
	err := path.SampleMany(dubinsInc, callback)

	if err != dubins.EDUBOK {
		return nil
	}

	return plan
}

//endregion

//region Parse

// Anything that does parsing goes here

/**
Read the map from stdin and build the corresponding grid.
*/
func buildGrid() *grid {
	printLog("Reading map dimensions")
	var width, height, resolution int
	fmt.Sscanf(getLine(), "map %d %d %d", &resolution, &width, &height)
	printLog("Building grid")
	grid := newGrid(width*resolution, height*resolution)
	for y := height - 1; y >= 0; y-- {
		var line string
		line = getLine()
		for x, c := range line {
			if c == '#' {
				grid.blockRange(x*resolution, y*resolution, resolution)
			}
		}
	}
	return &grid
}

/**
Parse a state from a string in the format: x y heading speed time.
Turns heading into angle.
*/
func parseState(line string) *State {
	//fmt.Println("parsing line", line)
	var x, y, heading, speed, t float64
	fmt.Sscanf(line, "%f %f %f %f %f", &x, &y, &heading, &speed, &t)
	return &State{x: x, y: y, heading: (heading * -1) + math.Pi/2, speed: speed, time: t}
}

func readPath() *path {
	p := new(path)
	var pathLength int
	printLog("Reading path to cover")
	fmt.Sscanf(getLine(), "path to cover %d", &pathLength)
	var x, y float64
	for l := 0; l < pathLength; l++ {
		fmt.Sscanf(getLine(), "%f %f", &x, &y)
		s := State{x: x, y: y, speed: maxSpeed}
		*p = append(*p, s)
		if l > 0 {
			sPrev := (*p)[l-1]
			sPrev.heading = sPrev.HeadingTo(&s)
		}
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
		s := &State{x: x, y: y, heading: (heading * -1) + math.Pi/2, speed: speed, time: t}
		o.update(id, s)
	}
}

//endregion

//region main

// globals
var maxSpeed, maxTurningRadius float64
var reader = bufio.NewReader(os.Stdin)

func main() {

	//var startTime = float64(time.Now().UnixNano()) / 10e9
	//printLog(fmt.Sprintf("Planner starting at %f", startTime))

	rand.Seed(3) // set seed for now

	// redoing the parsing stuff
	var line string
	getLine() // start

	fmt.Sscanf(getLine(), "max speed %f", &maxSpeed)
	fmt.Sscanf(getLine(), "max turning radius %f", &maxTurningRadius)

	var grid = buildGrid()

	//printLog(grid.dump())

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
			path.remove(State{x: float64(x), y: float64(y)})
		}
		line = getLine()
		line = strings.TrimPrefix(line, "start state ")
		start := parseState(line)

		var nObstacles int
		o := new(obstacles)
		fmt.Sscanf(getLine(), "dynamic obs %d", nObstacles)
		updateObstacles(o, nObstacles)

		plan := makePlan(grid, start, *path, o)
		fmt.Println(plan.String())

		printLog("ready to plan")
	}
}

//endregion
