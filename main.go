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
	"reflect"
	"strings"
	"time"
)

// region Constants
const (
	verbose                     = true
	rrtInc              float64 = 0.5
	dubinsInc           float64 = 0.1 // this might be low
	dubinsDensity       float64 = 1   // factor of dubinsInc
	planDistanceDensity float64 = 1
	planTimeDensity     float64 = 1
	timeToPlan          float64 = 0.09 // make parameter (why is it off by a factor of 10??)
	goalBias            float64 = 0
	maxSpeedBias        float64 = 1.0
	K                   int     = 5   // number of closest states to consider for BIT*
	bitStarSamples      int     = 200 // make this a parameter too
	timeHorizon         float64 = 30  // not used as intended yet
	// BIT* penalties
	coveragePenalty  float64 = 30
	collisionPenalty float64 = 600 // this is suspect... may need to be lower because it will be summed
	timePenalty      float64 = 1
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
func (p *path) remove(s State) path {
	b := (*p)[0:0]
	for _, x := range *p {
		if s != x {
			b = append(b, x)
		}
	}
	// p = &b // this was wrong... should fix in v1
	return b
}

func (p path) maxDistanceFrom(s State) (max float64) {
	for _, x := range p {
		if d := s.DistanceTo(&x); d > max {
			max = d
		}
	}
	return
}

func (p path) newlyCovered(s State) (covered path) {
	for _, x := range p {
		if s.DistanceTo(&x) < 1.0 {
			covered = append(covered, x)
		}
	}
	return
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
Append a state to the plan when the state is within the time horizon and either:
	1. The plan is empty
	2. There is a substantial distance gap between the last state and this one
	3. There is a substantial time gap between the last state and this one
*/
func (p *Plan) appendState(s *State) {
	if start.TimeUntil(p.states[len(p.states)-1]) > timeHorizon &&
		(len(p.states) == 0 ||
			!(p.states[len(p.states)-1].DistanceTo(s) < planDistanceDensity) ||
			p.states[len(p.states)-1].TimeUntil(s) > planTimeDensity) {
		p.states = append(p.states, s)
	}
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

// func makePlan(grid *grid, start *State, path path, o *obstacles) *Plan {
// 	printLog("Starting to plan")
//
// 	if len(path) == 0 {
// 		return defaultPlan(start)
// 	}
// 	// set goal as first item in path
// 	goalCount := 0
// 	goal := &path[goalCount]
//
// 	startTime := float64(time.Now().UnixNano()) / 10e9
// 	var p = new(Plan)
// 	for startTime+timeToPlan > now() {
// 		newPlan := rrt(grid, start, goal, o, (startTime+timeToPlan)-now())
// 		if newPlan == nil {
// 			if len(p.states) == 0 {
// 				printLog("Done planning")
// 				return defaultPlan(start)
// 			}
// 		} else {
// 			p.appendPlan(newPlan)
// 			if goalCount++; len(path) > goalCount {
// 				start = goal
// 				goal = &path[goalCount]
// 			} else {
// 				return p
// 			}
// 		}
// 	}
//
// 	printLog("Done planning")
// 	return p
// }

//endregion

//region BIT*

//region BIT* globals

// make sure to set these before you call bitStar()

// these should be immutable so no pointers necessary
var start State
var g grid
var o obstacles
var toCover path
var bestVertex *bitStarVertex

func initGlobals(g1 grid, p path) {
	g = g1
	toCover = p
}

//endregion

type rrtNode struct {
	state        *State
	parent       *rrtNode
	pathToParent *dubins.Path
	trajectory   *Plan // trajectory to parent
}

//region Vertex

type bitStarVertex struct {
	state            *State
	approxCost       float64
	currentCost      float64
	currentCostIsSet bool
	approxToGo       float64
	parentEdge       *bitStarEdge
	uncovered        path
}

// accessor methods that should handle caching and stuff
func (v *bitStarVertex) ApproxCost() float64 {
	if v.approxCost == 0 {
		v.approxCost = start.DistanceTo(v.state) / maxSpeed * timePenalty
	}
	printLog(fmt.Sprintf("Approx cost: %f", v.approxCost))
	return v.approxCost
}

func (v *bitStarVertex) CurrentCost() float64 {
	// updated in bitStarEdge.UpdateTrueCost()
	if v.currentCostIsSet {
		return v.currentCost
	} else {
		return math.MaxFloat64
	}
}

// get the cached heuristic value
func (v *bitStarVertex) ApproxToGo() float64 {
	return v.approxToGo
}

// update the cached heuristic value
func (v *bitStarVertex) UpdateApproxToGo(parent *bitStarVertex) float64 {
	// is the whole parent thing really necessary? Yeah it probably is. Damn.
	if parent == nil {
		parent = v.parentEdge.start
	}
	// max euclidean distance to an uncovered point - coverage penalty for covering all of them
	// This is actually accurate if they're all in a straight line from your current heading,
	// which is not a super unlikely scenario, making this heuristic not as horrible as it may seem.
	v.approxToGo = parent.uncovered.maxDistanceFrom(*v.state)/maxSpeed*timePenalty -
		float64(len(v.uncovered))*coveragePenalty
	return v.approxToGo
}

func (v bitStarVertex) fValue() float64 {
	return v.ApproxCost() + v.UpdateApproxToGo(nil)
}

// contains function for convenience.
// Should consider using maps instead for contains performance.
func containsVertex(s []*bitStarVertex, e *bitStarVertex) bool {
	for _, a := range s {
		if reflect.DeepEqual(e, a) {
			return true
		}
	}
	return false
}

/**
I know this has the name filter and mutates the collection but just let me be.
It also may be dumb to do it this way; I'm just not confident enough with Go to
be sure or do it better.
*/
func verticesFilter(vertices *[]*bitStarVertex, f func(edge *bitStarVertex) bool) {
	if vertices == nil {
		return
	}
	b := (*vertices)[:0]
	for _, x := range *vertices {
		if f(x) {
			b = append(b, x)
		}
	}
	*vertices = b
}

func removeVertex(vertices *[]*bitStarVertex, v *bitStarVertex) {
	verticesFilter(vertices, func(x *bitStarVertex) bool {
		return v != x
	})
}

//endregion

//region Edge

type bitStarEdge struct {
	start, end           *bitStarVertex
	approxCost, trueCost float64
	dPath                *dubins.Path
	plan                 *Plan // plan to traverse dPath
}

func (e bitStarEdge) ApproxCost() float64 {
	return e.approxCost
}

// get the cached true cost
func (e bitStarEdge) TrueCost() float64 {
	return e.trueCost
}

// Updates the cached true cost of this edge.
// This is expensive.
func (e *bitStarEdge) UpdateTrueCost() float64 {
	var collisionPenalty float64
	var newlyCovered path
	var err int
	e.dPath, err = shortestPath(e.start.state, e.end.state)
	if err != dubins.EDUBOK {
		printLog("Error computing dubins path")
		return math.MaxFloat64 // encountered an error making the dubins path
	}
	// compute the plan along the dubins path, the collision penalty, and the ending time
	// NOTE: this does a lot of work
	e.plan, collisionPenalty, newlyCovered, e.end.state.time = getSamples(e.dPath, e.start.state.time, e.start.uncovered)
	// update the uncovered path in e.end
	e.end.uncovered = e.start.uncovered
	for _, c := range newlyCovered {
		// TODO -- maybe make this more efficient... it shouldn't happen that much though
		e.end.uncovered = e.end.uncovered.remove(c)
	}
	// update e's true cost
	e.trueCost = e.netTime()*timePenalty + collisionPenalty
	// update e.end's current cost
	e.end.currentCost = e.start.currentCost + e.trueCost
	e.end.currentCostIsSet = true
	return e.trueCost
}

func (e bitStarEdge) netTime() float64 {
	return e.end.state.time - e.start.state.time
}

// contains function for convenience.
// Should consider using maps instead for contains performance.
func containsEdge(s []*bitStarEdge, e *bitStarEdge) bool {
	for _, a := range s {
		if reflect.DeepEqual(e, a) {
			return true
		}
	}
	return false
}

/**
I know this has the name filter and mutates the collection but just let me be.
It also may be dumb to do it this way; I'm just not confident enough with Go to
be sure or do it better.
*/
func edgesFilter(edges *[]*bitStarEdge, f func(edge *bitStarEdge) bool) {
	if edges == nil {
		return
	}
	b := (*edges)[:0]
	for _, x := range *edges {
		if f(x) {
			b = append(b, x)
		}
	}
	*edges = b
}

/**
Remove all the edges ending in a certain vertex (for line 18)
*/
func removeEdgesEndingIn(edges *[]*bitStarEdge, v *bitStarVertex) {
	edgesFilter(edges, func(e *bitStarEdge) bool {
		return e.end != v
	})
}

//endregion

//region Queues

//region VertexQueue

type VertexQueue struct {
	nodes []*bitStarVertex
	cost  func(node *bitStarVertex) float64
}

func (h VertexQueue) Len() int { return len(h.nodes) }
func (h VertexQueue) Less(i, j int) bool {
	return h.cost(h.nodes[i]) < h.cost(h.nodes[j])
}
func (h VertexQueue) Swap(i, j int) { h.nodes[i], h.nodes[j] = h.nodes[j], h.nodes[i] }

func (h *VertexQueue) Push(x interface{}) {
	h.nodes = append(h.nodes, x.(*bitStarVertex))
}

func (h *VertexQueue) Pop() interface{} {
	old := *h
	n := len(old.nodes)
	x := old.nodes[n-1]
	h.nodes = old.nodes[0 : n-1]
	return x
}

func (h *VertexQueue) Peek() interface{} {
	return h.nodes[len(h.nodes)-1]
}

func makeVertexQueue(nodes []*bitStarVertex, cost func(node *bitStarVertex) float64) *VertexQueue {
	var nodeHeap = VertexQueue{nodes: nodes, cost: cost}
	for i, n := range nodes {
		nodeHeap.nodes[i] = n
	}
	heap.Init(&nodeHeap)
	return &nodeHeap
}

func (h *VertexQueue) update(cost func(node *bitStarVertex) float64) {
	if cost != nil {
		h.cost = cost
	}
	heap.Init(h)
}

// func (h *VertexQueue) prune(cost float64) {
// 	newNodes := make([]*bitStarVertex, len(h.nodes))
// 	var j int
// 	for i, j := 0, 0; i < len(h.nodes); i++ {
// 		if n := h.nodes[i]; h.cost(n) < cost {
// 			newNodes[j] = n
// 			j++
// 		}
// 	}
// 	h.nodes = newNodes[0:j]
// }

//endregion

//region EdgeQueue

type EdgeQueue struct {
	nodes []*bitStarEdge
	cost  func(node *bitStarEdge) float64
}

func (h EdgeQueue) Len() int { return len(h.nodes) }
func (h EdgeQueue) Less(i, j int) bool {
	return h.cost(h.nodes[i]) < h.cost(h.nodes[j])
}
func (h EdgeQueue) Swap(i, j int) { h.nodes[i], h.nodes[j] = h.nodes[j], h.nodes[i] }

func (h *EdgeQueue) Push(x interface{}) {
	h.nodes = append(h.nodes, x.(*bitStarEdge))
}

func (h *EdgeQueue) Pop() interface{} {
	old := *h
	n := len(old.nodes)
	x := old.nodes[n-1]
	h.nodes = old.nodes[0 : n-1]
	return x
}

func (h *EdgeQueue) Peek() interface{} {
	return h.nodes[len(h.nodes)-1]
}

func makeEdgeQueue(nodes []*bitStarEdge, cost func(node *bitStarEdge) float64) *EdgeQueue {
	var nodeHeap = EdgeQueue{nodes: nodes, cost: cost}
	for i, n := range nodes {
		nodeHeap.nodes[i] = n
	}
	heap.Init(&nodeHeap)
	return &nodeHeap
}

func (h *EdgeQueue) update(cost func(node *bitStarEdge) float64) {
	if cost != nil {
		h.cost = cost
	}
	heap.Init(h)
}

// func (h *EdgeQueue) prune(cost float64, vertexCost func(node *bitStarVertex) float64) {
// 	newNodes := make([]*bitStarEdge, len(h.nodes))
// 	var j int
// 	for i, j := 0, 0; i < len(h.nodes); i++ {
// 		if n := h.nodes[i]; vertexCost(n.start) < cost || vertexCost(n.end) < cost { // this might be right?
// 			newNodes[j] = n
// 			j++
// 		}
// 	}
// 	h.nodes = newNodes[0:j]
// }

//endregion

//endregion

//region State generation

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

/**
Sample a state whose euclidean distance to start is less than the given distance bound.
*/
func boundedBiasedRandomState(bounds *grid, path path, start *State, distance float64) *State {
	printLog(fmt.Sprintf("Sampling state with distance less than %f", distance))
	s := biasedRandomState(bounds, nil) // TODO! -- path bias instead of goal bias
	for ; start.DistanceTo(s) > distance; s = biasedRandomState(bounds, nil) {
	} // can this be O(1)?
	return s
}

func randomNode(bounds *grid, goal *State) *rrtNode {
	n := rrtNode{state: biasedRandomState(bounds, goal)}
	return &n
}

//endregion

/**
Alg 3
*/
func prune(samples *[]*bitStarVertex, vertices *[]*bitStarVertex, edges *[]*bitStarEdge, goalCost float64) {
	// line 1
	verticesFilter(samples, func(v *bitStarVertex) bool {
		return !(v.fValue() >= goalCost)
	})
	// line 2
	verticesFilter(vertices, func(v *bitStarVertex) bool {
		return !(v.fValue() > goalCost)
	})
	// line 3
	edgesFilter(edges, func(e *bitStarEdge) bool {
		return !(e.start.fValue() > goalCost || e.end.fValue() > goalCost)
	})
	// lines 4-5
	verticesFilter(vertices, func(v *bitStarVertex) bool {
		if v.currentCostIsSet {
			return true
		} else {
			*samples = append(*samples, v)
			return false
		}
	})
}

/**
Alg 2
*/
func expandVertex(v *bitStarVertex, qV *VertexQueue, qE *EdgeQueue,
	samples []*bitStarVertex, vertices []*bitStarVertex, edges []*bitStarEdge,
	vOld []*bitStarVertex, goalCost float64) {

	printLog(fmt.Sprintf("Expanding vertex %v", v.state.String()))
	// already should have popped v from qV

	// find k nearest samples and make edges (Alg 2 lines 2-3)
	for _, e := range getKClosest(v, samples, goalCost) {
		qE.Push(e)
	}

	// find k nearest vertices already in the tree? (Alg 2 lines 4-6)
	if !containsVertex(vOld, v) {
		for _, e := range getKClosest(v, vertices, goalCost) {
			if !containsEdge(edges, e) {
				if v.CurrentCost()+e.ApproxCost() < e.end.CurrentCost() {
					// line 6.2 is in getKClosest
					qE.Push(e)
				}
			}
		}
	}
}

/**
samples doesn't have to be actual samples it can come from anywhere
*/
func getKClosest(v *bitStarVertex, samples []*bitStarVertex, goalCost float64) (closest []*bitStarEdge) {
	closest = make([]*bitStarEdge, K)
	var i int
	var x *bitStarVertex
	for i, x = range samples {
		distance := v.state.DistanceTo(x.state)
		// Can we assume that h has been calculated for all x we're being given?
		// This seems like a problematic assumption because h may depend on the branch
		// of the tree we're connecting to (path covered so far?)
		// No longer making that assumption but maybe we should in the future when h is more expensive?
		if !(v.ApproxCost()+distance+x.UpdateApproxToGo(v) < goalCost) {
			continue // skip edges that can't contribute to a better solution
		}
		// iterate through current best edges and replace the first one that's worse than this
		for j, edge := range closest {
			if edge == nil {
				closest[j] = &bitStarEdge{start: v, end: x, approxCost: distance}
				x.parentEdge = closest[j]
				break
			} else if distance < edge.ApproxCost() {
				edge.end = x
				edge.approxCost = distance
				x.parentEdge = edge
				break
			}
		}
	}
	if i < K {
		return closest[0:i]
	}
	return
}

// functions for queueing vertices and edges
func vertexCost(v *bitStarVertex) float64 {
	return v.CurrentCost() + v.UpdateApproxToGo(nil)
}

func edgeCost(edge *bitStarEdge) float64 {
	// NOTE: when the heuristic function becomes more expensive this will need to get changed
	return edge.start.CurrentCost() + edge.ApproxCost() + edge.end.UpdateApproxToGo(edge.start)
}

/**
Alg 1 (obviously)
*/
func bitStar(startState State, timeRemaining float64, o1 *obstacles) *Plan {
	endTime := timeRemaining + now()
	// setup
	o, start = *o1, startState // assign globals
	startV := &bitStarVertex{state: &start, currentCostIsSet: true, uncovered: toCover}
	startV.currentCost = -float64(len(toCover)) * coveragePenalty
	startV.parentEdge = &bitStarEdge{start: startV, end: startV}
	bestVertex = startV
	samples := make([]*bitStarVertex, 0)
	var vOld []*bitStarVertex
	// line 1
	vertices := []*bitStarVertex{startV}
	edges := make([]*bitStarEdge, 0)
	// line 2
	qE := new(EdgeQueue)
	qV := new(VertexQueue)
	qE.cost = edgeCost
	qV.cost = vertexCost
	// line 3
	for now() < endTime {
		// line 4
		if qE.Len() == 0 && qV.Len() == 0 {
			// line 5
			prune(&samples, &vertices, &edges, bestVertex.fValue())
			// line 6
			samples = make([]*bitStarVertex, bitStarSamples)
			for m := 0; m < bitStarSamples; m++ {
				samples[m] = &bitStarVertex{state: boundedBiasedRandomState(&g, toCover, &start, bestVertex.fValue())}
			}
			// line 7
			// vOld is used in expandVertex to make sure we only add
			vOld = append([]*bitStarVertex(nil), vertices...)
			// line 8
			qV.nodes = make([]*bitStarVertex, len(vertices))
			copy(qV.nodes, vertices)
			qV.update(nil) // refactor?
			// line 9 -- not doing that so shouldn't need to do anything
		}
		// shouldn't need this but I added it for safety
		if qV.Len() > 0 {
			// lines 10, 11
			for qV.cost(qV.Peek().(*bitStarVertex)) <= qE.cost(qE.Peek().(*bitStarEdge)) {
				expandVertex(qV.Pop().(*bitStarVertex), qV, qE, samples, vertices, edges, vOld, bestVertex.fValue())
			}
		}
		// lines 12, 13
		edge := qE.Pop().(*bitStarEdge)
		vM, xM := edge.start, edge.end
		// line 14
		// vM should  be fully up to date at this point, but xM likely is not
		// Should we be using the f value for the best vertex here? I think that's
		// right but the paper is giving me pause...
		// Yeah pretty sure it's right. This is one of those places we're going to
		// do something different than the paper because of our path coverage goal.
		if vM.CurrentCost()+edge.ApproxCost()+xM.UpdateApproxToGo(vM) < bestVertex.fValue() {
			// line 15
			if vM.ApproxCost()+edge.UpdateTrueCost()+xM.ApproxToGo() < bestVertex.fValue() {
				// by now xM is fully up to date and we have a path
				// line 16
				if vM.CurrentCost()+edge.TrueCost() < xM.currentCost { // xM.currentCost is up to date
					// line 17
					if containsVertex(vertices, xM) {
						// line 18
						// remove edges ending in xM
						removeEdgesEndingIn(&edges, xM)
					} else {
						// line 20
						// remove xM from samples
						removeVertex(&samples, xM)
						// line 21
						// add xM to vertices
						vertices = append(vertices, xM)
						// add xM to vertex queue
						qV.Push(xM)
					}
					// line 22
					edges = append(edges, edge)
					// line 23
					// do some pruning on qE
					edgesFilter(&qE.nodes, func(e *bitStarEdge) bool {
						return !(e.end == xM && e.start.CurrentCost()+e.ApproxCost() >= xM.CurrentCost())
					})
					// Should probably try to remove items from the heap while maintaining the
					// heap property but that sounds hard so I'm not gonna worry about it yet.
					// Or at least see if anything changed before doing this
					heap.Init(qE) // re-do heap order (O(n))
				}
				// this is different:
				// update best vertex (may not want to do it here but it seems convenient)
				if bestVertex.fValue() > xM.fValue() {
					bestVertex = xM
				}
			}
		} else {
			// line 25
			qV.nodes = make([]*bitStarVertex, 0)
			qE.nodes = make([]*bitStarEdge, 0)
		}
	}
	// figure out the plan I guess
	// turn tree into slice
	branch := make([]*bitStarEdge, 0)
	for cur := bestVertex; cur != startV; cur = cur.parentEdge.start {
		branch = append(branch, cur.parentEdge)
	}

	// reverse the plan order (this might look dumb)
	s := branch
	for i, j := 0, len(s)-1; i < j; i, j = i+1, j-1 {
		s[i], s[j] = s[j], s[i]
	}
	branch = s

	p := new(Plan)
	p.appendState(&start)
	for _, e := range branch {
		p.appendState(e.end.state)
		p.appendPlan(e.plan) // should be fully calculate by now
	}
	return p
}

/**
Find the closest node in a list of nodes.
O(n) time.
*/
// func getClosest(nodes []*rrtNode, n *rrtNode) (*rrtNode, float64, *dubins.Path) {
// 	return getClosestNoHeap(nodes, n)
// 	//return getClosestWithHeap(nodes, n)
// }

// func getClosestNoHeap(nodes []*rrtNode, n *rrtNode) (*rrtNode, float64, *dubins.Path) {
// 	var closest *rrtNode
// 	minDistance := math.MaxFloat64
// 	var bestPath dubins.Path
// 	for _, node := range nodes {
// 		//shortestPathCompCount++
// 		dPath, err := shortestPath(node.state, n.state)
// 		if err != dubins.EDUBOK {
// 			if verbose {
// 				printLog("Couldn't make dubins path")
// 			}
// 			continue
// 		}
// 		if d := dPath.Length(); d < minDistance {
// 			minDistance = d
// 			closest = node
// 			bestPath = *dPath
// 		}
// 	}
// 	//iterationCount++
// 	return closest, minDistance, &bestPath
// }

// func rrt(g *grid, start *State, goal *State, o *obstacles, timeRemaining float64) *Plan {
// 	if *start == *goal {
// 		return defaultPlan(start)
// 	}
// 	startTime := float64(time.Now().UnixNano()) / 10e9
// 	printLog(fmt.Sprintf("Start state is %s", start.String()))
// 	p := new(Plan)
// 	root := &rrtNode{state: start}
// 	nodes := []*rrtNode{root}
// 	var n *rrtNode
// 	var sampleCount, blockedCount int
// 	// RRT loop
// 	for startTime+timeRemaining > now() {
// 		//printLog("Starting RRT loop")
// 		n = randomNode(g, goal)
// 		sampleCount++
// 		//printLog(fmt.Sprintf("Sampled state %s", n.state.String()))
// 		closest, distance, dPath := getClosest(nodes, n)
// 		if distance == math.MaxFloat64 {
// 			blockedCount++
// 			//printLog("Could not find state to connect to")
// 			continue
// 		}
// 		n.trajectory = getSamples2(dPath, g, o)
// 		if n.trajectory == nil {
// 			blockedCount++
// 			//printLog("Could not find state to connect to")
// 			continue // no path so continue
// 		}
// 		//printLog(fmt.Sprintf("Found nearest state %s at distance %f", closest.state.String(), distance))
//
// 		n.parent = closest
// 		n.pathToParent = dPath
// 		nodes = append(nodes, n)
// 		if n.state == goal {
// 			break
// 		}
// 		//printLog((fmt.Sprintf("Current node count is now %d", len(nodes))))
// 	}
// 	printLog(fmt.Sprintf("Samples: %d, blocked: %d, ratio: %f", sampleCount, blockedCount, float64(blockedCount)/float64(sampleCount)))
// 	//printLog("Samples on map:" + showSamples(nodes, g, start, goal))
//
// 	if !(startTime+timeToPlan > now()) {
// 		printLog("Time elapsed before we found a plan")
// 		return nil // calling function handles default plan
// 	}
//
// 	// if we couldn't even start
// 	if n == nil {
// 		return nil
// 	}
//
// 	// collect all nodes into slice
// 	var branch []*rrtNode
//
// 	for cur := n; cur.state != start && cur.parent != nil; cur = cur.parent {
// 		branch = append(branch, cur)
// 	}
//
// 	// reverse the plan order (this might look dumb)
// 	s := branch
// 	for i, j := 0, len(s)-1; i < j; i, j = i+1, j-1 {
// 		s[i], s[j] = s[j], s[i]
// 	}
// 	branch = s
//
// 	// add states to plan and compute times
// 	t := start.time
// 	var prev *State = nil
// 	var headingDelta float64
// 	for _, n := range branch {
// 		//printLog(fmt.Sprintf("Trajectory from %s has type %d", n.state.String(), n.pathToParent.GetPathType()))
// 		t += dubinsInc / maxSpeed
// 		cur := n.state
// 		traj := n.trajectory // shorthand, I guess?
// 		for _, s := range traj.states {
// 			t += dubinsInc / maxSpeed
// 			s.time += t
// 			// filter which states to include in output plan
// 			if prev == nil ||
// 				(headingDelta != 0 &&
// 					!prev.IsSamePosition(s) &&
// 					// heading delta changes sign, i.e we moved to new dubins segment
// 					headingDelta*(prev.heading-s.heading) <= 0) {
// 				// so we want to include this state
// 				p.appendState(s)
// 			}
// 			if prev != nil {
// 				headingDelta = prev.heading - s.heading
// 			}
// 			prev = s
// 		}
// 		t += dubinsInc / maxSpeed
// 		cur.time = t
// 		p.appendState(cur)
// 		prev = cur
// 	}
// 	//printLog(fmt.Sprintf("Found a plan for goal %s", goal.String()))
// 	//printLog(p.String())
//
// 	return p
// }

// func showSamples(nodes []*rrtNode, g *grid, start *State, goal *State) string {
// 	var bytes = []byte(g.dump())
// 	var arrays [][]byte
// 	for i := g.height - 1; i >= 0; i-- {
// 		arrays = append(arrays, bytes[1+(i*(g.width+1)):1+(i+1)*(g.width+1)])
// 	}
// 	//printLog("All nodes sampled:")
// 	for _, n := range nodes {
// 		//printLog(n.state.String())
// 		arrays[int(n.state.y)][int(n.state.x)] = 'o'
// 	}
// 	arrays[int(start.y)][int(start.x)] = '@'
// 	arrays[int(goal.y)][int(goal.x)] = '*'
// 	//printLog("Map:")
// 	return string(bytes)
// }

//endregion

//region NodeHeap

type NodeHeap struct {
	nodes      []*rrtNode
	otherState *State
}

func (h NodeHeap) Len() int { return len(h.nodes) }
func (h NodeHeap) Less(i, j int) bool {
	return h.nodes[i].state.DistanceTo(h.otherState) <
		h.nodes[j].state.DistanceTo(h.otherState)
}
func (h NodeHeap) Swap(i, j int) { h.nodes[i], h.nodes[j] = h.nodes[j], h.nodes[i] }

func (h *NodeHeap) Push(x interface{}) {
	h.nodes = append(h.nodes, x.(*rrtNode))
}

func (h *NodeHeap) Pop() interface{} {
	old := *h
	n := len(old.nodes)
	x := old.nodes[n-1]
	h.nodes = old.nodes[0 : n-1]
	return x
}

func heapify(nodes []*rrtNode, otherState *State) *NodeHeap {
	var nodeHeap = NodeHeap{nodes: nodes, otherState: otherState}
	for i, n := range nodes {
		nodeHeap.nodes[i] = n
	}
	heap.Init(&nodeHeap)
	return &nodeHeap
}

func (h *NodeHeap) update(otherState *State) {
	h.otherState = otherState
	heap.Init(h)
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
func getSamples2(path *dubins.Path, g *grid, o *obstacles) (plan *Plan) {
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

/**
Convert the given path into a plan and compute the sum collision cost and the newly covered path.
*/
func getSamples(path *dubins.Path, startTime float64, toCover path) (plan *Plan, penalty float64, newlyCovered path, finalTime float64) {
	plan = new(Plan)
	t := startTime
	callback := func(q *[3]float64, inc float64) int {
		t += inc / maxSpeed
		s := &State{x: q[0], y: q[1], heading: q[2], speed: maxSpeed, time: t}
		s.collisionProbability = o.collisionExists(s)
		if g.isBlocked(s.x, s.y) {
			penalty += collisionPenalty
		} else if s.collisionProbability > 0 {
			penalty += collisionPenalty * s.collisionProbability
		}
		newlyCovered = append(newlyCovered, toCover.newlyCovered(*s)...) // splash operator I guess
		plan.appendState(s)
		return 0
	}
	err := path.SampleMany(dubinsInc, callback)

	if err != dubins.EDUBOK {
		return nil, math.MaxFloat64, newlyCovered, 0
	}
	return plan, penalty, newlyCovered, t
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

	initGlobals(*grid, *path)

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

		// plan := makePlan(grid, start, *path, o)
		plan := bitStar(*start, timeToPlan, o)
		fmt.Println(plan.String())

		// show shortest path statistics
		//printLog(fmt.Sprintf("Performed %d shortest path computations over %d iterations", shortestPathCompCount, iterationCount))
		//printLog(fmt.Sprintf("That's on average %f per iteration", float64(shortestPathCompCount) / float64(iterationCount)))

		printLog("ready to plan")
	}
}

//endregion
