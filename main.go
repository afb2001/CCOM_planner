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
	rrtInc         float64 = 0.5
	dubinsInc      float64 = 0.1  // this might be low
	dubinsDensity  float64 = 1    // factor of dubinsInc
	timeToPlan     float64 = 0.09 // TOOO! -- make parameter (why is it off by a factor of 10??)
	goalBias       float64 = 0    // TODO! -- refactor state sampling for BIT*
	maxSpeedBias   float64 = 1.0
	K              int     = 5   // number of closest states to consider for BIT*
	bitStarSamples int     = 200 // parameterize
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
	if len(p.states) == 0 || !p.states[len(p.states)-1].IsSamePosition(s) {
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

//region BIT*

type rrtNode struct {
	state        *State
	parent       *rrtNode
	pathToParent *dubins.Path
	trajectory   *Plan // trajectory to parent
}

//region Vertex

type bitStarVertex struct {
	state                   *State
	approxCost, currentCost float64
	approxToGo              float64
	parentEdge              *bitStarEdge
	uncovered               path
}

// accessor methods that should handle caching and stuff
func (v bitStarVertex) ApproxCost() float64 {
	return v.approxCost
}

func (v bitStarVertex) CurrentCost() float64 {
	return v.currentCost
}

func (v bitStarVertex) ApproxToGo(parent *bitStarVertex) float64 {
	if parent == nil {
		parent = v.parentEdge.start
	}
	return v.approxToGo
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
}

func (e bitStarEdge) ApproxCost() float64 {
	return e.approxCost
}

func (e bitStarEdge) TrueCost() float64 {
	return e.trueCost
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
parent is the potential parent for x that we're using for
the uncovered path.
*/
func h(x *bitStarVertex, parent *bitStarVertex) float64 {
	return 0
}

// TODO! -- compute current cost somewhere
// also make sure all other costs have been computed somewhere
// oh and figure out what to do about caching heuristic values (is it necessary/easy?)

/**
Alg 3
*/
func prune(vertices []*bitStarVertex, edges []*bitStarEdge, goalCost float64) {
	// TODO -- this is gonna be different if I convert the slices to maps so I'll hold off
}

/**
Alg 2
*/
func expandVertex(v *bitStarVertex, qV *VertexQueue, qE *EdgeQueue,
	samples []*bitStarVertex, vertices []*bitStarVertex, edges []*bitStarEdge,
	goalCost float64) {
	// already should have popped v from qV

	// find k nearest samples and make edges (Alg 2 lines 2-3)
	for _, e := range getKClosest(v, samples, goalCost) {
		qE.Push(e)
	}

	// find k nearest vertices already in the tree? (Alg 2 lines 4-6)
	if !containsVertex(vertices, v) {
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
		if !(v.ApproxCost()+distance+x.ApproxToGo(v) < goalCost) {
			continue // skip edges that can't contribute to a better solution
		}
		// iterate through current best edges and replace the first one that's worse than this
		for j, edge := range closest {
			if edge == nil {
				closest[j] = &bitStarEdge{start: v, end: x, approxCost: distance} // is approx cost right here?
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
	return v.CurrentCost() + v.ApproxToGo(nil) // TODO! -- gotta cache that heuristic value for sure...
}

func edgeCost(edge *bitStarEdge) float64 {
	return edge.start.CurrentCost() + edge.ApproxCost() + edge.end.ApproxToGo(edge.start)
}

/**
Alg 1 (obviously)
*/
func bitStar(g *grid, start *State, toCover path, o *obstacles, timeRemaining float64) *Plan {
	endTime := timeRemaining + now()
	startV := &bitStarVertex{state: start}
	goalCost := math.MaxFloat64
	samples := make([]*bitStarVertex, bitStarSamples)
	// line 1
	vertices := []*bitStarVertex{startV}
	edges := make([]*bitStarEdge, 0)
	// line 2
	qE := new(EdgeQueue)
	qV := new(VertexQueue)
	// TODO -- consider refactoring this as it doesn't really add any generality
	qE.cost = edgeCost
	qV.cost = vertexCost
	// line 3
	for now() < endTime {
		// line 4
		if qE.Len() == 0 && qV.Len() == 0 {
			// line 5
			prune(vertices, edges, goalCost)
			// line 6
			for m := 0; m < bitStarSamples; m++ {
				samples[m] = &bitStarVertex{state: boundedBiasedRandomState(g, toCover, start, goalCost)}
			}
			// TODO! -- what do we need V_old for? (line 7, A2 line 4)
			// line 8
			qV.nodes = make([]*bitStarVertex, len(vertices))
			copy(qV.nodes, vertices)
			qV.update(nil) // refactor?
			// line 9 -- TODO! -- how to find the cost of the best plan so far?
		}
		// lines 10, 11
		// this is all a for loop but the formatting is weird...
		for v := qV.Peek().(*bitStarVertex); qV.cost(v) <= qE.cost(qE.Peek().(*bitStarEdge)); expandVertex(qV.Pop().(*bitStarVertex), qV, qE, samples, vertices, edges, goalCost) {
		}
		// lines 12, 13
		edge := qE.Pop().(*bitStarEdge)
		vM, xM := edge.start, edge.end
		// line 14
		if vM.CurrentCost()+edge.ApproxCost()+xM.ApproxToGo(vM) < goalCost {
			// line 15
			if vM.ApproxCost()+edge.TrueCost()+xM.ApproxToGo(vM) < goalCost {
				// line 16
				if vM.CurrentCost()+edge.TrueCost() < goalCost {
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
					heap.Init(qE) // re-do heap order (O(n))
				}
			}
		} else {
			// line 25
			qV.nodes = make([]*bitStarVertex, 0)
			qE.nodes = make([]*bitStarEdge, 0)
		}
	}
	// figure out the plan I guess
	return nil
}

/**
Find the closest node in a list of nodes.
O(n) time.
*/
func getClosest(nodes []*rrtNode, n *rrtNode) (*rrtNode, float64, *dubins.Path) {
	return getClosestNoHeap(nodes, n)
	//return getClosestWithHeap(nodes, n)
}

// debugging variables for keeping track of statistics
//var iterationCount int
//var shortestPathCompCount int

func getClosestWithHeap(nodes []*rrtNode, n *rrtNode) (*rrtNode, float64, *dubins.Path) {
	var closest *rrtNode
	minDistance := math.MaxFloat64 // dubins distance
	var bestPath dubins.Path
	nodeHeap := heapify(nodes, n.state)
	for heapNode := nodeHeap.Pop().(*rrtNode);
	// euclidean distance (2d) vs dubins distance
	heapNode.state.DistanceTo(n.state) < minDistance; {
		//shortestPathCompCount++
		dPath, err := shortestPath(heapNode.state, n.state)
		if err != dubins.EDUBOK {
			if verbose {
				printLog("Couldn't make dubins path")
			}
			continue
		}
		if d := dPath.Length(); d < minDistance {
			minDistance = d
			closest = heapNode
			bestPath = *dPath
		}
		if nodeHeap.Len() > 0 {
			heapNode = nodeHeap.Pop().(*rrtNode)
		} else {
			break
		}
	}
	//iterationCount++
	return closest, minDistance, &bestPath
}

func getClosestNoHeap(nodes []*rrtNode, n *rrtNode) (*rrtNode, float64, *dubins.Path) {
	var closest *rrtNode
	minDistance := math.MaxFloat64
	var bestPath dubins.Path
	for _, node := range nodes {
		//shortestPathCompCount++
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
	//iterationCount++
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

		// show shortest path statistics
		//printLog(fmt.Sprintf("Performed %d shortest path computations over %d iterations", shortestPathCompCount, iterationCount))
		//printLog(fmt.Sprintf("That's on average %f per iteration", float64(shortestPathCompCount) / float64(iterationCount)))

		printLog("ready to plan")
	}
}

//endregion
