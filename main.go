package main

import (
	"./common"
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
	verbose                = true
	rrtInc         float64 = 0.5
	dubinsInc      float64 = 0.1  // this might be low
	dubinsDensity  float64 = 1    // factor of dubinsInc
	timeToPlan     float64 = 0.09 // make parameter (why is it off by a factor of 10??)
	goalBias       float64 = 0
	maxSpeedBias   float64 = 1.0
	K              int     = 5   // number of closest states to consider for BIT*
	bitStarSamples int     = 200 // (m in the paper) -- make this a parameter too
	// BIT* penalties (should all be made into parameters)
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

//region Dubins integration

/**
Find the shortest Dubins path between two states.
*/
func shortestPath(s1 *common.State, s2 *common.State) (path *dubins.Path, err int) {
	if verbose {
		printLog(fmt.Sprintf("Computing dubins path between %s, %s", s1.String(), s2.String()))
	}
	path = new(dubins.Path)
	err = dubins.ShortestPath(path, s1.ToArrayPointer(), s2.ToArrayPointer(), maxTurningRadius)
	return path, err
}

/**
Convert the given path into a plan and compute the sum collision cost and the newly covered path.
*/
func getSamples(path *dubins.Path, startTime float64, toCover common.Path) (plan *common.Plan, penalty float64, newlyCovered common.Path, finalTime float64) {
	plan = new(common.Plan)
	t := startTime
	callback := func(q *[3]float64, inc float64) int {
		t += inc / maxSpeed
		s := &common.State{X: q[0], Y: q[1], Heading: q[2], Speed: maxSpeed, Time: t}
		s.CollisionProbability = o.CollisionExists(s)
		if grid.IsBlocked(s.X, s.Y) {
			penalty += collisionPenalty
		} else if s.CollisionProbability > 0 {
			penalty += collisionPenalty * s.CollisionProbability
		}
		newlyCovered = append(newlyCovered, toCover.NewlyCovered(*s)...) // splash operator I guess
		plan.AppendState(s)
		return 0
	}
	err := path.SampleMany(dubinsInc, callback)

	if err != dubins.EDUBOK {
		return nil, math.MaxFloat64, newlyCovered, 0
	}
	return plan, penalty, newlyCovered, t
}

//endregion

//region BIT*

//region BIT* globals

// make sure to set these before you call bitStar()

// these should be immutable so no pointers necessary
var start common.State
var grid common.Grid
var o common.Obstacles
var toCover common.Path
var bestVertex *BitStarVertex

func initGlobals(g1 common.Grid, p common.Path) {
	grid = g1
	toCover = p
}

//endregion

//region Vertex

type BitStarVertex struct {
	state            *common.State
	approxCost       float64
	currentCost      float64
	currentCostIsSet bool
	approxToGo       float64
	parentEdge       *BitStarEdge
	uncovered        common.Path
}

// accessor methods that should handle caching and stuff
func (v *BitStarVertex) ApproxCost() float64 {
	if v.approxCost == 0 {
		v.approxCost = start.DistanceTo(v.state) / maxSpeed * timePenalty
	}
	printLog(fmt.Sprintf("Approx cost: %f", v.approxCost))
	return v.approxCost
}

func (v *BitStarVertex) CurrentCost() float64 {
	// updated in BitStarEdge.UpdateTrueCost()
	if v.currentCostIsSet {
		return v.currentCost
	} else {
		return math.MaxFloat64
	}
}

// get the cached heuristic value
func (v *BitStarVertex) ApproxToGo() float64 {
	return v.approxToGo
}

// update the cached heuristic value
func (v *BitStarVertex) UpdateApproxToGo(parent *BitStarVertex) float64 {
	// is the whole parent thing really necessary? Yeah it probably is. Damn.
	if parent == nil {
		parent = v.parentEdge.start
	}
	// max euclidean distance to an uncovered point - coverage penalty for covering all of them
	// This is actually accurate if they're all in a straight line from your current heading,
	// which is not a super unlikely scenario, making this heuristic not as horrible as it may seem.
	v.approxToGo = parent.uncovered.MaxDistanceFrom(*v.state)/maxSpeed*timePenalty -
		float64(len(v.uncovered))*coveragePenalty
	return v.approxToGo
}

func (v BitStarVertex) fValue() float64 {
	return v.ApproxCost() + v.UpdateApproxToGo(nil)
}

// contains function for convenience.
// Should consider using maps instead for contains performance.
func containsVertex(s []*BitStarVertex, e *BitStarVertex) bool {
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
func verticesFilter(vertices *[]*BitStarVertex, f func(edge *BitStarVertex) bool) {
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

func removeVertex(vertices *[]*BitStarVertex, v *BitStarVertex) {
	verticesFilter(vertices, func(x *BitStarVertex) bool {
		return v != x
	})
}

//endregion

//region Edge

type BitStarEdge struct {
	start, end           *BitStarVertex
	approxCost, trueCost float64
	dPath                *dubins.Path
	plan                 *common.Plan // plan to traverse dPath
}

func (e BitStarEdge) ApproxCost() float64 {
	return e.approxCost
}

// get the cached true cost
func (e BitStarEdge) TrueCost() float64 {
	return e.trueCost
}

// Updates the cached true cost of this edge.
// This is expensive.
func (e *BitStarEdge) UpdateTrueCost() float64 {
	var collisionPenalty float64
	var newlyCovered common.Path
	var err int
	e.dPath, err = shortestPath(e.start.state, e.end.state)
	if err != dubins.EDUBOK {
		printLog("Error computing dubins path")
		return math.MaxFloat64 // encountered an error making the dubins path
	}
	// compute the plan along the dubins path, the collision penalty, and the ending time
	// NOTE: this does a lot of work
	e.plan, collisionPenalty, newlyCovered, e.end.state.Time = getSamples(e.dPath, e.start.state.Time, e.start.uncovered)
	// update the uncovered path in e.end
	e.end.uncovered = e.start.uncovered
	for _, c := range newlyCovered {
		// TODO -- maybe make this more efficient... it shouldn't happen that much though
		e.end.uncovered = e.end.uncovered.Remove(c)
	}
	// update e's true cost
	e.trueCost = e.netTime()*timePenalty + collisionPenalty
	// update e.end's current cost
	e.end.currentCost = e.start.currentCost + e.trueCost
	e.end.currentCostIsSet = true
	return e.trueCost
}

func (e BitStarEdge) netTime() float64 {
	return e.end.state.Time - e.start.state.Time
}

// contains function for convenience.
// Should consider using maps instead for contains performance.
func ContainsEdge(s []*BitStarEdge, e *BitStarEdge) bool {
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
func EdgesFilter(edges *[]*BitStarEdge, f func(edge *BitStarEdge) bool) {
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
func RemoveEdgesEndingIn(edges *[]*BitStarEdge, v *BitStarVertex) {
	EdgesFilter(edges, func(e *BitStarEdge) bool {
		return e.end != v
	})
}

//endregion

//region Queues

//region VertexQueue

type VertexQueue struct {
	nodes []*BitStarVertex
	cost  func(node *BitStarVertex) float64
}

func (h VertexQueue) Len() int { return len(h.nodes) }
func (h VertexQueue) Less(i, j int) bool {
	return h.cost(h.nodes[i]) < h.cost(h.nodes[j])
}
func (h VertexQueue) Swap(i, j int) { h.nodes[i], h.nodes[j] = h.nodes[j], h.nodes[i] }

func (h *VertexQueue) Push(x interface{}) {
	h.nodes = append(h.nodes, x.(*BitStarVertex))
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

func makeVertexQueue(nodes []*BitStarVertex, cost func(node *BitStarVertex) float64) *VertexQueue {
	var nodeHeap = VertexQueue{nodes: nodes, cost: cost}
	for i, n := range nodes {
		nodeHeap.nodes[i] = n
	}
	heap.Init(&nodeHeap)
	return &nodeHeap
}

func (h *VertexQueue) update(cost func(node *BitStarVertex) float64) {
	if cost != nil {
		h.cost = cost
	}
	heap.Init(h)
}

// func (h *VertexQueue) prune(cost float64) {
// 	newNodes := make([]*BitStarVertex, len(h.nodes))
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
	nodes []*BitStarEdge
	cost  func(node *BitStarEdge) float64
}

func (h EdgeQueue) Len() int { return len(h.nodes) }
func (h EdgeQueue) Less(i, j int) bool {
	return h.cost(h.nodes[i]) < h.cost(h.nodes[j])
}
func (h EdgeQueue) Swap(i, j int) { h.nodes[i], h.nodes[j] = h.nodes[j], h.nodes[i] }

func (h *EdgeQueue) Push(x interface{}) {
	h.nodes = append(h.nodes, x.(*BitStarEdge))
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

func makeEdgeQueue(nodes []*BitStarEdge, cost func(node *BitStarEdge) float64) *EdgeQueue {
	var nodeHeap = EdgeQueue{nodes: nodes, cost: cost}
	for i, n := range nodes {
		nodeHeap.nodes[i] = n
	}
	heap.Init(&nodeHeap)
	return &nodeHeap
}

func (h *EdgeQueue) update(cost func(node *BitStarEdge) float64) {
	if cost != nil {
		h.cost = cost
	}
	heap.Init(h)
}

// func (h *EdgeQueue) prune(cost float64, vertexCost func(node *BitStarVertex) float64) {
// 	newNodes := make([]*BitStarEdge, len(h.nodes))
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

// functions for queueing vertices and edges
func vertexCost(v *BitStarVertex) float64 {
	return v.CurrentCost() + v.UpdateApproxToGo(nil)
}

func edgeCost(edge *BitStarEdge) float64 {
	// NOTE: when the heuristic function becomes more expensive this will need to get changed
	return edge.start.CurrentCost() + edge.ApproxCost() + edge.end.UpdateApproxToGo(edge.start)
}

//endregion

//region State generation

/**
Create a new state with random values.
Time is unset (zero).
*/
func randomState(bounds *common.Grid) *common.State {
	s := new(common.State)
	s.X = rand.Float64() * float64(bounds.Width)
	s.Y = rand.Float64() * float64(bounds.Height)
	s.Heading = rand.Float64() * math.Pi * 2
	s.Speed = rand.Float64() * maxSpeed
	return s
}

/**
Create a random sample using the biasing constants.
*/
func biasedRandomState(bounds *common.Grid, goal *common.State) *common.State {
	//verbose = false
	s := randomState(bounds)
	if r := rand.Float64(); r < maxSpeedBias {
		s.Speed = maxSpeed
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
func boundedBiasedRandomState(bounds *common.Grid, path common.Path, start *common.State, distance float64) *common.State {
	printLog(fmt.Sprintf("Sampling state with distance less than %f", distance))
	s := biasedRandomState(bounds, nil) // TODO! -- path bias instead of goal bias
	for ; start.DistanceTo(s) > distance; s = biasedRandomState(bounds, nil) {
	} // can this be O(1)?
	return s
}

//endregion

//region Algorithm 3 (Prune)

/**
Alg 3
*/
func prune(samples *[]*BitStarVertex, vertices *[]*BitStarVertex, edges *[]*BitStarEdge, goalCost float64) {
	// line 1
	verticesFilter(samples, func(v *BitStarVertex) bool {
		return !(v.fValue() >= goalCost)
	})
	// line 2
	verticesFilter(vertices, func(v *BitStarVertex) bool {
		return !(v.fValue() > goalCost)
	})
	// line 3
	EdgesFilter(edges, func(e *BitStarEdge) bool {
		return !(e.start.fValue() > goalCost || e.end.fValue() > goalCost)
	})
	// lines 4-5
	verticesFilter(vertices, func(v *BitStarVertex) bool {
		if v.currentCostIsSet {
			return true
		} else {
			*samples = append(*samples, v)
			return false
		}
	})
}

//endregion

//region Algorithm 2 (ExpandVertex)

/**
Alg 2
*/
func expandVertex(v *BitStarVertex, qV *VertexQueue, qE *EdgeQueue,
	samples []*BitStarVertex, vertices []*BitStarVertex, edges []*BitStarEdge,
	vOld []*BitStarVertex, goalCost float64) {

	printLog(fmt.Sprintf("Expanding vertex %v", v.state.String()))
	// already should have popped v from qV

	// find k nearest samples and make edges (Alg 2 lines 2-3)
	for _, e := range getKClosest(v, samples, goalCost) {
		qE.Push(e)
	}

	// find k nearest vertices already in the tree? (Alg 2 lines 4-6)
	if !containsVertex(vOld, v) {
		for _, e := range getKClosest(v, vertices, goalCost) {
			if !ContainsEdge(edges, e) {
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
func getKClosest(v *BitStarVertex, samples []*BitStarVertex, goalCost float64) (closest []*BitStarEdge) {
	closest = make([]*BitStarEdge, K)
	var i int
	var x *BitStarVertex
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
				closest[j] = &BitStarEdge{start: v, end: x, approxCost: distance}
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

//endregion

//region Algorithm 1 (BIT*)

/**
Alg 1 (obviously)
*/
func bitStar(startState common.State, timeRemaining float64, o1 *common.Obstacles) *common.Plan {
	endTime := timeRemaining + now()
	// setup
	o, start = *o1, startState // assign globals
	startV := &BitStarVertex{state: &start, currentCostIsSet: true, uncovered: toCover}
	startV.currentCost = -float64(len(toCover)) * coveragePenalty
	startV.parentEdge = &BitStarEdge{start: startV, end: startV}
	bestVertex = startV
	samples := make([]*BitStarVertex, 0)
	var vOld []*BitStarVertex
	// line 1
	vertices := []*BitStarVertex{startV}
	edges := make([]*BitStarEdge, 0)
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
			samples = make([]*BitStarVertex, bitStarSamples)
			for m := 0; m < bitStarSamples; m++ {
				samples[m] = &BitStarVertex{state: boundedBiasedRandomState(&grid, toCover, &start, bestVertex.fValue())}
			}
			// line 7
			// vOld is used in expandVertex to make sure we only add
			vOld = append([]*BitStarVertex(nil), vertices...)
			// line 8
			qV.nodes = make([]*BitStarVertex, len(vertices))
			copy(qV.nodes, vertices)
			qV.update(nil) // refactor?
			// line 9 -- not doing that so shouldn't need to do anything
		}
		// shouldn't need this but I added it for safety
		if qV.Len() > 0 {
			// lines 10, 11
			for qV.cost(qV.Peek().(*BitStarVertex)) <= qE.cost(qE.Peek().(*BitStarEdge)) {
				expandVertex(qV.Pop().(*BitStarVertex), qV, qE, samples, vertices, edges, vOld, bestVertex.fValue())
			}
		}
		// lines 12, 13
		edge := qE.Pop().(*BitStarEdge)
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
						RemoveEdgesEndingIn(&edges, xM)
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
					EdgesFilter(&qE.nodes, func(e *BitStarEdge) bool {
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
			qV.nodes = make([]*BitStarVertex, 0)
			qE.nodes = make([]*BitStarEdge, 0)
		}
	}
	// figure out the plan I guess
	// turn tree into slice
	branch := make([]*BitStarEdge, 0)
	for cur := bestVertex; cur != startV; cur = cur.parentEdge.start {
		branch = append(branch, cur.parentEdge)
	}

	// reverse the plan order (this might look dumb)
	s := branch
	for i, j := 0, len(s)-1; i < j; i, j = i+1, j-1 {
		s[i], s[j] = s[j], s[i]
	}
	branch = s

	p := new(common.Plan)
	p.Start = start
	p.AppendState(&start) // yes this is necessary
	for _, e := range branch {
		p.AppendState(e.end.state)
		p.AppendPlan(e.plan) // should be fully calculate by now
	}
	return p
}

//endregion

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
func updateObstacles(o *common.Obstacles, n int) {
	for i := 0; i < n; i++ {
		var id int
		var x, y, heading, speed, t float64
		var line string = getLine()
		fmt.Sscanf(line, "%d %f %f %f %f %f", &id, &x, &y, &heading, &speed, &t)
		s := &common.State{X: x, Y: y, Heading: (heading * -1) + math.Pi/2, Speed: speed, Time: t}
		o.Update(id, s)
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
			path.Remove(common.State{X: float64(x), Y: float64(y)})
		}
		line = getLine()
		line = strings.TrimPrefix(line, "start state ")
		start := parseState(line)

		var nObstacles int
		o := new(common.Obstacles)
		fmt.Sscanf(getLine(), "dynamic obs %d", nObstacles)
		updateObstacles(o, nObstacles)

		// plan := makePlan(grid, start, *path, o)
		plan := bitStar(*start, timeToPlan, o)
		fmt.Println(plan.String())

		printLog("ready to plan")
	}
}

//endregion
