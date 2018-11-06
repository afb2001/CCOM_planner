package bitStar

import (
	"container/heap"
	"fmt"
	"github.com/afb2001/CCOM_planner/common"
	"github.com/afb2001/CCOM_planner/dubins"
	"log"
	"math"
	"math/rand"
	"reflect"
	"time"
)

const (
	verbose        bool    = false
	goalBias       float64 = 0
	maxSpeedBias   float64 = 1.0
	dubinsInc      float64 = 0.1 // this might be low
	K              int     = 5   // number of closest states to consider for BIT*
	bitStarSamples int     = 20  // (m in the paper) -- make this a parameter too
	// BIT* penalties (should all be made into parameters)
	coveragePenalty  float64 = 30
	collisionPenalty float64 = 600 // this is suspect... may need to be lower because it will be summed
	timePenalty      float64 = 1
)

//region BIT* globals

// make sure to set these before you call BitStar()

// these should be immutable so no pointers necessary
var start common.State
var grid common.Grid
var o common.Obstacles
var toCover common.Path
var bestVertex *Vertex
var maxSpeed, maxTurningRadius float64

func InitGlobals(g1 common.Grid, p common.Path, speed, radius float64) {
	grid, toCover, maxSpeed, maxTurningRadius = g1, p, speed, radius
}

//endregion

//region Util

/**
Log a message to stderr.
This is a copy of the function in main.go for convenience.
Definitely bad practice but here we are.
*/
func printLog(v interface{}) {
	log.Println("Planner message:", v)
}

/**
Print a fatal error and die.
This is a copy too.
*/
func printError(v interface{}) {
	log.Fatal("Planner error:", v)
}

/**
Returns the current time in seconds as a float
*/
func now() float64 {
	return float64(time.Now().UnixNano()) / 10e9
}

func showSamples(nodes, allSamples []*Vertex, g *common.Grid, start *common.State, path common.Path) string {
	var bytes = []byte(g.Dump())
	var arrays [][]byte
	for i := g.Height - 1; i >= 0; i-- {
		arrays = append(arrays, bytes[1+(i*(g.Width+1)):1+(i+1)*(g.Width+1)])
	}
	for _, s := range allSamples {
		if arrays[int(s.state.Y)][int(s.state.X)] == '_' {
			arrays[int(s.state.Y)][int(s.state.X)] = '.'
		}
	}
	//printLog("All nodes sampled:")
	for _, n := range nodes {
		//printLog(n.state.String())
		arrays[int(n.state.Y)][int(n.state.X)] = 'o'
	}
	arrays[int(start.Y)][int(start.X)] = '@'
	for _, p := range path {
		//printLog(p.X)
		arrays[int(p.Y)][int(p.X)] = '*'
	}
	//arrays[int(goal.y)][int(goal.x)] = '*'
	//printLog("Map:")
	return string(bytes)
}

//endregion

//region Vertex

type Vertex struct {
	state            *common.State
	approxCost       float64
	currentCost      float64
	currentCostIsSet bool
	approxToGo       float64
	parentEdge       *Edge
	uncovered        common.Path
}

// accessor methods that should handle caching and stuff
func (v *Vertex) ApproxCost() float64 {
	if v.approxCost == 0 {
		dPath, err := shortestPath(&start, v.state)
		if err != dubins.EDUBOK {
			v.approxCost = math.MaxFloat64
		} else {
			v.approxCost = dPath.Length() / maxSpeed * timePenalty
		}
	}
	//printLog(fmt.Sprintf("Approx cost: %f", v.approxCost))
	return v.approxCost
}

func (v *Vertex) CurrentCost() float64 {
	// updated in Edge.UpdateTrueCost()
	if v.currentCostIsSet {
		return v.currentCost
	} else {
		return math.MaxFloat64
	}
}

// get the cached heuristic value
func (v *Vertex) ApproxToGo() float64 {
	return v.approxToGo
}

// update the cached heuristic value
func (v *Vertex) UpdateApproxToGo(parent *Vertex) float64 {
	// is the whole parent thing really necessary? Yeah it probably is. Damn.
	parentNil := parent == nil
	if parent == nil {
		if v.parentEdge == nil {
			// TODO! -- !!
			// assume we're a sample... hopefully we are
			parent = &Vertex{state: &start} // this is correct, right?
		} else {
			parent = v.parentEdge.start
		}
	}
	// max euclidean distance to an uncovered point - coverage penalty for covering all of them
	// This is actually accurate if they're all in a straight line from your current heading,
	// which is not a super unlikely scenario, making this heuristic not as horrible as it may seem.
	// The parent's uncovered path is used because we probably don't know ours yet,
	// and if we do it could be wrong.
	approxToGo := parent.uncovered.MaxDistanceFrom(*v.state)/maxSpeed*timePenalty -
		float64(len(parent.uncovered))*coveragePenalty
	// if we're updating without specifying a parent we can cache it, but not otherwise
	if parentNil {
		v.approxToGo = approxToGo
	}
	return approxToGo
}

// This is really f_hat, which is g_hat + h_hat
func (v *Vertex) fValue() float64 {
	return v.ApproxCost() + v.UpdateApproxToGo(nil)
}

// contains function for convenience.
// Should consider using maps instead for contains performance.
func containsVertex(s []*Vertex, e *Vertex) bool {
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
func verticesFilter(vertices *[]*Vertex, f func(edge *Vertex) bool) {
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

func removeVertex(vertices *[]*Vertex, v *Vertex) {
	verticesFilter(vertices, func(x *Vertex) bool {
		return v != x
	})
}

//endregion

//region Edge

type Edge struct {
	start, end           *Vertex
	approxCost, trueCost float64
	dPath                *dubins.Path
	plan                 *common.Plan // plan to traverse dPath
}

func (e *Edge) ApproxCost() float64 {
	// create a dubins path if one doesn't already exist
	if e.dPath == nil {
		var err int
		e.dPath, err = shortestPath(e.start.state, e.end.state)
		if err != dubins.EDUBOK {
			e.approxCost = math.MaxFloat64
		} else {
			e.approxCost = e.dPath.Length() / maxSpeed * timePenalty
		}
	}
	return e.approxCost
}

// I think I actually wanted update end
func (e *Edge) UpdateStart(newStart *Vertex) {
	e.start = newStart
	// zero out the path and plan so we don't use them out-of-date
	e.dPath, e.plan = nil, nil
}

func (e *Edge) UpdateEnd(newEnd *Vertex) {
	// TODO! -- make sure this is right
	// printLog("Doing a questionable thing")
	e.end.parentEdge = nil // I wanna know if someone tries to use the out of date edge
	e.end = newEnd
	// zero out the path and plan so we don't use them out-of-date
	e.dPath, e.plan = nil, nil
}

// get the cached true cost
func (e Edge) TrueCost() float64 {
	return e.trueCost
}

// Updates the cached true cost of this edge.
// This is expensive.
func (e *Edge) UpdateTrueCost() float64 {
	var collisionPenalty float64
	var newlyCovered common.Path
	e.ApproxCost() // compute dPath if it isn't already done
	// compute the plan along the dubins path, the collision penalty, and the ending time
	// NOTE: this does a lot of work
	e.plan, collisionPenalty, newlyCovered, e.end.state.Time = getSamples(e.dPath, e.start.state.Time, e.start.uncovered)
	// update the uncovered path in e.end
	e.end.uncovered = e.start.uncovered
	for _, c := range newlyCovered {
		// TODO -- maybe make this more efficient... it shouldn't happen that much though
		e.end.uncovered = *(e.end.uncovered.Without(c))
	}
	// update e's true cost
	e.trueCost = e.netTime()*timePenalty + collisionPenalty - float64(len(newlyCovered))*coveragePenalty

	// update e.end's current cost
	// Not doing this anymore. Could be a mistake who knows
	// e.end.currentCost = e.start.currentCost + e.trueCost
	// e.end.currentCostIsSet = true

	return e.trueCost
}

func (e Edge) netTime() float64 {
	return e.end.state.Time - e.start.state.Time
}

// contains function for convenience.
// Should consider using maps instead for contains performance.
func ContainsEdge(s []*Edge, e *Edge) bool {
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
func EdgesFilter(edges *[]*Edge, f func(edge *Edge) bool) {
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
func RemoveEdgesEndingIn(edges *[]*Edge, v *Vertex) {
	EdgesFilter(edges, func(e *Edge) bool {
		return e.end != v
	})
}

//endregion

//region Queues

//region VertexQueue

type VertexQueue struct {
	nodes []*Vertex
	cost  func(node *Vertex) float64
}

func (h VertexQueue) Len() int { return len(h.nodes) }
func (h VertexQueue) Less(i, j int) bool {
	return h.cost(h.nodes[i]) < h.cost(h.nodes[j])
}
func (h VertexQueue) Swap(i, j int) { h.nodes[i], h.nodes[j] = h.nodes[j], h.nodes[i] }

func (h *VertexQueue) Push(x interface{}) {
	if x.(*Vertex).parentEdge == nil {
		printLog("We're doing a bad thing")
	}
	h.nodes = append(h.nodes, x.(*Vertex))
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

func makeVertexQueue(nodes []*Vertex, cost func(node *Vertex) float64) *VertexQueue {
	var nodeHeap = VertexQueue{nodes: nodes, cost: cost}
	for i, n := range nodes {
		nodeHeap.nodes[i] = n
	}
	heap.Init(&nodeHeap)
	return &nodeHeap
}

func (h *VertexQueue) update(cost func(node *Vertex) float64) {
	if cost != nil {
		h.cost = cost
	}
	heap.Init(h)
}

// func (h *VertexQueue) prune(cost float64) {
// 	newNodes := make([]*Vertex, len(h.nodes))
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
	nodes []*Edge
	cost  func(node *Edge) float64
}

func (h EdgeQueue) Len() int { return len(h.nodes) }
func (h EdgeQueue) Less(i, j int) bool {
	return h.cost(h.nodes[i]) < h.cost(h.nodes[j])
}
func (h EdgeQueue) Swap(i, j int) { h.nodes[i], h.nodes[j] = h.nodes[j], h.nodes[i] }

func (h *EdgeQueue) Push(x interface{}) {
	// printLog("Adding edge")
	h.nodes = append(h.nodes, x.(*Edge))
}

func (h *EdgeQueue) Pop() interface{} {
	// printLog("Popping edge")
	old := *h
	n := len(old.nodes)
	x := old.nodes[n-1]
	h.nodes = old.nodes[0 : n-1]
	return x
}

func (h *EdgeQueue) Peek() interface{} {
	return h.nodes[len(h.nodes)-1]
}

func makeEdgeQueue(nodes []*Edge, cost func(node *Edge) float64) *EdgeQueue {
	var nodeHeap = EdgeQueue{nodes: nodes, cost: cost}
	for i, n := range nodes {
		nodeHeap.nodes[i] = n
	}
	heap.Init(&nodeHeap)
	return &nodeHeap
}

func (h *EdgeQueue) update(cost func(node *Edge) float64) {
	if cost != nil {
		h.cost = cost
	}
	heap.Init(h)
}

// func (h *EdgeQueue) prune(cost float64, vertexCost func(node *Vertex) float64) {
// 	newNodes := make([]*Edge, len(h.nodes))
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
func vertexCost(v *Vertex) float64 {
	return v.CurrentCost() + v.UpdateApproxToGo(nil)
}

func edgeCost(edge *Edge) float64 {
	// printLog(*edge) //debug
	// NOTE: when the heuristic function becomes more expensive this will need to get changed
	return edge.start.CurrentCost() +
		edge.ApproxCost() +
		edge.end.UpdateApproxToGo(edge.start)
}

//endregion

//region State generation

/**
Create a new state with random values.
Time is unset (zero).
*/
func randomState(xMin, xMax, yMin, yMax float64) *common.State {
	s := new(common.State)
	s.X = rand.Float64()*float64(xMax-xMin) + xMin
	s.Y = rand.Float64()*float64(yMax-yMin) + yMin
	s.Heading = rand.Float64() * math.Pi * 2
	s.Speed = rand.Float64() * maxSpeed
	return s
}

/**
Create a random sample using the biasing constants.
*/
func biasedRandomState(xMin, xMax, yMin, yMax float64) *common.State {
	//verbose = false
	s := randomState(xMin, xMax, yMin, yMax)
	if r := rand.Float64(); r < maxSpeedBias {
		s.Speed = maxSpeed
	}
	// if r := rand.Float64(); r < goalBias {
	//verbose = true
	// return goal
	// }
	return s
}

/**
Sample a state whose euclidean distance to start is less than the given distance bound.
*/
func BoundedBiasedRandomState(bounds *common.Grid, path common.Path, start *common.State, distance float64) *common.State {
	if distance < 0 {
		distance = 0
	}
	s := biasedRandomState(math.Max(0, start.X-distance),
		math.Min(float64(bounds.Width), start.X+distance),
		math.Max(0, start.Y-distance),
		math.Min(float64(bounds.Height), start.Y+distance)) // TODO! -- path bias
	return s
}

//endregion

//region Dubins integration

/**
Find the shortest Dubins path between two states.
*/
func shortestPath(s1 *common.State, s2 *common.State) (path *dubins.Path, err int) {
	// if verbose {
	// 	printLog(fmt.Sprintf("Computing dubins path between %s, %s", s1.String(), s2.String()))
	// }
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
		t = inc / maxSpeed
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
	newlyCoveredNoDup := common.Path{}
	for _, s := range newlyCovered {
		var contains bool
		for _, x := range newlyCoveredNoDup {
			if x.IsSamePosition(&s) {
				contains = true
				break
			}
		}
		if !contains {
			newlyCoveredNoDup = append(newlyCoveredNoDup, s)
		}
	}
	return plan, penalty, newlyCoveredNoDup, t
}

//endregion

//region Algorithm 3 (Prune)

/**
Alg 3
*/
func Prune(samples *[]*Vertex, vertices *[]*Vertex, edges *[]*Edge, goalCost float64) {
	// line 1
	verticesFilter(samples, func(v *Vertex) bool {
		return !(v.fValue() >= goalCost)
	})
	// line 2
	verticesFilter(vertices, func(v *Vertex) bool {
		return !(v.fValue() > goalCost)
	})
	// line 3
	EdgesFilter(edges, func(e *Edge) bool {
		return !(e.start.fValue() > goalCost || e.end.fValue() > goalCost)
	})
	// lines 4-5
	verticesFilter(vertices, func(v *Vertex) bool {
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
func ExpandVertex(v *Vertex, qV *VertexQueue, qE *EdgeQueue,
	samples []*Vertex, vertices []*Vertex, edges []*Edge,
	vOld []*Vertex, goalCost float64) {

	if verbose {
		printLog(fmt.Sprintf("Expanding vertex %v", v.state.String()))
	}
	// already should have popped v from qV
	// printLog(qV.nodes)
	// find k nearest samples and make edges (Alg 2 lines 2-3)
	for _, e := range getKClosest(v, samples, goalCost) {
		if e == nil { // TODO -- fix bug in getKClosest that's letting nil values get put in
			continue
		}
		// printLog("Line 2.2, 2.3")
		// printLog(*e) //debug
		qE.Push(e)
	}

	// find k nearest vertices already in the tree? (Alg 2 lines 4-6)
	if !containsVertex(vOld, v) {
		closest := getKClosest(v, vertices, goalCost)
		for _, e := range closest {
			if e == nil {
				// printError("Added nil edge to queue")
				continue
			}
			if !ContainsEdge(edges, e) {
				// printLog(e == nil)
				if v.CurrentCost()+e.ApproxCost() < e.end.CurrentCost() {
					if e.start == e.end {
						printError("Adding cycle to edge queue!!!!!!!!!!!!!!!!!!!!!!!!!!!!!!!!!!!!!!!!!!!!!!!!!!!!!!!!")
					}
					// line 6.2 is in getKClosest
					// printLog("Line 2.6.2")
					qE.Push(e)
				}
			}
		}
	}

	// printLog(qE.nodes) //debug
}

/**
samples doesn't have to be actual samples it can come from anywhere
*/
func getKClosest(v *Vertex, samples []*Vertex, goalCost float64) (closest []*Edge) {
	closest = make([]*Edge, K) // TODO! -- use heap
	var i int
	var x *Vertex
	// printLog(v.ApproxCost()) //debug
	// printLog(goalCost) //debug
	for i, x = range samples {
		if x == v {
			continue // skip edges to the same sample
		}
		newEdge := &Edge{start: v, end: x}
		distance := newEdge.ApproxCost()
		// printLog(v.ApproxCost()+distance+x.UpdateApproxToGo(v) < goalCost) //debug
		// Can we assume that h has been calculated for all x we're being given?
		// This seems like a problematic assumption because h may depend on the branch
		// of the tree we're connecting to (path covered so far?)
		// No longer making that assumption but maybe we should in the future when h is more expensive?
		if !(v.ApproxCost()+distance+x.UpdateApproxToGo(v) < goalCost) {
			continue // skip edges that can't contribute to a better solution
		}
		// iterate through current best edges and replace the first one that's worse than this
		for j, edge := range closest {
			// printLog("Here") //debug
			if edge == nil {
				// printLog("Edge was nil")
				closest[j], x.parentEdge = newEdge, newEdge
				break
			} else if distance < edge.ApproxCost() {
				// printLog("Found a better sample")
				// edge.UpdateEnd(x)
				// x.parentEdge = edge
				closest[j], x.parentEdge = newEdge, newEdge // just use the new one
				break
			}
		}
	}
	// printLog(closest) //debug
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
func BitStar(startState common.State, timeRemaining float64, o1 *common.Obstacles) *common.Plan {
	endTime := timeRemaining + now()
	// setup
	o, start = *o1, startState // assign globals
	startV := &Vertex{state: &start, currentCostIsSet: true, uncovered: toCover}
	// TODO! -- verify that startV.currentCost = 0
	startV.currentCostIsSet = true
	startV.currentCost = float64(len(toCover)) * coveragePenalty // changed this from - to +, which makes sense
	startV.parentEdge = &Edge{start: startV, end: startV}
	bestVertex = startV
	samples := make([]*Vertex, 0)
	allSamples := make([]*Vertex, 0)
	var totalSampleCount int
	var vOld []*Vertex
	// line 1
	vertices := []*Vertex{startV}
	edges := make([]*Edge, 0)
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
			Prune(&samples, &vertices, &edges, bestVertex.CurrentCost())
			// line 6
			if verbose {
				printLog("Starting sampling")
			}
			samples = make([]*Vertex, bitStarSamples)
			printLog(fmt.Sprintf("Sampling state with distance less than %f", bestVertex.CurrentCost()))
			for m := 0; m < bitStarSamples; m++ {
				samples[m] = &Vertex{state: BoundedBiasedRandomState(&grid, toCover, &start, bestVertex.CurrentCost())}
			}
			totalSampleCount += bitStarSamples
			allSamples = append(allSamples, samples...)
			if verbose {
				printLog("Finished sampling")
			}
			// line 7
			// vOld is used in ExpandVertex to make sure we only add
			vOld = append([]*Vertex(nil), vertices...)
			// line 8
			qV.nodes = make([]*Vertex, len(vertices))
			copy(qV.nodes, vertices)
			qV.update(nil) // TODO -- refactor?
			// line 9 -- not doing that so shouldn't need to do anything
		}
		// shouldn't need this but I added it for safety
		if qV.Len() > 0 {
			// lines 10, 11
			for qE.Len() == 0 || (
			// TODO! -- investigate why it wasn't breaking without the Len() != 0 bit
			qV.Len() != 0 && qV.cost(qV.Peek().(*Vertex)) <= qE.cost(qE.Peek().(*Edge))) { //qE should only be empty when we're just starting
				ExpandVertex(qV.Pop().(*Vertex), qV, qE, samples, vertices, edges, vOld, bestVertex.CurrentCost())
			}
		}
		// printLog("Starting meat of algorithm") //debug
		// lines 12, 13
		edge := qE.Pop().(*Edge)
		vM, xM := edge.start, edge.end
		// line 14
		// vM should  be fully up to date at this point, but xM likely is not
		// Should we be using the f value for the best vertex here? I think that's
		// right but the paper is giving me pause...
		// Yeah pretty sure it's right. This is one of those places we're going to
		// do something different than the paper because of our path coverage goal.
		if verbose {
			printLog(fmt.Sprintf("V_m current cost: %f", vM.CurrentCost())) //debug
			printLog(fmt.Sprintf("Edge approx cost: %f", edge.ApproxCost()))
			printLog(fmt.Sprintf("h(X_m): %f", xM.UpdateApproxToGo(vM)))
			printLog(fmt.Sprintf("g_T(x_goal): %f", bestVertex.CurrentCost()))
		}
		if vM.CurrentCost()+edge.ApproxCost()+xM.UpdateApproxToGo(vM) < bestVertex.CurrentCost() {
			// line 15
			if verbose {
				printLog("made it through the first IF") //debug
				printLog(fmt.Sprintf("g_hat(V_m) = %f", vM.ApproxCost()))
				printLog(fmt.Sprintf("c(v_m, x_m) = %f", edge.UpdateTrueCost())) //REMOVE THIS! It's very inefficient
				printLog(fmt.Sprintf("h(x_m) = %f", xM.ApproxToGo()))
				printLog(fmt.Sprintf("g_T(x_goal) %f", bestVertex.CurrentCost()))
			}
			if vM.ApproxCost()+edge.UpdateTrueCost()+xM.ApproxToGo() < bestVertex.CurrentCost() {
				// by now xM is fully up to date and we have a path
				// line 16
				if verbose {
					printLog("Made it through the second IF ---------------------------------------------------------")
					printLog(fmt.Sprintf("g_T(V_m) = %f", vM.CurrentCost()))
					printLog(fmt.Sprintf("c(v_m, x_m) = %f", edge.TrueCost()))
					printLog(fmt.Sprintf("g_T(x_m) = %f", xM.CurrentCost()))
				}
				if vM.CurrentCost()+edge.TrueCost() < xM.CurrentCost() { // xM.currentCost is up to date
					if verbose {
						printLog("Made it through third IF ********************************")
					}
					// This is different:
					// Update the cached current cost of xM
					xM.currentCost, xM.currentCostIsSet = vM.CurrentCost()+edge.TrueCost(), true
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
					EdgesFilter(&qE.nodes, func(e *Edge) bool {
						return !(e.end == xM && e.start.CurrentCost()+e.ApproxCost() >= xM.CurrentCost())
					})
					// Should probably try to remove items from the heap while maintaining the
					// heap property but that sounds hard so I'm not gonna worry about it yet.
					// Or at least see if anything changed before doing this
					heap.Init(qE) // re-do heap order (O(n))
				}
				// this is different:
				// update best vertex (may not want to do it here but it seems convenient)
				if bestVertex.CurrentCost() > xM.CurrentCost() {
					bestVertex = xM
				}
			}
		} else {
			if verbose {
				printLog("Resetting queues")
			}
			// line 25
			qV.nodes = make([]*Vertex, 0)
			qE.nodes = make([]*Edge, 0)
		}
		if verbose {
			printLog("Done iteration +++++++++++++++++++++++++++++++++++++++++++")
		}
	}
	printLog("Done with the main loop. Now to trace the tree...")
	printLog("But first: samples!")
	printLog(showSamples(vertices, allSamples, &grid, &start, toCover))
	printLog(fmt.Sprintf("%d total samples, %d vertices connected", totalSampleCount, len(vertices)))

	// figure out the plan I guess
	// turn tree into slice
	branch := make([]*Edge, 0)
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
