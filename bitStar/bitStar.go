package bitStar

import (
	"container/heap"
	"fmt"
	"github.com/afb2001/CCOM_planner/common"
	"github.com/afb2001/CCOM_planner/dubins"
	"github.com/afb2001/CCOM_planner/tsp"
	. "github.com/afb2001/CCOM_planner/util"
	"math"
	"math/rand"
	"reflect"
	"time"
)

const (
	verbose        bool    = false
	goalBias       float64 = 0.05
	maxSpeedBias   float64 = 1.0
	dubinsInc      float64 = 0.1 // this might be low
	K              int     = 2   // number of closest states to consider for BIT*
	bitStarSamples int     = 5   // (m in the paper) -- make this a parameter too
	// BIT* penalties (should all be made into parameters)
	coveragePenalty  float64 = 60
	collisionPenalty float64 = 600 // this is suspect... may need to be lower because it will be summed
	timePenalty      float64 = 1
)

//region BIT* globals

// make sure to set these before you call BitStar()

// these should be immutable so no pointers necessary
var start common.State
var grid common.Grid
var o common.Obstacles

var lastPlan []*Vertex

// var toCover *common.Path
var bestVertex *Vertex
var maxSpeed, maxTurningRadius float64

var solver tsp.Solver

func InitGlobals(g1 common.Grid, speed, radius float64, s tsp.Solver) {
	grid, maxSpeed, maxTurningRadius, solver = g1, speed, radius, s
}

func resetGlobals() {
	start = common.State{}
	o = nil
	bestVertex = nil
}

//endregion

//region Util

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
	for _, n := range nodes {
		arrays[int(n.state.Y)][int(n.state.X)] = 'o'
	}
	arrays[int(start.Y)][int(start.X)] = '@'
	for _, p := range path {
		arrays[int(p.Y)][int(p.X)] = '*'
	}
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
	// is the whole parent thing really necessary? Yeah it probably is.
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
	//approxToGo := parent.uncovered.MaxDistanceFrom(*v.state)/maxSpeed*timePenalty -
	//	float64(len(parent.uncovered))*coveragePenalty

	// switching to TSP heuristic (inadmissible)
	approxToGo := solver.Solve(v.state.X, v.state.Y, parent.uncovered)/maxSpeed*timePenalty -
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

/**
Create a string suitable for debug visualization
*/
func (v *Vertex) String() string {
	return fmt.Sprintf("%s g = %f h = %f", v.state.String(), v.CurrentCost(), v.ApproxToGo())
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

// unused
func (e *Edge) UpdateEnd(newEnd *Vertex) {
	// should make sure this is right
	// PrintLog("Doing a questionable thing")
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
	// compute dPath if it isn't already done
	if e.dPath == nil {
		e.ApproxCost()
	}
	// compute the plan along the dubins path, the collision penalty, and the ending time
	// don't make a plan yet, that's expensive, just sample for collision checking
	collisionPenalty, newlyCovered, e.end.state.Time = getSamples(e.dPath, e.start.state.Time, e.start.uncovered)
	// update the uncovered path in e.end
	e.end.uncovered = e.start.uncovered
	for _, c := range newlyCovered {
		// maybe make this more efficient... it shouldn't happen that much though
		e.end.uncovered = e.end.uncovered.Without(c)
	}
	timeFromStart := e.end.state.Time - start.Time
	timeFromStart = 0 // remove for greediness
	// update e's true cost
	e.trueCost = e.netTime()*timePenalty + collisionPenalty - float64(len(newlyCovered))*(coveragePenalty-timeFromStart)

	// update e.end's current cost
	// Not doing this anymore. Could be a mistake who knows
	// e.end.currentCost = e.start.currentCost + e.trueCost
	// e.end.currentCostIsSet = true

	return e.trueCost
}

func (e *Edge) Smooth() {
	// if e is the start (or somehow there's a cycle...)
	if e.start.parentEdge == e {
		return
	}
	parentCost := e.start.parentEdge.TrueCost() // should be up to date in A*, check for BIT*
	currentCost := e.TrueCost()
	smoothedEdge := &Edge{start: e.start.parentEdge.start, end: e.end}
	smoothedEdge.UpdateTrueCost()
	if smoothedEdge.TrueCost() < parentCost+currentCost {
		// PrintLog(fmt.Sprintf("Smoothing edge %v to %v", *e, *smoothedEdge))
		*e = *smoothedEdge
		e.end.parentEdge = smoothedEdge

		e.end.currentCost = e.start.currentCost + e.trueCost
		e.end.currentCostIsSet = true

		e.Smooth()
	}
}

func (e Edge) netTime() float64 {
	if e.end.state.Time < e.start.state.Time {
		PrintError(fmt.Sprintf("Found backwards edge: %s to %s", e.start.state.String(), e.end.state.String()))
	}
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
		PrintLog("Added a vertex to qV with no parent edge!")
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
	h.nodes = append(h.nodes, x.(*Edge))
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

//endregion

// functions for queueing vertices and edges
func vertexCost(v *Vertex) float64 {
	return v.CurrentCost() + v.UpdateApproxToGo(nil)
}

func edgeCost(edge *Edge) float64 {
	// PrintLog(*edge) //debug
	// NOTE: when the heuristic function becomes more expensive this will need to get changed
	return edge.start.CurrentCost() +
		edge.ApproxCost() +
		edge.end.UpdateApproxToGo(edge.start)
}

//endregion

//region State generation

func chance(probability float64) bool {
	return rand.Float64() < probability
}

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
	s := randomState(xMin, xMax, yMin, yMax)
	if r := rand.Float64(); r < maxSpeedBias {
		s.Speed = maxSpeed
	}
	return s
}

// unused
func b(bounds common.Grid, point common.State, distance float64) *common.State {
	s := randomState(math.Max(0, point.X-distance),
		math.Min(float64(bounds.Width), point.X+distance),
		math.Max(0, point.Y-distance),
		math.Min(float64(bounds.Height), point.Y+distance))
	if r := rand.Float64(); r < maxSpeedBias {
		s.Speed = maxSpeed
	}
	if r := rand.Float64(); r < goalBias {
		return &point
	}
	return s
}

/**
Sample a state whose euclidean distance to start is less than the given distance bound.
*/
func BoundedBiasedRandomState(bounds *common.Grid, path common.Path, start *common.State, cost float64) *common.State {
	distance := cost * maxSpeed
	horizon := (common.TimeHorizon + 1) * maxSpeed
	distance = math.Min(distance, horizon)
	var s, point *common.State
	var l = int32(len(path))
	if i := rand.Int31n(l * 2); i >= l {
		point = start
	} else {
		point = &path[i]
	}
	s = biasedRandomState(math.Max(0, point.X-distance),
		math.Min(float64(bounds.Width), point.X+distance),
		math.Max(0, point.Y-distance),
		math.Min(float64(bounds.Height), point.Y+distance))
	if chance(goalBias) {
		s.X, s.Y = point.X, point.Y
	}
	return s
}

//endregion

//region Dubins integration

/**
Find the shortest Dubins path between two states.
*/
func shortestPath(s1 *common.State, s2 *common.State) (path *dubins.Path, err int) {
	// if verbose {
	// 	PrintLog(fmt.Sprintf("Computing dubins path between %s, %s", s1.String(), s2.String()))
	// }
	path = new(dubins.Path)
	err = dubins.ShortestPath(path, s1.ToArrayPointer(), s2.ToArrayPointer(), maxTurningRadius)
	return path, err
}

/**
Convert the given path into a plan and compute the sum collision cost and the newly covered path.
*/
func getSamplesAndPlan(path *dubins.Path, startTime float64, toCover common.Path) (plan *common.Plan, penalty float64, newlyCovered common.Path, finalTime float64) {
	plan = new(common.Plan)
	plan.Start.Time = startTime
	t := startTime // unused?
	callback := func(q *[3]float64, inc float64) int {
		t = inc/maxSpeed + startTime
		s := &common.State{X: q[0], Y: q[1], Heading: q[2], Speed: maxSpeed, Time: t}
		s.CollisionProbability = o.CollisionExists(s)
		if grid.IsBlocked(s.X, s.Y) {
			penalty += collisionPenalty
		} else if s.CollisionProbability > 0 {
			penalty += collisionPenalty * s.CollisionProbability
		}
		// PrintDebug(s.String(), "cost =", penalty, "color = 1 shape = dot")
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

func getPlan(edge *Edge) (plan *common.Plan) {
	plan = new(common.Plan)
	plan.Start.Time = edge.start.state.Time
	var t float64
	callback := func(q *[3]float64, inc float64) int {
		t = inc/maxSpeed + plan.Start.Time
		s := &common.State{X: q[0], Y: q[1], Heading: q[2], Speed: maxSpeed, Time: t}

		// this is for debugging and can be removed in production
		s.CollisionProbability = o.CollisionExistsWithArray(*q, t)
		if grid.IsBlocked(q[0], q[1]) {
			s.CollisionProbability = 1
		}

		plan.AppendState(s)
		return 0
	}
	err := edge.dPath.SampleMany(dubinsInc, callback)

	if err != dubins.EDUBOK {
		return nil
	}
	return
}

func getSamples(path *dubins.Path, startTime float64, toCover common.Path) (penalty float64, newlyCovered common.Path, finalTime float64) {
	t := startTime // unused?
	callback := func(q *[3]float64, inc float64) int {
		t = inc/maxSpeed + startTime
		collisionProbability := o.CollisionExistsWithArray(*q, t)
		if grid.IsBlocked(q[0], q[1]) {
			penalty += collisionPenalty
		} else if collisionProbability > 0 {
			penalty += collisionPenalty * collisionProbability
		}
		newlyCovered = append(newlyCovered, toCover.NewlyCoveredArray(*q)...) // splash operator I guess

		PrintTrajectoryState(q[0], q[1], q[2], (inc/maxSpeed*timePenalty)+penalty)

		return 0
	}
	err := path.SampleMany(dubinsInc, callback)

	if err != dubins.EDUBOK {
		return math.MaxFloat64, newlyCovered, 0
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
	return penalty, newlyCoveredNoDup, t
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
		PrintLog(fmt.Sprintf("Expanding vertex %v", v.state.String()))
	}
	// already should have popped v from qV
	// PrintLog(qV.nodes)
	// find k nearest samples and make edges (Alg 2 lines 2-3)
	for _, e := range getKClosest(v, samples, goalCost) {
		if e == nil { // TODO -- fix bug in getKClosest that's letting nil values get put in
			continue
		}
		// PrintLog("Line 2.2, 2.3")
		// PrintLog(*e) //debug
		qE.Push(e)
	}

	// find k nearest vertices already in the tree? (Alg 2 lines 4-6)
	if !containsVertex(vOld, v) {
		closest := getKClosest(v, vertices, goalCost)
		for _, e := range closest {
			if e == nil {
				// PrintError("Added nil edge to queue")
				continue
			}
			if !ContainsEdge(edges, e) {
				// PrintLog(e == nil)
				if v.CurrentCost()+e.ApproxCost() < e.end.CurrentCost() {
					if e.start == e.end {
						PrintError("Adding cycle to edge queue!!!!!!!!!!!!!!!!!!!!!!!!!!!!!!!!!!!!!!!!!!!!!!!!!!!!!!!!")
					}
					// line 6.2 is in getKClosest
					// PrintLog("Line 2.6.2")
					qE.Push(e)
				}
			}
		}
	}

	// PrintLog(qE.nodes) //debug
}

/**
samples doesn't have to be actual samples it can come from anywhere
*/
func getKClosest(v *Vertex, samples []*Vertex, goalCost float64) (closest []*Edge) {
	closest = make([]*Edge, K) // TODO! -- use heap
	var i int
	var x *Vertex
	// PrintLog(v.ApproxCost()) //debug
	// PrintLog(goalCost) //debug
	for i, x = range samples {
		if x == v {
			continue // skip edges to the same sample
		}
		newEdge := &Edge{start: v, end: x}
		distance := newEdge.ApproxCost()
		// PrintLog(v.ApproxCost()+distance+x.UpdateApproxToGo(v) < goalCost) //debug
		// Can we assume that h has been calculated for all x we're being given?
		// This seems like a problematic assumption because h may depend on the branch
		// of the tree we're connecting to (path covered so far?)
		// No longer making that assumption but maybe we should in the future when h is more expensive?
		if !(v.ApproxCost()+distance+x.UpdateApproxToGo(v) < goalCost) {
			continue // skip edges that can't contribute to a better solution
		}
		// iterate through current best edges and replace the first one that's worse than this
		for j, edge := range closest {
			// PrintLog("Here") //debug
			if edge == nil {
				// PrintLog("Edge was nil")
				closest[j], x.parentEdge = newEdge, newEdge
				break
			} else if distance < edge.ApproxCost() {
				// PrintLog("Found a better sample")
				// edge.UpdateEnd(x)
				// x.parentEdge = edge
				closest[j], x.parentEdge = newEdge, newEdge // just use the new one
				break
			}
		}
	}
	// PrintLog(closest) //debug
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
func BitStar(startState common.State, toCover *common.Path, timeRemaining float64, o1 common.Obstacles) *common.Plan {
	endTime := timeRemaining + now()
	// setup
	o, start = o1, startState // assign globals
	startV := &Vertex{state: &start, currentCostIsSet: true, uncovered: *toCover}
	// TODO! -- verify that startV.currentCost = 0
	startV.currentCostIsSet = true
	startV.currentCost = float64(len(*toCover)) * coveragePenalty // changed this from - to +, which makes sense
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
				PrintLog("Starting sampling")
			}
			samples = make([]*Vertex, bitStarSamples)
			if verbose {
				PrintLog(fmt.Sprintf("Sampling state with distance less than %f", bestVertex.CurrentCost()))
			}
			for m := 0; m < bitStarSamples; m++ {
				samples[m] = &Vertex{state: BoundedBiasedRandomState(&grid, *toCover, &start, bestVertex.CurrentCost())}
			}
			totalSampleCount += bitStarSamples
			allSamples = append(allSamples, samples...)
			if verbose {
				PrintLog("Finished sampling")
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
				ExpandVertex(heap.Pop(qV).(*Vertex), qV, qE, samples, vertices, edges, vOld, bestVertex.CurrentCost())
			}
		}
		// PrintLog("Starting meat of algorithm") //debug
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
			PrintLog(fmt.Sprintf("V_m current cost: %f", vM.CurrentCost())) //debug
			PrintLog(fmt.Sprintf("Edge approx cost: %f", edge.ApproxCost()))
			PrintLog(fmt.Sprintf("h(X_m): %f", xM.UpdateApproxToGo(vM)))
			PrintLog(fmt.Sprintf("g_T(x_goal): %f", bestVertex.CurrentCost()))
		}
		if vM.CurrentCost()+edge.ApproxCost()+xM.UpdateApproxToGo(vM) < bestVertex.CurrentCost() {
			// line 15
			if verbose {
				PrintLog("made it through the first IF") //debug
				PrintLog(fmt.Sprintf("g_hat(V_m) = %f", vM.ApproxCost()))
				PrintLog(fmt.Sprintf("c(v_m, x_m) = %f", edge.UpdateTrueCost())) //REMOVE THIS! It's very inefficient
				PrintLog(fmt.Sprintf("h(x_m) = %f", xM.ApproxToGo()))
				PrintLog(fmt.Sprintf("g_T(x_goal) %f", bestVertex.CurrentCost()))
			}
			if vM.ApproxCost()+edge.UpdateTrueCost()+xM.ApproxToGo() < bestVertex.CurrentCost() {
				// by now xM is fully up to date and we have a path
				// line 16
				if verbose {
					PrintLog("Made it through the second IF ---------------------------------------------------------")
					PrintLog(fmt.Sprintf("g_T(V_m) = %f", vM.CurrentCost()))
					PrintLog(fmt.Sprintf("c(v_m, x_m) = %f", edge.TrueCost()))
					PrintLog(fmt.Sprintf("g_T(x_m) = %f", xM.CurrentCost()))
				}
				if vM.CurrentCost()+edge.TrueCost() < xM.CurrentCost() { // xM.currentCost is up to date
					if verbose {
						PrintLog("Made it through third IF ********************************")
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
						heap.Push(qV, xM)
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
				PrintLog("Resetting queues")
			}
			// line 25
			qV.nodes = make([]*Vertex, 0)
			qE.nodes = make([]*Edge, 0)
		}
		if verbose {
			PrintLog("++++++++++++++++++++++++++++++++++++++ Done iteration ++++++++++++++++++++++++++++++++++++++")
		}
	}
	PrintLog("Done with the main loop. Now to trace the tree...")
	PrintLog("But first: samples!")
	//PrintLog(showSamples(vertices, allSamples, &grid, &start, *toCover))
	PrintLog(fmt.Sprintf("%d total samples, %d vertices connected", totalSampleCount, len(vertices)))

	//// figure out the plan I guess
	//// turn tree into slice
	//branch := make([]*Edge, 0)
	//for cur := bestVertex; cur != startV; cur = cur.parentEdge.start {
	//	branch = append(branch, cur.parentEdge)
	//}
	//
	//// reverse the plan order (this might look dumb)
	//s := branch
	//for i, j := 0, len(s)-1; i < j; i, j = i+1, j-1 {
	//	s[i], s[j] = s[j], s[i]
	//}
	//branch = s
	//
	//p := new(common.Plan)
	//p.Start = start
	//p.AppendState(&start) // yes this is necessary
	//for _, e := range branch {
	//	p.AppendState(e.end.state)
	//	p.AppendPlan(e.plan) // should be fully calculate by now
	//}
	p := TracePlan(bestVertex, true)
	return p
}

//endregion

//region A*

func Expand(v *Vertex, qV *VertexQueue, samples *[]*Vertex) {
	for _, e := range getKClosest(v, *samples, math.MaxFloat64) {
		if e == nil {
			continue
		}
		// remove the sample from the list (could be more efficient)
		verticesFilter(samples, func(x *Vertex) bool {
			return e.end != x
		})
		if verbose {
			PrintLog(fmt.Sprintf("Connected to vertex at %s", e.end.state.String()))
		}
		e.UpdateTrueCost()
		if verbose {
			PrintLog(fmt.Sprintf("Cost: %f", e.TrueCost()))
		}

		// aggressive smoothing
		//e.Smooth()

		// used to do these in UpdateTrueCost...
		e.end.currentCost = e.start.currentCost + e.trueCost
		e.end.currentCostIsSet = true
		// if bestVertex == nil || e.end.CurrentCost() + e.end.UpdateApproxToGo(nil) < bestVertex.CurrentCost(){

		e.end.UpdateApproxToGo(nil)

		PrintDebugVertex(e.end.String(), "vertex")

		heap.Push(qV, e.end)
		// TracePlan(e.end)
		// }

	}
}

func AStar(qV *VertexQueue, samples *[]*Vertex, endTime float64) (vertex *Vertex) {
	if verbose {
		PrintLog("Starting A*")
	}
	for vertex = heap.Pop(qV).(*Vertex); vertex.state.Time < 30+start.Time; {
		if now() > endTime {
			return nil
		}
		if verbose {
			PrintLog("Popping vertex at " + vertex.state.String())
			PrintLog(fmt.Sprintf("f = g + h = %f + %f = %f", vertex.CurrentCost(), vertex.ApproxToGo(), vertex.CurrentCost()+vertex.ApproxToGo()))
		}
		Expand(vertex, qV, samples)
		if vertex.state.Time > 30+start.Time {
			PrintDebugVertex(vertex.String(), "goal")
			return
		}
		if qV.Len() == 0 {
			return nil
		}
		vertex = heap.Pop(qV).(*Vertex)
	}
	return
}

func TracePlan(v *Vertex, smoothing bool) *common.Plan {
	branch := make([]*Edge, 0)
	if v == nil {
		return nil
	}
	if v.parentEdge == nil {
		PrintError("Nil parent edge")
	}
	if v.parentEdge.start == nil {
		PrintError("Nil parent edge start")
	}

	if smoothing {
		// smoothing
		v.parentEdge.Smooth()
	}

	// only cycle should be in start vertex
	for cur := v; cur.parentEdge.start != cur; cur = cur.parentEdge.start {
		branch = append(branch, cur.parentEdge)
	}

	// reverse the plan order (this might look dumb)
	s := branch
	for i, j := 0, len(s)-1; i < j; i, j = i+1, j-1 {
		s[i], s[j] = s[j], s[i]
	}
	branch = s

	if verbose && len(branch) > 0 {
		PrintLog("Current tree: ")
		PrintLog(branch[0].start.state.String())
		for _, x := range branch {
			PrintLog(x.end.state.String())
		}
		PrintLog("Done printing tree.")
	}

	p := new(common.Plan)
	p.Start = start
	lastPlan = make([]*Vertex, len(branch))
	// p.AppendState(&start) // yes this is necessary
	for i, e := range branch {
		lastPlan[i] = e.end
		p.AppendPlan(getPlan(e))
		p.AppendState(e.end.state)
	}
	return p
}

func FindAStarPlan(startState common.State, toCover *common.Path, timeRemaining float64, o1 common.Obstacles) (bestPlan *common.Plan) {
	PrintDebug("done") // reset visuals
	resetGlobals()
	// PrintLog("\n\n\n\n\n")
	// defer profile.Start().Stop()
	endTime := timeRemaining + now()
	// setup
	o, start = o1, startState // assign globals
	startV := &Vertex{state: &start, currentCostIsSet: true, uncovered: *toCover}
	startV.currentCostIsSet = true
	startV.currentCost = float64(len(*toCover)) * coveragePenalty
	startV.parentEdge = &Edge{start: startV, end: startV}
	bestVertex = nil
	// bestVertex = startV
	PrintDebugVertex(startV.String(), "start")
	samples := make([]*Vertex, 0)
	allSamples := make([]*Vertex, 0)
	currentSampleCount := bitStarSamples
	var totalSampleCount int
	qV := new(VertexQueue)
	qV.cost = func(v *Vertex) float64 {
		return v.currentCost + v.UpdateApproxToGo(nil)
	}
	for now() < endTime {
		qV.nodes = make([]*Vertex, 0) // wipe out old nodes
		heap.Push(qV, startV)
		if verbose {
			PrintLog("Starting sampling")
		}
		samples = make([]*Vertex, len(allSamples)+currentSampleCount)
		// samples = make([]*Vertex, bitStarSamples)
		copy(samples, allSamples)
		//PrintLog(fmt.Sprintf("Sampling state with distance less than %f", bestVertex.CurrentCost()))
		for m := len(allSamples); m < len(allSamples)+currentSampleCount; m++ {
			// for m := 0; m < bitStarSamples; m++ {
			samples[m] = &Vertex{state: BoundedBiasedRandomState(&grid, *toCover, &start, math.MaxFloat64)}
		}
		totalSampleCount += currentSampleCount
		// also sample on the last best plan
		samples = append(samples, lastPlan...)
		// allSamples = append(allSamples, samples...)
		allSamples = samples
		if verbose {
			PrintLog("Finished sampling")
		}
		v := AStar(qV, &samples, endTime)
		// Assume the approx to go has been calculated
		if bestVertex == nil || (v != nil && v.currentCost+v.ApproxToGo() < bestVertex.currentCost+bestVertex.ApproxToGo()) {
			// PrintLog("Found a plan")
			bestVertex = v
			bestPlan = TracePlan(bestVertex, true)
			if verbose {
				PrintLog(fmt.Sprintf("Cost of the current best plan: %f", bestVertex.CurrentCost()))
				PrintLog("Current best plan:")
				PrintLog(bestPlan.String())
			}
		}
		// else {
		// 	PrintLog("Did not find a better plan than the incumbent: ")
		// 	if bestVertex != nil {
		// 		PrintLog(fmt.Sprintf("Cost of the current best plan: %f", bestVertex.CurrentCost()))
		// 	} else {
		// 		PrintLog("Infinity (incumbent is nil)")
		// 	}
		// }
		if verbose {
			PrintLog("++++++++++++++++++++++++++++++++++++++ Done iteration ++++++++++++++++++++++++++++++++++++++")
		}
		//currentSampleCount = len(samples)
		currentSampleCount += currentSampleCount
		//PrintLog(currentSampleCount)
	}
	if verbose {
		PrintLog(showSamples(make([]*Vertex, 0), allSamples, &grid, &start, *toCover))
	}
	if bestVertex == startV {
		PrintLog("Couldn't find a plan any better than staying put.")
	}
	PrintLog(fmt.Sprintf("%d total samples", totalSampleCount))
	return
}

func PointToPointPlan(startState common.State, toCover *common.Path, timeRemaining float64, o1 common.Obstacles) (bestPlan *common.Plan) {
	start = startState
	var cur, prev *Vertex
	cur = &Vertex{state: &startState}
	cur.parentEdge = &Edge{start: cur, end: cur}
	for _, p := range *toCover {
		prev = cur
		p1 := p
		cur = &Vertex{state: &p1}
		cur.parentEdge = &Edge{start: prev, end: cur}
		cur.parentEdge.UpdateTrueCost()
	}
	return TracePlan(cur, false)
}

//endregion
