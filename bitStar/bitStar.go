package bitStar

import (
	"container/heap"
	"fmt"
	"github.com/afb2001/CCOM_planner/common"
	. "github.com/afb2001/CCOM_planner/globals"
	. "github.com/afb2001/CCOM_planner/search"
	. "github.com/afb2001/CCOM_planner/util"
	"reflect"
)

//region BIT* globals

var BestVertex *Vertex

//endregion

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

func removeSample(samples *[]*common.State, s *common.State) {
	if samples == nil {
		return
	}
	b := (*samples)[:0]
	for _, x := range *samples {
		if s != x {
			b = append(b, x)
		}
	}
	*samples = b
}

//region Algorithm 3 (Prune)

/**
Alg 3
*/
func Prune(samples *[]*Vertex, vertices *[]*Vertex, edges *[]*Edge, goalCost float64) {
	// line 1
	verticesFilter(samples, func(v *Vertex) bool {
		return !(v.FValue() >= goalCost)
	})
	// line 2
	verticesFilter(vertices, func(v *Vertex) bool {
		return !(v.FValue() > goalCost)
	})
	// line 3
	EdgesFilter(edges, func(e *Edge) bool {
		return !(e.Start.FValue() > goalCost || e.End.FValue() > goalCost)
	})
	// lines 4-5
	verticesFilter(vertices, func(v *Vertex) bool {
		if v.CurrentCostIsSet {
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

	if Verbose {
		PrintLog(fmt.Sprintf("Expanding vertex %v", v.State.String()))
	}
	// already should have popped v from qV
	// PrintLog(qV.nodes)
	// find k nearest samples and make edges (Alg 2 lines 2-3)
	for _, e := range GetKClosest_old(v, samples, goalCost) {
		if e == nil { // TODO -- fix bug in GetKClosest_old that's letting nil values get put in
			continue
		}
		// PrintLog("Line 2.2, 2.3")
		// PrintLog(*e) //debug
		qE.Push(e)
	}

	// find k nearest vertices already in the tree? (Alg 2 lines 4-6)
	if !containsVertex(vOld, v) {
		closest := GetKClosest_old(v, vertices, goalCost)
		for _, e := range closest {
			if e == nil {
				// PrintError("Added nil edge to queue")
				continue
			}
			if !ContainsEdge(edges, e) {
				// PrintLog(e == nil)
				if v.GetCurrentCost()+e.ApproxCost() < e.End.GetCurrentCost() {
					if e.Start == e.End {
						PrintError("Adding cycle to edge queue!!!!!!!!!!!!!!!!!!!!!!!!!!!!!!!!!!!!!!!!!!!!!!!!!!!!!!!!")
					}
					// line 6.2 is in GetKClosest_old
					// PrintLog("Line 2.6.2")
					qE.Push(e)
				}
			}
		}
	}

	// PrintLog(qE.nodes) //debug
}

//endregion

//region Algorithm 1 (BIT*)

/**
Alg 1 (obviously)
*/
func BitStar(startState common.State, toCover *common.Path, timeRemaining float64, o1 common.Obstacles) *common.Plan {
	endTime := timeRemaining + Now()
	// setup
	Obst, Start = o1, startState // assign globals
	startV := &Vertex{State: &Start, CurrentCostIsSet: true, Uncovered: *toCover}
	// TODO! -- verify that startV.CurrentCost = 0
	startV.CurrentCostIsSet = true
	startV.CurrentCost = float64(len(*toCover)) * CoveragePenalty // changed this from - to +, which makes sense
	startV.ParentEdge = &Edge{Start: startV, End: startV}
	BestVertex = startV
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
	qE.Cost = EdgeCost
	qV.Cost = VertexCost
	// line 3
	for Now() < endTime {
		// line 4
		if qE.Len() == 0 && qV.Len() == 0 {
			// line 5
			Prune(&samples, &vertices, &edges, BestVertex.GetCurrentCost())
			// line 6
			if Verbose {
				PrintLog("Starting sampling")
			}
			samples = make([]*Vertex, BitStarSamples)
			if Verbose {
				PrintLog(fmt.Sprintf("Sampling State with distance less than %f", BestVertex.GetCurrentCost()))
			}
			for m := 0; m < BitStarSamples; m++ {
				samples[m] = &Vertex{State: BoundedBiasedRandomState(&Grid, *toCover, &Start, BestVertex.GetCurrentCost())}
			}
			totalSampleCount += BitStarSamples
			allSamples = append(allSamples, samples...)
			if Verbose {
				PrintLog("Finished sampling")
			}
			// line 7
			// vOld is used in ExpandVertex to make sure we only add
			vOld = append([]*Vertex(nil), vertices...)
			// line 8
			qV.Nodes = make([]*Vertex, len(vertices))
			copy(qV.Nodes, vertices)
			qV.Update(nil) // TODO -- refactor?
			// line 9 -- not doing that so shouldn't need to do anything
		}
		// shouldn't need this but I added it for safety
		if qV.Len() > 0 {
			// lines 10, 11
			for qE.Len() == 0 || (
			// TODO! -- investigate why it wasn't breaking without the Len() != 0 bit
			qV.Len() != 0 && qV.Cost(qV.Peek().(*Vertex)) <= qE.Cost(qE.Peek().(*Edge))) { //qE should only be empty when we're just starting
				ExpandVertex(heap.Pop(qV).(*Vertex), qV, qE, samples, vertices, edges, vOld, BestVertex.GetCurrentCost())
			}
		}
		// PrintLog("Starting meat of algorithm") //debug
		// lines 12, 13
		edge := qE.Pop().(*Edge)
		vM, xM := edge.Start, edge.End
		// line 14
		// vM should  be fully up to date at this point, but xM likely is not
		// Should we be using the f value for the best vertex here? I think that's
		// right but the paper is giving me pause...
		// Yeah pretty sure it's right. This is one of those places we're going to
		// do something different than the paper because of our path coverage goal.
		if Verbose {
			PrintLog(fmt.Sprintf("V_m current cost: %f", vM.GetCurrentCost())) //debug
			PrintLog(fmt.Sprintf("Edge approx cost: %f", edge.ApproxCost()))
			PrintLog(fmt.Sprintf("h(X_m): %f", xM.UpdateApproxToGo(vM)))
			PrintLog(fmt.Sprintf("g_T(x_goal): %f", BestVertex.GetCurrentCost()))
		}
		if vM.GetCurrentCost()+edge.ApproxCost()+xM.UpdateApproxToGo(vM) < BestVertex.GetCurrentCost() {
			// line 15
			if Verbose {
				PrintLog("made it through the first IF") //debug
				PrintLog(fmt.Sprintf("g_hat(V_m) = %f", vM.ApproxCost()))
				PrintLog(fmt.Sprintf("c(v_m, x_m) = %f", edge.UpdateTrueCost())) //REMOVE THIS! It's very inefficient
				PrintLog(fmt.Sprintf("h(x_m) = %f", xM.ApproxToGo()))
				PrintLog(fmt.Sprintf("g_T(x_goal) %f", BestVertex.GetCurrentCost()))
			}
			if vM.ApproxCost()+edge.UpdateTrueCost()+xM.ApproxToGo() < BestVertex.GetCurrentCost() {
				// by Now xM is fully up to date and we have a path
				// line 16
				if Verbose {
					PrintLog("Made it through the second IF ---------------------------------------------------------")
					PrintLog(fmt.Sprintf("g_T(V_m) = %f", vM.GetCurrentCost()))
					PrintLog(fmt.Sprintf("c(v_m, x_m) = %f", edge.TrueCost()))
					PrintLog(fmt.Sprintf("g_T(x_m) = %f", xM.GetCurrentCost()))
				}
				if vM.GetCurrentCost()+edge.TrueCost() < xM.GetCurrentCost() { // xM.CurrentCost is up to date
					if Verbose {
						PrintLog("Made it through third IF ********************************")
					}
					// This is different:
					// Update the cached current cost of xM
					xM.CurrentCost, xM.CurrentCostIsSet = vM.GetCurrentCost()+edge.TrueCost(), true
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
					EdgesFilter(&qE.Nodes, func(e *Edge) bool {
						return !(e.End == xM && e.Start.GetCurrentCost()+e.ApproxCost() >= xM.GetCurrentCost())
					})
					// Should probably try to remove items from the heap while maintaining the
					// heap property but that sounds hard so I'm not gonna worry about it yet.
					// Or at least see if anything changed before doing this
					heap.Init(qE) // re-do heap order (O(n))
				}
				// this is different:
				// Update best vertex (may not want to do it here but it seems convenient)
				if BestVertex.GetCurrentCost() > xM.GetCurrentCost() {
					BestVertex = xM
				}
			}
		} else {
			if Verbose {
				PrintLog("Resetting queues")
			}
			// line 25
			qV.Nodes = make([]*Vertex, 0)
			qE.Nodes = make([]*Edge, 0)
		}
		if Verbose {
			PrintLog("++++++++++++++++++++++++++++++++++++++ Done iteration ++++++++++++++++++++++++++++++++++++++")
		}
	}
	PrintLog("Done with the main loop. Now to trace the tree...")
	PrintLog("But first: samples!")
	//PrintLog(ShowSamples(vertices, allSamples, &Grid, &Start, *toCover))
	PrintLog(fmt.Sprintf("%d total samples, %d vertices connected", totalSampleCount, len(vertices)))

	//// figure out the Plan I guess
	//// turn tree into slice
	//branch := make([]*Edge, 0)
	//for cur := BestVertex; cur != startV; cur = cur.ParentEdge.Start {
	//	branch = append(branch, cur.ParentEdge)
	//}
	//
	//// reverse the Plan order (this might look dumb)
	//s := branch
	//for i, j := 0, len(s)-1; i < j; i, j = i+1, j-1 {
	//	s[i], s[j] = s[j], s[i]
	//}
	//branch = s
	//
	//p := new(common.Plan)
	//p.Start = Start
	//p.AppendState(&Start) // yes this is necessary
	//for _, e := range branch {
	//	p.AppendState(e.End.State)
	//	p.AppendPlan(e.Plan) // should be fully calculate by Now
	//}
	p := TracePlan(BestVertex, true)
	return p
}

//endregion
