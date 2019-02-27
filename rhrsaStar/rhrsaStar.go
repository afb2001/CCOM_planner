package rhrsaStar

import (
	"container/heap"
	"fmt"
	"github.com/afb2001/CCOM_planner/common"
	. "github.com/afb2001/CCOM_planner/globals"
	. "github.com/afb2001/CCOM_planner/search"
	. "github.com/afb2001/CCOM_planner/util"
	"math"
)

//region RHRSA* globals

//var LastPlan []*Vertex

var BestVertex *Vertex

func ResetGlobals() {
	Start = common.State{}
	Obst = nil
	BestVertex = nil
}

//endregion

//region Expand

func Expand(sourceVertex *Vertex, qV *VertexQueue, samples *[]*common.State) {
	for _, e := range GetKClosest(sourceVertex, *samples) {
		if e == nil {
			continue
		}
		// remove the sample from the list (could be more efficient)
		//removeSample(samples, e.End.State)

		PrintVerbose(fmt.Sprintf("Connected to vertex at %s", e.End.State.String()))

		e.UpdateTrueCost()
		PrintVerbose(fmt.Sprintf("Edge Cost: %f", e.TrueCost()))

		if AggressiveSmoothing {
			e.Smooth()
		}

		destinationVertex := e.End

		// used to do these in UpdateTrueCost...
		destinationVertex.CurrentCost = e.Start.CurrentCost + e.TrueCost()
		destinationVertex.CurrentCostIsSet = true
		// is this pruning?
		// if BestVertex == nil || destinationVertex.GetCurrentCost() + destinationVertex.UpdateApproxToGo(nil) < BestVertex.GetCurrentCost(){

		destinationVertex.UpdateApproxToGo(nil)

		PrintVerbose(fmt.Sprintf("Destination vertex f value is: %f", destinationVertex.FValue()))
		PrintDebugVertex(destinationVertex.String(), "vertex")

		heap.Push(qV, destinationVertex)

		//VerifyBranch(destinationVertex)
		// }

	}
}

//endregion

func AStar(qV *VertexQueue, samples *[]*common.State, endTime float64) (vertex *Vertex) {
	PrintVerbose("Starting A*")
	for vertex = heap.Pop(qV).(*Vertex); ; {
		if Now() > endTime {
			return nil
		}
		PrintVerbose("Popping vertex at " + vertex.State.String())
		PrintVerbose(fmt.Sprintf(" whose cost is: f = g + h = %f + %f = %f", vertex.GetCurrentCost(), vertex.ApproxToGo(), vertex.GetCurrentCost()+vertex.ApproxToGo()))
		if vertex.State.Time > common.TimeHorizon+Start.Time || len(vertex.Uncovered) == 0 {
			PrintDebugVertex(vertex.String(), "goal")
			return
		}
		Expand(vertex, qV, samples)
		if qV.Len() == 0 {
			return nil
		}
		vertex = heap.Pop(qV).(*Vertex)
	}
}

func FindAStarPlan(startState common.State, toCover *common.Path, timeRemaining float64, o1 common.Obstacles) (bestPlan *common.Plan) {
	PrintDebug("done") // reset visuals
	ResetGlobals()
	// PrintLog("\n\n\n\n\n")
	// defer profile.Start().Stop()
	endTime := timeRemaining + Now()
	// setup
	Obst, Start = o1, startState // assign globals
	startV := &Vertex{State: &Start, CurrentCostIsSet: true, Uncovered: *toCover}
	startV.CurrentCostIsSet = true
	//startV.CurrentCost = float64(len(*toCover)) * CoveragePenalty
	startV.ParentEdge = &Edge{Start: startV, End: startV}
	BestVertex = nil
	// BestVertex = startV
	PrintDebugVertex(startV.String(), "Start")
	samples := make([]*common.State, 0)
	allSamples := make([]*common.State, 0)
	currentSampleCount := BitStarSamples
	var totalSampleCount int
	qV := new(VertexQueue)
	qV.Cost = func(v *Vertex) float64 {
		return v.CurrentCost + v.UpdateApproxToGo(nil)
	}
	for Now() < endTime {
		qV.Nodes = make([]*Vertex, 0) // wipe out old nodes
		heap.Push(qV, startV)
		PrintVerbose("Starting sampling")
		samples = make([]*common.State, len(allSamples)+currentSampleCount)
		// samples = make([]*Vertex, BitStarSamples)
		copy(samples, allSamples)
		//PrintLog(fmt.Sprintf("Sampling State with distance less than %f", BestVertex.GetCurrentCost()))
		for m := len(allSamples); m < len(allSamples)+currentSampleCount; m++ {
			// for m := 0; m < BitStarSamples; m++ {
			samples[m] = BoundedBiasedRandomState(&Grid, *toCover, &Start, math.MaxFloat64)
			PrintDebugVertex(samples[m].String()+" g = 0 h = 0", "sample")
		}
		totalSampleCount += currentSampleCount
		// also sample on the last best Plan
		//samples = append(samples, LastPlan...) // TODO

		// allSamples = append(allSamples, samples...)
		allSamples = samples
		PrintVerbose("Finished sampling")
		v := AStar(qV, &samples, endTime)
		// Assume the approx to go has been calculated
		if BestVertex == nil || (v != nil && v.CurrentCost+v.ApproxToGo() < BestVertex.CurrentCost+BestVertex.ApproxToGo()) {
			// found a plan
			BestVertex = v
			bestPlan = TracePlan(BestVertex, false)
			if Verbose {
				if BestVertex == nil {
					PrintLog(fmt.Sprintf("Couldn't find a plan this round."))
				} else {
					PrintLog(fmt.Sprintf("Cost of the current best plan: %f", BestVertex.GetCurrentCost()))
					PrintLog("Current best plan:")
					PrintLog(bestPlan.String())
				}
			}
		}
		PrintVerbose("++++++++++++++++++++++++++++++++++++++ Done iteration ++++++++++++++++++++++++++++++++++++++")
		currentSampleCount += currentSampleCount
	}
	PrintVerbose(ShowSamples(make([]*Vertex, 0), allSamples, &Grid, &Start, *toCover))
	if BestVertex == startV {
		PrintLog("Couldn't find a plan any better than staying put.")
	}
	PrintLog(fmt.Sprintf("%d total samples", totalSampleCount))
	return
}

// Just gonna leave this here for now
func PointToPointPlan(startState common.State, toCover *common.Path, timeRemaining float64, o1 common.Obstacles) (bestPlan *common.Plan) {
	Start = startState
	var cur, prev *Vertex
	cur = &Vertex{State: &startState}
	cur.ParentEdge = &Edge{Start: cur, End: cur}
	for _, p := range *toCover {
		prev = cur
		p1 := p
		cur = &Vertex{State: &p1}
		cur.ParentEdge = &Edge{Start: prev, End: cur}
		cur.ParentEdge.UpdateTrueCost()
	}
	return TracePlan(cur, false)
}

//endregion
