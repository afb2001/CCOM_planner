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

func Expand(v *Vertex, qV *VertexQueue, samples *[]*common.State) {
	for _, e := range GetKClosest(v, *samples, math.MaxFloat64) {
		if e == nil {
			continue
		}
		// remove the sample from the list (could be more efficient)
		//removeSample(samples, e.End.State)

		if Verbose {
			PrintLog(fmt.Sprintf("Connected to vertex at %s", e.End.State.String()))
		}
		e.UpdateTrueCost()
		if Verbose {
			PrintLog(fmt.Sprintf("Cost: %f", e.TrueCost()))
		}

		if AggressiveSmoothing {
			e.Smooth()
		}

		// used to do these in UpdateTrueCost...
		e.End.CurrentCost = e.Start.CurrentCost + e.TrueCost()
		e.End.CurrentCostIsSet = true
		// if BestVertex == nil || e.End.GetCurrentCost() + e.End.UpdateApproxToGo(nil) < BestVertex.GetCurrentCost(){

		e.End.UpdateApproxToGo(nil)

		PrintDebugVertex(e.End.String(), "vertex")

		heap.Push(qV, e.End)
		// TracePlan(e.End)
		// }

	}
}

//endregion

func RHRSAStar(qV *VertexQueue, samples *[]*common.State, endTime float64) (vertex *Vertex) {
	if Verbose {
		PrintLog("Starting A*")
	}
	for vertex = heap.Pop(qV).(*Vertex); vertex.State.Time < common.TimeHorizon+Start.Time; {
		if Now() > endTime {
			return nil
		}
		if Verbose {
			PrintLog("Popping vertex at " + vertex.State.String())
			PrintLog(fmt.Sprintf("f = g + h = %f + %f = %f", vertex.GetCurrentCost(), vertex.ApproxToGo(), vertex.GetCurrentCost()+vertex.ApproxToGo()))
		}
		Expand(vertex, qV, samples)
		if vertex.State.Time > common.TimeHorizon+Start.Time {
			// NOTE! -- this never seems to get hit
			PrintDebugVertex(vertex.String(), "goal")
			return
		}
		if qV.Len() == 0 {
			return nil
		}
		vertex = heap.Pop(qV).(*Vertex)
	}
	PrintDebugVertex(vertex.String(), "goal")
	return
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
	startV.CurrentCost = float64(len(*toCover)) * CoveragePenalty
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
		if Verbose {
			PrintLog("Starting sampling")
		}
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
		if Verbose {
			PrintLog("Finished sampling")
		}
		v := RHRSAStar(qV, &samples, endTime)
		// Assume the approx to go has been calculated
		if BestVertex == nil || (v != nil && v.CurrentCost+v.ApproxToGo() < BestVertex.CurrentCost+BestVertex.ApproxToGo()) {
			// PrintLog("Found a Plan")
			BestVertex = v
			bestPlan = TracePlan(BestVertex, true)
			if Verbose {
				if BestVertex == nil {
					PrintLog(fmt.Sprintf("Couldn't find a Plan this round."))
				} else {
					PrintLog(fmt.Sprintf("Cost of the current best Plan: %f", BestVertex.GetCurrentCost()))
					PrintLog("Current best Plan:")
					PrintLog(bestPlan.String())
				}
			}
		}
		// else {
		// 	PrintLog("Did not find a better Plan than the incumbent: ")
		// 	if BestVertex != nil {
		// 		PrintLog(fmt.Sprintf("Cost of the current best Plan: %f", BestVertex.GetCurrentCost()))
		// 	} else {
		// 		PrintLog("Infinity (incumbent is nil)")
		// 	}
		// }
		if Verbose {
			PrintLog("++++++++++++++++++++++++++++++++++++++ Done iteration ++++++++++++++++++++++++++++++++++++++")
		}
		//currentSampleCount = len(samples)
		currentSampleCount += currentSampleCount
		//PrintLog(currentSampleCount)
	}
	if Verbose {
		PrintLog(ShowSamples(make([]*Vertex, 0), allSamples, &Grid, &Start, *toCover))
	}
	if BestVertex == startV {
		PrintLog("Couldn't find a Plan any better than staying put.")
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
