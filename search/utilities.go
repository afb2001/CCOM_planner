package search

import (
	"github.com/afb2001/CCOM_planner/common"
	. "github.com/afb2001/CCOM_planner/globals"
	. "github.com/afb2001/CCOM_planner/util"
	"time"
)

//region GetKClosest

/**
samples doesn't have to be actual samples it can come from anywhere
*/
func GetKClosestVertices(v *Vertex, samples []*Vertex, goalCost float64) (closest []*Edge) {
	closest = make([]*Edge, K+1) // TODO! -- use heap!
	var i int
	var x *Vertex
	for i, x = range samples {
		if x == v {
			continue // skip edges to the same sample
		}
		if x.ParentEdge != nil {
			PrintError("Reassigning parent edge")
		}
		newEdge := &Edge{Start: v, End: x}
		distance := newEdge.ApproxCost()
		// PrintLog(v.ApproxCost()+distance+x.UpdateApproxToGo(v) < goalCost) //debug
		// Can we assume that h has been calculated for all x we're being given?
		// This seems like a problematic assumption because h may depend on the branch
		// of the tree we're connecting to (path covered so far?)
		// No longer making that assumption but maybe we should in the future when h is more expensive?
		// 2/22/19: This doesn't do anything for RHRSA* because it gets passed math.MaxFloat64 as goalCost
		if !(v.ApproxCost()+distance+x.UpdateApproxToGo(v) < goalCost) {
			continue // skip edges that can't contribute to a better solution
		}

		// iterate through current best edges and replace the first one that's worse than this
		for j, edge := range closest {
			if edge == nil {
				closest[j], x.ParentEdge = newEdge, newEdge
				break
			} else if distance < edge.ApproxCost() {
				tmp := closest[j]
				closest[j], x.ParentEdge = newEdge, newEdge
				newEdge = tmp
				//break
			}
		}
	}
	if i < K {
		if len(v.Uncovered) == 0 {
			return closest[0:i]
		} else {
			s := v.Uncovered.GetClosest(*v.State)
			closest[i+1] = &Edge{Start: v, End: &Vertex{State: &s, Uncovered: v.Uncovered}}
			i += 1
			return closest[0:i]
		}
	} else if len(v.Uncovered) > 0 {
		s := v.Uncovered.GetClosest(*v.State)
		closest[K] = &Edge{Start: v, End: &Vertex{State: &s, Uncovered: v.Uncovered}}
		closest[K].End.ParentEdge = closest[K]
		return
	} else {
		return
	}
}

func GetKClosest(v *Vertex, samples []*common.State) (closest []*Edge) {
	closest = make([]*Edge, K+1) // TODO! -- use heap!
	var i int
	var sample *common.State
	for i, sample = range samples {
		if sample == v.State {
			continue // skip edges to the same sample
		}
		x := &Vertex{State: sample}
		newEdge := &Edge{Start: v, End: x}
		distance := newEdge.ApproxCost()

		// iterate through current best edges and replace the first one that's worse than this
		for j, edge := range closest {
			if edge == nil {
				closest[j], x.ParentEdge = newEdge, newEdge
				break
			} else if distance < edge.ApproxCost() {
				tmp := closest[j]
				closest[j], x.ParentEdge = newEdge, newEdge
				newEdge = tmp
			}
		}
	}
	if i < K {
		if len(v.Uncovered) == 0 {
			return closest[0:i]
		} else {
			s := v.Uncovered.GetClosest(*v.State)
			closest[i+1] = &Edge{Start: v, End: &Vertex{State: &s}}
			i += 1
			return closest[0:i]
		}
	} else if len(v.Uncovered) > 0 {
		s := v.Uncovered.GetClosest(*v.State)
		closest[K] = &Edge{Start: v, End: &Vertex{State: &s}}
		closest[K].End.ParentEdge = closest[K]
		return
	} else {
		return
	}
}

//endregion

//region TracePlan

func TracePlan(v *Vertex, smoothing bool) *common.Plan {
	branch := make([]*Edge, 0)
	if v == nil {
		return nil
	}
	if v.ParentEdge == nil {
		PrintError("Nil parent edge")
	}
	if v.ParentEdge.Start == nil {
		PrintError("Nil parent edge Start")
	}

	if smoothing {
		// smoothing
		v.ParentEdge.Smooth()
	}

	// only cycle should be in Start vertex
	for cur := v; cur.ParentEdge.Start != cur; cur = cur.ParentEdge.Start {
		branch = append(branch, cur.ParentEdge)
	}

	// reverse the Plan order (this might look dumb)
	s := branch
	for i, j := 0, len(s)-1; i < j; i, j = i+1, j-1 {
		s[i], s[j] = s[j], s[i]
	}
	branch = s

	if Verbose && len(branch) > 0 {
		PrintLog("Current tree: ")
		PrintLog(branch[0].Start.State.String())
		for _, x := range branch {
			PrintLog(x.End.State.String())
		}
		PrintLog("Done printing tree.")
	}

	p := new(common.Plan)
	p.Start = Start
	//LastPlan = make([]*Vertex, len(branch))
	// p.AppendState(&Start) // yes this is necessary
	for _, e := range branch {
		//LastPlan[i] = e.End
		p.AppendPlan(GetPlan(e))
		p.AppendState(e.End.State)
	}
	return p
}

//endregion

/**
Returns the current time in seconds as a float
*/
func Now() float64 {
	return float64(time.Now().UnixNano()) / 10e9
}

func ShowSamples(nodes []*Vertex, allSamples []*common.State, g *common.Grid, start *common.State, path common.Path) string {
	var bytes = []byte(g.Dump())
	var arrays [][]byte
	for i := g.Height - 1; i >= 0; i-- {
		arrays = append(arrays, bytes[1+(i*(g.Width+1)):1+(i+1)*(g.Width+1)])
	}
	for _, s := range allSamples {
		if arrays[int(s.Y)][int(s.X)] == '_' {
			arrays[int(s.Y)][int(s.X)] = '.'
		}
	}
	for _, n := range nodes {
		arrays[int(n.State.Y)][int(n.State.X)] = 'o'
	}
	arrays[int(start.Y)][int(start.X)] = '@'
	for _, p := range path {
		arrays[int(p.Y)][int(p.X)] = '*'
	}
	return string(bytes)
}
