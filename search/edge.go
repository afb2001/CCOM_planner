package search

import (
	"fmt"
	"github.com/afb2001/CCOM_planner/common"
	"github.com/afb2001/CCOM_planner/dubins"
	. "github.com/afb2001/CCOM_planner/globals"
	. "github.com/afb2001/CCOM_planner/util"
	"math"
	"reflect"
)

//region Edge

type Edge struct {
	Start, End           *Vertex
	approxCost, trueCost float64
	DPath                *dubins.Path
	Plan                 *common.Plan // Plan to traverse DPath
}

func (e *Edge) ApproxCost() float64 {
	// create a dubins path if one doesn't already exist
	if e.DPath == nil {
		var err int
		e.DPath, err = shortestPath(e.Start.State, e.End.State)
		if err != dubins.EDUBOK {
			e.approxCost = math.MaxFloat64
		} else {
			e.approxCost = e.DPath.Length() / MaxSpeed * TimePenalty
		}
	}
	return e.approxCost
}

// I think I actually wanted update End
func (e *Edge) UpdateStart(newStart *Vertex) {
	e.Start = newStart
	// zero out the path and Plan so we don't use them out-of-date
	e.DPath, e.Plan = nil, nil
}

// unused
func (e *Edge) UpdateEnd(newEnd *Vertex) {
	// should make sure this is right
	// PrintLog("Doing a questionable thing")
	e.End.ParentEdge = nil // I wanna know if someone tries to use the out of date edge
	e.End = newEnd
	// zero out the path and Plan so we don't use them out-of-date
	e.DPath, e.Plan = nil, nil
}

// get the cached true Cost
func (e Edge) TrueCost() float64 {
	return e.trueCost
}

// Updates the cached true Cost of this edge.
// This is expensive.
func (e *Edge) UpdateTrueCost() float64 {
	var collisionPenalty float64
	var newlyCovered common.Path
	// compute DPath if it isn't already done
	if e.DPath == nil {
		e.ApproxCost()
	}
	// compute the Plan along the dubins path, the collision penalty, and the ending time
	// don't make a Plan yet, that's expensive, just sample for collision checking
	collisionPenalty, newlyCovered, e.End.State.Time = getSamples(e.DPath, e.Start.State.Time, e.Start.Uncovered)
	// update the Uncovered path in e.End
	e.End.Uncovered = e.Start.Uncovered
	for _, c := range newlyCovered {
		// maybe make this more efficient... it shouldn't happen that much though
		e.End.Uncovered = e.End.Uncovered.Without(c)
	}
	timeFromStart := e.End.State.Time - Start.Time
	timeFromStart = 0 // remove for greediness
	// update e's true Cost
	e.trueCost = e.netTime()*TimePenalty + collisionPenalty - float64(len(newlyCovered))*(CoveragePenalty-timeFromStart)

	// update e.End's current Cost
	// Not doing this anymore. Could be a mistake who knows
	// e.End.CurrentCost = e.Start.CurrentCost + e.trueCost
	// e.End.CurrentCostIsSet = true

	return e.trueCost
}

func (e *Edge) Smooth() {

	// if e is the Start (or somehow there's a cycle...)
	if e.Start.ParentEdge == e {
		return
	}
	parentCost := e.Start.ParentEdge.TrueCost() // should be up to date in A*, check for BIT*
	currentCost := e.TrueCost()
	smoothedEdge := &Edge{Start: e.Start.ParentEdge.Start, End: e.End}
	smoothedEdge.UpdateTrueCost()
	if smoothedEdge.TrueCost() < parentCost+currentCost {
		// PrintLog(fmt.Sprintf("Smoothing edge %v to %v", *e, *smoothedEdge))
		*e = *smoothedEdge
		e.End.ParentEdge = smoothedEdge

		e.End.CurrentCost = e.Start.CurrentCost + e.trueCost
		e.End.CurrentCostIsSet = true

		e.Smooth()
	}
}

func (e Edge) netTime() float64 {
	if e.End.State.Time < e.Start.State.Time {
		PrintError(fmt.Sprintf("Found backwards edge: %s to %s", e.Start.State.String(), e.End.State.String()))
	}
	return e.End.State.Time - e.Start.State.Time
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
		return e.End != v
	})
}

//endregion
