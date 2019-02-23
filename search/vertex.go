package search

import (
	"fmt"
	"github.com/afb2001/CCOM_planner/common"
	"github.com/afb2001/CCOM_planner/dubins"
	. "github.com/afb2001/CCOM_planner/globals"
	"math"
)

//region Vertex

type Vertex struct {
	State            *common.State
	approxCost       float64
	CurrentCost      float64
	CurrentCostIsSet bool
	approxToGo       float64
	ParentEdge       *Edge
	Uncovered        common.Path
}

// accessor methods that should handle caching and stuff
func (v *Vertex) ApproxCost() float64 {
	if v.approxCost == 0 {
		dPath, err := shortestPath(&Start, v.State)
		if err != dubins.EDUBOK {
			v.approxCost = math.MaxFloat64
		} else {
			v.approxCost = dPath.Length() / MaxSpeed * TimePenalty
		}
	}
	return v.approxCost
}

func (v *Vertex) GetCurrentCost() float64 {
	// updated in Edge.UpdateTrueCost()
	if v.CurrentCostIsSet {
		return v.CurrentCost
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
		if v.ParentEdge == nil {
			// TODO! -- !!
			// assume we're a sample... hopefully we are
			parent = &Vertex{State: &Start} // this is correct, right?
		} else {
			parent = v.ParentEdge.Start
		}
	}
	// max euclidean distance to an Uncovered point - coverage penalty for covering all of them
	// This is actually accurate if they're all in a straight line from your current heading,
	// which is not a super unlikely scenario, making this heuristic not as horrible as it may seem.
	// The parent's Uncovered path is used because we probably don't know ours yet,
	// and if we do it could be wrong.
	//approxToGo := parent.Uncovered.MaxDistanceFrom(*v.State)/maxSpeed*timePenalty -
	//	float64(len(parent.Uncovered))*coveragePenalty

	// switching to TSP heuristic (inadmissible)
	approxToGo := Solver.Solve(v.State.X, v.State.Y, parent.Uncovered)/MaxSpeed*TimePenalty -
		float64(len(parent.Uncovered))*CoveragePenalty

	// if we're updating without specifying a parent we can cache it, but not otherwise
	if parentNil {
		v.approxToGo = approxToGo
	}
	return approxToGo
}

// This is really f_hat, which is g_hat + h_hat
func (v *Vertex) FValue() float64 {
	return v.ApproxCost() + v.UpdateApproxToGo(nil)
}

/**
Create a string suitable for debug visualization
*/
func (v *Vertex) String() string {
	return fmt.Sprintf("%s g = %f h = %f", v.State.String(), v.GetCurrentCost(), v.ApproxToGo())
}

//endregion
