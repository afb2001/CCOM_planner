package search

import (
	"github.com/afb2001/CCOM_planner/common"
	"github.com/afb2001/CCOM_planner/dubins"
	. "github.com/afb2001/CCOM_planner/globals"
	. "github.com/afb2001/CCOM_planner/util"
	"math"
)

//region Dubins integration

/**
Find the shortest Dubins path between two states.
*/
func shortestPath(s1 *common.State, s2 *common.State) (path *dubins.Path, err int) {
	// if Verbose {
	// 	PrintLog(fmt.Sprintf("Computing dubins path between %s, %s", s1.String(), s2.String()))
	// }
	path = new(dubins.Path)
	err = dubins.ShortestPath(path, s1.ToArrayPointer(), s2.ToArrayPointer(), MaxTurningRadius)
	return path, err
}

/**
Convert the given path into a plan and compute the sum collision Cost and the newly covered path.
*/
func getSamplesAndPlan(path *dubins.Path, startTime float64, toCover common.Path) (plan *common.Plan, penalty float64, newlyCovered common.Path, finalTime float64) {
	plan = new(common.Plan)
	plan.Start.Time = startTime
	t := startTime // unused?
	callback := func(q *[3]float64, inc float64) int {
		t = inc/MaxSpeed + startTime
		s := &common.State{X: q[0], Y: q[1], Heading: q[2], Speed: MaxSpeed, Time: t}
		s.CollisionProbability = Obst.CollisionExists(s)
		if Grid.IsBlocked(s.X, s.Y) {
			penalty += CollisionPenalty
		} else if s.CollisionProbability > 0 {
			penalty += CollisionPenalty * s.CollisionProbability
		}
		// PrintDebug(s.String(), "Cost =", penalty, "color = 1 shape = dot")
		newlyCovered = append(newlyCovered, toCover.NewlyCovered(*s)...) // splash operator I guess
		plan.AppendState(s)
		return 0
	}
	err := path.SampleMany(DubinsInc, callback)

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

func GetPlan(edge *Edge) (plan *common.Plan) {
	plan = new(common.Plan)
	plan.Start.Time = edge.Start.State.Time
	var t float64
	callback := func(q *[3]float64, inc float64) int {
		t = inc/MaxSpeed + plan.Start.Time
		s := &common.State{X: q[0], Y: q[1], Heading: q[2], Speed: MaxSpeed, Time: t}

		// this is for debugging and can be removed in production
		s.CollisionProbability = Obst.CollisionExistsWithArray(*q, t)
		if Grid.IsBlocked(q[0], q[1]) {
			s.CollisionProbability = 1
		}

		plan.AppendState(s)
		return 0
	}
	err := edge.DPath.SampleMany(DubinsInc, callback)

	if err != dubins.EDUBOK {
		return nil
	}
	return
}

func getSamples(path *dubins.Path, startTime float64, toCover common.Path) (penalty float64, newlyCovered common.Path, finalTime float64) {
	t := startTime // unused?
	callback := func(q *[3]float64, inc float64) int {
		t = inc/MaxSpeed + startTime
		collisionProbability := Obst.CollisionExistsWithArray(*q, t)
		if Grid.IsBlocked(q[0], q[1]) {
			penalty += CollisionPenalty
		} else if collisionProbability > 0 {
			penalty += CollisionPenalty * collisionProbability
		}
		newlyCovered = append(newlyCovered, toCover.NewlyCoveredArray(*q)...) // splash operator I guess

		if DebugToFile {
			PrintTrajectoryState(q[0], q[1], q[2], (inc/MaxSpeed*TimePenalty)+penalty)
		}

		return 0
	}
	err := path.SampleMany(DubinsInc, callback)

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
