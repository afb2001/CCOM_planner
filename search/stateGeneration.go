package search

import (
	"github.com/afb2001/CCOM_planner/common"
	. "github.com/afb2001/CCOM_planner/globals"
	"math"
	"math/rand"
)

//region State generation

func chance(probability float64) bool {
	return rand.Float64() < probability
}

/**
Create a new State with random values.
Time is unset (zero).
*/
func RandomState(xMin, xMax, yMin, yMax float64) *common.State {
	s := new(common.State)
	s.X = rand.Float64()*float64(xMax-xMin) + xMin
	s.Y = rand.Float64()*float64(yMax-yMin) + yMin
	s.Heading = rand.Float64() * math.Pi * 2
	s.Speed = rand.Float64() * MaxSpeed
	return s
}

/**
Create a random sample using the biasing constants.
*/
func biasedRandomState(xMin, xMax, yMin, yMax float64) *common.State {
	s := RandomState(xMin, xMax, yMin, yMax)
	if r := rand.Float64(); r < MaxSpeedBias {
		s.Speed = MaxSpeed
	}
	return s
}

// unused
func b(bounds common.Grid, point common.State, distance float64) *common.State {
	s := RandomState(math.Max(0, point.X-distance),
		math.Min(float64(bounds.Width), point.X+distance),
		math.Max(0, point.Y-distance),
		math.Min(float64(bounds.Height), point.Y+distance))
	if r := rand.Float64(); r < MaxSpeedBias {
		s.Speed = MaxSpeed
	}
	if r := rand.Float64(); r < GoalBias {
		return &point
	}
	return s
}

/**
Sample a State whose euclidean distance to Start is less than the given distance bound.
*/
func BoundedBiasedRandomState(bounds *common.Grid, path common.Path, start *common.State, cost float64) *common.State {
	distance := cost * MaxSpeed
	horizon := (common.TimeHorizon + 1) * MaxSpeed
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
	if chance(GoalBias) {
		s.X, s.Y = point.X, point.Y
	}
	return s
}

//endregion
