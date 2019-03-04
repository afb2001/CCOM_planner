package globals

import (
	"github.com/afb2001/CCOM_planner/common"
	"github.com/afb2001/CCOM_planner/tsp"
)

const (
	GoalBias       float64 = 0.05
	MaxSpeedBias   float64 = 1.0
	DubinsInc      float64 = 0.1 // this might be low
	K              int     = 3   // number of closest states to consider for BIT*
	BitStarSamples int     = 10  // (m in the paper) -- make this a parameter too
	// BIT* penalties (should all be made into parameters)
	CoveragePenalty     float64 = 10
	CollisionPenalty    float64 = 600
	TimePenalty         float64 = 1
	AggressiveSmoothing bool    = false
	Heuristic                   = "tsp"
	Weight                      = 1.0
)

// make sure to set these before you call BitStar() or FindAStarPlan
var (
	//Verbose          = true
	MaxTurningRadius float64
	MaxSpeed         float64
	// these should be immutable so no pointers necessary
	Start common.State
	Grid  common.Grid
	Obst  common.Obstacles
)

var Solver tsp.Solver

func InitGlobals(g1 common.Grid, speed, radius float64, s tsp.Solver) {
	Grid, MaxSpeed, MaxTurningRadius, Solver = g1, speed, radius, s
}
