package rhrsaStar

import (
	"fmt"
	"github.com/afb2001/CCOM_planner/common"
	. "github.com/afb2001/CCOM_planner/globals"
	. "github.com/afb2001/CCOM_planner/search"
	"github.com/afb2001/CCOM_planner/tsp"
	. "github.com/afb2001/CCOM_planner/util"
	"math"
	"math/rand"
	"os"
	"testing"
	"time"
)

var g common.Grid = common.NewGrid(3, 3)
var p1, p2 = common.State{X: 0.5, Y: 1.5}, common.State{X: 0.5, Y: 2.5}
var p = common.Path{p1, p2}

func setUp() {
	DebugVis = false
	g.BlockRange(1, 1, 1)
	solver := tsp.NewSolver(p)
	InitGlobals(g, 0.5, 0.20, solver)
	Start = common.State{X: 2.5, Y: 1.25, Heading: 3 * math.Pi / 2}
}

func bigGrid() common.Grid {
	g1 := common.NewGrid(100, 100)
	g1.BlockRange(10, 20, 10)
	g1.BlockRange(20, 20, 10)
	g1.BlockRange(30, 20, 10)
	g1.BlockRange(40, 20, 10)
	g1.BlockRange(50, 20, 10)
	g1.BlockRange(60, 20, 10)
	g1.BlockRange(70, 20, 10)
	g1.BlockRange(80, 20, 10)
	g1.BlockRange(90, 20, 10)
	g1.BlockRange(0, 40, 10)
	g1.BlockRange(10, 40, 10)
	g1.BlockRange(20, 40, 10)
	g1.BlockRange(30, 40, 10)
	g1.BlockRange(40, 40, 10)
	g1.BlockRange(50, 40, 10)
	g1.BlockRange(60, 40, 10)
	g1.BlockRange(70, 40, 10)
	g1.BlockRange(80, 40, 10)
	g1.BlockRange(10, 60, 10)
	g1.BlockRange(20, 60, 10)
	g1.BlockRange(30, 60, 10)
	g1.BlockRange(40, 60, 10)
	g1.BlockRange(50, 60, 10)
	g1.BlockRange(60, 60, 10)
	g1.BlockRange(70, 60, 10)
	g1.BlockRange(80, 60, 10)
	g1.BlockRange(90, 60, 10)
	g1.BlockRange(0, 80, 10)
	g1.BlockRange(10, 80, 10)
	g1.BlockRange(20, 80, 10)
	g1.BlockRange(30, 80, 10)
	g1.BlockRange(40, 80, 10)
	g1.BlockRange(50, 80, 10)
	g1.BlockRange(60, 80, 10)
	g1.BlockRange(70, 80, 10)
	g1.BlockRange(80, 80, 10)
	return g1
}

func bigPath() common.Path {
	path1 := common.Path{
		common.State{X: 90, Y: 6},
		common.State{X: 85, Y: 7},
		common.State{X: 80, Y: 8},
		common.State{X: 75, Y: 9},
		common.State{X: 70, Y: 10},
		common.State{X: 65, Y: 11},
		common.State{X: 60, Y: 12},
		common.State{X: 55, Y: 13},
		common.State{X: 50, Y: 14},
		common.State{X: 45, Y: 15},
	}
	return path1
}

func TestMain(m *testing.M) {
	setUp()
	SetupDebugWriter()
	defer CleanupDebugWriter()
	retCode := m.Run()
	os.Exit(retCode)
}

func TestFindAStarPlan(t *testing.T) {
	t.Log("Testing A* on a large world")
	seed := time.Now().UnixNano()
	rand.Seed(1551724267362385641)
	PrintLog("Seed:", seed)
	// redo setup
	var p = bigPath()
	solver := tsp.NewSolver(p)
	InitGlobals(bigGrid(), 2.5, 0.75, solver)
	o1 := new(common.Obstacles)
	plan := FindAStarPlan(common.State{X: 95, Y: 5, Heading: 4.75, Speed: 0, Time: 100}, &p, 0.095, *o1)
	if plan == nil {
		t.Error("Could not find a plan. There definitely should be one.")
		return
	}
	fmt.Println(plan.String())
	if len(plan.States) == 1 {
		t.Errorf("Plan was only length 1")
	}
	for j, s := range plan.States {
		if j < len(plan.States)-1 && s.DistanceTo(plan.States[j+1]) > 3 {
			t.Error("Plan has states too far away")
			return
		}
	}
}

func TestFindAStarPlan2(t *testing.T) {
	t.SkipNow() // I changed the semantics so this still finds a path
	t.Log("Testing A* without any path to cover")
	// redo setup
	var p = new(common.Path)
	solver := tsp.NewSolver(*p)
	InitGlobals(bigGrid(), 2.5, 0.75, solver)
	o1 := new(common.Obstacles)
	plan := FindAStarPlan(common.State{X: 95, Y: 5, Heading: -1.5, Speed: 0, Time: 100}, p, 10e100, *o1)
	if plan != nil {
		t.Errorf("Plan was too long")
	}
}

func TestFindAStarPlan3(t *testing.T) {
	// t.SkipNow() // this takes a while
	t.Log("Testing A* a bunch of times in a row from random states")
	seed := time.Now().UnixNano()
	rand.Seed(1551724267362385641)
	PrintLog("Seed:", seed)
	// redo setup
	var p = bigPath()
	solver := tsp.NewSolver(p)
	InitGlobals(bigGrid(), 2.5, 0.75, solver)
	o1 := new(common.Obstacles)
	for i := 0; i < 10; i++ {
		s := RandomState(50, 100, 0, 20)
		s.Time = float64(i)
		plan := FindAStarPlan(*s, &p, 0.09, *o1)
		if plan == nil || plan.States == nil {
			if !Grid.IsBlocked(s.X, s.Y) {
				t.Errorf("Empty Plan for unblocked State")
				break
			} else {
				PrintLog("Sampled blocked State " + s.String())
			}
		}
		for j, s := range plan.States {
			if j < len(plan.States)-1 && s.DistanceTo(plan.States[j+1]) > 3 {
				t.Error("Plan has states too far away")
				return
			}
		}
	}
}

func TestPointToPointPlan(t *testing.T) {
	t.Log("Testing the Point to Point planner a bunch of times in a row from random states")
	rand.Seed(time.Now().UnixNano())
	// redo setup
	var p = bigPath()
	solver := tsp.NewSolver(p)
	InitGlobals(bigGrid(), 2.5, 0.75, solver)
	o1 := new(common.Obstacles)
	for i := 0; i < 60; i++ {
		s := RandomState(50, 100, 0, 20)
		s.Time = float64(i)
		plan := PointToPointPlan(*s, &p, 0.09, *o1)
		if plan == nil || plan.States == nil {
			if !Grid.IsBlocked(s.X, s.Y) {
				t.Errorf("Empty Plan for unblocked State")
				break
			} else {
				PrintLog("Sampled blocked State " + s.String())
			}
		}
	}
}
