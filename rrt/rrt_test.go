package rrt

import (
	"fmt"
	"github.com/afb2001/CCOM_planner/common"
	. "github.com/afb2001/CCOM_planner/util"
	"math/rand"
	"testing"
	"time"
)

var g common.Grid = common.NewGrid(3, 3)
var p1, p2 = common.State{X: 0.5, Y: 1.5}, common.State{X: 0.5, Y: 2.5}
var p = common.Path{p1, p2}

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

func TestRRT(t *testing.T) {
	//t.SkipNow()
	t.Log("Testing RRT on a larger world")
	// redo setup
	var p = bigPath()
	g := bigGrid()
	o1 := new(common.Obstacles)
	SetBoatConstants(2.5, 0.75)
	plan := MakePlan(&g, &common.State{X: 95, Y: 5, Heading: -1.5, Speed: 1}, p, o1, 0.09)
	fmt.Println(plan.String())
	if len(plan.States) == 1 {
		t.Errorf("Plan was only length 1")
	}
}

func TestRRT2(t *testing.T) {
	t.Log("Testing RRT a bunch of times in a row from random states")
	rand.Seed(time.Now().UnixNano())
	// redo setup
	var p = bigPath()
	g := bigGrid()
	bounds := common.Grid{Width: 100, Height: 20}
	SetBoatConstants(2.5, 0.75)
	o1 := new(common.Obstacles)
	for i := 0; i < 60; i++ {
		s := randomState(&bounds)
		s.Time = float64(i)
		plan := MakePlan(&g, s, p, o1, 0.09)
		if plan == nil || plan.States == nil {
			if !g.IsBlocked(s.X, s.Y) {
				t.Errorf("Empty plan for unblocked state")
				break
			} else {
				PrintLog("Sampled blocked state " + s.String())
			}
		} else {
			PrintLog(plan.String())
		}
	}
}
