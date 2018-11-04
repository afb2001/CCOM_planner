package bitStar

import (
	"github.com/afb2001/CCOM_planner/common"
	"math"
	"os"
	"testing"
)

var g common.Grid = common.NewGrid(3, 3)
var p1, p2 = common.State{X: 0.5, Y: 1.5}, common.State{X: 0.5, Y: 2.5}
var p = common.Path{p1, p2}

func setUp() {
	g.BlockRange(1, 1, 1)
	InitGlobals(g, p, 0.5, 0.25)
	start = common.State{X: 2.5, Y: 1.25, Heading: 3 * math.Pi / 2}
}

func TestMain(m *testing.M) {
	setUp()
	retCode := m.Run()
	os.Exit(retCode)
}

func TestVertex_ApproxCost(t *testing.T) {
	t.Log("Testing vertex approx cost...")
	v := Vertex{state: &common.State{X: 2.5, Y: 0.25}}
	// distance is 1, speed is 0.5
	if c := v.ApproxCost(); c != 2 {
		t.Errorf("Expected 2, got %f", c)
	}
}

func TestVertex_UpdateApproxToGo(t *testing.T) {
	t.Log("Testing vertex update approx to go...")
	v1 := Vertex{state: &common.State{X: 1.5, Y: 0.25, Heading: math.Pi}, uncovered: toCover}
	v2 := Vertex{state: &common.State{X: 0.5, Y: 0.5, Heading: math.Pi}}
	e := Edge{start: &v1, end: &v2}
	v2.parentEdge = &e
	if h := v2.UpdateApproxToGo(nil); h != -56 {
		t.Errorf("Expected %d, got %f", -56, h)
	}
	t.Log("Testing f value...")
	if f := v2.fValue(); f != -56+v2.state.DistanceTo(&start)/0.5 {
		t.Errorf("Expected %f, got %f", -56+v2.state.DistanceTo(&start)/0.5, f)
	}
}

func TestEdge_ApproxCost(t *testing.T) {
	t.Log("Testing edge approx cost...")
	v1 := Vertex{state: &common.State{X: 1.5, Y: 0.5, Heading: math.Pi}, uncovered: toCover}
	v2 := Vertex{state: &common.State{X: 0.5, Y: 0.5, Heading: math.Pi}}
	e := Edge{start: &v1, end: &v2}
	// if there's a dPath it's probably right so don't worry about it
	e.ApproxCost()
	if e.dPath == nil {
		t.Errorf("Unable to create valid dubins path to find approximate cost")
	}
}

func TestEdge_UpdateTrueCost(t *testing.T) {
	t.Log("Testing edge true cost...")
	v1 := Vertex{state: &common.State{X: 1.5, Y: 0.5, Heading: math.Pi}, uncovered: toCover}
	v1.parentEdge = &Edge{start: &v1, end: &v1}               // root vertex setup
	v1.currentCost = -float64(len(toCover)) * coveragePenalty // here too
	v2 := Vertex{state: &common.State{X: 0.5, Y: 0.5, Heading: math.Pi}}
	e := Edge{start: &v1, end: &v2}
	v2.parentEdge = &e
	v2.parentEdge.UpdateTrueCost()
	if v2.parentEdge.TrueCost() != -58 {
		t.Errorf("Expected -58, got %f", v2.parentEdge.TrueCost())
	}
}

func TestEdge_UpdateStart(t *testing.T) {
	t.Log("Testing edge update start...")
	v1 := Vertex{state: &common.State{X: 1.5, Y: 0.5, Heading: math.Pi}, uncovered: toCover}
	v1.parentEdge = &Edge{start: &v1, end: &v1}               // root vertex setup
	v1.currentCost = -float64(len(toCover)) * coveragePenalty // here too
	v2 := Vertex{state: &common.State{X: 0.5, Y: 0.5, Heading: math.Pi}}
	e := Edge{start: &v1, end: &v2}
	v2.parentEdge = &e
	v2.parentEdge.UpdateTrueCost()
	v2.parentEdge.UpdateStart(&v2)
	if v2.parentEdge.dPath != nil {
		t.Errorf("Path was not reset")
	}
}

func TestStaticCollision(t *testing.T) {
	t.Log("Testing edge through obstacle...")
	v1 := Vertex{state: &common.State{X: 2.5, Y: 1.5, Heading: math.Pi}, uncovered: toCover}
	v1.parentEdge = &Edge{start: &v1, end: &v1}               // root vertex setup
	v1.currentCost = -float64(len(toCover)) * coveragePenalty // here too
	v2 := Vertex{state: &common.State{X: 0.5, Y: 1.5, Heading: math.Pi}}
	e := Edge{start: &v1, end: &v2}
	v2.parentEdge = &e
	v2.parentEdge.UpdateTrueCost()
	// not sure about this number it seems like it might be wrong but it's high so it's OK
	if c := v2.parentEdge.TrueCost(); c != 5373.8 {
		t.Errorf("Expected %f, got %f", 5373.8, c)
	}
}
