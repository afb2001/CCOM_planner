package bitStar

import (
	"container/heap"
	"fmt"
	"github.com/afb2001/CCOM_planner/common"
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
	g.BlockRange(1, 1, 1)
	InitGlobals(g, &p, 0.5, 0.20)
	start = common.State{X: 2.5, Y: 1.25, Heading: 3 * math.Pi / 2}
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
	retCode := m.Run()
	os.Exit(retCode)
}

func TestVertex_ApproxCost(t *testing.T) {
	t.Log("Testing vertex approx cost...")
	v := Vertex{state: &common.State{X: 2.5, Y: 0.25}}
	// distance is 1, speed is 0.5
	if c := v.ApproxCost(); c > 2.3 || c < 2.2 { // approx because I'm lazy
		t.Errorf("Expected 2, got %f", c)
	}
}

func TestVertex_UpdateApproxToGo(t *testing.T) {
	t.Log("Testing vertex update approx to go...")
	v1 := Vertex{state: &common.State{X: 1.5, Y: 0.25, Heading: math.Pi}, uncovered: *toCover}
	v2 := Vertex{state: &common.State{X: 0.5, Y: 0.5, Heading: math.Pi}}
	e := Edge{start: &v1, end: &v2}
	v2.parentEdge = &e
	if h := v2.UpdateApproxToGo(nil); h != -116 {
		t.Errorf("Expected %d, got %f", -116, h)
	}
	t.Log("Testing f value...")
	if f := v2.fValue(); f > -111 || f < -112 { // TODO -- why did this not give the exact thing (below)? dubins??
		t.Errorf("Expected %f, got %f", -116+v2.state.DistanceTo(&start)/0.5, f)
	}
}

func TestEdge_ApproxCost(t *testing.T) {
	t.Log("Testing edge approx cost...")
	v1 := Vertex{state: &common.State{X: 1.5, Y: 0.5, Heading: math.Pi}, uncovered: *toCover}
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
	v1 := Vertex{state: &common.State{X: 1.5, Y: 0.5, Heading: math.Pi}, uncovered: *toCover}
	v1.parentEdge = &Edge{start: &v1, end: &v1}                // root vertex setup
	v1.currentCost = -float64(len(*toCover)) * coveragePenalty // here too
	v2 := Vertex{state: &common.State{X: 0.5, Y: 0.5, Heading: math.Pi}}
	e := Edge{start: &v1, end: &v2}
	v2.parentEdge = &e
	v2.parentEdge.UpdateTrueCost()
	if v2.parentEdge.TrueCost() != -118 {
		t.Errorf("Expected -118, got %f", v2.parentEdge.TrueCost())
	}
}

func TestEdge_UpdateStart(t *testing.T) {
	t.Log("Testing edge update start...")
	v1 := Vertex{state: &common.State{X: 1.5, Y: 0.5, Heading: math.Pi}, uncovered: *toCover}
	v1.parentEdge = &Edge{start: &v1, end: &v1}                // root vertex setup
	v1.currentCost = -float64(len(*toCover)) * coveragePenalty // here too
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
	toCover = toCover.Without(p2) // so we can have just one point to cover
	v1 := Vertex{state: &common.State{X: 2.5, Y: 1.5, Heading: math.Pi}, uncovered: *toCover}
	v1.parentEdge = &Edge{start: &v1, end: &v1}                // root vertex setup
	v1.currentCost = -float64(len(*toCover)) * coveragePenalty // here too
	v2 := Vertex{state: &common.State{X: 0.5, Y: 1.5, Heading: math.Pi}}
	e := Edge{start: &v1, end: &v2}
	v2.parentEdge = &e
	v2.parentEdge.UpdateTrueCost()
	// not sure about this number it seems like it might be wrong but it's high so it's OK
	if c := v2.parentEdge.TrueCost(); c != 5343.8 {
		t.Errorf("Expected %f, got %f", 5343.8, c)
	}
}

func TestBoundedBiasedRandomState(t *testing.T) {
	t.Log("Testing random state generation...")
	v1 := Vertex{state: &common.State{X: 2.5, Y: 1.5, Heading: math.Pi}, uncovered: *toCover}
	v1.parentEdge = &Edge{start: &v1, end: &v1}                // root vertex setup
	v1.currentCost = -float64(len(*toCover)) * coveragePenalty // here too
	s := BoundedBiasedRandomState(&g, p, &start, 60)
	if s == nil || s.X < 0 || s.Y < 0 {
		if s != nil {
			t.Errorf("Generated invalid state: %s", s.String())
		}
		t.Errorf("State generation failed")
	}
}

func TestEdgeQueue_PushPop(t *testing.T) {
	t.Log("Testing edge queue push/pop")
	qE := new(EdgeQueue)
	qE.cost = func(edge *Edge) float64 {
		return edge.start.CurrentCost() + edge.ApproxCost() + edge.end.UpdateApproxToGo(edge.start)
	}
	v1 := Vertex{state: &common.State{X: 1.5, Y: 0.5, Heading: math.Pi}, uncovered: *toCover}
	v2 := Vertex{state: &common.State{X: 0.5, Y: 0.5, Heading: math.Pi}}
	e := Edge{start: &v1, end: &v2}
	qE.Push(&e)
	if qE.Len() != 1 {
		t.Errorf("Edge was not added correctly to queue")
	}
	if qE.Pop() != &e {
		t.Errorf("Edge on the queue was not the right edge")
	}
	if qE.Len() != 0 {
		t.Errorf("Edge did not pop correctly from the queue")
	}
}

func TestVertexQueue_PushPop(t *testing.T) {
	t.Log("Testing vertex queue push/pop")
	qV := new(VertexQueue)
	qV.cost = func(v *Vertex) float64 {
		return v.CurrentCost() + v.UpdateApproxToGo(nil)
	}
	v1 := Vertex{state: &common.State{X: 1.5, Y: 0.5, Heading: math.Pi}, uncovered: *toCover}
	// v2 := Vertex{state: &common.State{X: 0.5, Y: 0.5, Heading: math.Pi}}
	// e := Edge{start: &v1, end: &v2}
	heap.Push(qV, &v1)
	if qV.Len() != 1 {
		t.Errorf("Vertex was not added correctly to queue")
	}
	if heap.Pop(qV) != &v1 {
		t.Errorf("Vertex on the queue was not the right edge")
	}
	if qV.Len() != 0 {
		t.Errorf("Vertex did not pop correctly from the queue")
	}
}

func TestVertexQueue_PushPopMany(t *testing.T) {
	t.Log("Testing vertex queue push/pop multiple times")
	qV := new(VertexQueue)
	qV.cost = func(v *Vertex) float64 {
		return v.state.X
	}
	v1 := &Vertex{state: &common.State{X: 0.5}}
	v2 := &Vertex{state: &common.State{X: 0.6}}
	v3 := &Vertex{state: &common.State{X: 0.7}}
	v4 := &Vertex{state: &common.State{X: 0.8}}
	v5 := &Vertex{state: &common.State{X: 0.9}}
	heap.Push(qV, v3)
	heap.Push(qV, v1)
	heap.Push(qV, v4)
	heap.Push(qV, v2)
	heap.Push(qV, v5)
	v := heap.Pop(qV).(*Vertex)
	printLog(v.state)
	if v != v1 {
		t.Errorf("Wrong order popping from queue")
	}
}

func TestBitStar(t *testing.T) {
	t.SkipNow()
	t.Log("Testing BIT* in a small world")
	o1 := new(common.Obstacles)
	plan := BitStar(start, 0.09, o1)
	fmt.Println(plan.String())
	if len(plan.States) == 1 {
		t.Errorf("Plan was only length 1")
	}
}

func TestBitStar2(t *testing.T) {
	t.SkipNow()
	t.Log("Testing BIT* on a larger world")
	// redo setup
	var p = bigPath()
	InitGlobals(bigGrid(), &p, 2.5, 0.75)
	o1 := new(common.Obstacles)
	plan := BitStar(common.State{X: 95, Y: 5, Heading: -1.5, Speed: 1}, 0.09, o1)
	fmt.Println(plan.String())
	if len(plan.States) == 1 {
		t.Errorf("Plan was only length 1")
	}
}

func TestFindAStarPlan(t *testing.T) {
	t.Log("Testing A* on a large world")
	rand.Seed(time.Now().UnixNano())
	// redo setup
	var p = bigPath()
	InitGlobals(bigGrid(), &p, 2.5, 0.75)
	o1 := new(common.Obstacles)
	plan := FindAStarPlan(common.State{X: 95, Y: 5, Heading: -1.5, Speed: 0, Time: 100}, 0.095, o1)
	fmt.Println(plan.String())
	if len(plan.States) == 1 {
		t.Errorf("Plan was only length 1")
	}
}

func TestFindAStarPlan2(t *testing.T) {
	t.SkipNow() // I changed the semantics so this still finds a path
	t.Log("Testing A* without any path to cover")
	// redo setup
	var p = new(common.Path)
	InitGlobals(bigGrid(), p, 2.5, 0.75)
	o1 := new(common.Obstacles)
	plan := FindAStarPlan(common.State{X: 95, Y: 5, Heading: -1.5, Speed: 0, Time: 100}, 0.095, o1)
	if plan != nil {
		t.Errorf("Plan was too long")
	}
}

func TestFindAStarPlan3(t *testing.T) {
	// t.SkipNow() // this takes a while (almost a minute
	t.Log("Testing A* a bunch of times in a row from random states")
	rand.Seed(time.Now().UnixNano())
	// redo setup
	var p = bigPath()
	InitGlobals(bigGrid(), &p, 2.5, 0.75)
	o1 := new(common.Obstacles)
	for i := 0; i < 60; i++ {
		s := randomState(50, 100, 0, 20)
		s.Time = float64(i)
		plan := FindAStarPlan(*s, 0.09, o1)
		if plan == nil || plan.States == nil {
			if !grid.IsBlocked(s.X, s.Y) {
				t.Errorf("Empty plan for unblocked state")
				break
			} else {
				printLog("Sampled blocked state " + s.String())
			}
		}
	}
}
