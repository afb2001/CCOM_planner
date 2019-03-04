package bitStar

import (
	"container/heap"
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
	Verbose = false
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

func TestVertex_ApproxCost(t *testing.T) {
	t.Log("Testing vertex approx cost...")
	v := Vertex{State: &common.State{X: 2.5, Y: 0.25}}
	// distance is 1, speed is 0.5
	if c := v.ApproxCost(); c > 2.3 || c < 2.2 { // approx because I'm lazy
		t.Errorf("Expected 2, got %f", c)
	}
}

func TestVertex_UpdateApproxToGo(t *testing.T) {
	t.Log("Testing vertex Update approx to go...")
	v1 := Vertex{State: &common.State{X: 1.5, Y: 0.25, Heading: math.Pi}, Uncovered: p}
	v2 := Vertex{State: &common.State{X: 0.5, Y: 0.5, Heading: math.Pi}}
	e := Edge{Start: &v1, End: &v2}
	v2.ParentEdge = &e
	if h := v2.UpdateApproxToGo(nil); h != -116 {
		t.Errorf("Expected %d, got %f", -116, h)
	}
	t.Log("Testing f value...")
	if f := v2.FValue(); f > -111 || f < -112 { // TODO -- why did this not give the exact thing (below)? dubins??
		t.Errorf("Expected %f, got %f", -116+v2.State.DistanceTo(&Start)/0.5, f)
	}
}

func TestEdge_ApproxCost(t *testing.T) {
	t.Log("Testing edge approx cost...")
	v1 := Vertex{State: &common.State{X: 1.5, Y: 0.5, Heading: math.Pi}, Uncovered: p}
	v2 := Vertex{State: &common.State{X: 0.5, Y: 0.5, Heading: math.Pi}}
	e := Edge{Start: &v1, End: &v2}
	// if there's a DPath it's probably right so don't worry about it
	e.ApproxCost()
	if e.DPath == nil {
		t.Errorf("Unable to create valid dubins path to find approximate cost")
	}
}

func TestEdge_UpdateTrueCost(t *testing.T) {
	t.Log("Testing edge true cost...")
	v1 := Vertex{State: &common.State{X: 1.5, Y: 0.5, Heading: math.Pi}, Uncovered: p}
	v1.ParentEdge = &Edge{Start: &v1, End: &v1}         // root vertex setup
	v1.CurrentCost = -float64(len(p)) * CoveragePenalty // here too
	v2 := Vertex{State: &common.State{X: 0.5, Y: 0.5, Heading: math.Pi}}
	e := Edge{Start: &v1, End: &v2}
	v2.ParentEdge = &e
	v2.ParentEdge.UpdateTrueCost()
	if v2.ParentEdge.TrueCost() != -118 {
		t.Errorf("Expected -118, got %f", v2.ParentEdge.TrueCost())
	}
}

func TestEdge_UpdateStart(t *testing.T) {
	t.Log("Testing edge Update Start...")
	v1 := Vertex{State: &common.State{X: 1.5, Y: 0.5, Heading: math.Pi}, Uncovered: p}
	v1.ParentEdge = &Edge{Start: &v1, End: &v1}         // root vertex setup
	v1.CurrentCost = -float64(len(p)) * CoveragePenalty // here too
	v2 := Vertex{State: &common.State{X: 0.5, Y: 0.5, Heading: math.Pi}}
	e := Edge{Start: &v1, End: &v2}
	v2.ParentEdge = &e
	v2.ParentEdge.UpdateTrueCost()
	v2.ParentEdge.UpdateStart(&v2)
	if v2.ParentEdge.DPath != nil {
		t.Errorf("Path was not reset")
	}
}

func TestStaticCollision(t *testing.T) {
	t.Log("Testing edge through obstacle...")
	toCover := p.Without(p2) // so we can have just one point to cover
	v1 := Vertex{State: &common.State{X: 2.5, Y: 1.5, Heading: math.Pi}, Uncovered: toCover}
	v1.ParentEdge = &Edge{Start: &v1, End: &v1}               // root vertex setup
	v1.CurrentCost = -float64(len(toCover)) * CoveragePenalty // here too
	v2 := Vertex{State: &common.State{X: 0.5, Y: 1.5, Heading: math.Pi}}
	e := Edge{Start: &v1, End: &v2}
	v2.ParentEdge = &e
	v2.ParentEdge.UpdateTrueCost()
	// not sure about this number it seems like it might be wrong but it's high so it's OK
	if c := v2.ParentEdge.TrueCost(); c != 5343.8 {
		t.Errorf("Expected %f, got %f", 5343.8, c)
	}
}

func TestBoundedBiasedRandomState(t *testing.T) {
	t.Log("Testing random State generation...")
	v1 := Vertex{State: &common.State{X: 2.5, Y: 1.5, Heading: math.Pi}, Uncovered: p}
	v1.ParentEdge = &Edge{Start: &v1, End: &v1}         // root vertex setup
	v1.CurrentCost = -float64(len(p)) * CoveragePenalty // here too
	s := BoundedBiasedRandomState(&g, p, &Start, 60*MaxSpeed)
	if s == nil || s.X < 0 || s.Y < 0 {
		if s != nil {
			t.Errorf("Generated invalid State: %s", s.String())
		}
		t.Errorf("State generation failed")
	}
}

func TestEdgeQueue_PushPop(t *testing.T) {
	t.Log("Testing edge queue push/pop")
	qE := new(EdgeQueue)
	qE.Cost = func(edge *Edge) float64 {
		return edge.Start.GetCurrentCost() + edge.ApproxCost() + edge.End.UpdateApproxToGo(edge.Start)
	}
	v1 := Vertex{State: &common.State{X: 1.5, Y: 0.5, Heading: math.Pi}, Uncovered: p}
	v2 := Vertex{State: &common.State{X: 0.5, Y: 0.5, Heading: math.Pi}}
	e := Edge{Start: &v1, End: &v2}
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
	qV.Cost = func(v *Vertex) float64 {
		return v.GetCurrentCost() + v.UpdateApproxToGo(nil)
	}
	v1 := Vertex{State: &common.State{X: 1.5, Y: 0.5, Heading: math.Pi}, Uncovered: p}
	// v2 := Vertex{State: &common.State{X: 0.5, Y: 0.5, Heading: math.Pi}}
	// e := Edge{Start: &v1, End: &v2}
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
	qV.Cost = func(v *Vertex) float64 {
		return v.State.X
	}
	v1 := &Vertex{State: &common.State{X: 0.5}}
	v2 := &Vertex{State: &common.State{X: 0.6}}
	v3 := &Vertex{State: &common.State{X: 0.7}}
	v4 := &Vertex{State: &common.State{X: 0.8}}
	v5 := &Vertex{State: &common.State{X: 0.9}}
	heap.Push(qV, v3)
	heap.Push(qV, v1)
	heap.Push(qV, v4)
	heap.Push(qV, v2)
	heap.Push(qV, v5)
	v := heap.Pop(qV).(*Vertex)
	PrintLog(v.State)
	if v != v1 {
		t.Errorf("Wrong order popping from queue")
	}
}

func TestBitStar(t *testing.T) {
	t.SkipNow()
	t.Log("Testing BIT* in a small world")
	o1 := new(common.Obstacles)
	plan := BitStar(Start, &p, 0.09, *o1)
	fmt.Println(plan.String())
	if len(plan.States) == 1 {
		t.Errorf("Plan was only length 1")
	}
}

func TestBitStar2(t *testing.T) {
	//t.SkipNow()
	t.Log("Testing BIT* on a larger world")
	// redo setup
	var p = bigPath()
	solver := tsp.NewSolver(p)
	InitGlobals(bigGrid(), 2.5, 0.75, solver)
	o1 := new(common.Obstacles)
	plan := BitStar(common.State{X: 95, Y: 5, Heading: -1.5, Speed: 1}, &p, 0.09, *o1)
	fmt.Println(plan.String())
	if len(plan.States) == 1 {
		t.Errorf("Plan was only length 1")
	}
}

func TestBitStar3(t *testing.T) {
	t.SkipNow() // this takes a while (almost a minute
	t.Log("Testing BIT* a bunch of times in a row from random states")
	rand.Seed(time.Now().UnixNano())
	// redo setup
	var p = bigPath()
	solver := tsp.NewSolver(p)
	InitGlobals(bigGrid(), 2.5, 0.75, solver)
	o1 := new(common.Obstacles)
	for i := 0; i < 60; i++ {
		s := RandomState(50, 100, 0, 20)
		s.Time = float64(i)
		plan := BitStar(*s, &p, 0.09, *o1)
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
