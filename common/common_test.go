package common

import (
	"math"
	"reflect"
	"testing"
)

//region State

func TestState_TimeUntil(t *testing.T) {
	t.Log("Testing state time until...")
	s1 := &State{X: 0, Y: 0, Heading: 0, Speed: 0, Time: 0}
	s2 := &State{X: 0.5, Y: 0.5, Heading: 0, Speed: 0, Time: 12.46}
	if time := s1.TimeUntil(s2); time != 12.46 {
		t.Errorf("Expected 12.46, got %f", time)
	}
}

func TestState_DistanceTo(t *testing.T) {
	t.Log("Testing state distance...")
	s1 := &State{X: 0, Y: 0, Heading: 0, Speed: 0, Time: 0}
	s2 := &State{X: 3, Y: 4, Heading: 0, Speed: 0, Time: 0}
	if s1.DistanceTo(s2) != 5 {
		t.Errorf("Expected 5, got %f", s1.DistanceTo(s2))
	}
}

func TestState_HeadingTo(t *testing.T) {
	t.Log("Testing state heading to...")
	s1 := &State{X: 0, Y: 0, Heading: 0, Speed: 0, Time: 0}
	s2 := &State{X: 0, Y: 4, Heading: 0, Speed: 0, Time: 0}
	if s1.HeadingTo(s2) != math.Pi/2 {
		t.Errorf("Expected %f, got %f", math.Pi/2, s1.HeadingTo(s2))
	}
}

func TestState_Collides(t *testing.T) {
	t.Log("Testing state collisions...")
	s1 := &State{X: 0, Y: 0, Heading: 0, Speed: 0, Time: 0}
	s2 := &State{X: 0.5, Y: 0.5, Heading: 0, Speed: 0, Time: 0}
	if !s1.Collides(s2) {
		t.Error("Expected collision, got no collision instead")
	}
	s2.X = 2
	if s1.Collides(s2) {
		t.Error("Expected no collision, got collision instead")
	}
}

func TestState_Project(t *testing.T) {
	t.Log("Testing state projecting...")
	s1 := &State{X: 0, Y: 0, Heading: 1, Speed: 2.0, Time: 0}
	s2 := s1.Project(1.0)
	if d := s1.DistanceTo(s2); d != 2 {
		t.Errorf("Expected distance of 2, got %f", d)
	}
}

//endregion

//region Path

func TestPath_Remove(t *testing.T) {
	t.Log("Testing path remove")
	s1 := State{X: 0, Y: 0, Heading: 0, Speed: 0, Time: 0}
	s2 := State{X: 0.5, Y: 0.5, Heading: 0, Speed: 0, Time: 12.46}
	p := &Path{s1, s2}
	p = p.Without(s1)
	if !reflect.DeepEqual(*p, Path{s2}) {
		t.Errorf("Removal failed: expected %v, got %v", Path{s2}, *p)
	}
}

func TestPath_MaxDistanceFrom(t *testing.T) {
	t.Log("Testing path MaxDistanceFrom")
	s1 := State{X: 0, Y: 0, Heading: 0, Speed: 0, Time: 0}
	s2 := State{X: 0.5, Y: 0.5, Heading: 0, Speed: 0, Time: 12.46}
	s3 := State{X: 3, Y: 4, Time: 12}
	p := &Path{s1, s2, s3}
	if p.MaxDistanceFrom(s3) != 5 {
		t.Errorf("Expected 5, got %f", p.MaxDistanceFrom(s3))
	}
}

func TestPath_NewlyCovered(t *testing.T) {
	t.Log("Testing path newly covered")
	s1 := State{X: 0, Y: 0, Heading: 0, Speed: 0, Time: 0}
	s2 := State{X: 0.5, Y: 0.5, Heading: 0, Speed: 0, Time: 12.46}
	s3 := State{X: 3, Y: 4, Time: 12}
	p := &Path{s1, s2, s3}
	if n := p.NewlyCovered(s2); !reflect.DeepEqual(n, Path{s1, s2}) {
		t.Errorf("Expected %v, got %v", Path{s1, s2}, n)
	}
}

//endregion

func TestPlan_AppendState(t *testing.T) {
	t.Log("Testing plan append state (basic)")
	s1 := &State{X: 0, Y: 0, Heading: 0, Speed: 0, Time: 0}
	p := new(Plan)
	p.States = make([]*State, 0)
	p.AppendState(s1)
	if !reflect.DeepEqual(p.States, []*State{s1}) {
		t.Errorf("Expected %v, got %v", []*State{s1}, p.States)
	}
}

func TestPlan_AppendState2(t *testing.T) {
	t.Log("Testing plan append state (not adding too close a state)")
	s1 := &State{X: 0, Y: 0, Heading: 0, Speed: 0, Time: 0}
	s2 := &State{X: 0.5, Y: 0.5, Heading: 0, Speed: 0, Time: 0.25}
	s3 := &State{X: 3, Y: 4, Time: 12}
	p := new(Plan)
	p.States = make([]*State, 0)
	p.AppendState(s1)
	p.AppendState(s2)
	p.AppendState(s3)
	if !reflect.DeepEqual(p.States, []*State{s1, s3}) {
		t.Errorf("Expected %v, got %v", []*State{s1, s3}, p.States)
	}
}
