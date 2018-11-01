package common

import (
	"math"
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
