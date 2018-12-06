package tsp

import (
	"bufio"
	"github.com/afb2001/CCOM_planner/parse"
	"strings"
	"testing"
)

func TestSolver_Solve(t *testing.T) {
	t.Log("Testing greedy TSP solver")
	path := parse.ReadPath(bufio.NewReader(strings.NewReader(`path to cover 5
1 0
1 2
4 2
4 6
9 6
`)), 0)
	solver := NewSolver(*path)
	if d := solver.Solve(0, 0, *path); d != 15 {
		t.Errorf("Expected %f, got %f", float64(15), d)
	}
	if d := solver.Solve(1, 0, *path); d != 14 {
		t.Errorf("Expected %f, got %f", float64(14), d)
	}
}
