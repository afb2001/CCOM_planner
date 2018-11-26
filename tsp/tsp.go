package tsp

import (
	"github.com/afb2001/CCOM_planner/common"
	"math"
	"sort"
)

type Solver struct {
	points    []point
	distances map[point][]pointDistance // sorted
}

type point struct {
	x, y float64
}

func (p point) getPointDistance(other point) pointDistance {
	d := math.Sqrt((other.x-p.x)*(other.x-p.x) + (other.y-p.y)*(other.y-p.y))
	return pointDistance{point: p, distance: d}
}

// these are just for the distances lists (the distance is from the key to the map)
type pointDistance struct {
	point    point
	distance float64
}

type byDistance []pointDistance

func (s byDistance) Len() int {
	return len(s)
}
func (s byDistance) Swap(i, j int) {
	s[i], s[j] = s[j], s[i]
}
func (s byDistance) Less(i, j int) bool {
	return s[i].distance < s[j].distance
}

func NewSolver(path common.Path) (solver Solver) {
	solver.points = make([]point, len(path))
	for _, p := range solver.points {
		list := make([]pointDistance, len(solver.points)-1)
		skipped := 0
		for i, d := range solver.points {
			if d == p {
				skipped = 1
				continue
			}
			list[i-skipped] = p.getPointDistance(d)
		}
		sort.Sort(byDistance(list))
		solver.distances[p] = list
	}
	return solver
}

func (s Solver) Solve(x, y float64, toCover common.Path) float64 {
	points := make([]point, len(toCover))
	// This is inefficient but I want to be able to do reference comparison later
	// Oh, and hashing. Mostly hashing
	for i, x := range toCover {
		for _, p := range s.points {
			if x.X == p.x && x.Y == p.y {
				points[i] = p
			}
		}
	}
	start := point{x: x, y: y}
	list := make([]pointDistance, len(points)) // distances list for starting point
	for i, p := range points {
		list[i] = start.getPointDistance(p)
	}
	sort.Sort(byDistance(list))
	// TODO -- do the greedy TSP solving
	// var covered map[point] bool
	// var p point = start
	// var distance float64
	// for {
	//
	// 	d := s.distances[p]
	//
	// }
	return 0
}
