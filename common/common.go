package common

import (
	"fmt"
	"github.com/afb2001/CCOM_planner/util"
	"math"
	"strings"
)

const (
	planDistanceDensity float64 = 1
	planTimeDensity     float64 = 0.5
	TimeHorizon         float64 = 30
	coverageThreshold   float64 = 3
	collisionDistance   float64 = 1.8
)

//region State

/**
Represents a singular state in the world.
*/
type State struct {
	X, Y, Heading, Speed, Time float64
	CollisionProbability       float64
}

/**
Returns the time in seconds until state other.
*/
func (s *State) TimeUntil(other *State) float64 {
	return other.Time - s.Time
}

/**
Returns the Euclidean distance in two dimensions (x,y).
*/
func (s *State) DistanceTo(other *State) float64 {
	// return math.Sqrt(math.Pow(s.X-other.X, 2) + math.Pow(s.Y-other.Y, 2))
	return math.Sqrt((s.X-other.X)*(s.X-other.X) + (s.Y-other.Y)*(s.Y-other.Y))
}

func (s State) DistanceToArray(other [3]float64) float64 {
	return math.Sqrt((s.X-other[0])*(s.X-other[0]) + (s.Y-other[1])*(s.Y-other[1]))
}

func (s *State) HeadingTo(other *State) float64 {
	dx := other.X - s.X
	dy := other.Y - s.Y
	h := math.Atan2(dy, dx)
	if h < 0 {
		return h + (2 * math.Pi) // may not need this? I don't remember how tangents work
	}
	return h
}

/**
True iff other is within 1.5m in the x and y directions.
*/
func (s *State) Collides(other *State) bool {
	return s.Time == other.Time &&
		(math.Abs(s.X-other.X) < collisionDistance) &&
		(math.Abs(s.Y-other.Y) < collisionDistance)
}

func (s *State) CollidesWithArray(other [3]float64) bool {
	// assume times are equal
	return math.Abs(s.X-other[0]) < collisionDistance &&
		math.Abs(s.Y-other[1]) < collisionDistance
}

/**
Tests whether the states have same (x, y)
*/
func (s *State) IsSamePosition(other *State) bool {
	return s.X == other.X && s.Y == other.Y
}

/**
Convert this state to a 3D vector for Dubins functions.
*/
func (s *State) ToArrayPointer() *[3]float64 {
	return &[3]float64{s.X, s.Y, s.Heading}
}

/**
Create a string representation of the state.
Angle is turned back into heading.
*/
func (s *State) String() string {
	return fmt.Sprintf("%f %f %f %f %f", s.X, s.Y, (-1*s.Heading)+math.Pi/2, s.Speed, s.Time)
}

/**
Projects a state to a specified time assuming constant speed and heading.
Creates a new state.

Meant to be used with future times but should work either way.
*/
func (s *State) Project(time float64) *State {
	deltaT := time - s.Time
	magnitude := deltaT * s.Speed
	deltaX := math.Cos(s.Heading) * magnitude
	deltaY := math.Sin(s.Heading) * magnitude
	return &State{X: s.X + deltaX, Y: s.Y + deltaY, Heading: s.Heading, Speed: s.Speed, Time: time}
}

/**
Push a state at a given angle for a given distance.
Mutates the current state.

Written for ray-casting of sorts during collision checking.
Probably should not use past version 0.
*/
func (s *State) Push(heading float64, distance float64) {
	dx := distance * math.Cos(heading)
	dy := distance * math.Sin(heading)
	s.X += dx
	s.Y += dy
}

//endregion

//region Path

type Path []State

/**
Remove the given state from the Path. Modifies the original Path.
*/
func (p Path) Without(s State) Path {
	b := Path{}
	for _, x := range p {
		if s.X != x.X || s.Y != x.Y {
			b = append(b, x)
		}
	}
	//*p = b
	return b
}

func (p Path) MaxDistanceFrom(s State) (max float64) {
	for _, x := range p {
		if d := s.DistanceTo(&x); d > max {
			max = d
		}
	}
	return
}

func (p Path) MinDistanceFrom(s State) (min float64) {
	min = math.MaxFloat64
	for _, x := range p {
		if d := s.DistanceTo(&x); d < min {
			min = d
		}
	}
	return
}

func (p Path) NewlyCovered(s State) (covered Path) {
	for _, x := range p {
		if s.DistanceTo(&x) < coverageThreshold {
			covered = append(covered, x)
		}
	}
	return
}

func (p Path) NewlyCoveredArray(s [3]float64) (covered Path) {
	for _, x := range p {
		if x.DistanceToArray(s) < coverageThreshold {
			covered = append(covered, x)
		}
	}
	return
}

//endregion

//region Plan
type Plan struct {
	Start  State // be careful using this because it isn't always fully set...
	States []*State
}

func (p *Plan) String() string {
	s := fmt.Sprintf("plan %d", len(p.States))
	for _, state := range p.States {
		s += "\n" + state.String()
	}
	return s
}

/**
Append a state to the plan when the state is within the time horizon and either:
	1. The plan is empty
	2. There is a substantial distance gap between the last state and this one
	3. There is a substantial time gap between the last state and this one
*/
func (p *Plan) AppendState(s *State) {
	if len(p.States) == 0 ||
		(p.Start.TimeUntil(p.States[len(p.States)-1]) < TimeHorizon &&
			//(!(p.States[len(p.States)-1].DistanceTo(s) < planDistanceDensity) ||
			p.States[len(p.States)-1].TimeUntil(s) >= planTimeDensity) {
		p.States = append(p.States, s)
		//util.PrintDebugVertex(s.String(), "boat", 2)
		util.PrintDebug(s.String(), "g = 0 h = 0 tag = plan")
	}
}

/**
Concatenate two plans.
*/
func (p *Plan) AppendPlan(other *Plan) {
	for _, s := range other.States {
		p.AppendState(s)
	}
}

/**
Default do-nothing plan.
*/
func DefaultPlan(start *State) *Plan {
	plan := new(Plan)
	s := State{X: start.X, Y: start.Y, Heading: start.Heading, Speed: start.Speed, Time: start.Time + 1.0}
	plan.States = append(plan.States, &s)
	return plan
}

//endregion

//region Grid
/**
Cell struct to make up grid
*/
type cell struct {
	x, y            int // location
	distanceToShore int // in cells
}

/**
Cell constructor.
*/
func newCell(x int, y int) cell {
	return cell{x: x, y: y, distanceToShore: math.MaxInt32}
}

/**
Convenience function to tell if cell is a static obstacle
*/
func (c *cell) isBlocked() bool {
	return c.distanceToShore == 0
}

/**
Grid struct for holding distances to static obstacles (shore)
*/
type Grid struct {
	cells         [][]bool
	Width, Height int
}

/**
Create a new grid of given dimensions
*/
func NewGrid(width int, height int) Grid {
	cells := make([][]bool, height)
	for y := range cells {
		cells[y] = make([]bool, width)
	}
	return Grid{cells: cells, Width: width, Height: height}
}

/**
Get the cell at x, y.
*/
func (g *Grid) get(x int, y int) bool {
	return g.cells[y][x]
}

/**
Block the cell at x, y. For initialization only (probably).
*/
func (g *Grid) block(x int, y int) {
	g.cells[y][x] = true
}

/**
Block cells at the specified resolution
*/
func (g *Grid) BlockRange(x int, y int, r int) {
	for i := 0; i < r; i++ {
		for j := 0; j < r; j++ {
			g.block(x+i, y+j)
		}
	}
}

func (g *Grid) Dump() string {
	var sb = strings.Builder{}
	sb.WriteRune('\n')
	for y := g.Height - 1; y >= 0; y-- {
		for x := 0; x < g.Width; x++ {
			if g.IsBlocked(float64(x), float64(y)) {
				sb.WriteRune('#')
			} else {
				sb.WriteRune('_')
			}
		}
		sb.WriteRune('\n')
	}
	return sb.String()
}

/**
Determine if a given point is within a static obstacle.
*/
func (g *Grid) IsBlocked(x float64, y float64) bool {
	if x < 0 || x > float64(g.Width) || y < 0 || y > float64(g.Height) {
		return true
	}
	return g.get(int(x), int(y))
}

//endregion

//region Obstacles

type Obstacle struct {
	Initial State
	Stddev  float64
}

/**
Type alias for (dynamic) obstacle collection.
*/
type Obstacles []*State

// /**
// Add or update the obstacle collection with the new obstacle.
// */
// func (o *Obstacles) Update(id int, newState *State) {
// 	(*o)[id] = newState
// }
//
// func (o *Obstacles) Remove(id int) {
// 	delete(*o, id)
// }

/**
Check if any of the obstacles collide with the given state.

Returns a float64 probability of collision.
*/
func (o *Obstacles) CollisionExists(state *State) float64 {
	for _, s := range *o {
		if s.Project(s.TimeUntil(state)).Collides(state) {
			return 1.0
		}
	}
	return 0
}

func (o *Obstacles) CollisionExistsWithArray(state [3]float64, t float64) float64 {
	for _, s := range *o {
		if s.Project(t - s.Time).CollidesWithArray(state) {
			util.PrintLog("Collision exists")
			return 1.0
		}
	}
	return 0
}

//endregion
