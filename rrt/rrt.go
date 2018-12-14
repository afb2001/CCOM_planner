package rrt

import (
	"container/heap"
	"fmt"
	. "github.com/afb2001/CCOM_planner/common"
	"github.com/afb2001/CCOM_planner/dubins"
	. "github.com/afb2001/CCOM_planner/util"
	"math"
	"math/rand"
	"time"
)

const (
	verbose        bool    = false
	goalBias       float64 = 0.05
	maxSpeedBias   float64 = 1.0
	dubinsInc      float64 = 0.1 // this might be low
	K              int     = 5   // number of closest states to consider for BIT*
	bitStarSamples int     = 32  // (m in the paper) -- make this a parameter too
	// BIT* penalties (should all be made into parameters)
	coveragePenalty  float64 = 60
	collisionPenalty float64 = 600 // this is suspect... may need to be lower because it will be summed
	timePenalty      float64 = 1
)

var maxSpeed, maxTurningRadius float64

func SetBoatConstants(s, r float64) {
	maxSpeed, maxTurningRadius = s, r
}

/**
Returns the current time in seconds as a float
*/
func now() float64 {
	return float64(time.Now().UnixNano()) / 10e9
}

/**
Default do-nothing plan.
*/
func defaultPlan(start *State) *Plan {
	return nil
	//plan := new(Plan)
	//s := State{X: start.X, Y: start.Y, Heading: start.Heading, Speed: start.Speed, Time: start.Time + 1.0}
	//plan.States = append(plan.States, &s)
	//return plan
}

func MakePlan(grid *Grid, start *State, path Path, o *Obstacles, timeToPlan float64) *Plan {
	PrintLog("Starting to plan")

	if len(path) == 0 {
		return defaultPlan(start)
	}
	// set goal as first item in path
	goalCount := 0
	goal := &path[goalCount]

	startTime := float64(time.Now().UnixNano()) / 10e9
	var p = new(Plan)
	for startTime+timeToPlan > now() {
		newPlan := rrt(grid, start, goal, o, (startTime+timeToPlan)-now())
		if newPlan == nil {
			if len(p.States) == 0 {
				PrintLog("Done planning")
				return defaultPlan(start)
			}
		} else {
			p.AppendPlan(newPlan)
			if goalCount++; len(path) > goalCount {
				start = goal
				goal = &path[goalCount]
			} else {
				PrintLog("Done planning")

				//if p.states[0].IsSamePosition(start) { // doesn't work for some reason??
				//	p.states = p.states[1:] // remove start state from plan
				//}
				return p
			}
		}
	}

	PrintLog("Done planning")
	return p
}

type rrtNode struct {
	state        *State
	parent       *rrtNode
	pathToParent *dubins.Path
	trajectory   *Plan // trajectory to parent
}

/**
Create a new state with random values.
Time is unset (zero).
*/
func randomState(bounds *Grid) *State {
	s := new(State)
	s.X = rand.Float64() * float64(bounds.Width)
	s.Y = rand.Float64() * float64(bounds.Height)
	s.Heading = rand.Float64() * math.Pi * 2
	s.Speed = rand.Float64() * maxSpeed
	return s
}

/**
Create a random sample using the biasing constants.
*/
func biasedRandomState(bounds *Grid, goal *State) *State {
	//verbose = false
	s := randomState(bounds)
	if r := rand.Float64(); r < maxSpeedBias {
		s.Speed = maxSpeed
	}
	if r := rand.Float64(); r < goalBias {
		//verbose = true
		return goal
	}
	return s
}

func randomNode(bounds *Grid, goal *State) *rrtNode {
	n := rrtNode{state: biasedRandomState(bounds, goal)}
	return &n
}

/**
Find the closest node in a list of nodes.
O(n) time.
*/
func getClosest(nodes []*rrtNode, n *rrtNode) (*rrtNode, float64, *dubins.Path) {
	return getClosestNoHeap(nodes, n)
	//return getClosestWithHeap(nodes, n)
}

// debugging variables for keeping track of statistics
//var iterationCount int
//var shortestPathCompCount int

func getClosestWithHeap(nodes []*rrtNode, n *rrtNode) (*rrtNode, float64, *dubins.Path) {
	var closest *rrtNode
	minDistance := math.MaxFloat64 // dubins distance
	var bestPath dubins.Path
	nodeHeap := heapify(nodes, n.state)
	for heapNode := nodeHeap.Pop().(*rrtNode);
	// euclidean distance (2d) vs dubins distance
	heapNode.state.DistanceTo(n.state) < minDistance; {
		//shortestPathCompCount++
		dPath, err := shortestPath(heapNode.state, n.state)
		if err != dubins.EDUBOK {
			if verbose {
				PrintLog("Couldn't make dubins path")
			}
			continue
		}
		if d := dPath.Length(); d < minDistance {
			minDistance = d
			closest = heapNode
			bestPath = *dPath
		}
		if nodeHeap.Len() > 0 {
			heapNode = nodeHeap.Pop().(*rrtNode)
		} else {
			break
		}
	}
	//iterationCount++
	return closest, minDistance, &bestPath
}

func getClosestNoHeap(nodes []*rrtNode, n *rrtNode) (*rrtNode, float64, *dubins.Path) {
	var closest *rrtNode
	minDistance := math.MaxFloat64
	var bestPath dubins.Path
	for _, node := range nodes {
		//shortestPathCompCount++
		dPath, err := shortestPath(node.state, n.state)
		if err != dubins.EDUBOK {
			if verbose {
				PrintLog("Couldn't make dubins path")
			}
			continue
		}
		if d := dPath.Length(); d < minDistance {
			minDistance = d
			closest = node
			bestPath = *dPath
		}
	}
	//iterationCount++
	return closest, minDistance, &bestPath
}

func rrt(g *Grid, start *State, goal *State, o *Obstacles, timeRemaining float64) *Plan {
	if *start == *goal {
		return nil
	}
	//trueStartTime := start.Time
	startTime := float64(time.Now().UnixNano()) / 10e9
	PrintLog(fmt.Sprintf("Start state is %s", start.String()))
	p := new(Plan)
	root := &rrtNode{state: start}
	nodes := []*rrtNode{root}
	var n *rrtNode
	var sampleCount, blockedCount int
	// RRT loop
	for startTime+timeRemaining > now() {
		//PrintLog("Starting RRT loop")
		n = randomNode(g, goal)
		sampleCount++
		//PrintLog(fmt.Sprintf("Sampled state %s", n.state.String()))
		closest, distance, dPath := getClosest(nodes, n)
		if distance == math.MaxFloat64 {
			blockedCount++
			//PrintLog("Could not find state to connect to")
			continue
		}
		n.trajectory = getSamples(dPath, g, o)
		if n.trajectory == nil {
			blockedCount++
			//PrintLog("Could not find state to connect to")
			continue // no path so continue
		}
		//PrintLog(fmt.Sprintf("Found nearest state %s at distance %f", closest.state.String(), distance))

		n.parent = closest
		n.pathToParent = dPath
		nodes = append(nodes, n)
		if n.state == goal {
			break
		}
		//PrintLog((fmt.Sprintf("Current node count is now %d", len(nodes))))
	}
	PrintLog(fmt.Sprintf("Samples: %d, blocked: %d, ratio: %f", sampleCount, blockedCount, float64(blockedCount)/float64(sampleCount)))
	//PrintLog("Samples on map:" + showSamples(nodes, g, start, goal))

	if !(startTime+timeRemaining > now()) {
		PrintLog("Time elapsed before we found a plan")
		return nil // calling function handles default plan
	}

	// if we couldn't even start
	if n == nil {
		return nil
	}

	// collect all nodes into slice
	var branch []*rrtNode

	for cur := n; cur.state != start && cur.parent != nil; cur = cur.parent {
		branch = append(branch, cur)
	}

	// reverse the plan order (this might look dumb)
	s := branch
	for i, j := 0, len(s)-1; i < j; i, j = i+1, j-1 {
		s[i], s[j] = s[j], s[i]
	}
	branch = s

	// add states to plan and compute times
	t := start.Time
	//var prev *State = nil
	//var headingDelta float64
	for _, n := range branch {
		//PrintLog(fmt.Sprintf("Trajectory from %s has type %d", n.state.String(), n.pathToParent.GetPathType()))
		t += dubinsInc / maxSpeed
		cur := n.state
		traj := n.trajectory // shorthand, I guess?
		for _, s := range traj.States {
			t += dubinsInc / maxSpeed
			s.Time += t
			p.AppendState(s)
			//// filter which states to include in output plan
			//if s.Time-trueStartTime < 5 && (prev == nil ||
			//	prev.TimeUntil(s) > 1.0 ||
			//	prev.DistanceTo(s) > 1.0) {
			//	//(headingDelta != 0 &&
			//	//	!prev.IsSamePosition(s) &&
			//	// heading delta changes sign, i.e we moved to new dubins segment
			//	//headingDelta*(prev.Heading-s.Heading) <= 0)
			//	//{
			//	// so we want to include this state
			//	p.AppendState(s)
			//	prev = s
			//}

			//if prev != nil {
			//	headingDelta = prev.Heading - s.Heading
			//}
			//prev = s
		}
		t += dubinsInc / maxSpeed
		cur.Time = t
		p.AppendState(cur)
		//prev = cur
	}
	//PrintLog(fmt.Sprintf("Found a plan for goal %s", goal.String()))
	//PrintLog(p.String())

	return p
}

func showSamples(nodes []*rrtNode, g *Grid, start *State, goal *State) string {
	var bytes = []byte(g.Dump())
	var arrays [][]byte
	for i := g.Height - 1; i >= 0; i-- {
		arrays = append(arrays, bytes[1+(i*(g.Width+1)):1+(i+1)*(g.Width+1)])
	}
	//PrintLog("All nodes sampled:")
	for _, n := range nodes {
		//PrintLog(n.state.String())
		arrays[int(n.state.Y)][int(n.state.X)] = 'o'
	}
	arrays[int(start.Y)][int(start.X)] = '@'
	arrays[int(goal.Y)][int(goal.X)] = '*'
	//PrintLog("Map:")
	return string(bytes)
}

//endregion

//region NodeHeap

type NodeHeap struct {
	nodes      []*rrtNode
	otherState *State
}

func (h NodeHeap) Len() int { return len(h.nodes) }
func (h NodeHeap) Less(i, j int) bool {
	return h.nodes[i].state.DistanceTo(h.otherState) <
		h.nodes[j].state.DistanceTo(h.otherState)
}
func (h NodeHeap) Swap(i, j int) { h.nodes[i], h.nodes[j] = h.nodes[j], h.nodes[i] }

func (h *NodeHeap) Push(x interface{}) {
	h.nodes = append(h.nodes, x.(*rrtNode))
}

func (h *NodeHeap) Pop() interface{} {
	old := *h
	n := len(old.nodes)
	x := old.nodes[n-1]
	h.nodes = old.nodes[0 : n-1]
	return x
}

func heapify(nodes []*rrtNode, otherState *State) *NodeHeap {
	var nodeHeap = NodeHeap{nodes: nodes, otherState: otherState}
	for i, n := range nodes {
		nodeHeap.nodes[i] = n
	}
	heap.Init(&nodeHeap)
	return &nodeHeap
}

func (h *NodeHeap) update(otherState *State) {
	h.otherState = otherState
	heap.Init(h)
}

//endregion

//region Dubins integration

/**
Find the shortest Dubins path between two states.
*/
func shortestPath(s1 *State, s2 *State) (path *dubins.Path, err int) {
	if verbose {
		PrintLog(fmt.Sprintf("Computing dubins path between %s, %s", s1.String(), s2.String()))
	}
	path = new(dubins.Path)
	err = dubins.ShortestPath(path, s1.ToArrayPointer(), s2.ToArrayPointer(), maxTurningRadius)
	return path, err
}

/**
Convert the given path into a feasible plan.
*/
func getSamples(path *dubins.Path, g *Grid, o *Obstacles) (plan *Plan) {
	plan = new(Plan)
	var t float64
	callback := func(q *[3]float64, inc float64) int {
		//PrintLog(q)
		//t += inc / maxSpeed
		s := &State{X: q[0], Y: q[1], Heading: q[2], Speed: maxSpeed, Time: t} // t = 0 now
		s.CollisionProbability = o.CollisionExists(s)
		// collision probability is 0 or 1 for now
		if g.IsBlocked(s.X, s.Y) || s.CollisionProbability > 0 {
			if verbose {
				PrintLog(fmt.Sprintf("Blocked path: %f %f %f", s.X, s.Y, s.CollisionProbability))
			}
			return dubins.EDUBNOPATH
		}
		plan.AppendState(s)
		return 0
	}
	err := path.SampleMany(dubinsInc, callback)

	if err != dubins.EDUBOK {
		return nil
	}

	return plan
}

//endregion
