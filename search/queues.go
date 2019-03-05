package search

import (
	"container/heap"
	. "github.com/afb2001/CCOM_planner/util"
)

//region Queues

//region VertexQueue

type VertexQueue struct {
	Nodes []*Vertex
	Cost  func(node *Vertex) float64
}

func (h VertexQueue) Len() int { return len(h.Nodes) }
func (h VertexQueue) Less(i, j int) bool {
	return h.Cost(h.Nodes[i]) < h.Cost(h.Nodes[j])
}
func (h VertexQueue) Swap(i, j int) { h.Nodes[i], h.Nodes[j] = h.Nodes[j], h.Nodes[i] }

func (h *VertexQueue) Push(x interface{}) {
	if x.(*Vertex).ParentEdge == nil {
		PrintLog("Added a vertex to qV with no parent edge!")
	}
	h.Nodes = append(h.Nodes, x.(*Vertex))
}

func (h *VertexQueue) Pop() interface{} {
	old := *h
	n := len(old.Nodes)
	x := old.Nodes[n-1]
	h.Nodes = old.Nodes[0 : n-1]
	return x
}

func (h *VertexQueue) Peek() interface{} {
	return h.Nodes[len(h.Nodes)-1]
}

func makeVertexQueue(nodes []*Vertex, cost func(node *Vertex) float64) *VertexQueue {
	var nodeHeap = VertexQueue{Nodes: nodes, Cost: cost}
	for i, n := range nodes {
		nodeHeap.Nodes[i] = n
	}
	heap.Init(&nodeHeap)
	return &nodeHeap
}

func (h *VertexQueue) Update(cost func(node *Vertex) float64) {
	if cost != nil {
		h.Cost = cost
	}
	heap.Init(h)
}

// A costly (O(n)) operation that verifies the item in spot 0 is the smallest.
func (qV *VertexQueue) Verify() {
	for _, n := range qV.Nodes {
		if qV.Nodes[0].FValue() > n.FValue() {
			PrintError("Popped wrong value from queue")
		}
	}
}

//endregion

//region EdgeQueue

type EdgeQueue struct {
	Nodes []*Edge
	Cost  func(node *Edge) float64
}

func (h EdgeQueue) Len() int { return len(h.Nodes) }
func (h EdgeQueue) Less(i, j int) bool {
	return h.Cost(h.Nodes[i]) < h.Cost(h.Nodes[j])
}
func (h EdgeQueue) Swap(i, j int) { h.Nodes[i], h.Nodes[j] = h.Nodes[j], h.Nodes[i] }

func (h *EdgeQueue) Push(x interface{}) {
	h.Nodes = append(h.Nodes, x.(*Edge))
}

func (h *EdgeQueue) Pop() interface{} {
	old := *h
	n := len(old.Nodes)
	x := old.Nodes[n-1]
	h.Nodes = old.Nodes[0 : n-1]
	return x
}

func (h *EdgeQueue) Peek() interface{} {
	return h.Nodes[len(h.Nodes)-1]
}

func makeEdgeQueue(nodes []*Edge, cost func(node *Edge) float64) *EdgeQueue {
	var nodeHeap = EdgeQueue{Nodes: nodes, Cost: cost}
	for i, n := range nodes {
		nodeHeap.Nodes[i] = n
	}
	heap.Init(&nodeHeap)
	return &nodeHeap
}

func (h *EdgeQueue) Update(cost func(node *Edge) float64) {
	if cost != nil {
		h.Cost = cost
	}
	heap.Init(h)
}

//endregion

// functions for queueing vertices and edges
func VertexCost(v *Vertex) float64 {
	return v.FValue()
}

func EdgeCost(edge *Edge) float64 {
	// PrintLog(*edge) //debug
	// NOTE: when the heuristic function becomes more expensive this will need to get changed
	return edge.Start.GetCurrentCost() +
		edge.ApproxCost() +
		edge.End.HValue()
}

//endregion
