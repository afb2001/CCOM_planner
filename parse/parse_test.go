package parse

import (
	"bufio"
	"github.com/afb2001/CCOM_planner/common"
	"strings"
	"testing"
)

func TestBuildGrid(t *testing.T) {
	t.Log("Testing BuildGrid...")
	rd := strings.NewReader(`map 1 10 10
_
# 9
_
_ 1
_
# 9
_
_ 1
_
_
`)
	reader := bufio.NewReader(rd)
	grid := BuildGrid(reader)
	dump := grid.Dump()
	if dump != `
__________
#########_
__________
_#########
__________
#########_
__________
_#########
__________
__________
` {
		println(dump)
		t.Errorf("Wrong grid dumped")
	}
}

func TestReadObstacles(t *testing.T) {
	t.Log("Testing reading obstacles")
	rd := strings.NewReader(`1 10 10 1.5 1 0
2 17 8 0.2 1 0
3 81 44.2 0 0 0
`)
	reader := bufio.NewReader(rd)
	o := make(common.Obstacles, 3)
	ReadObstacles(reader, o, 3)
	if o[0].X != 10 ||
		o[1].X != 17 ||
		o[2].X != 81 {
		t.Errorf("Read wrong obstacle(s)")
	}
}

func TestUpdatePath(t *testing.T) {
	t.Log("Testing updating the path to cover")
	rd := strings.NewReader(`newly covered 1
0.5 1.5
`)
	reader := bufio.NewReader(rd)
	var p1, p2 = common.State{X: 0.5, Y: 1.5}, common.State{X: 0.5, Y: 2.5}
	var p = &common.Path{p1, p2}
	println(len(*p))
	UpdatePath(reader, p)
	if len(*p) != 1 {
		println(len(*p))
		t.Errorf("Failed to remove element from path")
	}
}
