package parse

import (
	"bufio"
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
