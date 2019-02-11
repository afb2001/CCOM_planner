package util

import (
	"bufio"
	"fmt"
	"log"
	"strings"
)

var DebugVis = false
var DebugToFile = true
var VisWriter *bufio.Writer

type ErrorPolicy int

const (
	IgnoreErr ErrorPolicy = iota
	LogErr
	ParseErr
	FatalErr
)

/**
Print a fatal error and die.
*/
func PrintError(v ...interface{}) {
	log.Fatal(append([]interface{}{"Planner error:"}, v...)...)
}

/**
LogErr a message to stderr.
*/
func PrintLog(v ...interface{}) {
	log.Println(append([]interface{}{"Planner message:"}, v...)...)
}

/**
Print a debug visualization for the planner.
*/
func PrintDebug(v ...string) {
	v = append([]string{"Planner visualization:"}, v...)
	if DebugVis {
		log.Println(strings.Join(v, " "))
	}
	if DebugToFile && VisWriter != nil {
		_, err := VisWriter.WriteString(strings.Join(v, " "))
		_, err = VisWriter.WriteString("\n")
		err = VisWriter.Flush()
		HandleError(err, LogErr)
	}
}

/**
Show a vertex in the specified visualizer with the specified shape.
*/
func PrintDebugVertex(vertex string, shape string, vis int) {
	PrintDebug(vertex, fmt.Sprintf("shape = %s vis = vis%d", shape, vis))
}

/**
Error handling
*/
func HandleError(err error, policy ErrorPolicy) {
	if err == nil {
		return
	}
	switch policy {
	case IgnoreErr:
	case LogErr:
		PrintLog("Encountered an error:", err)
		PrintError(err)
	case ParseErr:
		fallthrough
	case FatalErr:
		PrintError(err)
	}
}
