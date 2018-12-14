package util

import (
	"fmt"
	"log"
)

var DebugVis = false

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
func PrintDebug(v ...interface{}) {
	if DebugVis {
		log.Println(append([]interface{}{"Planner visualization:"}, v...)...)
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
	case ParseErr:
		fallthrough
	case FatalErr:
		PrintError(err)
	}
}
