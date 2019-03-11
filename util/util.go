package util

import (
	"bufio"
	"fmt"
	"log"
	"os"
	"strings"
)

var DebugVis = false
var DebugToFile = false
var VisWriter *bufio.Writer
var visInputFile *os.File
var Verbose = false

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
	log.Fatal(append([]interface{}{"Planner error: "}, v...)...)
}

/**
LogErr a message to stderr.
*/
func PrintLog(v ...interface{}) {
	log.Println(append([]interface{}{"Planner message:"}, v...)...)
}

/**
Logs a message only in verbose mode.
*/
func PrintVerbose(v ...interface{}) {
	if Verbose {
		PrintLog(v...)
	}
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
func PrintDebugVertex(vertex string, tag string) {
	if DebugToFile {
		PrintDebug(vertex, fmt.Sprintf("tag = %s", tag))
	}
}

func PrintTrajectoryState(x, y, h, cost float64) {
	PrintDebugVertex(fmt.Sprintf("%f %f %f 0 0 g = %f h = 0", x, y, h, cost), "trajectory")
}

func SetupDebugWriter() {
	err := os.Remove("../vis_input")
	HandleError(err, LogErr)
	visInputFile, err = os.OpenFile("../vis_input", os.O_CREATE|os.O_WRONLY, 0755)
	HandleError(err, LogErr)
	VisWriter = bufio.NewWriter(visInputFile)
}

func CleanupDebugWriter() {
	err := visInputFile.Close()
	HandleError(err, LogErr)
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
		//PrintError(err)
	case ParseErr:
		fallthrough
	case FatalErr:
		PrintError(err)
	}
}
