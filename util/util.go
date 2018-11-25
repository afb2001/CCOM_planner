package util

import "log"

const debugVis bool = false

type ErrorPolicy int

const (
	IgnoreErr ErrorPolicy = iota
	LogErr
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
	if debugVis {
		log.Println(append([]interface{}{"Planner visualization:"}, v...)...)
	}
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
		return
	case LogErr:
		PrintLog("Encountered an error:", err)
		return
	case FatalErr:
		PrintError(err)
	}
}
