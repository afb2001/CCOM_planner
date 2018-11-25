package util

import "log"

const debugVis bool = false

/**
Print a fatal error and die.
*/
func PrintError(v ...interface{}) {
	log.Println(append([]interface{}{"Planner error:"}, v...)...)
}

/**
Log a message to stderr.
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
