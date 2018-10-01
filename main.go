package main

import (
	"fmt"
	"log"
	"time"
)

func main() {
	var startTime = time.Now()
	fmt.Println("Planner starting at", startTime.Format("15:04:05.1234"))
	buildGrid()
}

func buildGrid() grid {
	log.Println("Reading map dimensions")
	var mapWidth, mapHeight int
	_, err := fmt.Scanf("%d %d", &mapWidth, &mapHeight)
	if err != nil { log.Fatal(err) }
	grid := newGrid(mapWidth, mapHeight)
	for y := 0; y < mapHeight; y++ {
		var line string
		_, err = fmt.Scanln(&line)
		//if(len(line) != mapWidth) { log.Fatal("Error: wrong map dimension") }
		//for x := 0; x < mapWidth; x++ {
		//	fmt.Println(line[x])
		//}
		for x, c := range line {
			fmt.Println(x, c)
		}
	}
	return grid
}
