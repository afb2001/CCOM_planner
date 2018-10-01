package main

import "math"

/**
Cell struct to make up grid
 */
type cell struct {
	x, y int // location
	distanceToShore int // in cells
}

/**
Cell constructor.
 */
func newCell( x int, y int ) cell {
	return cell{x: x, y: y, distanceToShore: math.MaxInt32}
}

/**
Convenience function to tell if cell is a static obstacle
 */
func (c *cell) isBlocked() bool {
	return c.distanceToShore == 0
}

/**
Grid struct for holding distances to static obstacles (shore)
 */
type grid struct {
	cells [][]cell
}

/**
Create a new grid of given dimensions
 */
func newGrid(width int, height int) grid {
	cells := new([][]cell)
	for x := 0; x < width; x++ {
		col := new([]cell)
		for y := 0; y < height; y++ {
			*col = append(*col, newCell(x, y))
		}
		*cells = append(*cells, *col)
	}
	return grid{cells: *cells}
}




