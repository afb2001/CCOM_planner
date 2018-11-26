#!/usr/bin/env python
import depth_map
import sys
import numpy as np
if __name__ == '__main__':

    geotiff = sys.argv[1]
    if geotiff != '':
        geotiff = depth_map.BathyGrid(geotiff)
        x1, x2, y1, y2 = geotiff.getBound()
        maxx = np.abs(x2-x1)
        maxy = np.abs(y2-y1)
        geodata = geotiff.getGrid()
        by, bx = np.shape(geodata)
        xlim = bx
        ylim = by

        factor = maxx / bx
        print factor
        print bx
        print by
        for j in range(by):
            s = ""
            for i in range(bx):
                if geodata[j, i] < 2:
                    s += "#"
                else:
                    s += "_"
            print s
