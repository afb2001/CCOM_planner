#!/usr/bin/env python

import sys
import gdal

class BathyGrid:
    def __init__(self, fname):
        self.dataset = gdal.Open(fname, gdal.GA_ReadOnly)
        
        self.band = self.dataset.GetRasterBand(1)
        self.geoTransform = self.dataset.GetGeoTransform()
        self.inverseGeoTransorm = gdal.InvGeoTransform(self.geoTransform)

        self.data = self.band.ReadAsArray()
        ulx, xres, xskew, uly, yskew, yres  = self.dataset.GetGeoTransform()
        lrx = ulx + (self.dataset.RasterXSize * xres)
        lry = uly + (self.dataset.RasterYSize * yres)
        self.x1 = ulx;
        self.x2 = lrx;
        self.y1 = lry;
        self.y2 = uly;
        

    def getBound(self):
        return int(self.x1),int(self.x2),int(self.y1),int(self.y2)
    def getDepth(self,x,y):
        xi = self.inverseGeoTransorm[0]+x*self.inverseGeoTransorm[1]+y*self.inverseGeoTransorm[2]
        yi = self.inverseGeoTransorm[3]+x*self.inverseGeoTransorm[4]+y*self.inverseGeoTransorm[5]
        # print y,x,int(yi),int(xi)
        try:
            return self.data[int(yi),int(xi)]
        except IndexError:
            return None
    def getGrid(self):
        return self.data

if __name__ == '__main__':

    # TODO: add arg checking and usage display, for now, expect geotiff or similar raster passed in as first cmd line arg.
    grid = BathyGrid(sys.argv[1])

    while True:
        # TODO: more robust input parsing, for now, pass in x y whitspace seperated.
        data = sys.stdin.readline()
        coordinates = data.split(None,1)
        x = float(coordinates[0].strip())
        y = float(coordinates[1].strip())
        print grid.getDepth(x,y)
