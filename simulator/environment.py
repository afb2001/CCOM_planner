#!/usr/bin/env python

# Roland Arsenault
# Center for Coastal and Ocean Mapping
# University of New Hampshire
# Copyright 2017, All rights reserved.


class Environment:
    def __init__(self):
        self.current = {"speed": 1.0, "direction": 90.0}
        
    def getCurrent(self, lat, long):
        # plan to allow current to differ with location, uniform for now.
        return self.current
    
    
