#!/usr/bin/env python

# Roland Arsenault and Val Schmidt
# Center for Coastal and Ocean Mapping
# University of New Hampshire
# Copyright 2017, All rights reserved.

# Model for the C-Worker 4 ASV
# Engine: 30 bHP -> *.4 for losses * 745 to watts = 8948.4
# 3200 rpm: 30 bHP
# draft 0.4m
# 4m long, 1.7 wide, and has a draft of 0.4 m.
# Tonnage: 1 T
# Top Speed:       5.5 knots (2.75 m/s)
# Minimum survey speed:    1 knots
# Propulsion:      1 x 30 HP Yanmar Diesel direct drive.
cw4 = {'max_rpm':3200,
	   'max_power':8948.4,
	   'idle_rpm':0,
	   'prop_ratio':0.389105058,      # FIX
	   'prop_pitch':20,               # FIX
	   'max_rpm_change_rate':1000,    # FIX
	   'max_speed':2.75,
	   'mass':2000,                   
	   'max_rudder_angle':30,
	   'rudder_distance':2,           
	   'rudder_coefficient':.25,      # FIX
	  }

