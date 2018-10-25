#!/usr/bin/env python

# Roland Arsenault
# Center for Coastal and Ocean Mapping
# University of New Hampshire
# Copyright 2017, All rights reserved.

# Engine: 200 bHP -> *.5 for lossed * 745 to watts = 74500
# 2400 rpm: 200 bHP
# 2200 rpm: 185 bHP
# draft 5.5ft
# 40' (12.19m) long, 12' (3.66m) wide, and has a draft of 5.5' (1.8m).
# Tonnage: 16 GRT, 11 DWT
# Top Speed:       10 knots (5.14444444 m/s)
# Minimum speed for full roll stabilization:       5 knots
# Minimum survey speed:    2.5 knots
# Propulsion:      1 x Caterpillar 3116; 200HP Marine Diesel; 2.57:1 reduction
# prop pitch of 20 is just a random number for testing, mass is random as well
coastal_surveyor = {'max_rpm':2400,
                    'max_power':74500,
                    'idle_rpm':100,
                    'prop_ratio':0.389105058,
                    'prop_pitch':20,
                    'max_rpm_change_rate':1000,
                    'max_speed':5.14444444,
                    'mass':5000,
                    'max_rudder_angle':30,
                    'rudder_distance':6,
                    'rudder_coefficient':.25,
                   }

