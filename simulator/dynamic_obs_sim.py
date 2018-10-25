#!/usr/bin/env python



import dynamics
import cw4
import numpy as np

# Create an instance of the asv_sim model
# Set parameters for the model.
boat = {'max_rpm':3200,
           'max_power':8948.4,
           'idle_rpm':0,
           'prop_ratio':0.389105058,      # FIX --- What is it ?!?
           'prop_pitch':20,               # FIX [deg]
           'max_rpm_change_rate':1000,    # FIX
           'max_speed':2.75,		  # [m/s]?!?
           'mass':2000,                   # [kg]
           'max_rudder_angle':30,         # [deg]
           'rudder_distance':2,           # ---- what is rudder distance ?!?
           'rudder_coefficient':.25,      # FIX
	  }

# Set the starting location
start_lat = 43.0
start_long = -70.1
start_x = 1
start_y = 1

# Set target waypoint
desired_lat  = 42.997797    # Does it need to be updated from somewhere else?
desired_long = -70.102602   # Does it need to be updated from somewhere else?

# deg to rad	
desired_lat = np.radians(desired_lat)
desired_long = np.radians(desired_long)
wpt_x = 10
wpt_y = 10

# set target speed
desired_speed = 2.5 #get via UDP server

# Loop at some rate
dt=0.01 # [sec]
t = 0.0

boat_dynamics = dynamics.Dynamics(boat)

def	heading_to_rudder(heading):
	#heading_delta = data.heading.heading - current_heading
    heading_delta = desired_heading - curr_heading
    if heading_delta>np.pi:
        heading_delta -= np.pi*2.0
    if heading_delta<-np.pi:
        heading_delta += np.pi*2.0
    rudder = max(-1.0,min(1.0,heading_delta))
    return rudder
     

def speed_to_throttle(speed):
    #throttle = speed / dynamics.model["max_speed"]
	throttle = speed / boat["max_speed"]
	return throttle


while t < 30: #[sec] 
	#    Monitor for updates to wpts/speed
	

	if t == 0.0: 
		curr_lat  = start_lat
		curr_long = start_long
		curr_x = wpt_x
		curr_y = wpt_y
		curr_heading = 0.0	   
        
	# deg to rad
	curr_lat = np.radians(curr_lat)
	curr_long = np.radians(curr_long)


	#        Calculate desired heading from current position and desired position.
	#	 Referene: https://www.igismap.com/formula-to-find-bearing-or-heading-angle-between-two-points-latitude-longitude/	
 
	#X = np.cos(desired_lat)*np.sin(desired_long-curr_long)
	#Y = np.cos(curr_lat)*np.sin(desired_lat)-np.sin(curr_lat)*np.cos(desired_lat)*np.cos(desired_long-curr_long)
	desired_heading = np.arctan2(wpt_y-curr_y,wpt_x-curr_x) 
 
	#        Calculate rudder/throttle from desired heading and speed.
	rudder = heading_to_rudder(desired_heading)
	throttle = speed_to_throttle(desired_speed)
	 
	#    Update the model

	boat_dynamics.update(throttle,rudder,t)
	curr_x = boat_dynamics.x
	curr_y = boat_dynamics.y
	curr_heading = boat_dynamics.heading

	curr_speed = boat_dynamics.speed 
	print("x:%0.2f,y:%0.2f,s:%0.2f,h%0.2f" % (curr_x,curr_y,curr_speed,curr_heading))
	
	#    Report the new position
	t += dt


