#!/usr/bin/env python

# Roland Arsenault, Coral Moreno and Val Schmidt
# Center for Coastal and Ocean Mapping
# University of New Hampshire
# Copyright 2017, All rights reserved.

import math
import random
import geodesic
import time

start_lat =  43.0725381
start_lon = -70.7105114


class Dynamics:
    def __init__(self,model,init_x=0.0,init_y=0.0):
        self.model = model
        self.x_o = init_x
        self.y_o = init_y
        
        self.reset()

        max_prop_speed = (self.model['max_rpm']*self.model['prop_ratio'])/self.model['prop_pitch']
        max_force = self.model['max_power']/self.model['max_speed']
        
        self.prop_coefficient = max_force/(max_prop_speed**2-self.model['max_speed']**2)
        

        self.drag_coefficient = max_force /(self.model['max_speed']**3)

        #self.prop_coefficient = (self.model['max_speed']**3)*self.drag_coefficient/self.model['max_rpm']
        
        #self.last_update = None


    def reset(self):
        self.rpm = 0
        self.speed = 0.0
        self.longitude = math.radians(start_lon)
        self.latitude = math.radians(start_lat)
        
        self.heading = 0.0
        self.pitch = 0.0
        self.roll = 0.0

        self.x = self.x_o
        self.y = self.y_o
        self.last_update = time.time()

    def update(self,throttle, rudder, timestamp):
        
        throttle = min(1.0,max(0.0,throttle))

        
        if self.last_update is None:
            delta_t = None
        else:
            #delta_t = (timestamp-self.last_update).to_sec()
            delta_t = timestamp-self.last_update
		# ADDED FOR DEBUGGING
        #print("last_update: %0.2f" % self.last_update)
        self.last_update = timestamp
        

        target_rpm = self.model['idle_rpm']+ throttle * (self.model['max_rpm']-self.model['idle_rpm'])
        if(target_rpm != self.rpm):
            if delta_t is not None:
                rcr = (target_rpm - self.rpm)/delta_t
                if abs(rcr) > self.model['max_rpm_change_rate']:
                    if rcr < 0:
                        rcr = -self.model['max_rpm_change_rate']
                    else:
                        rcr = self.model['max_rpm_change_rate']
                self.rpm += rcr*delta_t

        prop_rpm = self.model['prop_ratio']*self.rpm
        prop_speed = prop_rpm/self.model['prop_pitch']

        rudder_speed = max(math.sqrt(prop_speed),self.speed)

        thrust = self.prop_coefficient*(prop_speed**2-self.speed**2)
        thrust = random.gauss(thrust,thrust*0.1)

        rudder_angle = rudder*self.model['max_rudder_angle']
        rudder_angle += random.gauss(0.0,0.25)
        rudder_rads = math.radians(rudder_angle)

        thrust_fwd = thrust*math.cos(rudder_rads)

        rudder_speed_yaw = rudder_speed*math.sin(rudder_rads)
        yaw_rate = self.model['rudder_coefficient']*rudder_speed_yaw/self.model['rudder_distance']
        if delta_t is not None:
            self.heading += yaw_rate*delta_t
            self.heading = math.fmod(self.heading,math.radians(360))
            if self.heading < 0:
                self.heading += math.radians(360)

        drag = self.speed**3*self.drag_coefficient
        drag = random.gauss(drag,drag*0.1)

        a = (thrust_fwd-drag)/self.model['mass']

        if delta_t is not None:
            self.speed += a*delta_t

        if self.speed > 0:
            (prop_rpm/self.model['prop_pitch'])/self.speed


        if delta_t is not None:
            delta = self.speed*delta_t
            self.longitude,self.latitude = geodesic.direct(self.longitude,self.latitude,self.heading,delta)
			#ADDED - CM -
			#ADDED FOR DEBUGGING
            #print("delta: %0.3f" % (delta))
            #print("delta_t: %0.3f , speed: %0.2f" % (delta_t,self.speed))
            self.x = self.x + delta*math.sin(self.heading)  
            self.y = self.y + delta*math.cos(self.heading)

        #self.roll = math.radians(math.sin(time.time()) * 2.5)
        #self.pitch = math.radians(math.sin(time.time()/2.1) * 5.0)
        
