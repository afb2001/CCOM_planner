#!/usr/bin/env python
#Coral Moreno

import socket
import argparse

# Receive from command line desired state:
parser = argparse.ArgumentParser(description='''
./UDPsend_hs.py -ds desired_heading,desired_speed 
units: desired_heading in [rad] , desired_speed in [m/s].
This script send to the ASV the desired waypoint and speed via UDP client ('localhost', port 9000). 
Should work with the simulation in the file: dynamic_obs_sim_2.py
''')
parser.add_argument('-ds','--desired_state', dest='desired_asv_state',action='store',default='0,1.5',
			help='Desired ASV state (heading and speed): "heading,speed" (rad,m/s). Default is 0,1.5.')

parser.add_argument('-p','--port', dest='UDPport',action='store',default='9000',
			help='set UDP port to send the desired ASV state. Default is 9000.')
args = parser.parse_args()
#print(args)
#print(args.desired_asv_state)


#Send using UDP client desired state
s = socket.socket(family=socket.AF_INET, type=socket.SOCK_DGRAM)

#s.sendto("1.5,2.75".encode('utf-8'),('localhost',9000))
s.sendto(args.desired_asv_state.encode('utf-8'),('localhost',int(args.UDPport)))
print("sending desired_heading , desired_speed: ",args.desired_asv_state)
