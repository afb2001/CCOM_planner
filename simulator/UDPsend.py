#!/usr/bin/env python
#Coral Moreno

import socket
import argparse

# Receive from command line desired state:
parser = argparse.ArgumentParser(description='''
./UDPsend.py -ds wpt_x,wpt_y,desired_speed 
units: wpt_x and wpt_y in [m] , desired_speed in [m/s].
This script send to the ASV the desired waypoint and speed via UDP client ('localhost', port 9000)
''')
parser.add_argument('-ds','--desired_state', dest='desired_asv_state',action='store',default='0,0,1.5',
			help='Desired ASV state (waypoint and speed): "x,y,speed" (m,m,m/s). Default is 0,0,1.5.')

parser.add_argument('-p','--port', dest='UDPport',action='store',default='9000',
			help='set UDP port to send the desired ASV state. Default is 9000.')
args = parser.parse_args()
#print(args)
#print(args.desired_asv_state)


#Send using UDP client desired state
s = socket.socket(family=socket.AF_INET, type=socket.SOCK_DGRAM)

#s.sendto("100,0,2.75".encode('utf-8'),('localhost',9000))
s.sendto(args.desired_asv_state.encode('utf-8'),('localhost',int(args.UDPport)))
print("sending wpt_x , wpt_y , desired_speed: ",args.desired_asv_state)
