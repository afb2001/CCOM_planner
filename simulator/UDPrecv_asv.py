#!/usr/bin/env python
#Coral Moreno 

import socket
import sys
s = socket.socket(family=socket.AF_INET, type=socket.SOCK_DGRAM)
s.bind(('localhost', 9012))
 
while True:
     #data, address = s.recvfrom(4096)
     data = s.recv(4096)	
     data = data.decode('utf-8')
     #print("rx: %s [%s] from %s" % (data, len(data), address))
     print("rx: %s [%s] " % (data, len(data)))
     sys.stdout.flush()
