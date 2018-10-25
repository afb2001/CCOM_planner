#!/usr/bin/env python
# Alex Brown
# Shim to interface between the simulator and the
# rest of the project.

import socket
import threading
import fileinput
import sys
import os
import signal

UDP_ASV_PORT = 9012
UDP_OBS_PORT = 9020
UDP_CTRL_PORT = 9000

# listen for vehicle state on port 9012

s1 = socket.socket(family=socket.AF_INET, type=socket.SOCK_DGRAM)
s2 = socket.socket(family=socket.AF_INET, type=socket.SOCK_DGRAM)
s3 = socket.socket(family=socket.AF_INET, type=socket.SOCK_DGRAM)


def recv_asv():
    # s1 = socket.socket(family=socket.AF_INET, type=socket.SOCK_DGRAM)
    s1.bind(('localhost', UDP_ASV_PORT))
    print('Starting to listen for vehicle state on port %d' % UDP_ASV_PORT)
    while getattr(threading.current_thread(), 'do_run'):
        data = s1.recv(4096)
        data = data.decode('utf-8')
        print("Location:\n%s [%s] " % (data, len(data)))
        print("\0")
        sys.stdout.flush()



# listen for obstacle states on port 9020
def recv_obs():
    # s2 = socket.socket(family=socket.AF_INET, type=socket.SOCK_DGRAM)
    s2.bind(('localhost', UDP_OBS_PORT))
    print('Starting to listen for obstacle states on port %d' % UDP_OBS_PORT)
    while getattr(threading.current_thread(), 'do_run'):
        data1 = s2.recv(4096)
        data1 = data1.decode('utf-8')
        print("Obstacle: \n%s [%s] " % (data1, len(data1)))
        print("\0")
        sys.stdout.flush()


# listen for control updates on stdin
# each line should be given as heading,speed
def recv_control():
    # s3 = socket.socket(family=socket.AF_INET, type=socket.SOCK_DGRAM)
    # for line in fileinput.input():
    while True:
        line = raw_input()
        if(line[:4] == 'path'):
            sending = line + '\n'
            s3.sendto(sending.encode('utf-8'), ('localhost', UDP_CTRL_PORT))
            count = int(line.split(' ')[1])
            while count != 0:
                sending = raw_input() + '\n'
                s3.sendto(sending.encode('utf-8'), ('localhost', UDP_CTRL_PORT))
                count -= 1
            sending = raw_input() + '\n'
            s3.sendto(sending.encode('utf-8'), ('localhost', UDP_CTRL_PORT))
            line = raw_input()
            s3.sendto(line.encode('utf-8'), ('localhost', UDP_CTRL_PORT))
    sys.stderr.write('EOF encountered.')

# open the pipe to communicate with executive
def open_executive():
    pread, cwrite = os.pipe()
    cread, pwrite = os.pipe()
    processid = os.fork()
    if processid == 0:
        os.dup2(pread, 0)
        os.dup2(pwrite, 1)
        os.close(pread)
        os.close(pwrite) 
        os.close(cread)
        os.close(cwrite)
    else:
        os.dup2(cread, 0)
        os.dup2(cwrite, 1)
        os.close(pread)
        os.close(pwrite)
        os.close(cread)
        os.close(cwrite)
        if(len(sys.argv) > 2):
            os.execl('executive', 'executive', sys.argv[1], sys.argv[2])
        elif(len(sys.argv) > 1):
            os.execl('executive', 'executive', sys.argv[1])
        else:
            os.execl('executive', 'executive')
        sys.exit(0)


if __name__ == "__main__":
    # start up threads to listen on UDP ports
    open_executive()
    asv_thread = threading.Thread(target=recv_asv)
    asv_thread.do_run = True
    asv_thread.start()
    obs_thread = threading.Thread(target=recv_obs)
    obs_thread.do_run = True
    obs_thread.start()
    try:
        recv_control()
    finally:
        sys.stderr.write('Shutting down the state listeners\n')
        obs_thread.do_run = False
        asv_thread.do_run = False
        s1.close()
        s2.close()
        s3.close()
        os._exit(0)

    obs_thread.do_run = False
    asv_thread.join(timeout=1000)
    asv_thread.do_run = False
    obs_thread.join(timeout=1000)
    os._exit(0)
