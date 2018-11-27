#!/usr/bin/env python
# Coral Moreno, Val Schmidt and Roland Arsenault
# In this code the ASV receives heading and speed


import argparse
import dynamics
import cw4
import coastal_surveyor
import numpy as np
import threading
import time
import environment as ev
import depth_map
from bitarray import bitarray
# import matplotlib
import test
# matplotlib.use("TkAgg")
# import matplotlib.pyplot as plt
import socket
import datetime
#import matplotlib.animation as animation
# from matplotlib import rcParams, cycler


class DynamicObsSim:
    def __init__(self, start_x=0.0, start_y=0.0, start_heading=0.0, start_speed=0.0, nobs=4, xlim=1000.0, ylim=1000.0, plot_bool=False, file_world='',goal_location = '',environment = None, geotiff = None,boat_model = cw4.cw4,dynamic_obs=''):
        self.debug = False
        self.throttle = 0.0
        self.rudder = 0.0
        # Create an instance of the asv_sim model
        # Set parameters for the model.
        self.environment = environment
        self.boat_model = boat_model
        self.boat_dynamics = dynamics.Dynamics(self.boat_model, start_x, start_y,start_heading,start_speed,self.environment)

        # Set the starting state
        self.start_lat = 43.0
        self.start_long = -70.1
        self.start_heading = start_heading
        self.start_speed = start_speed

        # Set target waypoint
        self.desired_lat = 42.997797    # Does it need to be updated from somewhere else?
        self.desired_long = -70.102602   # Does it need to be updated from somewhere else?
        self.wpt_x = start_x  # 0.0
        self.wpt_y = start_y  # 0.0

        # deg to rad
        self.desired_lat = np.radians(self.desired_lat)
        self.desired_long = np.radians(self.desired_long)

        # set target speed
        self.desired_speed = self.start_speed
        self.desired_heading = self.start_heading

        # current state:
        self.curr_lat = self.start_lat
        self.curr_long = self.start_long
        self.curr_x = start_x
        self.curr_y = start_y
        self.curr_heading = self.start_heading

        # Loop at some rate
        self.period = 0.05

        self.last_update_time = None

        self.xx = []
        self.yy = []

        # Define the socket for input data...
        self.soc = socket.socket(family=socket.AF_INET, type=socket.SOCK_DGRAM)

        # Define the socket for output data - ASV current state
        self.soc_asv_out = socket.socket(
            family=socket.AF_INET, type=socket.SOCK_DGRAM)

        # Define the socket for output data - Obstacles current state
        self.soc_obs_out = socket.socket(
            family=socket.AF_INET, type=socket.SOCK_DGRAM)

        # Limits of the area
        self.xlim = np.abs(xlim)
        self.ylim = np.abs(ylim)

        # read the grid and update the boat model, limit, curr_x, curr_y, wpt_x, wpt_y
        self.static_obs = None
        self.static_draw = []
        self.goal_location = []
        self.factor = 1
        self.read_world(file_world,geotiff, goal_location)

        
        if dynamic_obs:
            self.read_dynamic(dynamic_obs)
        else:
            # Initializing obstacles:
            self.nobs = nobs  # number of obstacles

            self.xobs = np.random.uniform(-self.xlim, self.xlim, self.nobs)
            self.yobs = np.random.uniform(-self.ylim, self.ylim, self.nobs)
            # speed of obstacles [m/sec]
            self.vobs = np.random.uniform(2.57, 7.72, self.nobs)
            # heading of obstacles [rad]
            self.hobs = np.random.uniform(0, 2*np.pi, self.nobs)
            self.xx_obs = []
            self.yy_obs = []
            self.obs_id = np.arange(self.nobs)
            #self.cmap = plt.cm.summer
            #rcParams['axes.prop_cycle'] = cycler(color=self.cmap(np.linspace(0, 1, self.nobs)))
            # self.obstacles = np.zeros(self.nobs, dtype=[('position',float,2),
            #											('id', float, 1),
            #											('color',float, 4)])

            #futrue path

        self.future_heading = []
        self.future_x = []
        self.future_y = []
        self.estimateStart = (0,0,0)

        # Plot
        self.plot_boolean = plot_bool
        # Verifying self.plot_boolean is of type Boolean and not str:
        if self.plot_boolean == 'True':
            self.plot_boolean = True
        if self.plot_boolean == 'False':
            self.plot_boolean = False

        if self.debug:
            print("Initial plot_boolean = ", self.plot_boolean)
        # plt.ion()
        # # Set up the figure
        # fig = plt.figure()
        # plt.xlabel("x [m]")
        # plt.ylabel("y [m]")
        self.plotaxes = None
        self.plot_draw = test.PLOT(self.static_obs,self.static_draw,self.xlim,self.ylim,self.factor)
        self.plotaxes_obs = list(range(self.nobs))






    # read the grid and update the boat model, limit, curr_x, curr_y, wpt_x, wpt_y
    def read_dynamic(self,dynamics_file):
        f=open(dynamics_file, "r")
        f1 = f.readline()

        self.xobs  = []
        self.yobs  = []
        self.vobs  = []
        self.hobs  = []
        self.nobs = int(f1)  # number of obstacles
        self.obs_id = np.arange(self.nobs)

        for i in range(self.nobs):
            f1 = f.readline()
            while f1.strip() == '':
                f1 = f.readline()
            f2 = f1.split(' ')
            self.xobs.append(float(f2[0]))
            self.yobs.append(float(f2[1]))
            self.vobs.append(float(f2[2]))
            self.hobs.append(float(f2[3]))
        self.xobs = np.array(self.xobs)
        self.yobs = np.array(self.yobs)
        self.vobs = np.array(self.vobs)
        self.hobs = np.array(self.hobs)
            

    def read_world(self,file_world,geotiff,goal_location):
        maxx = 0
        maxy = 0
        if  geotiff:
            geotiff = depth_map.BathyGrid(geotiff)
            x1,x2,y1,y2 = geotiff.getBound()
            maxx = np.abs(x2-x1)
            maxy = np.abs(y2-y1)
            geodata = geotiff.getGrid()
            by,bx = np.shape(geodata)
            previous = None
            self.xlim = max(bx,by)
            self.ylim = self.xlim
            self.factor = maxx / bx 
            self.static_obs = [bitarray(by) for i in xrange(bx)]
            for j in range(by):
                for i in range(bx):    
                    if geodata[j,i] < 2:
                        self.static_obs[i ][by - 1 -j] = True;
                        if previous == None:
                            previous = ((i , (by - 1 -j) ))
                    else:
                        self.static_obs[i ][by - 1 -j] = False;
                        if previous != None:
                            x,y = previous
                            self.static_draw.append((previous,(i,y+1) ))
                            previous = None
                if previous != None:
                            x,y = previous
                            self.static_draw.append((previous,(bx,y+1) ))
                            previous = None
                    
            self.boat_dynamics = dynamics.Dynamics(self.boat_model, self.curr_x, self.curr_y,self.curr_heading,self.start_speed,self.environment)

        elif file_world != '':
            f=open(file_world, "r")
            f1 = f.readlines()
            previous = None
            
            for i in range(0, len(f1)):
                    if  i == 1 :
                        self.xlim = np.abs(int(int(f1[i])))
                        maxx = np.abs(int(int(f1[i])))
                    elif i == 2:
                        self.ylim = np.abs(int(int(f1[i])))
                        maxy = np.abs(int(int(f1[i])))
                        self.static_obs = [bitarray(maxy) for i in xrange(maxx)]
                        print maxx,maxy
                    elif i == 0:
                        self.factor = np.abs(int(int(f1[i])))
                    else:
                        string = f1[i].strip()
                        for j in range(0, len(string)):
                            if f1[i][j] == '#':
                                self.static_obs[j][maxy - i + 2] = True;
                                if previous == None:
                                    previous = ((j , maxy - i + 2 ))
                            else:
                                self.static_obs[j][maxy - i + 2] = False;
                                if previous != None:
                                    x,y = previous
                                    self.static_draw.append(( previous,(j ,y+1)))
                                    previous = None
                        if previous != None:
                            x,y = previous
                            self.static_draw.append((previous,(maxx,y+1) ))
                            previous = None
            self.boat_dynamics = dynamics.Dynamics(self.boat_model, self.curr_x, self.curr_y,self.curr_heading,self.start_speed,self.environment)
        if goal_location != '':
            f=open(goal_location, "r")
            f1 = f.readlines()
            numOfGoal = int(f1[0])
            for i in range(1, len(f1)):
                s = f1[i].split(' ')
                self.goal_location.append((float(s[0]),float(s[1])))



    def heading_to_rudder(self, heading):
        #heading_delta = data.heading.heading - current_heading
        self.heading_delta = self.desired_heading - self.curr_heading
        if self.heading_delta > np.pi:
            self.heading_delta -= np.pi*2.0
        if self.heading_delta < -np.pi:
            self.heading_delta += np.pi*2.0
        self.rudder = max(-1.0, min(1.0, self.heading_delta))
        return self.rudder

    def speed_to_throttle(self, speed):
        #throttle = speed / dynamics.model["max_speed"]
        self.throttle = speed / self.boat_dynamics.model["max_speed"]
        return self.throttle

    def g_tick(self, period):
        t = time.time()
        count = 0
        while True:
            count += 1
            timetoexecute = t + count*period
            timetosleeep = max(t + count*period - time.time(), 0)
            yield timetoexecute, timetosleeep

    def do_every(self, period, f, *args):
        g = self.g_tick(period)
        while True and getattr(threading.current_thread(), 'do_run'):
            self.update_time, self.sleeptime = next(g)
            time.sleep(self.sleeptime)
            f(*args)
            
            

    # def hello(self,s):   #TEST
    #	print('hello {} ({:.4f})'.format(s,time.time()))
    #	time.sleep(.3)

    def update_asv(self):
        # deg to rad
        self.curr_lat = np.radians(self.curr_lat)
        self.curr_long = np.radians(self.curr_long)

        #        Calculate desired heading from current position and desired position.
        #	 Referene: https://www.igismap.com/formula-to-find-bearing-or-heading-angle-between-two-points-latitude-longitude/

        #X = np.cos(desired_lat)*np.sin(desired_long-curr_long)
        #Y = np.cos(curr_lat)*np.sin(desired_lat)-np.sin(curr_lat)*np.cos(desired_lat)*np.cos(desired_long-curr_long)

        #self.desired_heading = np.arctan2(self.wpt_x-self.curr_x,self.wpt_y-self.curr_y)
        if self.debug:
            print("desired_x: %0.2f , desired_y: %0.2f" %
                  (self.wpt_x, self.wpt_y))
            print("desired heading: %0.2f " % (self.desired_heading))

        #        Calculate rudder/throttle from desired heading and speed.
        # self.rudder = self.heading_to_rudder(self.desired_heading)
        # self.throttle = self.speed_to_throttle(self.desired_speed)
        if self.debug:
            print("rudder: %0.2f , throttle: %0.2f" %
                  (self.rudder, self.throttle))
        #    Update the model
        self.boat_dynamics.update(self.throttle, self.rudder, self.update_time)
        self.curr_x = self.boat_dynamics.x
        self.curr_y = self.boat_dynamics.y
    
        self.curr_heading = self.boat_dynamics.heading
       
        self.curr_speed = self.boat_dynamics.speed
        self.curr_lat = self.boat_dynamics.longitude
        self.curr_long = self.boat_dynamics.latitude

    def get_new_waypoint(self):
        ''' Monitor for updates to wpts/speed '''

        try:
            data = self.soc.recv(4096)
            if(data[:4]== 'path'):
                n = int(data.split(' ')[1])
                self.future_x = []
                self.future_y = []
                self.future_heading = []
                for i in range(0,n):
                    data = self.soc.recv(4096)
                    data1 = data.split(' ');
                    self.future_x.append(float(data1[0]))
                    self.future_y.append(float(data1[1]))
                    self.future_heading.append(float(data1[2]))
                data = self.soc.recv(4096)
                data1 = data.split(' ');
                self.estimateStart = (float(data1[0]),float(data1[1]),float(data1[2]))
                data = self.soc.recv(4096)
                print('***************', data, '*******************')
                dummy = data.decode('utf-8').split(",")
                self.rudder = float(dummy[0])
                self.throttle = float(dummy[1])
        except socket.timeout as e:
            pass

    def send_asv_state_UDP(self):
        ''' Sending the state of the ASV to the world
                data = (current_x , current_y , current_speed , current_heading , time) 
                units of data: (m , m , m/s , rad , sec)
                sent over UDP client:
                host = 'localhost'
                port = 9012 '''
        data = str("%f,%f,%f,%f,%f" % (self.curr_x, self.curr_y,
                                       self.curr_speed, self.curr_heading, self.update_time))
        self.soc_asv_out.sendto(data.encode('utf-8'), ('localhost', 9012))

    def send_obs_state_UDP(self):
        ''' Sending the state of the obstacles to the world 
                data = (obs_ID, curr_obs_x , curr_obs_y , curr_obs_speed , curr_obs_heading , time) 
                units of data: (m , m , m/s , rad , sec)
                sent over UDP client:
                host = 'localhost'
                port = 9020 '''
        data = ""
        for i in range(self.nobs):
            data = data+str("%i,%f,%f,%f,%f,%f\n" % (
                self.obs_id[i], self.xobs[i], self.yobs[i], self.vobs[i], self.hobs[i], self.update_time))
        self.soc_obs_out.sendto(data.encode('utf-8'), ('localhost', 9020))

    def update(self):
        ''' Update the state of the world. '''
        self.get_new_waypoint()
        self.update_asv()
        self.update_obs()
        
        # Insert here --> send UPD messages to world ---> make it independent method
        self.send_asv_state_UDP()
        self.send_obs_state_UDP()

        #    Report the new position ----> need to publish!!!
        if self.debug:
            self.print_state()

        # Update plot
        if self.debug:
            print("plot_boolean in update = ", self.plot_boolean)

        # if self.plot_boolean:
        #     if self.debug:
        #         print("plot_boolean in update inside if = ", self.plot_boolean)
        #     self.plot()
        

    def print_state(self):
        print("x: %0.2f, y: %0.2f, s: %0.2f , h: %0.2f" %
              (self.curr_x, self.curr_y, self.curr_speed, self.curr_heading))

    # def mypause(self, interval):
    #     # reference: https://stackoverflow.com/questions/45729092/make-interactive-matplotlib-window-not-pop-to-front-on-each-update-windows-7
    #     # https://github.com/matplotlib/matplotlib/issues/11131
    #     backend = plt.rcParams['backend']
    #     if backend in matplotlib.rcsetup.interactive_bk:
    #         figManager = matplotlib._pylab_helpers.Gcf.get_active()
    #         if figManager is not None:
    #             canvas = figManager.canvas
    #             if canvas.figure.stale:
    #                 canvas.draw()
    #             canvas.start_event_loop(interval)
    #             return

    def plot(self):  # can also function as plot_init for animation.FuncAnimation()
        # ASV:
        self.xx.append(self.curr_x)
        self.yy.append(self.curr_y)
        # Dynamic obstacles:
        '''		
		self.xx_obs.append(self.xobs)
		self.yy_obs.append(self.yobs)
		

		if self.xx_obs.__len__() > 10:
			self.xx_obs.pop(0)
			self.yy_obs.pop(0)
		'''

        if self.xx.__len__() > 20:
            self.xx.pop(0)
            self.yy.pop(0)


        if self.plotaxes == None:
            # start_time = time.time()
            self.plot_draw.updateInformation(self.curr_x, self.curr_y, self.curr_heading, self.nobs, self.xobs, self.yobs, self.hobs,self.future_x,self.future_y,self.future_heading,self.estimateStart)
            self.plot_draw.update()
            # print("--- %s seconds ---" % ((time.time() - start_time)*1000))
            # self.plotaxes = plt.plot(self.xx, self.yy, ".b")

            # # plt.hold(True)
            # self.plotaxes_wpt = plt.plot(self.wpt_x, self.wpt_y, '+r')
            # for ind in range(self.nobs):
            #     self.plotaxes_obs[ind] = plt.plot(
            #         self.xobs[ind], self.yobs[ind], 'ok')
            # # plt.hold(False)
            # plt.axis([-self.xlim, self.xlim, -self.ylim, self.ylim])
            # plt.grid(True)
            # plt.draw()
            # self.mypause(.001)
            # plt.show(block=False)

        else:
            pass
            # self.plotaxes[0].set_xdata(self.xx)
            # self.plotaxes[0].set_ydata(self.yy)
            # # plt.hold(True)
            # self.plotaxes_wpt[0].set_xdata(self.wpt_x)
            # self.plotaxes_wpt[0].set_ydata(self.wpt_y)
            # for ind in range(self.nobs):
            #     self.plotaxes_obs[ind][0].set_xdata(self.xobs[ind])
            #     self.plotaxes_obs[ind][0].set_ydata(self.yobs[ind])
            # # plt.hold(False)
            # self.mypause(.001)

        # plt.draw()
        # plt.pause(.001)
        # plt.show(block=False)
    '''	
	def plot_init(self):
		self.plotaxes = plt.plot(self.xx,self.yy,".b")
		plt.hold(True)
		plt.plot(self.wpt_x,self.wpt_y,'+')
		plt.hold(False)
		plt.axis([-200,200,-200,200])

	def plot_update(self):
		self.xx.append(self.curr_x)
		self.yy.append(self.curr_y)

		if self.xx.__len__() > 20:
			self.xx.pop(0)
			self.yy.pop(0)

		self.plotaxes[0].set_xdata(self.xx)
		self.plotaxes[0].set_ydata(self.yy)
	'''

    

    def update_obs(self):
        if self.last_update_time is None:
            self.last_update_time = self.update_time

        select = self.plot_draw.selects()
        for key, value in select.iteritems():
            self.hobs[key] = value[0]
            self.vobs[key] = value[1]
            
        
        self.tobs = self.update_time - self.last_update_time  # time [sec]
        self.delta_obs = self.vobs*self.tobs  # distance [m]
        self.xobs = self.xobs + self.delta_obs * \
            np.sin(self.hobs)  # x position of obstacles
        self.yobs = self.yobs + self.delta_obs * \
            np.cos(self.hobs)  # y position of obstacles
        for ii in range(self.nobs):
            if self.xobs[ii] > self.xlim:
                self.xobs[ii] = self.xobs[ii] - 2*self.xlim
            if self.xobs[ii] < -self.xlim:
                self.xobs[ii] = self.xobs[ii] + 2*self.xlim
            if self.yobs[ii] > self.ylim:
                self.yobs[ii] = self.yobs[ii] - 2*self.ylim
            if self.yobs[ii] < -self.ylim:
                self.yobs[ii] = self.yobs[ii] + 2*self.ylim

        self.last_update_time = self.update_time

    def update_th(self):
        self.do_every(self.period, self.update)
    

    def run(self):
        # self.do_every(1.0,self.hello,'foo')		#TEST
        
    
        if self.plot_boolean:
            self.plot_draw.on_execute(self.curr_x, self.curr_y, self.start_heading, self.nobs, self.xobs, self.yobs, self.hobs)
            self.plot_draw.update()
        
        update_thread = threading.Thread(target=self.update_th)
        update_thread.do_run = True
        update_thread.start() 

        try:
            if self.plot_boolean:
                while True:
                    self.plot_draw.updateInformation(self.curr_x, self.curr_y, self.curr_heading, self.nobs, self.xobs, self.yobs, self.hobs,self.future_x,self.future_y,self.future_heading,self.estimateStart,self.goal_location)
                    self.plot_draw.update()
                    time.sleep(0.05)
                update_thread.join(timeout=1000)
            else:
                update_thread.join(timeout=1000)
        finally:
            update_thread.do_run = False
            if self.plot_boolean:
                self.plot_draw.stop()
            update_thread.join(timeout=1000)
            
        
        

        '''		
		# WHERE TO PUT THIS ?!?
		#animation = animation.FuncAnimation(fig,self.update,frames = self.plot_update_time,init_func = self.plot_init,interval=self.sleeptime)
		#plt.show()
		'''


if __name__ == '__main__':

    parser = argparse.ArgumentParser(description=("This is a simulation of an ASV model in the presence of dynamic obstacles in its environment."
                                                  "The following operations are available for this simulation:"
                                                  "1)Set initial state for the ASV via the command line. "
                                                  "2)The simulation accepts via UDP server the desired state of the ASV -"
                                                  "Data: (wpt_x,wpt_y,speed), units: (m,m,m/s), Note: speed was not set to be negative."
                                                  "Host:'localhost', Port:'9000'. "
                                                  "3)The simulation sends the current state of the ASV via UDP client. "
                                                  "Host:'localhost', Port:'9012'. "
                                                  "Data: (current_x , current_y , current_speed , current_heading , time),"
                                                  " units of data: (m , m , m/s , rad , sec)"
                                                  "4)The simulation sends the current state of the obstacles via UDP client. "
                                                  "Host:'localhost', Port:'9020'. "
                                                  "Data = (obs_ID, curr_obs_x , curr_obs_y , curr_obs_speed , curr_obs_heading , time), "
                                                  "units of data: (integer , m , m , m/s , rad , sec)"))

    # Set ASV initial state:

    # No point of using heading in the initial state since the simultion behaves the same.
    parser.add_argument('-a', '--asv', dest='initial_asv_state', action='store', default='0,0,0,0',
                        help='Initial ASV state: "x,y,speed,heading" (m,m,m/s,rad). Default is 0,0,0,0')  # ,default='0,0,0,0' <-- add to the help
    
    parser.add_argument('-af', '--asvfile', dest='initial_asv_state_file', action='store', default='',
                        help='File of Initial ASV state: "x,y,speed,heading" (m,m,m/s,rad). Default is 0,0,0,0')
    '''
	parser.add_argument('-a','--asv', dest='initial_asv_state',action='store',default='0,0,0',	
			help='Initial ASV state: "x,y,speed" (m,m,m/s). Default is 0,0,0') 
	'''
    parser.add_argument('-nobs', '--n_obstacles', dest='nobs', action='store', default='4',
                        help='Set number of obstacles. Default is 4 ')

    parser.add_argument('-l', '--limits', dest='lim', action='store', default='1000,1000',
                        help='Set limits of the area. The area is centered around the origin. [-xlim,xlim]X[-ylim,ylim]. xlim can be different than ylim. Default is 1000,1000 [m] ')

    parser.add_argument('-p', '--plot', dest='plot', action='store', default='False',
                        help='Generates a plot of the simulation. Receives True for plotting or False for not plotting. Default is False')

    parser.add_argument('-m', '--map', dest='file', action='store', default='',
                        help='File for Grid world')

    parser.add_argument('-g', '--goal', dest='goal', action='store', default='',
                        help='File for goal location')
    parser.add_argument('-e', '--environment', dest='environment', action='store', default='False',
                        help='File for enviroment affect, either true with defalt 1,90 or true,[speed],[direction]')

    parser.add_argument('-model', '--boatmodel', dest='model', action='store', default='cw4',
                        help='File for name for model')

    parser.add_argument('-tiff', '--geotiff', dest='geotiff', action='store', default='',
                        help='File for geo tiff')
    
    parser.add_argument('-dynamic', '--dynamicobs', dest='dynamicobs', action='store', default='',
                        help='File for dynamic obstacle')

    # Handle arguments
    args = parser.parse_args()

    if args.initial_asv_state_file:
        fields=open(args.initial_asv_state_file, "r").readline().split(',')
    else:
        fields = args.initial_asv_state.split(',')

    print("Initial ASV state, (x,y,speed,heading): ", fields)

    start_x = float(fields[0])
    start_y = float(fields[1])
    start_speed = float(fields[2])
    start_heading = float(fields[3])

    nobs = int(args.nobs)
    # print(nobs)
    limits = args.lim.split(',')
    xlim = float(limits[0])
    ylim = float(limits[1])

    environment = None
    if args.environment.lower() != 'false':
        data = args.environment.lower().split(',')
        if(len(data) > 1):
            print data[2]
            try:
                environment = ev.Environment(float(data[1]),float(data[2]))
            except:
                print "WRONG argument, No current affect is given"
        else:
            environment = ev.Environment()
        

    boat_model = args.model

    dynamic_obs = args.dynamicobs

    if boat_model.lower() == "coastal_surveyor" :
        boat_model = coastal_surveyor.coastal_surveyor
    else:
        boat_model = cw4.cw4



    plot_bool = args.plot

    file_world = args.file

    geotiff = args.geotiff

    goal_location = args.goal
    '''
	print("plot_bool = ",plot_bool)
	print("Type of plot_bool = ",type(plot_bool))
	'''
    # Setup simulation
    sim = DynamicObsSim(start_x, start_y, start_heading,
                        start_speed, nobs, xlim, ylim, plot_bool,file_world,goal_location,environment,geotiff,boat_model,dynamic_obs)
    '''
	# Example - Con'd:
	sim.curr_x = float(fields[0])
	sim.curr_y = float(fields[1])
	sim.curr_speed = float(fields[2])
	sim.curr_heading = float(fields[3])
	'''

    # Receive via UDP server the desired ASV state

    sim.host = 'localhost'
    sim.port = 9000

    try:
        sim.soc.settimeout(0.015)  # needs to be lower then self.period
        sim.soc.bind((sim.host, sim.port))
    except socket.error as e:
        exit("unable to bind to: %s:%s" % (sim.host, sim.port))
    print("socket to receive desired ASV state: %s %s" % sim.soc.getsockname())

    # Send to UDP server (via UDP client) the current ASV state
    sim.soc_asv_out.settimeout(0.015)  # needs to be lower then self.period

    # Send to UDP server (via UDP client) the current Obstacles states
    sim.soc_obs_out.settimeout(0.015)

    try:
        sim.run()
    except KeyboardInterrupt:
        sim.soc.close()
        sim.soc_asv_out.close()
        sim.soc_obs_out.close()


##################################################################################################
