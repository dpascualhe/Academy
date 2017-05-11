import numpy as np
import threading
import time
from datetime import datetime
import jderobot
import math
from Target import Target
from Parser import Parser

time_cycle = 120

class MyAlgorithm(threading.Thread):

    def __init__(self, cameraL, cameraR, pose3d, laser, motors):
        self.cameraL = cameraL
        self.cameraR = cameraR
        self.pose3d = pose3d
        self.laser = laser
        self.motors = motors

        self.imageRight=None
        self.imageLeft=None

        # Car direction
        self.carx = 0.0
        self.cary = 0.0

        # Obstacles direction
        self.obsx = 0.0
        self.obsy = 0.0

        # Average direction
        self.avgx = 0.0
        self.avgy = 0.0

        # Current target
        self.targetx = 0.0
        self.targety = 0.0

        self.stop_event = threading.Event()
        self.kill_event = threading.Event()
        self.lock = threading.Lock()
        threading.Thread.__init__(self, args=self.stop_event)

        # Init targets
        parser = Parser('targets.json')
        self.targets = parser.getTargets()

    def getNextTarget(self):
        for target in self.targets:
            if target.isReached() == False:
                return target

        return None

    def setRightImageFiltered(self, image):
        self.lock.acquire()
        self.imageRight=image
        self.lock.release()


    def setLeftImageFiltered(self, image):
        self.lock.acquire()
        self.imageLeft=image
        self.lock.release()

    def getRightImageFiltered(self):
        self.lock.acquire()
        tempImage=self.imageRight
        self.lock.release()
        return tempImage

    def getLeftImageFiltered(self):
        self.lock.acquire()
        tempImage=self.imageLeft
        self.lock.release()
        return tempImage

    def getCarDirection(self):
        return (self.carx, self.cary)

    def getObstaclesDirection(self):
        return (self.obsx, self.obsy)

    def getAverageDirection(self):
        return (self.avgx, self.avgy)

    def getCurrentTarget(self):
        return (self.targetx, self.targety)

    def run (self):

        while (not self.kill_event.is_set()):
           
            start_time = datetime.now()

            if not self.stop_event.is_set():
                self.execute()

            finish_Time = datetime.now()

            dt = finish_Time - start_time
            ms = (dt.days * 24 * 60 * 60 + dt.seconds) * 1000 + dt.microseconds / 1000.0
            #print (ms)
            if (ms < time_cycle):
                time.sleep((time_cycle - ms) / 1000.0)

    def stop (self):
        self.stop_event.set()

    def play (self):
        if self.is_alive():
            self.stop_event.clear()
        else:
            self.start()

    def kill (self):
        self.kill_event.set()


    def execute(self):
        # We get the destination coordinates.
        self.currentTarget=self.getNextTarget()
        self.targetx = self.currentTarget.getPose().x
        self.targety = self.currentTarget.getPose().y
        print("Target: ", self.targetx, self.targety)
        
        # We get the car coordinates and orientation.
        myx = self.pose3d.getX()/1000.
        myy = self.pose3d.getY()/1000.
        myyaw = -self.pose3d.getYaw()
        print("Car: ", myx, myy, myyaw)
        
        if ((myx + 2 >= self.targetx >= myx - 2) 
            and (myy + 2 >= self.targety >= myy - 2)):
            self.currentTarget.setReached(True)
            
        # We apply axis translation and rotation to get the relative 
        # target coordinates with respect to the car position.
        transx = self.targetx - myx
        transy = self.targety - myy
        rotx = (transx * math.cos(myyaw)) - (transy * math.sin(myyaw))
        roty = (transx * math.sin(myyaw)) + (transy * math.cos(myyaw))
        
        self.carx = rotx
        self.cary = roty
        print("Relative target: ", self.carx, self.cary)

        # We get the laser data.
        laser_data = self.laser.getLaserData()
        laser = []
        for i in range(laser_data.numLaser):
            distance = laser_data.distanceData[i]/1000.
            if distance < 5:
                yaw = math.radians(i)
                laser += [(distance, yaw)]
            
        # We enlarge the obstacles.
        car_radius = 1
        for i in range(len(laser)):
            data = list(laser[i])
            data[0] = data[0] - car_radius
            laser[i] = tuple(data)
            
        # Obstacles direction
        x = 0
        y = 0
        for i in range(len(laser)):
            x -= laser[i][0] * math.cos(laser[i][1])
            y -= laser[i][0] * math.sin(laser[i][1])
        
        self.obsx = x/len(laser)
        self.obsy = y/len(laser)
        print("Obstacles: ", self.obsx, self.obsy)
         
        # Average direction
        alfa = 0.15
        beta = 1
        car_module = math.sqrt((self.carx ** 2) + (self.cary ** 2))
        car_angle = math.atan2(self.cary, self.carx)
        if car_module > 3:
            car_module = 3
            self.carx = car_module*math.cos(car_angle)
            self.cary = car_module*math.sin(car_angle)

        self.avgx = alfa * self.carx + beta * self.obsx
        self.avgy = alfa * self.cary + beta * self.obsy
        
        module = math.sqrt((self.avgx ** 2) + (self.avgy ** 2))
        angle = math.atan(self.avgy / self.avgx)
        print("Avg: ", module, angle)
        
        v = module
        angle =  math.atan2(self.avgy, self.avgx)
        if -math.pi/2 + 0.1 >= angle >= -math.pi/2 - 0.1:
            w = 0
        else:
            w = (angle + math.pi/2)
            print(w)
            if w > 0.5:
                w = 1.5
            if w < -0.5:
                w = -1.5
        
        if (w == 1.5) or (w == -1.5):
            v = 1
        elif w == 0:
            v = 8
        else:
            v = (math.pi/2 - abs(w)) * 2

        # We set the car velocities.
        self.motors.setV(v)
        self.motors.setW(w)
        self.motors.sendVelocities()

        

