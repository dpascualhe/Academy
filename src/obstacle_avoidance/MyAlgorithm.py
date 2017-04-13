import numpy as np
import threading
import time
from datetime import datetime
import jderobot
import math
from Target import Target
from Parser import Parser

time_cycle = 80

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
        
        # We get the car coordinates and orientation.
        myx = self.pose3d.getX()/1000.
        myy = self.pose3d.getY()/1000.
        myyaw = self.pose3d.getYaw()
        
        
        # We get the laser data.
        laser_data = self.laser.getLaserData()
        laser = []
        i = 0
        for i in range(laser_data.numLaser):
            distance = laser_data.distanceData[i]/1000.
            yaw = math.radians(i)-math.pi/2
            laser += [(distance, yaw)]
            i += 1            
        
        print("Car coordinates: " + str(myx) + ", " + str(myy))
        print("Car position: " + str(myyaw))
        print("Target coordinates: " + str(self.targetx) + ", "
              + str(self.targety))
        print("Laser data: " + str(laser))
        
        v = 3
        w = 0
        # We set the car velocities.
        self.motors.setV(v)
        self.motors.setW(w)

        

