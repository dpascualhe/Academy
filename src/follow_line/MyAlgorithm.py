import time
import threading
from datetime import datetime

import cv2
import numpy as np


time_cycle = 120

class MyAlgorithm(threading.Thread):

    def __init__(self, cameraL, cameraR, motors):
        self.cameraL = cameraL
        self.cameraR = cameraR
        self.motors = motors
        self.imageRight=None
        self.imageLeft=None
        self.stop_event = threading.Event()
        self.kill_event = threading.Event()
        self.lock = threading.Lock()
        threading.Thread.__init__(self, args=self.stop_event)
        
        self.first_frame = 1
        self.abs_center = 0
        self.derivator = 0

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
        im = self.cameraL.getImage() # Getting the images.

        print ("Running")

        # We "threshold" the HSV image to segment the line.
        im_hsv = cv2.cvtColor(im, cv2.COLOR_BGR2HSV)

        hsv_max = np.array([120, 255, 255])
        hsv_min = np.array([110, 245, 0])
        
        mask = cv2.inRange(im_hsv, hsv_min, hsv_max)
        im_line = cv2.cvtColor(mask, cv2.COLOR_GRAY2BGR)
        
        # We obtain the points that we're going to use as a
        # reference to check whether we are centered or not.
        lower_row = mask[360,:]
        upper_row = mask[280,:]

        if self.first_frame:
            self.abs_center = np.mean(np.where(lower_row == 255))
            self.first_frame = 0
        
        lower_indices = np.where(lower_row == 255)
        if lower_indices[0].any():
            tmp_lower_center = np.mean(lower_indices)
        else:
            tmp_lower_center = self.abs_center

        upper_indices = np.where(upper_row == 255)
        if upper_indices[0].any():
            tmp_upper_center = np.mean(upper_indices)
        else:
            tmp_upper_center = tmp_lower_center      

        # Normalizing error value.    
        error = self.abs_center - tmp_lower_center
        if error > 0:
            error /= mask.shape[1] - self.abs_center
        else:
            error /= self.abs_center

        # PD control.
        kp = 1
        kd = 8

        if error:
            print("error: " + str(error))
            p = kp * error
            d = kd * (error - self.derivator)
            
            w = p + d
        else:
            w = 0
        
        self.derivator = error
        
        if abs(w) > 0.4:
            v = abs((1-w)*2.2)
        elif abs(w) > 0.1:
            v = 4
        else:
            v = 7

        # Sending parameters to actuators.
        self.motors.setV(v)
        self.motors.setW(w)
        self.motors.sendVelocities()
        
        im_line[:,int(tmp_lower_center)] = [255, 0, 0]
        im_line[:,int(tmp_upper_center)] = [0, 0, 255]
        im_line[:,int(self.abs_center)] = [0, 255, 0]
        self.setLeftImageFiltered(im_line) # Displaying filtered image.


