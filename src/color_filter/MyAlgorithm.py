import threading
import time
from datetime import datetime
import cv2
import numpy as np

from sensors.cameraFilter import CameraFilter
from parallelIce.navDataClient import NavDataClient
from parallelIce.cmdvel import CMDVel
from parallelIce.extra import Extra
from parallelIce.pose3dClient import Pose3DClient


time_cycle = 80

class MyAlgorithm(threading.Thread):

    def __init__(self, camera, navdata, pose, cmdvel, extra):
        self.camera = camera
        self.navdata = navdata
        self.pose = pose
        self.cmdvel = cmdvel
        self.extra = extra

        self.stop_event = threading.Event()
        self.kill_event = threading.Event()
        self.lock = threading.Lock()
        threading.Thread.__init__(self, args=self.stop_event)


    def run (self):

        self.stop_event.clear()

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
        # We get the image 
        im = self.camera.getImage()

        if im is not None:
            # We blur the image in order to avoid noise
            kernel = (5, 5)
            im_blur = cv2.GaussianBlur(im, kernel, 0)
            
            # We convert the image to HSV color space to avoid a negative
            # influence of the variable ilumination
            im_hsv = cv2.cvtColor(im_blur, cv2.COLOR_BGR2HSV)
            
            # We "threshold" the HSV image to segment the red ball
            if self.video == "drone1":
                hsv_max_red = np.array([180, 255, 205])
                hsv_min_red = np.array([100, 135, 76])
            else:    
                hsv_max_red = np.array([140, 255, 215])
                hsv_min_red = np.array([120, 230, 115])
            
            im_filter_red = cv2.inRange(im_hsv, hsv_min_red, hsv_max_red)
            im_copy_red = np.copy(im_filter_red)            
            
            # We "threshold" the HSV image to segment the blue ball
            im_filter_blue = im_filter_red
            if self.video == "pelotas_roja_azul":                
                hsv_max_blue = np.array([100, 255, 190])
                hsv_min_blue = np.array([0, 80, 60])
                im_filter_blue = cv2.inRange(im_hsv, hsv_min_blue, hsv_max_blue)
            im_copy_blue = np.copy(im_filter_blue)            
            
            # We display the b&w image with the segmented objects
            self.camera.setThresholdImage(im_filter_red+im_filter_blue)
            
            # We find the contours and draw the detected objects
            im_cont_red, cont_red, h_red= cv2.findContours(im_copy_red,cv2.RETR_TREE, 
                                       cv2.CHAIN_APPROX_SIMPLE)

            for i in cont_red:
                if cv2.contourArea(i)>= 100 and cv2.contourArea(i)<= 1000:
                    xr,yr,wr,hr = cv2.boundingRect(i)
                    cv2.rectangle(im,(xr,yr),(xr+wr,yr+hr),(0,255,0),2)
                    print("Red ball coordinates: x=" + str((2*xr + wr)/2) + "; y="
                          + str((2*yr + hr)/2))
            
            # im_cont_blue, cont_blue, hier_blue = cv2.findContours(im_copy_blue,
                                                               #- cv2.RETR_TREE,
                                                               # cv2.CHAIN_APPROX_SIMPLE)
            #---------------------- xb,yb,wb,hb = cv2.boundingRect(im_cont_blue)
#------------------------------------------------------------------------------ 
            #----------------- # We calculate the center of the detected regions
            #--------------- cv2.rectangle(im,(xb,yb),(xb+wb,yb+hb),(0,255,0),2)
#------------------------------------------------------------------------------ 
            #--- print("Blue ball coordinates: x=" + str((2*xb + wb)/2) + "; y="
                  #--------------------------------------- + str((2*yb + hb)/2))
#------------------------------------------------------------------------------ 
            # We display the color image
            self.camera.setColorImage(im)       
            '''
