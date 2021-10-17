
##
#   This module defines a spatial filter based off of the standard Kalman filter.
#
#   @author Geoffrey Clark <gmclark1@asu.edu>

import numpy as np
import cv2
width=3264
height=2464
flip=21
disp_width=800
disp_height=600
camSet='nvarguscamerasrc sensor-id=0 ! video/x-raw(memory:NVMM), width='+str(width)+', height='+str(height)+', framerate='+str(flip)+'/1,format=NV12 ! nvvidconv flip-method=2 ! video/x-raw, width='+str(disp_width)+', height='+str(disp_height)+', format=BGRx ! videoconvert ! video/x-raw, format=BGR ! appsink'

cam=cv2.VideoCapture(camSet)
while True:
    _, frame = cam.read()
    cv2.imshow('mycam',frame)
    cv2.moveWindow('mycam',0,0)
    if cv2.waitKey(1)==ord('q'):
        break
cam.release()
cv2.destroyAllWindows


