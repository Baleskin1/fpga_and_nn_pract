import cv2
import matplotlib.pyplot as plt
import numpy as np
from time import sleep
#import io
from RPi import GPIO as io

io.setmode(io.BOARD)
freq = 50
dc_min = 2.5
dc_max = 12.5
dc = dc_min
delta = 0
servo = 12
led = 10
x_mid = 240
eps = 5 # (error margin)
eta = 0.1 # step modifier
io.setup(servo, io.OUT)
io.setup(led, io.OUT)

sg90 = io.PWM(servo, freq)
sg90.start(dc)

#use sudo modprobe bcm2835-v4l2 before first launch
cam = cv2.VideoCapture(0)
cam.set(3, 480)
cam.set(4, 360)
ret,img = cam.read()
d_img = img# cv2.cvtColor(img, cv2.COLOR_BGR2RGB)
plt.subplot(1,2,1)
plt.xticks([]), plt.yticks([])
plt.title("Original")
ax1 = plt.imshow(d_img)
plt.subplot(1,2,2)
plt.xticks([]), plt.yticks([])
plt.title("processed red")
ax2 = plt.imshow(d_img)
old_circle = [0,0,0]
c = [0,0,0]
plt.ion()
try:
    while True:
        ret,img = cam.read()
        d_img = img#cv2.cvtColor(img, cv2.COLOR_BGR2RGB)
        img2 = img.copy()[:,:,0]
        
        #our camera considers red to be blue
        #img2 = cv2.inRange(hsv_img, np.array((90,50,70), np.uint8), np.array((128,255,255), np.uint8))
        img2 = cv2.adaptiveThreshold(img2, 255, cv2.ADAPTIVE_THRESH_MEAN_C, cv2.THRESH_BINARY_INV, 17, 10)
        #values for  red in case of non-daltonic camera
        #img2_lower = cv2.inRange(hsv_img, np.array((0,100,100), np.uint8), np.array((10,255,255), np.uint8))
        #img2_upper = cv2.inRange(hsv_img, np.array((160,100,100), np.uint8), np.array((179,255,255), np.uint8))
        #img2 = cv2.addWeighted(img2_lower, 1.0, img2_upper, 1.0, 0.0)
        img2 = cv2.GaussianBlur(img2, (3,3), cv2.BORDER_DEFAULT)
        circles = cv2.HoughCircles(img2, cv2.HOUGH_GRADIENT, 1, 22)
        if circles is not None:
            for circle in circles:
                print(circle)
                x = circle[0][0]
                y = circle[0][1]
                r = circle[0][2]
                if x*y*r > 0:
                    cv2.rectangle(d_img, (x-r,y-r), (x+r, y+r), (0,255,0), 5)
                    old_circle = [x,y,r]
                    c = [x,y,r]
                    io.output(led, io.HIGH)
                elif old_circle[0]*old_circle[1]*old_circle[2]> 0:
                    x = old_circle[0]
                    y = old_circle[1]
                    r = old_circle[2]
                    cv2.rectangle(d_img, (x-r,y-r), (x+r, y+r), (0,255,0), 5)
        elif old_circle[0]*old_circle[1]*old_circle[2]> 0:
            x = old_circle[0]
            y = old_circle[1]
            r = old_circle[2]
            io.output(led, io.LOW)
            cv2.rectangle(d_img, (x-r,y-r), (x+r, y+r), (0,255,0), 5)
        else:
            io.output(led, io.LOW)
        ax1.set_data(d_img)
        ax2.set_data(img2)
        if c[0]*c[1]>0:
            if (c[0] > x_mid+eps)or(c[0] < x_mid-eps):
                delta = eta*(1-c[0]/x_mid)
            else:
                delta = 0
            dc += delta
            if (dc > dc_max):
                dc = dc_max
            if (dc < dc_min):
                dc = dc_min
            c= [0,0,0]
        print("dc = ", dc)
        sg90.ChangeDutyCycle(dc)
        plt.pause(0.1)
        
        

finally:
    plt.ioff()
    plt.show()
    sg90.stop()
    io.cleanup()
    cam.release()
