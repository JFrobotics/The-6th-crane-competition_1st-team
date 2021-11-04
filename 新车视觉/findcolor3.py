from shapedetector import ShapeDetector
import numpy as np
import cv2
import cv2 as cv
import imutils
import time
import serial
kernel=np.ones((5,5),np.uint8)
ser = serial.Serial('/dev/ttyAMA0',115200,timeout=1)
capture=cv.VideoCapture(0)
#capture.set(cv2.CAP_PROP_FOURCC, cv2.VideoWriter.fourcc('M', 'J', 'P', 'G'))
def video_demo():
    
    while(True):
        Area=10000
        ref,frame=capture.read()
        start=time.perf_counter()
        sd = ShapeDetector()
        gray = cv2.cvtColor(frame, cv2.COLOR_BGR2GRAY)
        b,g,r=cv2.split(frame)
        err_red=cv2.subtract(r,b)
        err_blue=cv2.subtract(b,r)
        
        gray_red = cv2.morphologyEx(err_red, cv2.MORPH_OPEN, kernel)
        gray_blue = cv2.morphologyEx(err_blue, cv2.MORPH_OPEN, kernel)
        
        ret,red=cv2.threshold(gray_red,70,255,0)
        ret,blue=cv2.threshold(gray_blue,60,255,0)
        
        cnts1, red1 = cv2.findContours(red, cv2.RETR_EXTERNAL, cv2.CHAIN_APPROX_NONE)#轮廓检测红色
        cnts2, blue2 = cv2.findContours(blue, cv2.RETR_EXTERNAL, cv2.CHAIN_APPROX_NONE)#轮廓检测蓝色

        for cnt in cnts1:
            area=cv2.contourArea(cnt)
            
            if area>Area :
                shape=sd.detect(cnt,err_red)
                if shape==3:
                    cv2.drawContours(frame,[cnt], -1, (0, 255, 0), 2)
                    cv2.imshow('frame', frame)
                    ser.write('a'.encode("utf-8"))
                    end=time.perf_counter()
                    print('a')
                elif shape==4 :
                    cv2.drawContours(frame,[cnt], -1, (0, 255, 0), 2)
                    cv2.imshow('frame', frame)
                    ser.write('c'.encode("utf-8"))
                    end=time.perf_counter()
                    print('c')
                elif shape==5 :
                    cv2.drawContours(frame,[cnt], -1, (0, 255, 0), 2)
                    cv2.imshow('frame', frame)
                    ser.write('b'.encode("utf-8"))
                    end=time.perf_counter() 
                    print('b')
                
        for cnt in cnts2:
            area=cv2.contourArea(cnt)
            
            if area>Area :
                shape=sd.detect(cnt,err_blue)
                if shape==3:
                    cv2.drawContours(frame,[cnt], -1, (0, 255, 0), 2)
                    cv2.imshow('frame', frame)
                    ser.write('A'.encode("utf-8"))
                    end=time.perf_counter()
                    print('A')
                elif shape==4 :
                    cv2.drawContours(frame,[cnt], -1, (0, 255, 0), 2)
                    cv2.imshow('frame', frame)
                    ser.write('C'.encode("utf-8"))
                    end=time.perf_counter()
                    print('C')
                elif shape==5 :
                    cv2.drawContours(frame,[cnt], -1, (0, 255, 0), 2)
                    cv2.imshow('frame', frame)
                    ser.write('B'.encode("utf-8"))
                    end=time.perf_counter() 
                    print('B')
 
            #cv2.imshow('frame', frame)
        
        c= cv.waitKey(30) & 0xff
        if c==27:
            capture.release()
            break


video_demo()
cv.waitKey()
cv.destroyAllWindows()

