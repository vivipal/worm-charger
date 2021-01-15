import serial
import cv2
import time
import numpy as np
from datetime import datetime
import struct

# to get button state on the raspberry (not implemented)
# import RPi.GPIO as GPIO

import robot

send_time = 0
centered = 0
num_cam = 2 # numero X de /dev/videoX
num_port = 0 # numero X du port serie /dev/ttyUSBX
delay_between_send = 1 # delay different consecutive send in second
mid_error = 10 # set the precision for the center

distance=["OK","Go forward","Go backward","Go little forward","Go little backward"]
dir=["OK","Turn Left","Turn Right","Turn little Left","Turn little Right"]

def setup_serial(num_port) :
    port = "/dev/ttyUSB"+str(num_port)
    rate = 28800 #baud
    connection = serial.Serial(port,rate)

    return connection

def open_camera(num_cam):
    cv2.namedWindow("Robot POV")
    vc = cv2.VideoCapture(num_cam)

    if vc.isOpened(): # try to get the first frame
        rval, frame = vc.read()
    else:
        raise TypeError("Could not open the first frame")


    return vc,rval,frame

def frame_shape(frame):
    # find the center of the image (in px)
    y_px,x_px,*_ = np.shape(frame)
    x_px = int(x_px)
    y_px=int(y_px)
    x_mid,y_mid = int(x_px/2),int(y_px/2)

    return x_px,y_px,x_mid,y_mid

def detect_circle(frame):
    # process image for detecting circles
    gray = cv2.cvtColor(frame,cv2.COLOR_BGR2GRAY)
    gray_blurred = cv2.blur(gray, (5, 5))

    # detect circles
    detected_circles = cv2.HoughCircles(gray, cv2.HOUGH_GRADIENT, 1, 20, param1 = 50, param2 = 80, minRadius = 1, maxRadius = 150)

    return detected_circles

def draw(frame,a,b,r):

    global x_px,y_px,x_mid,y_mid

    # draw the circle.
    cv2.circle(frame, (a, b), r, (0, 255, 0), 2)
    # draw the center (filled circle)
    cv2.circle(frame, (a, b), 5, (0, 0, 255), -1)
    # draw the line of the angle
    cv2.line(frame,(x_mid,y_px),(x_mid,y_mid),(0,0,255),3)
    cv2.line(frame,(x_mid,y_px),(a,b),(0,0,255),3)

def next_command(a,b):
    global x_mid,y_mid,mid_error

    turn,dist = 0,0

    if a < x_mid-mid_error : # need to turn to the left
        turn = 1
    elif a > x_mid+mid_error : # need to turn to the right
        turn = 2

    if abs(a-x_mid)<70 and turn !=0: # need to perform small movement
        turn+=2

    if b < y_mid-mid_error : # need to go forward
        dist = 1
    elif b > y_mid+mid_error : # need to go backward
        dist = 2

    return turn,dist



if __name__ == '__main__':

    # setup for the communication with the arduino
    connection = setup_serial(num_port)

    # open the camera
    vc,rval,frame = open_camera(num_cam)
    print(rval)

    # get dimension and the middle of the frame in px
    x_px,y_px,x_mid,y_mid = frame_shape(frame)


    while rval and centered<=5 :

        cv2.imshow("Robot POV", frame)

        rval, frame = vc.read()
        # draw mid lines
        cv2.line(frame,(x_mid,0),(x_mid,y_px),(0,0,0))
        cv2.line(frame,(0,y_mid),(x_px,y_mid),(0,0,0))

        # get the new frame

        detected_circles = detect_circle(frame)

        if detected_circles is not None :

            #check if it sees only ONE circle
            if len(detected_circles[0,:])==1 :

                detected_circles = np.uint16(np.around(detected_circles))

                # get circle info
                circle = detected_circles[0, :][0]
                a, b, r = circle[0], circle[1], circle[2]

                draw(frame,a,b,r)

                turn, dist = next_command(a,b)

                current_time = time.time()
                if current_time - send_time > delay_between_send:

                    print("----------------------------------")
                    print(dir[turn]+" -  "+distance[dist])
                    print(">>> Sent at ", datetime.now().strftime("%H:%M:%S"))
                    print("----------------------------------\n")

                    send_time = time.time()

                    connection.write(b'C')
                    connection.write(struct.pack("BB",turn,dist))

                    # count the number of time it is centered
                    if dist==0 and turn == 0 :
                        centered += 1
                    else :
                        centered = 0

        key = cv2.waitKey(20)
        if key == 27: # exit on ESC
            break


    print("Centered for 5 seconds --> exit ")

    cv2.destroyWindow("Robot POV")

    time.sleep(0.2)

    robot.go_down(connection)



    # close the serial connection
    connection.close()
