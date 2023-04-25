import cv2
import numpy as np
import serial
from time import sleep 

import urllib.request
import imutils
url = 'http://192.168.1.8:8080/shot.jpg'
 
camera = cv2.VideoCapture(1)
image_width = 640
image_height = 380
minimum_area = 20000
maximum_area = 80000


min_mau = np.array([0, 100, 100]) 
max_mau = np.array([10, 255, 255])

ser = serial.Serial(
	port = '/dev/ttyACM0',
	baudrate = 115200,
	parity = serial.PARITY_NONE,
	stopbits = serial.STOPBITS_ONE,
	bytesize = serial.EIGHTBITS,
	timeout = 1
)

def serial_send_num(num):
	if(num>=0):
		send_string = (('<'+str(len(str(num)))+'+'+str(num)+'>').encode("utf-8"))	
	else:
		send_string = (('<'+str(len(str(num))-1)+str(num)+'>').encode("utf-8"))
	ser.write(send_string)
	
	
while (camera.isOpened):
    imgPath = urllib.request.urlopen(url)
    imgNp = np.array(bytearray(imgPath.read()), dtype=np.uint8)
    image = cv2.imdecode(imgNp, -1)

    image = cv2.resize(src=image, dsize=(image_width,image_height))

    #image = imutils.resize(img,width=450)

    hsv = cv2.cvtColor(image, cv2.COLOR_BGR2HSV)
    color_mask = cv2.inRange(hsv, min_mau, max_mau)
    countours, hierarchy = cv2.findContours(color_mask, cv2.RETR_LIST, cv2.CHAIN_APPROX_SIMPLE)
    result = cv2.bitwise_and(image, image, mask= color_mask)
    object_area = 0
    object_x = 0
    object_y = 0
    
    cv2.imshow("Real", image)
    cv2.imshow("HSV", hsv)
    cv2.imshow("Final Result", result)
        
        
    for contour in countours:
        
        x, y, width, height = cv2.boundingRect(contour)
        found_area = width * height
        center_x = x + (width / 2)
        center_y = y + (height / 2)
       
        if object_area < found_area:
            object_area = found_area
            object_x = center_x
            object_y = center_y
    if object_area > 0:
        ball_location = [object_area, object_x, object_y]
    else:
        ball_location = None
 
    if ball_location:
        if (ball_location[0] > minimum_area) and (ball_location[0] < maximum_area):
            if ball_location[1] < image_width/4:
                print("Turning right")
                ser.write("right\r".encode())
            elif ball_location[1] > 3*image_width/4:
                print("Turning left")
                ser.write("left\r".encode())
            else:
                print("Stop")
                ser.write("stop\r".encode())
        elif (ball_location[0] < minimum_area and ball_location[0] > minimum_area/4 ):
            print("Forward")
            ser.write("forward\r".encode())
        elif (ball_location[0] > maximum_area):
            print("Target large enough, backward")
            ser.write("backward\r".encode())
        else:
            print("Target not found, searching")
            ser.write("stop\r".encode())
    if cv2.waitKey(1) == ord('q'):
    	break
print("Stop")
ser.write("stop\r".encode())