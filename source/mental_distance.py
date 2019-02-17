#encoding=utf-8
# measuring the distance between two person who wear the same hat (yellow is recommond.) and stand within the identified area
# run with python-3.5, opencv-3.1, raspberry pi 3 model B, pi-camera(SCI)
# usage: standerd place the PiCamera to suite the playgrund, than the distance of two person will be measured
# by wanckl @ 2019-01-08

# importing modules
import os
import cv2
import math
import numpy as np
# import serial
# import time

# define the serial port founction
# def port_close():
# 	ser.close()
# 	if (ser.isOpen()):
# 		print("Close port failed.")
# 	else:
# 		print("Finish.")
 
# def send(send_data):
# 	if (ser.isOpen()):
# 		ser.write(send_data.encode('utf-8'))  # send data with utf-8 encoding
# 		#ser.write(binascii.a2b_hex(send_data))  # send with hex format
# 		print("Successfully send.")
# 	else:
# 		print("Send failed")

fullfill = 19
gauss = 15
resize_rate = 0.8
start = 0
shrink = 53
cmp = 30

# pre define position data structure of two person
people_count = 0
position = [[0, 0, 0, 0], [0, 0, 0, 0]]
distance = 0

# Open the PiCamera and capturing video through PiCamera
cap=cv2.VideoCapture(0)
# obtain the resolution of video
hight = int(cap.get(3))
width = int(cap.get(4))
print (hight," x ",width,'\n')

# configur the serial of raspberry Pi
# ser = serial.Serial("/dev/ttyAMA0", 9600)

try:
	while(1):
		(ret, img) = cap.read()
		if not ret:
			if (start):
				print ("finished.")
			else:
				print ("open video failed")
			break
		else:
			# ser.open()
			img = cv2.resize(img, (int(hight*resize_rate), int(width*resize_rate)))
			start = 1
			img = cv2.flip(img, 1)	# overturn the img if need

			#converting frame(img i.e BGR) to HSV (hue-saturation-value)
			hsv=cv2.cvtColor(img,cv2.COLOR_BGR2HSV)

			#definig the range of hat color (yellow is recommoned)
			color_lower=np.array([136,87,111],np.uint8)
			color_upper=np.array([180,255,255],np.uint8)

			#finding the range of yellow color in the image
			hat=cv2.inRange(hsv, color_lower, color_upper)
			cv2.imshow("Original Found",hat)

			# Morphological flit transformation, GaussianBlur, Dilation
			hat = cv2.GaussianBlur(hat, (gauss, gauss), 0)
			hat = cv2.threshold(hat, cmp, 255, cv2.THRESH_BINARY)[1]
			hat=cv2.dilate(hat, None, iterations=fullfill)
			cv2.imshow("Flitered",hat)

			kernel = np.ones((shrink, shrink), np.uint8)
			hat = cv2.erode(hat, kernel)

			# Mark the hat Color in original imagen
			(_,contours,hierarchy)=cv2.findContours(hat,cv2.RETR_EXTERNAL,cv2.CHAIN_APPROX_NONE)
			
			for pic, contour in enumerate(contours):
				area = cv2.contourArea(contour)
				# limit the area to reduce noise
				if(area > 5000):
					x,y,w,h = cv2.boundingRect(contour)
					people_count += 1
					if (people_count < 3):
						# recorde the position infomation of two person
						position[people_count-1] = [x, y, w, h]
					img = cv2.rectangle(img,(x,y),(x+w,y+h),(0,255,0),2)
					# comment = "hat "+str(x)+", "+str(y)
					# cv2.putText(img,comment,(x,y),cv2.FONT_HERSHEY_SIMPLEX, 0.7, (0,0,255))

			if (people_count == 0):
				pass
			elif (people_count == 1):
				# the condition that too close two person is to same as one.
				distance = 0
			elif (people_count == 2):
				# Mack the distance path between two person. (from the center point of person)
				person_1 = (position[0][0] + int(w/2), position[0][1] + int(h/2))
				person_2 = (position[1][0] + int(w/2), position[1][1] + int(h/2))
				cv2.line(img, person_1, person_2, (0, 255, 0), 2)

				# compute the distance path between two person. (from the center point of person)
				bias_x = math.fabs(person_2[0] - person_1[0])
				bias_y = math.fabs(person_2[1] - person_1[1])
				distance = int(math.sqrt(bias_x**2 + bias_y**2))
				cv2.putText(img,str(distance), (int((person_1[0] + person_2[0])/2), \
												int((person_1[1] + person_2[1])/2)),cv2.FONT_HERSHEY_SIMPLEX, 0.7, (0,255,0))
			else:
				pass
			people_count = 0

			# send the distance information calculated via serial port
			# send(str(distance))
			print(distance)
			cv2.imshow("Hat Tracking",img)

			# define keybord to debug
			Key = cv2.waitKey(1) & 0xFF
			if Key == ord('q'):
				break
			# --------------------------------------------------------------
			elif Key == ord('+'):	# change the fulfill valume
				fullfill += 1
				print ("			fullfill =",fullfill)
			elif Key == ord('_'):
				fullfill -= 1
				print ("			fullfill =",fullfill)
			# --------------------------------------------------------------
			elif Key == ord('z'):	# change the gussblur depth
				if gauss >= 3:
					gauss -= 2
					print ("			gauss =",gauss)
				else : print ("			Stop Decratint Gauss Value !")
			elif Key == ord('x'):
				gauss += 2
				print ("			gauss =",gauss)
			# --------------------------------------------------------------
			elif Key == ord('v'):	# change the erode intensity
				shrink += 1
				print ("			shrink =",shrink)
			elif Key == ord('c'): 
				shrink -= 1
				print ("			shrink =",shrink)
			# --------------------------------------------------------------
			elif Key == ord('s'):	# change the binary compare threshhold
				cmp += 1
				print ("			cmp =",cmp)
			elif Key == ord('a'): 
				cmp -= 1
				print ("			cmp =",cmp)
			elif Key == ord(' '):	#Press blank to pause;
				cv2.waitKey(0)
	print("* " *13)

finally:
	cap.release()
	cv2.destroyAllWindows()
	port_close()
	print("exit !")
