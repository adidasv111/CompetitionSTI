#from scipy.spatial import distance as dist ----   Use explicit Euclidian distance instead
from picamera.array import PiRGBArray
from picamera import PiCamera
import time
import numpy as np
import cv2
import argparse
import math
#import imutils
import perspective
import contours
import RPi.GPIO as GPIO
import os

# ******** Tunning Variables ********
filterKernelSize = 3
threshod = 200
C = -60
closeSize1 = 65
openSize = 10
closeSize2 = 70
imgHeight = 360
imgWidth = 480
camFrameRate = 10

# ******** Auxilary methods ********
#Finds the midpoint between two points
def midpoint(p1, p2):
	return ((p1[0]+p2[0])/2, (p1[1]+p2[1])/2)

# Finds the Euclidian distance between two points
def euclidianDist(p1, p2):
	dx = p1[0]-p2[0]
	dy = p1[1]-p2[1]
	return (np.sqrt(pow(dx,2)+pow(dy,2)))

#Find the real distance to a point
def findDistance(x, y, imgWidth, imgHeight):		#*********** Need to correect according to camera! ******
	if x>imgWidth/2 and y>0.5*imgHeight/2:
		return 1;
	if x<imgWidth/2 and y>imgHeight/2:
		return 2;
	if x<imgWidth/2 and y<imgHeight/2:
		return 3;
	if x>imgWidth/2 and y<imgHeight/2:
		return 4;

# Shut down the RPI 
def shutdown(channel):
        print("Shutting Down")
        os.system("sudo reboot -h now")	#can add sudo

# ******** GPIO Configuration ********
GPIO.setmode(GPIO.BCM)					#pin numbering of the SoC
GPIO.setup(12, GPIO.IN, pull_up_down=GPIO.PUD_UP)		#pulled up. Goes to GND if switch is pressed
GPIO.add_event_detect(12, GPIO.FALLING, callback = shutdown, bouncetime = 2000)   #call shutdown() in case of falling edge in switch
								#bouncetime = time in which other similar events are ignored

# ******** Main Program ********
# construct the argument parse and parse the arguments
'''ap = argparse.ArgumentParser()
ap.add_argument("-v", "--video",
	help = "path to the (optional) video file")
args = vars(ap.parse_args())

if not args.get("video", False):				#if the video path was not supplied, grab the reference to the camera
	camera = cv2.VideoCapture(0)				#number is number of video device
else:					
	camera = cv2.VideoCapture(args["video"])		#otherwise, load the video
'''

#initialize the camera and gran a reference to the raw camera capture
camera = PiCamera()
camera.resolution = (imgWidth, imgHeight)
camera.framerate = camFrameRate
rawCapture = PiRGBArray(camera, size=(imgWidth, imgHeight))
		    
origin = (imgWidth/2, imgHeight)

#allow the camera to warmup
time.sleep(0.1)

# keep looping over the frames
'''while True:
	(grabbed, frame) = camera.read()			# grab the current frame
	if not grabbed:						# check to see if we have reached the end of the video
		break
	
	key = cv2.waitKey(1) & 0xFF
	if key == ord("q"):
		break
	
	imgHeight, imgWidth, _ = frame.shape			#Calculate image size
	origin = (imgWidth/2, imgHeight)
	'''
for frame in camera.capture_continuous(rawCapture, format="bgr", use_video_port=True):
        image = frame.array

        #show the frame
        cv2.imshow("Frame", image)
        key = cv2.waitKey(1) & 0xFF

        #clear the stream in preparation for the next frame
        rawCapture.truncate(0)
        
        #press q key to exit
        if key == ord("q"):
                break
        
        # --- Image Processing ---
        image = cv2.cvtColor(image, cv2.COLOR_BGR2GRAY)		#Calculte image in grayscale
        cv2.putText(image, "origin", (int(origin[0]), int(origin[1])), cv2.FONT_HERSHEY_SIMPLEX, 0.5, (255, 255, 255), 2)
        image = cv2.GaussianBlur(image, (filterKernelSize, filterKernelSize), 0)
	#image2 = cv2.GaussianBlur(image2, (3, 3), 0)
	#cv2.imshow('grayFiltered', image)
	#cv2.imshow('thresh2', image2)
        
	#_,image = cv2.threshold(image,threshold,255,cv2.THRESH_BINARY)
	#cv2.imshow('thresh', image2)
        
	# !!! Should not blur binary image, only the gray level (works much much better) !!!
        mean = cv2.mean(image)[0]
        thresholdAdapt = mean - C
	#print(thresholdAdapt)
        _,image = cv2.threshold(image,thresholdAdapt,255,cv2.THRESH_BINARY)
        #cv2.imshow('threshGauss', image)
	#_,image2 = cv2.threshold(image2,thresholdAdapt,255,cv2.THRESH_BINARY)
	#cv2.imshow('threshblur', image2)
        
        strEleClose = cv2.getStructuringElement(cv2.MORPH_RECT, (closeSize1,closeSize1), (-1,-1))
        image = cv2.morphologyEx(image, cv2.MORPH_CLOSE, strEleClose)
        #cv2.imshow('closed', image)
        
        strEleOpen = cv2.getStructuringElement(cv2.MORPH_RECT, (openSize,openSize), (-1,-1))
        image = cv2.morphologyEx(image, cv2.MORPH_OPEN, strEleOpen)
        #cv2.imshow('opened', image)
        
        strEleClose = cv2.getStructuringElement(cv2.MORPH_RECT, (closeSize2,closeSize2), (-1,-1))
        image = cv2.morphologyEx(image, cv2.MORPH_CLOSE, strEleClose)
        #cv2.imshow('closed2', image)
	
	#*************** Might not need edge detector *************
	#image = cv2.Canny(image, 128, 128*2)
	#cv2.imshow('edged',image)
	
	#strEleClose = cv2.getStructuringElement(cv2.MORPH_RECT, (10,10), (-1,-1))
	#edged = cv2.morphologyEx(image, cv2.MORPH_CLOSE, strEleClose)
        #cv2.imshow('edged', edged)

# --- Distance and orientation ---
        _, cnts, _ = cv2.findContours(image.copy(), cv2.RETR_EXTERNAL,cv2.CHAIN_APPROX_SIMPLE)
	#print("full cnt no: " + str(len(cnts)))
        if len(cnts) == 0:
                print("Warning: No countours found!")
        else:
                (cnts, _) = contours.sort_contours(cnts)
	
                for c in cnts:					#loop over the contours individually
                        if cv2.contourArea(c) < 1000:		#if the contour is not sufficiently large, ignore it
                                continue

		# --- Find center of bottle--
			# compute the center of the contour
                        M = cv2.moments(c)
                        cX = int(M["m10"] / M["m00"])
                        cY = int(M["m01"] / M["m00"])

                        cv2.line(image, (int(cX), int(cY)), (int(origin[0]), int(origin[1])),(255, 0, 255), 2)

		# --- Find oriention ---
			#box = cv2.fitEllipse(c)		#*********** Can try fitting an ellipse ***************
                        box = cv2.minAreaRect(c)		#Fit a box arround the countour
                        box = cv2.boxPoints(box)		#cv2.cv.BoxPoints for Python 2.4
                        box = np.array(box, dtype="int")
                        box = perspective.order_points(box)
                        cv2.drawContours(image, [box.astype("int")], -1, (255, 255, 0), 2)
                        
                        '''# loop over the box points and draw them
			for (x, y) in box:
				cv2.circle(image, (int(x), int(y)), 5, (128, 0, 255), -1)
			'''
			# unpack the ordered bounding box, then compute the midpoints
                        (tl, tr, br, bl) = box
                        tltr = midpoint(tl, tr)	#top-left and top-right
                        blbr = midpoint(bl, br)	#bottom-left and bottom-right
                        tlbl = midpoint(tl, bl)	#top-left and bottom-left
                        trbr = midpoint(tr, br)	#top-right and bottom-right

                        '''# draw the midpoints on the image
			cv2.circle(image, (int(tltr[0]), int(tltr[1])), 5, (200, 0, 0), -1)
			cv2.circle(image, (int(blbr[0]), int(blbr[1])), 5, (200, 0, 0), -1)
			cv2.circle(image, (int(tlbl[0]), int(tlbl[1])), 5, (200, 0, 0), -1)
			cv2.circle(image, (int(trbr[0]), int(trbr[1])), 5, (200, 0, 0), -1)
			'''	 
			# draw lines between the midpoints
                        cv2.line(image, (int(tltr[0]), int(tltr[1])), (int(blbr[0]), int(blbr[1])),(255, 0, 255), 2)
                        cv2.line(image, (int(tlbl[0]), int(tlbl[1])), (int(trbr[0]), int(trbr[1])),(255, 0, 255), 2)
			
			# compute the Euclidean distance between the midpoints
                        dA = euclidianDist(tltr, blbr)
                        dB = euclidianDist(tlbl, trbr)
	 		
                        if dA >= dB:
                                majorAxis = [tltr, blbr]		
                        else:
                                majorAxis = [tlbl, trbr]

                        majorAxis = np.array(majorAxis, dtype="int")
                        majorAxis = majorAxis[np.argsort(majorAxis[:, 1]), :]	#sort according to y
                        majorAxisTop = majorAxis[0]		#top is point with lowest y
                        cv2.putText(image, "top", (majorAxisTop[0], majorAxisTop[1]), cv2.FONT_HERSHEY_SIMPLEX, 0.5, (255, 255, 255), 2)
			#print("dx")
			#print(majorAxisTop[0]-cX)
			#print("dy")
			#print(cY - majorAxisTop[1])

			#compute distance to the center of the object
                        distance = findDistance(cX, cY, imgWidth, imgHeight)
                        #print("Distance: " + str(distance))

			#compute direction to the center of the object
                        direction = np.degrees(np.arctan2(origin[1] - cY, cX - origin[0]))	#y is upside down
                        #print("Direction: " + str(direction))

			#compute orientation of the bottle
                        orientation = np.degrees(np.arctan2(cY - majorAxisTop[1], majorAxisTop[0]-cX))	#y is upside down
                        #print("Orientation: " + str(orientation))

                        #cv2.imshow('cnt', image)
		
# cleanup the camera and close any open windows
#camera.release()
#cv2.destroyAllWindows()
