import numpy as np
import cv2
import argparse


# construct the argument parse and parse the arguments
ap = argparse.ArgumentParser()
ap.add_argument("-v", "--video",
	help = "path to the (optional) video file")
args = vars(ap.parse_args())

# if the video path was not supplied, grab the reference to the camera
if not args.get("video", False):
	camera = cv2.VideoCapture(0)

# otherwise, load the video
else:
	camera = cv2.VideoCapture(args["video"])

# keep looping over the frames
while True:
	# grab the current frame and make duplicates for ourputs
	(grabbed, frame) = camera.read()
	# check to see if we have reached the end of the video
	if not grabbed:
		break
	
	key = cv2.waitKey(1) & 0xFF
	# if the 'q' key is pressed, stop the loop
	if key == ord("q"):
		break
	
	#Calculte image in grayscale
	gray = cv2.cvtColor(frame, cv2.COLOR_BGR2GRAY)
	gray = cv2.GaussianBlur(gray, (5, 5), 0)
	edged = cv2.Canny(gray, 75, 200)
	#cv2.imshow('edged',edged)

	_, cnts, hierarchy = cv2.findContours(edged.copy(),cv2.RETR_LIST, cv2.CHAIN_APPROX_SIMPLE)
	cnts = sorted(cnts, key = cv2.contourArea, reverse = True)[:5]
	contours = [cv2.approxPolyDP(cnt, 3, True) for cnt in cnts]
	'''
	# loop over the contours
	for c in cnts:
		# approximate the contour
		peri = cv2.arcLength(c, True)
		approx = cv2.approxPolyDP(c, 0.02 * peri, True)
	 	#print(approx)
		screenCnt = approx
	'''
	cv2.drawContours( edged, contours, -1, (128,255,255), 3, cv2.LINE_AA)
	#cv2.drawContours( edged, contours, -1, (0, 255, 0), 2)
	
	cv2.imshow('Outline', edged)

# cleanup the camera and close any open windows
camera.release()
cv2.destroyAllWindows()
