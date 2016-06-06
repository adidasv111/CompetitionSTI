import numpy as np
import cv2
import argparse

face_haar_cascade = cv2.CascadeClassifier('/home/adi/opencv/data/haarcascades/haarcascade_frontalface_default.xml')
eye_haar_cascade = cv2.CascadeClassifier('/home/adi/opencv/data/haarcascades/haarcascade_eye.xml')
face_lbp_cascade = cv2.CascadeClassifier('/home/adi/opencv/data/lbpcascades/lbpcascade_frontalface.xml')

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
	haar_frame = frame.copy()
	lbp_frame = frame.copy()
	# check to see if we have reached the end of the video
	if not grabbed:
		break
	
	key = cv2.waitKey(1) & 0xFF
	# if the 'q' key is pressed, stop the loop
	if key == ord("q"):
		break
	
	#Calculte image in grayscale
	gray = cv2.cvtColor(frame, cv2.COLOR_BGR2GRAY)

	#face detection using haar cascade
	faces = face_haar_cascade.detectMultiScale(gray, 1.3, 5)
	for (x,y,w,h) in faces:
		cv2.rectangle(haar_frame,(x,y),(x+w,y+h),(255,0,0),2)
		roi_gray = gray[y:y+h, x:x+w]
		roi_color = haar_frame[y:y+h, x:x+w]
		eyes = eye_haar_cascade.detectMultiScale(roi_gray)
		for (ex,ey,ew,eh) in eyes:
			cv2.rectangle(roi_color,(ex,ey),(ex+ew,ey+eh),(0,255,0),2)
	
	#cv2.imshow('original',frame)
	cv2.imshow("haar",haar_frame)
	'''
	faces = face_lbp_cascade.detectMultiScale(gray, 1.3, 5)
	for (x,y,w,h) in faces:
		cv2.rectangle(lbp_frame,(x,y),(x+w,y+h),(255,0,0),2)
		roi_gray = gray[y:y+h, x:x+w]
		roi_color = lbp_frame[y:y+h, x:x+w]

	cv2.imshow('lbp',lbp_frame)
	'''
# cleanup the camera and close any open windows
camera.release()
cv2.destroyAllWindows()
