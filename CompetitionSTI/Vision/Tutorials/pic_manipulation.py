# import the necessary packages
import cv2
import argparse
 
# construct the argument parser and parse the arguments
ap = argparse.ArgumentParser()
ap.add_argument("-i", "--image", required = True,
	help = "Path to the image to be scanned")
args = vars(ap.parse_args())

# load the image and show it
image = cv2.imread(args["image"])
cv2.imshow("original", image)
cv2.waitKey(0)

print (image.shape)	#rows x columns x channels (r-g-b). rows = height, columns = width

# we need to keep in mind aspect ratio so the image does
# not look skewed or distorted -- therefore, we calculate
# the ratio of the new image to the old image
r = 100.0 / image.shape[1]
dim = (100, int(image.shape[0] * r))	#dim = (new width, new height)
 
# perform the actual resizing of the image and show it
resized = cv2.resize(image, dim, interpolation = cv2.INTER_AREA)
cv2.imshow("resized", resized)
cv2.waitKey(0)

# grab the dimensions of the image and calculate the center of the image
(h, w) = image.shape[:2]
center = (w / 2, h / 2)
 
# rotate the image by 180 degrees
M = cv2.getRotationMatrix2D(center, 180, 1.0)		#Pt around which we rotate, degrees of rotation, scaling factor)
rotated = cv2.warpAffine(image, M, (w, h))
cv2.imshow("rotated", rotated)
cv2.waitKey(0)

# crop the image using array slices -- it's a NumPy array after all!
cropped = image[70:170, 440:540]	#startY:endY, startX:endX
cv2.imshow("cropped", cropped)
cv2.waitKey(0)

# write the cropped image to disk in PNG format
cv2.imwrite("/images/thumbnail.png", cropped)
