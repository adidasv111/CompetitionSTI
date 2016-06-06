# import the necessary packages
import numpy as np
import argparse
import cv2
 
# construct the argument parse and parse the arguments
ap = argparse.ArgumentParser()
ap.add_argument("-i", "--image", help = "path to the image")
args = vars(ap.parse_args())
 
# load the image
image = cv2.imread(args["image"])

# define the list of boundaries for red, blue, yellow, and gray
# Images are represented as BGR arrays
boundaries = [
	([17, 15, 100], [50, 56, 200]),
	([86, 31, 4], [220, 88, 50]),
	([25, 146, 190], [62, 174, 250]),
	([103, 86, 65], [145, 133, 128])
]

# loop over the boundaries
for (lower, upper) in boundaries:
	# create NumPy arrays from the boundaries (Needed for cv2.inRange)
	lower = np.array(lower, dtype = "uint8")
	upper = np.array(upper, dtype = "uint8")

 	#names = ["Sam", "Peter", "James", "Julian", "Ann"] - IGNORE

	# find the colors in the image within the specified boundaries
	#Returns mask where each pixel is 255 (white) if the original pixel is in the boundaries
	#or 0 (black) if not
	mask = cv2.inRange(image, lower, upper)
	# apply the mask
	output = cv2.bitwise_and(image, image, mask = mask)
 
	# show the images
	cv2.imshow("images", np.hstack([image, output]))
	cv2.waitKey(0)
