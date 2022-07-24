##This is the main file that will run the QR code state machine and feed the outputs to the Arduino
##The general structure is as follows
##Upon each state change the Arduino will send its new state to the Raspberry Pi
##Raspberry Pi will then:
##Detect and decode QR code
##Identify the state the Arduino needs to be in from the QR code
##If Arduino is in wrong state, send the new state to the Arduino and wait for reply with confirmation of state change
##Once the Arduino is in the correct state:
##If the Arduino is in grapple state, get distance and angle measurements and send to arduino
##If the Arduino is in roam state do nothing
import cv2 as cv
import numpy as np
import math
import pyzbar.pyzbar as pyzbar

FOCAL_LENGTH = 3.04 #This is the focal length of the camera being used in mm
QR_CODE_SIZE = 5 #This is the size of the QR code being used
CALIBRATE = 0.23 #This will be calibrated for optimal results
#receiveMessage()

#detectQR()

#sendMessage()

#getMeasurements()

camera = cv.VideoCapture("nvarguscamerasrc ! nvvidconv ! video/x-raw, width=1024, height=576, format=BGRx ! videoconvert ! video/x-raw, format=BGR ! appsink", cv.CAP_GSTREAMER) #This sets up the camera object
QR = cv.QRCodeDetector() #This creates the QR code detection object


def detectQR(img):

	data = QR.detectAndDecode(img)[0]



def main():
	flag, img = camera.read() #This captures an image from the camera.
	if flag == True:
		getMeasurements(img)

def getMeasurements(img):
	imgSize = findImgSize(img) #Find the size of the QR code in the image
	if imgSize is not None:
		Distance = distanceFinder(imgSize)
		Distance = Distance - Distance*CALIBRATE
		return Distance


# finding Distance between two points

def eucaldainDistance(x, y, x1, y1):

    eucaldainDist = math.sqrt((x1 - x) ** 2 + (y1 - y) ** 2)

    return eucaldainDist

def distanceFinder(widthInImage):
  
    distance = ((QR_CODE_SIZE * FOCAL_LENGTH) / widthInImage)

    return distance

def findImgSize(image):
    # convert the color image to gray scale image
    Gray = cv.cvtColor(image, cv.COLOR_BGR2GRAY)

    # create QR code object
    objectQRcode = pyzbar.decode(Gray)

    for obDecoded in objectQRcode:

        points = obDecoded.polygon

        if len(points) > 4:
            hull = cv.convexHull(
                np.array([points for point in points], dtype=np.float32))
            hull = list(map(tuple, np.squeeze(hull)))
        else:
            hull = points

        x, x1 = hull[0][0], hull[1][0]
        y, y1 = hull[0][1], hull[1][1]

        # using Eucaldain distance finder function to find the width 
        euclaDistance = eucaldainDistance(x, y, x1, y1)

        return euclaDistance


while True:
	main()

