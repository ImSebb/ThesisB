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
import sys

FOCAL_LENGTH = 3.04 #This is the focal length of the camera being used in mm
QR_CODE_SIZE = 5 #This is the size of the QR code being used
CALIBRATE = 0.23 #This will be calibrated for optimal results
#receiveMessage()

#detectQR()

#sendMessage()

#getMeasurements()

def detectQR(img):

    data = QR.detectAndDecode(img)[0]

def getMeasurements(img):
    imgSize = findImgSize(img) #Find the size of the QR code in the image
    if imgSize is not None:
        Distance = distanceFinder(imgSize)
        Distance = Distance - Distance*CALIBRATE
        Angle = show_axes(cameraMatrix, distortionCoeff)
        return Distance, Angle


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

def read_camera_parameters(filepath = 'camera_parameters/intrinsic.dat'):

    inf = open(filepath, 'r')

    cmtx = []
    dist = []

    #ignore first line
    line = inf.readline()
    for _ in range(3):
        line = inf.readline().split()
        line = [float(en) for en in line]
        cmtx.append(line)

    #ignore line that says "distortion"
    line = inf.readline()
    line = inf.readline().split()
    line = [float(en) for en in line]
    dist.append(line)

    #cmtx = camera matrix, dist = distortion parameters
    return np.array(cmtx), np.array(dist)

def get_qr_coords(cmtx, dist, points):

    #Selected coordinate points for each corner of QR code.
    qr_edges = np.array([[0,0,0],
                         [0,1,0],
                         [1,1,0],
                         [1,0,0]], dtype = 'float32').reshape((4,1,3))

    #determine the orientation of QR code coordinate system with respect to camera coorindate system.
    rvec = cv.solvePnP(qr_edges, points, cameraMatrix, distortionCoeff)[1]

    result = rotParam(rvec)

    return result

def rotParam(rvec):
    from math import pi,atan2,asin 
    R = cv.Rodrigues(rvec)[0]
    roll = 180*atan2(-R[2][1], R[2][2])/pi
    #pitch = 180*asin(R[2][0])/pi
    #yaw = 180*atan2(-R[1][0], R[0][0])/pi
    result = roll + 180
    if result > 180:
        result = result - 360

    if result > 0:
        result = result + 3.2
    else:
        result = result + 5.4

    return result


def getAngle(cmtx, dist):

    ret_qr, points = qr.detect(img)

    if ret_qr:
        resultAngle = get_qr_coords(cmtx, dist, points)

    return resultAngle


def main():
    flag, img = camera.read() #This captures an image from the camera.
    if flag == True:
        distanceM, angleM = getMeasurements(img)
        print angleM


camera = cv.VideoCapture("nvarguscamerasrc ! nvvidconv ! video/x-raw, width=1024, height=576, format=BGRx ! videoconvert ! video/x-raw, format=BGR ! appsink", cv.CAP_GSTREAMER) #This sets up the camera object
QR = cv.QRCodeDetector() #This creates the QR code detection object
cameraMatrix, distortionCoeff = read_camera_parameters()

while True:
    main()

