
import cv2 as cv
import numpy as np
import math
import pyzbar.pyzbar as pyzbar
import betterLook

# Variable
camID = 0  # camera ID, or pass string as filename. to the camID

# Real world measured Distance and width of QR code
KNOWN_DISTANCE = 30.1  # inches
KNOWN_WIDTH = 5.0  # inches

# define the fonts
fonts = cv.FONT_HERSHEY_COMPLEX
Pos =(50,50)
# colors (BGR)
WHITE = (255, 255, 255)
BLACK = (0, 0, 0)
MAGENTA = (255, 0, 255)
GREEN = (0, 255, 0)
CYAN = (255, 255, 0)
GOLD = (0, 255, 215)
YELLOW = (0, 255, 255)
ORANGE = (0, 165, 230)

# functions

# finding Distance between two points


def eucaldainDistance(x, y, x1, y1):

    eucaldainDist = math.sqrt((x1 - x) ** 2 + (y1 - y) ** 2)

    return eucaldainDist

# focal length finder function


def focalLengthFinder(knowDistance, knownWidth, widthInImage):
    '''This function calculates the focal length. which is used to find the distance between  object and camera 
    :param1 knownDistance(int/float) : it is Distance form object to camera measured in real world.
    :param2 knownWidth(float): it is the real width of object, in real world
    :param3 widthInImage(float): the width of object in the image, it will be in pixels.
    return FocalLength(float): '''
    
    focalLength = ((widthInImage * knowDistance) / knownWidth)
    return focalLength

def distanceFinder(focalLength, knownWidth, widthInImage):
    '''
    This function basically estimate the distance, it takes the three arguments: focallength, knownwidth, widthInImage
    :param1 focalLength: focal length found through another function .
    param2 knownWidth : it is the width of object in the real world.
    param3 width of object: the width of object in the image .
    :returns the distance:


    '''
    distance = ((knownWidth * focalLength) / widthInImage)
    return distance

def DetectQRcode(image):
    codeWidth = 0
    x, y = 0, 0
    euclaDistance = 0
    global Pos 
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

        n = len(hull)
        # draw the lines on the QR code 
        for j in range(0, n):
            # print(j, "      ", (j + 1) % n, "    ", n)

            cv.line(image, hull[j], hull[(j + 1) % n], ORANGE, 2)
        # finding width of QR code in the image 
        x, x1 = hull[0][0], hull[1][0]
        y, y1 = hull[0][1], hull[1][1]
        
        Pos = hull[3]
        # using Eucaldain distance finder function to find the width 
        euclaDistance = eucaldainDistance(x, y, x1, y1)
        # retruing the Eucaldain distance/ QR code width other words  
        return euclaDistance

# video recording 
fourcc = cv.VideoWriter_fourcc(*'XVID')
out = cv.VideoWriter('Demo.mp4', fourcc, 15.0, (640, 480))

# creating camera object
camera = cv.VideoCapture("nvarguscamerasrc ! nvvidconv ! video/x-raw, width=1024, height=576, format=BGRx ! videoconvert ! video/x-raw, format=BGR ! appsink", cv.CAP_GSTREAMER)


refernceImage = cv.imread("referenceImage2.png")
# getting the width of QR code in the reference image 
Rwidth= DetectQRcode(refernceImage)

# finding the focal length 
focalLength = focalLengthFinder(KNOWN_DISTANCE, KNOWN_WIDTH, Rwidth)
print("Focal length:  ", focalLengthFinder)

counter =0

while True:
    ret, frame = camera.read()

    # finding width of QR code width in the frame 
    codeWidth= DetectQRcode(frame)
    # print(Value)
    # print(codeWidth)
    
    if codeWidth is not None:
        
        # print("not none")
        Distance = distanceFinder(focalLength, KNOWN_WIDTH, codeWidth)
        Distance = (Distance - Distance*0.23)*2.54
        # cv.putText(frame, f"Distance: {Distance}", (50,50), fonts, 0.6, (GOLD), 2)
        
        betterLook.showText(frame, f"Distnace: {round(Distance,2)} cm", Pos, GOLD, int(Distance * 4.5))
    out.write(frame)
    

    cv.imshow("frame", frame)
    key = cv.waitKey(1)
    if key == ord('s'):
        # saving frame
        counter += 1
        print("frame saved")
        cv.imwrite(f"frames/frame{counter}.png", frame)
    if key == ord('q'):
        break
camera.release()
cv.destroyAllWindows()
out.release()
