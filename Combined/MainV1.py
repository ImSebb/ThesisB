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
#receiveMessage()

#detectQR()

#sendMessage()

#getMeasurements()

camera = cv.VideoCapture("nvarguscamerasrc ! nvvidconv ! video/x-raw, width=1024, height=576, format=BGRx ! videoconvert ! video/x-raw, format=BGR ! appsink", cv.CAP_GSTREAMER) #This sets up the camera object
QR = cv.QRCodeDetector() #This creates the QR code detection object


def detectQR(img):

	data = QR.detectAndDecode(img)[0]
	print(data)


def main():
	flag, img = camera.read() #This captures an image from the camera.
	if flag == True:
		detectQR(img)


while True:
	main()