# import the necessary packages
from imutils.video import VideoStream
import numpy as np
import RPi.GPIO as GPIO
import cv2
import time
import os

# store the frame for the ROI
r = None

# servo max and min postitions to activate motors
x_max = 200
x_min = 100

# pwm for motors
pwm = 80

# servo angle increment for scanning
scan_increment = 2

# initial scan direction
scan_direction = "left"

# define the GPIO pins
motor_ena = 11
motor_1a = 13
motor_1b = 15

motor_enb = 22
motor_2a = 16
motor_2b = 18

trig = 29
echo = 31

GPIO.setmode(GPIO.BOARD)

# set the GPIO pins as input or output
GPIO.setup(motor_ena, GPIO.OUT)
GPIO.setup(motor_1a, GPIO.OUT)
GPIO.setup(motor_1b, GPIO.OUT)
GPIO.setup(motor_enb, GPIO.OUT)
GPIO.setup(motor_2a, GPIO.OUT)
GPIO.setup(motor_2b, GPIO.OUT)
GPIO.setup(trig, GPIO.OUT)
GPIO.setup(echo, GPIO.IN)

# start the pwm for motors
p = GPIO.PWM(motor_ena, 100)
q = GPIO.PWM(motor_enb, 100)
p.start(0)
q.start(0)

# functions to move the robot


def go_forward():
    p.ChangeDutyCycle(pwm)
    q.ChangeDutyCycle(pwm)
    GPIO.output(motor_1a, True)
    GPIO.output(motor_1b, False)
    GPIO.output(motor_2a, True)
    GPIO.output(motor_2b, False)


def go_backward():
    p.ChangeDutyCycle(pwm)
    q.ChangeDutyCycle(pwm)
    GPIO.output(motor_1a, False)
    GPIO.output(motor_1b, True)
    GPIO.output(motor_2a, False)
    GPIO.output(motor_2b, True)


def turn_left():
    p.ChangeDutyCycle(pwm)
    q.ChangeDutyCycle(pwm)
    GPIO.output(motor_1a, True)
    GPIO.output(motor_1b, False)
    GPIO.output(motor_2a, False)
    GPIO.output(motor_2b, True)


def turn_right():
    p.ChangeDutyCycle(pwm)
    q.ChangeDutyCycle(pwm)
    GPIO.output(motor_1a, False)
    GPIO.output(motor_1b, True)
    GPIO.output(motor_2a, True)
    GPIO.output(motor_2b, False)


def stop():
    p.ChangeDutyCycle(0)
    q.ChangeDutyCycle(0)

# calculate distance using ultrasonic sensor


def distance(trig, echo):
    global dist
    start, stop = 0

    # produce an ultrasonic signal
    GPIO.output(trig, True)
    time.sleep(0.00001)
    GPIO.output(trig, False)

    # record the time for echo to be detected
    while GPIO.input(echo) == False:
        start = time.time()

    while GPIO.input(echo) == True:
        stop = time.time()

    # calculate the duration using 2d/t
    # speed of sound approximately 343m/s
    duration = stop-start
    dist = round(duration*34300/2, 2)

    # distance calculated in cm
    return dist

# detect circle in the frame


def HoughCircles(mask):
    global frame, circles, x, y, radius

    # apply houghcircles
    circles = cv2.HoughCircles(
        mask, cv2.HOUGH_GRADIENT, 1, 300, param1=15, param2=20, minRadius=20)

    if circles is not None:
        circles = np.round(circles[0, :]).astype("int")

        for (x, y, radius) in circles:

            # draw a circle around the object with a centre point
            cv2.circle(frame, (x, y), radius, (0, 0, 255), 3)
            cv2.circle(frame, (x, y), 2, (0, 255, 255), 3)

# detect colour in the frame


def detect_colour(ROI, frame):

    # convert ROI to HSV
    hsv_ROI = cv2.cvtColor(ROI, cv2.COLOR_BGR2HSV)

    # upper and lower ranges for threshold
    lower = np.array(
        [hsv_ROI[:, :, 0].min(), hsv_ROI[:, :, 1].min(), hsv_ROI[:, :, 2].min()])
    upper = np.array(
        [hsv_ROI[:, :, 0].max(), hsv_ROI[:, :, 1].max(), hsv_ROI[:, :, 2].max()])

    # convert video frame from BGR to HSV and apply threshold
    hsv_frame = cv2.cvtColor(frame, cv2.COLOR_BGR2HSV)
    threshold = cv2.inRange(hsv_frame, lower, upper)

    # blur the image and perform morphological operations
    mask = cv2.GaussianBlur(threshold, (1, 1), 0)
    mask = cv2.erode(mask, None, iterations=2)
    mask = cv2.dilate(mask, None, iterations=2)

    return mask

# rotate servo to keep the object in the frame


def rotate_servo():
    global circles, h, w, x, y

    # calculate frame centre and the difference with the detected object
    frame_centre = (w/2, h/2)
    if circles is not None:
        pan_error = x-frame_centre[0]
        tilt_error = y-frame_centre[1]
    else:
        pan_error = 0
        tilt_error = 0

    # rotate servos when the error is large
    if abs(pan_error) > 30:
        panServo.update(pan_error)
        os.system("echo 0=%s > /dev/servoblaster" % panServo.pos)
    if abs(tilt_error) > 30:
        tiltServo.update(tilt_error)
        os.system("echo 2=%s > /dev/servoblaster" % tiltServo.pos)

# move the robot to follow object


def follow():
    global x_max, x_min, radius

    # if servo rotates to certain angle, rotate the robot
    if panServo.pos < x_min:
        turn_right()
    elif panServo.pos > x_max:
        turn_left()

    # move towards or away from the object
    elif radius < 30:
        go_forward()
    elif radius > 50:
        go_backward()

    else:
        stop()

# scan for objecs


def scan():
    global scan_increment, scan_direction, dist

    # continuously rotate the servo
    panServo.pos += scan_increment
    if panServo.pos > 240 or panServo.pos < 60:
        scan_increment = -scan_increment
    os.system("echo 0=%s > /dev/servoblaster" % panServo.pos)

    # calculate distance and go forward if no obstacle
    distance(trig, echo)
    if dist > 10:
        go_forward()

    # Turn left and right alternatively everytime it encounters an obstacle
    # rotate servos simultaneously
    else:
        if scan_direction == "left":
            for x in range(90):
                panServo.pos += scan_increment
                if panServo.pos > 240 or panServo.pos < 60:
                    scan_increment = -scan_increment
                os.system("echo 0=%s > /dev/servoblaster" % panServo.pos)
                if x < 20 or x > 70:
                    turn_left()
                else:
                    go_forward()
            scan_direction = "right"
        elif scan_direction == "right":
            for x in range(90):
                panServo.pos += scan_increment
                if panServo.pos > 240 or panServo.pos < 60:
                    scan_increment = -scan_increment
                os.system("echo 0=%s > /dev/servoblaster" % panServo.pos)
                if x < 20 or x > 70:
                    turn_right()
                else:
                    go_forward()
            scan_direction = "left"


class Servo:
    def __init__(self, p_gain, d_gain):

        # varibles for a pid servo
        self.pos = 150
        self.prev_error = 0
        self.p_gain = p_gain
        self.d_gain = d_gain

    def update(self, error):

        # rotate servo based on the magnitude of error
        if self.prev_error != 0:
            pid = (self.p_gain*error+(error-self.prev_error)*self.d_gain)
            self.pos += pid
            if self.pos > 240:
                self.pos = 240
            if self.pos < 60:
                self.pos = 60
        self.prev_error = error


# set the servos p and d gains
panServo = Servo(0.03, 0.02)
tiltServo = Servo(0.01, 0.01)

# initiate the Picamera and allow it to warm up
vs = VideoStream(usePiCamera=True).start()
time.sleep(1.0)

while True:

    # grab a frame
    frame = vs.read()
    frame = cv2.flip(frame, 0)
    (h, w) = frame.shape[:2]

    key = cv2.waitKey(1) & 0xFF

    # if the s key is pressed, activate the ROI selection
    if key == ord("s"):
        r = cv2.selectROI("Frame", frame, fromCenter=False, showCrosshair=True)
        frame_ROI = frame[int(r[1]):int(r[1]+r[3]), int(r[0]):int(r[0]+r[2])]

    # if ROI is selected, perform object detection
    if r is not None:
        mask = detect_colour(frame_ROI, frame)
        HoughCircles(mask)

        # if an object is present in the frame, track the object
        if circles is not None:
            rotate_servo()
            follow()

        cv2.imshow("Mask", mask)

    # if no object detected, scan for objects
    else:
        scan()

    # show the frame
    cv2.imshow("Frame", frame)

    # if the 'q' key is pressed, stop the loop
    if key == ord("q"):
        break

# destroy the windows, stop the camera, and clean the GPIO pins
cv2.destroyAllWindows()
vs.stop()
GPIO.cleanup()
