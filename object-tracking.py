import numpy as np
import cv2
import time
import imutils

low_H=0
high_H=180
low_S=0
high_S=255
low_V=0
high_V=255

def on_trackbar_low_H(val):
    global low_H
    global high_H
    low_H=val
    low_H=min(high_H,low_H)
    cv2.setTrackbarPos("Low H","Trackbar",low_H)

def on_trackbar_high_H(val):
    global low_H
    global high_H
    high_H=val
    high_H=max(high_H,low_H)
    cv2.setTrackbarPos("High H","Trackbar",high_H)

def on_trackbar_low_S(val):
    global low_S
    global high_S
    low_S=val
    low_S=min(high_S,low_S)
    cv2.setTrackbarPos("Low S","Trackbar",low_S)

def on_trackbar_high_S(val):
    global low_S
    global high_S
    high_S=val
    high_S=max(high_S,low_S)
    cv2.setTrackbarPos("High S","Trackbar",high_S)

def on_trackbar_low_V(val):
    global low_V
    global high_V
    low_V=val
    low_V=min(high_V,low_V)
    cv2.setTrackbarPos("Low V","Trackbar",low_V)

def on_trackbar_high_V(val):
    global low_V
    global high_V
    high_V=val
    high_V=max(high_V,low_V)
    cv2.setTrackbarPos("High H","Trackbar",high_H)

cap = cv2.VideoCapture(0)

time.sleep(1)

cv2.namedWindow("Frame")
cv2.namedWindow("Thresh frame")
cv2.namedWindow("Trackbar")

cv2.createTrackbar("Low H","Trackbar",low_H, high_H,on_trackbar_low_H)
cv2.createTrackbar("High H","Trackbar",high_H,high_H,on_trackbar_high_H)
cv2.createTrackbar("Low S","Trackbar",low_S,high_S,on_trackbar_low_S)
cv2.createTrackbar("High S","Trackbar",high_S,high_S,on_trackbar_high_S)
cv2.createTrackbar("Low V","Trackbar",low_V,high_V,on_trackbar_low_V)
cv2.createTrackbar("High V","Trackbar",high_V,high_V,on_trackbar_high_V)

while(True):
    ret,frame=cap.read()
    hsv_frame=cv2.cvtColor(frame,cv2.COLOR_BGR2HSV)
    thresh_frame=cv2.inRange(hsv_frame,(low_H,low_S,low_V),
                            (high_H,high_S,high_V))

    mask=cv2.GaussianBlur(thresh_frame,(1,1),0)
    mask=cv2.erode(mask,None,iterations=2)
    mask=cv2.dilate(mask,None,iterations=2)

    cnts = cv2.findContours(mask.copy(), cv2.RETR_EXTERNAL,
		cv2.CHAIN_APPROX_SIMPLE)
    cnts = cnts[0] if imutils.is_cv2() else cnts[1]
    centroid = None

    # only proceed if at least one contour was found
    if len(cnts) > 0:
	    # find the largest contour in the mask, then use
	    # it to compute the minimum enclosing circle and
	    # centroid
	    c = max(cnts, key=cv2.contourArea)
	    ((x, y), radius) = cv2.minEnclosingCircle(c)
	    centre=(int(x),int(y))
	    M = cv2.moments(c)
	    centroid = (int(M["m10"] / M["m00"]), int(M["m01"] / M["m00"]))

	    # only proceed if the radius meets a minimum size
	    if radius > 10 and radius < 200:
	    	# draw the circle and centroid on the frame,
	    	# then update the list of tracked points
	    	cv2.circle(frame, centre, int(radius),
	    		(0, 255, 255), 2)
	    	cv2.circle(frame, centroid, 5, (0, 0, 255), -1)

    
    cv2.imshow("Frame",frame)
    cv2.imshow("Thresh frame",mask)
    if cv2.waitKey(10)==27:
        break

    if int(x)<320:
        print "Left"

    if int(x)>320:
        print "Right"
        
  

cap.release()
cv2.destroyAllWindows()
