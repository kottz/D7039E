import numpy as np
import cv2

from imutils.video import VideoStream
from imutils.video import FPS
import imutils

vs = cv2.VideoCapture('red_line.mp4')



while True:
    ret, frame = vs.read()
    
    if ret != True:
        break
    
    if cv2.waitKey(1) & 0xFF == ord('q'):
        break

    hsv = cv2.cvtColor(frame, cv2.COLOR_BGR2HSV)

    lower_red = np.array([0,0,50])
    upper_red = np.array([255,255,240])
    
    mask = cv2.inRange(hsv, lower_red, upper_red)
    #output = cv2.bitwise_and(frame,frame, mask= mask)

    element = cv2.getStructuringElement(cv2.MORPH_CROSS,(9,9))
    kernel = np.ones((3,3),np.uint8)
    #eroded = cv2.erode(mask,kernel, iterations = 1)
    #dilate = cv2.dilate(eroded, element)
    #skeleton = cv2.subtract(mask, dilate)
    #skeleton = cv2.ximgproc.thinning(eroded)

    edges = cv2.Canny(mask, 50, 200)
    #edges = cv2.dilate(edges, element)
    minLineLength = 100
    maxLineGap = 15
    lines = cv2.HoughLinesP(
        edges,
        1,
        np.pi / 180,
        100,
        minLineLength,
        maxLineGap,
        )
    if lines is not None:
        for (x1, y1, x2, y2) in lines[0]:
            cv2.line(frame, (x1, y1), (x2, y2), (0, 255, 0), 2)

    # show the images

    cv2.imshow('mask', mask)
    cv2.imshow('skeleton', edges)
    cv2.imshow("images", np.hstack([frame]))



vs.release()
cv2.destroyAllWindows()