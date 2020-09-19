import numpy as np
import cv2

from imutils.video import VideoStream
from imutils.video import FPS
import imutils

vs = cv2.VideoCapture('red_cable.mp4')
width = int(vs.get(cv2.CAP_PROP_FRAME_WIDTH) + 0.5)
height = int(vs.get(cv2.CAP_PROP_FRAME_HEIGHT) + 0.5)
size = (width, height)
fourcc = cv2.VideoWriter_fourcc(*'XVID')
out = cv2.VideoWriter('your_video.avi', fourcc, 20.0, size)
while True:
    ret, frame = vs.read()
    
    if ret != True:
        break
    
    if cv2.waitKey(1) & 0xFF == ord('q'):
        break

    hsv = cv2.cvtColor(frame, cv2.COLOR_BGR2HSV)
    # lower mask (0-10)
    lower_red = np.array([0,50,50])
    upper_red = np.array([10,255,255])
    mask0 = cv2.inRange(hsv, lower_red, upper_red)

    # upper mask (170-180)
    lower_red = np.array([170,50,50])
    upper_red = np.array([180,255,255])
    mask1 = cv2.inRange(hsv, lower_red, upper_red)

    mask = mask0 + mask1
    #output = cv2.bitwise_and(frame,frame, mask= mask)

    element = cv2.getStructuringElement(cv2.MORPH_CROSS,(9,9))
    kernel = np.ones((3,3),np.uint8)
    #eroded = cv2.erode(mask,kernel, iterations = 1)
    #dilate = cv2.dilate(eroded, element)
    #skeleton = cv2.subtract(mask, dilate)
    #skeleton = cv2.ximgproc.thinning(eroded)

    edges = cv2.Canny(mask, 50, 200)
    edges = cv2.dilate(edges, element)
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
    outmask = cv2.cvtColor(mask, cv2.COLOR_GRAY2RGB)
    out.write(outmask)
    cv2.imshow('mask', mask)
    cv2.imshow('skeleton', edges)
    cv2.imshow("images", np.hstack([frame]))


out.release()
vs.release()
cv2.destroyAllWindows()
