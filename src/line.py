import numpy as np
import cv2
from vector import Vector

from imutils.video import VideoStream
from imutils.video import FPS
import imutils


vs = cv2.VideoCapture('red_cable.mp4')

width = int(vs.get(cv2.CAP_PROP_FRAME_WIDTH) + 0.5)
height = int(vs.get(cv2.CAP_PROP_FRAME_HEIGHT) + 0.5)
size = (width*2, height)                        # change here if you want to output multiple frames in one video
#fourcc = cv2.VideoWriter_fourcc(*'XVID')
#out = cv2.VideoWriter('your_video.avi', fourcc, 20.0, size)

cv2.namedWindow('image', cv2.WINDOW_NORMAL)
cv2.resizeWindow('image', 1024,768)
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
    #lines = cv2.HoughLinesP(
    #    edges,
    #    1,
    #    np.pi / 180,
    #    100,
    #    minLineLength,
     #   maxLineGap,
    #    )
    #if lines is not None:
    #    for (x1, y1, x2, y2) in lines[0]:
    #        cv2.line(frame, (x1, y1), (x2, y2), (0, 255, 0), 2)


    #extract slice
    sizeX = frame.shape[1]
    sizeY = frame.shape[0]
    camX = sizeX // 2
    camY = sizeY - 0

    vertical_size = 10
    crop_img = mask[(sizeY-vertical_size)//2:(sizeY+vertical_size)//2, 0:sizeX]
    
    m = cv2.moments(crop_img)
    if m["m00"] == 0:
        cX = 0
    else:
        cX = int(m["m10"] / m["m00"])
    
    
    #cam to line
    track_point = Vector(cX, sizeY//2)
    cam = Vector(camX, camY)
    cv2.line(frame, tuple(cam), tuple(track_point), (255, 0, 0), 2)

    #calculate normalized line (unit circle)
    radius = 80
    u = track_point - cam
    u = u.normalize() * radius
    u = u.as_int()
    angle = u.argument() - 90
    print(int(angle))
    cv2.line(frame, (camX, camY), tuple(cam+u), (0, 0, 0), 5)
    cv2.ellipse(frame, tuple(cam), (int(u.norm()), int(u.norm())), 0, 0, -u.argument()+90, (0,0,0), 5)
    #
    cv2.putText(frame, str(int(angle)), tuple(cam+Vector(int(radius)+10, -10)), cv2.FONT_HERSHEY_SIMPLEX, 1, (0,0,0), 2)

    #cropped region
    upper_bound = (sizeY-vertical_size)//2
    lower_bound = (sizeY+vertical_size)//2
    cv2.line(frame, (0,upper_bound), (sizeX,upper_bound), (0, 0, 255), 2)
    cv2.line(frame, (0,lower_bound), (sizeX,lower_bound), (0, 0, 255), 2)

    # show the images
    #outmask = cv2.cvtColor(mask, cv2.COLOR_GRAY2RGB)
    outmask = cv2.cvtColor(mask, cv2.COLOR_BGR2RGB)
    outskeleton = cv2.cvtColor(edges, cv2.COLOR_GRAY2RGB)
    #cv2.imshow('mask', mask)
    #cv2.imshow('skeleton', edges)
    row1 = np.hstack([frame, outmask])
    row2 = np.hstack([outskeleton, outskeleton])
    output = np.vstack([row1, row2])
    cv2.imshow("image", row1)
    #cv2.imshow('crop', crop_img)
    #row1 = cv2.cvtColor(row1, cv2.COLOR_)
    #out.write(row1)


#out.release()
vs.release()
cv2.destroyAllWindows()


