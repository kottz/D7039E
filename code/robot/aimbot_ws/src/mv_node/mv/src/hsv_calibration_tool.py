import cv2
import numpy as np

def get_mask(image, hsv_start, hsv_range, s_min, s_max, v_min, v_max):
    lower = np.array([hsv_start, s_min, v_min])
    upper = np.array([min(179,hsv_start+hsv_range), s_max, v_max])
    hsv = cv2.cvtColor(image, cv2.COLOR_BGR2HSV)
    mask = cv2.inRange(hsv, lower, upper)
    if hsv_start + hsv_range > 179:
        hsv_end = (hsv_start+hsv_range) % 180
        lower = np.array([0, s_min, v_min])
        upper = np.array([hsv_end, s_max, v_max])
        hsv = cv2.cvtColor(image, cv2.COLOR_BGR2HSV)
        mask2 = cv2.inRange(hsv, lower, upper)
        mask = cv2.bitwise_or(mask, mask2)

    return mask
    
def nothing(x):
    pass

# Load image
image = cv2.imread('red_line.jpg')

# Create a window
cv2.namedWindow('image')

# Create trackbars for color change
# Hue is from 0-179 for Opencv
cv2.createTrackbar('hsv_start', 'image', 0, 179, nothing)
cv2.createTrackbar('hsv_range', 'image', 0, 179, nothing)
cv2.createTrackbar('s_min', 'image', 0, 255, nothing)
cv2.createTrackbar('v_min', 'image', 0, 255, nothing)
cv2.createTrackbar('s_max', 'image', 0, 255, nothing)
cv2.createTrackbar('v_max', 'image', 0, 255, nothing)

# Set default value for Max HSV trackbars
cv2.setTrackbarPos('hsv_range', 'image', 10)
cv2.setTrackbarPos('s_max', 'image', 255)
cv2.setTrackbarPos('v_max', 'image', 255)

# Initialize HSV min/max values
hsv_start = s_min = v_min = h_max = s_max = v_max = 0
prev_hsv_start = prev_s_min = prev_v_min = prev_h_max = prev_s_max = prev_v_max = 0

while(1):
    # Get current positions of all trackbars
    hsv_start = cv2.getTrackbarPos('hsv_start', 'image')
    hsv_range = cv2.getTrackbarPos('hsv_range', 'image')
    s_min = cv2.getTrackbarPos('s_min', 'image')
    v_min = cv2.getTrackbarPos('v_min', 'image')
    s_max = cv2.getTrackbarPos('s_max', 'image')
    v_max = cv2.getTrackbarPos('v_max', 'image')

    # Set minimum and maximum HSV values to display
    #lower = np.array([hsv_start, s_min, v_min])
    #upper = np.array([h_max, s_max, v_max])

    # Convert to HSV format and color threshold
    #hsv = cv2.cvtColor(image, cv2.COLOR_BGR2HSV)
    #mask = cv2.inRange(hsv, lower, upper)
    mask = get_mask(image, hsv_start, hsv_range, s_min, s_max, v_min, v_max)
    result = cv2.bitwise_and(image, image, mask=mask)

    # Print if there is a change in HSV value
    if((prev_hsv_start != hsv_start) | (prev_s_min != s_min) | (prev_v_min != v_min) | (prev_h_max != h_max) | (prev_s_max != s_max) | (prev_v_max != v_max) ):
        print("(hsv_start = %d , s_min = %d, v_min = %d), (h_max = %d , s_max = %d, v_max = %d)" % (hsv_start , s_min , v_min, h_max, s_max , v_max))
        prev_hsv_start = hsv_start
        prev_s_min = s_min
        prev_v_min = v_min
        prev_h_max = h_max
        prev_s_max = s_max
        prev_v_max = v_max

    # Display result image
    cv2.imshow('image', result)
    if cv2.waitKey(10) & 0xFF == ord('q'):
        break

cv2.destroyAllWindows()


