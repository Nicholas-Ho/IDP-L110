import cv2
import matplotlib.pyplot as plt
import numpy as np

from scipy.spatial import distance

# termination criteria
criteria = (cv2.TERM_CRITERIA_EPS + cv2.TERM_CRITERIA_MAX_ITER, 30, 0.001)

def detect_qr(img):
    
    img = cv2.cvtColor(img, cv2.COLOR_RGB2GRAY)
    
    # Detect key points using KAZE detector
    # Find the chess board corners
    ret, corners = cv2.findChessboardCorners(img, (9,6), None)

    # If found, add object points, image points (after refining them)
    if ret == True:
        corners2 = cv2.cornerSubPix(img,corners, (11,11), (-1,-1), criteria)
        # Draw and display the corners
        cv2.drawChessboardCorners(img, (9,6), corners2, ret)
        cv2.imshow('img', img)
        cv2.waitKey(500)
    else:
        print('No corners found.')

    return corners

img_path = 'sample_pictures/camera_calibration/camera_2/sample_picture_camera2_calib1_27.01.2023.png'

img = cv2.imread(img_path, cv2.IMREAD_COLOR)
img = cv2.cvtColor(img, cv2.COLOR_BGR2RGB)
img = img[50:700, 100:900]
plt.imshow(img)
plt.show()

print(img.shape)

qr = detect_qr(img)

# # Draw junctions
# img = cv2.circle(img, qr, radius=10, color=(0, 255, 0), thickness=-1)

# plt.imshow(img)
# plt.show()