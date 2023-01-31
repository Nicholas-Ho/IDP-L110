import cv2
from urllib import request
import numpy as np

filename = 'sample_pictures/sample_picture_circles_30.01.2023.png'# 'sample_pictures/camera_calibration/camera_2/sample_picture_camera1_calib'# _27.01.2023.png'
capture = True

stream = request.urlopen('http://localhost:8081/stream/video.mjpeg')
bytes = b''
while True:
    bytes += stream.read(1024)
    a = bytes.find(b'\xff\xd8')
    b = bytes.find(b'\xff\xd9')
    if a != -1 and b != -1:
        jpg = bytes[a:b+2]
        bytes = bytes[b+2:]
        i = cv2.imdecode(np.fromstring(jpg, dtype=np.uint8), cv2.IMREAD_COLOR)
        cv2.imshow('Frame', i)

        # Capture image
        if cv2.waitKey(1) == ord('s') and capture:
            cv2.imwrite(filename, i)
            img = cv2.imread(filename, cv2.IMREAD_COLOR)
            print('img res: ', img.shape)

        # ESC to exit
        if cv2.waitKey(1) == 27:
            exit(0)