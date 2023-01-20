import cv2
from urllib import request
import numpy as np

from .image_processing import process_img

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
        processed_img = process_img(i)
        cv2.imshow('Processed', processed_img)
        break
        # if cv2.waitKey(1) == 27:
        #     exit(0)