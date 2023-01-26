import cv2
from urllib import request
import numpy as np

name = 'sample_picture_qr_26.01.2023.png'

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
        if cv2.waitKey(1) == ord('s'):
            cv2.imwrite(name, i)
            img = cv2.imread(name, cv2.IMREAD_COLOR)
            print('img res: ', img.shape)
        if cv2.waitKey(1) == 27:
            exit(0)