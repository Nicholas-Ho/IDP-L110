import cv2
import matplotlib.pyplot as plt

def detect_qr(img):
    img = cv2.cvtColor(img, cv2.COLOR_RGB2GRAY)
    ret, img = cv2.threshold(img, 240, 255, cv2.THRESH_BINARY)
    img = cv2.GaussianBlur(img, (3, 3), 0)
    cv2.imshow("Mask", img)
    cv2.waitKey(0)
    detector = cv2.QRCodeDetector()
    ret, points = detector.detect(img)
    print(points)

    return points

img_path = 'sample_picture_qr_26.01.2023.png'

img = cv2.imread(img_path, cv2.IMREAD_COLOR)
cv2.imshow("Mask", img)
cv2.waitKey(0)
img = cv2.cvtColor(img, cv2.COLOR_BGR2RGB)

print(img.shape)

qr = detect_qr(img)

for point in qr:
    img = cv2.circle(img, point, radius=5, color=(255, 0, 0), thickness=-1)

plt.imshow(img)
plt.show()