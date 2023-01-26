import cv2
import matplotlib.pyplot as plt
import numpy as np

def detect_qr(img, qr):
    img = cv2.cvtColor(img, cv2.COLOR_RGB2GRAY)
    
    # Detect key points using KAZE detector
    detector = cv2.KAZE_create()
    keypoints_qr, descriptors_qr = detector.detectAndCompute(qr, None)
    keypoints_img, descriptors_img = detector.detectAndCompute(img, None)

    img2 = cv2.drawKeypoints(img, keypoints_img, None, (255,0,0),4)
    plt.imshow(img2)
    plt.show()
    img3 = cv2.drawKeypoints(qr, keypoints_img, None, (255,0,0),4)
    plt.imshow(img3)
    plt.show()

    # Match descriptor vectors with a FLANN based matcher
    FLANN_INDEX_KDTREE = 1
    index_params = dict(algorithm = FLANN_INDEX_KDTREE,
                        trees = 5)
    search_params = dict(checks = 50)

    descriptors_img = np.float32(descriptors_img)
    descriptors_qr = np.float32(descriptors_qr)

    matcher = cv2.FlannBasedMatcher(indexParams=index_params, searchParams=search_params)
    knn_matches = matcher.knnMatch(descriptors_qr, descriptors_img, 2)

    # Filter matches using the Lowe's ratio test
    ratio_thresh = 0.75
    good_matches = []
    for m, n in knn_matches:
        if m.distance < ratio_thresh * n.distance:
            good_matches.append(m)

    #-- Draw matches
    img_matches = np.empty((max(qr.shape[0], img.shape[0]), qr.shape[1]+img.shape[1], 3), dtype=np.uint8)
    cv2.drawMatches(qr, keypoints_qr, img, keypoints_img, good_matches, img_matches, matchColor=(255,0,0), flags=cv2.DrawMatchesFlags_NOT_DRAW_SINGLE_POINTS)
    plt.imshow(img_matches)
    plt.show()

    print(len(good_matches))

    return []

img_path = 'sample_picture_qr_26.01.2023.png'
qr_path = 'qr_code.png'

img = cv2.imread(img_path, cv2.IMREAD_COLOR)
img = cv2.cvtColor(img, cv2.COLOR_BGR2RGB)
img = img[50:700, 100:900]
qr = cv2.imread(qr_path, cv2.IMREAD_GRAYSCALE)
# qr = cv2.resize(qr, (100, 100), interpolation=cv2.INTER_AREA)
plt.imshow(img)
plt.show()

plt.imshow(qr, cmap='gray')
plt.show()

print(img.shape)
print(qr.shape)

qr = detect_qr(img, qr)

for point in qr:
    img = cv2.circle(img, point, radius=5, color=(255, 0, 0), thickness=-1)

# plt.imshow(img)
# plt.show()