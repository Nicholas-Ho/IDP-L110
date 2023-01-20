import cv2
import matplotlib.pyplot as plt
from skimage.morphology import skeletonize
from sklearn.cluster import AgglomerativeClustering

# Parameters
thresh_ratio = 3
low_thresh = 50
hi_thresh = thresh_ratio*low_thresh

binary_thresh = 15

def find_junctions(img):
    max_val = img.max()
    img = img//max_val

    junctions = []

    h, w = img.shape

    # We're not expecting the path to be at the very edges of the image, so it's okay to leave out the edges
    for i in range(1,h-1):
        for j in range(1, w-1):
            # Since the image should only contain 1 pixel-wide skeletons, any activated pixel that has more than 2 neighbours is most likely a junction.
            if img[i][j] == 1:
                if (img[i-1][j-1] 
                        + img[i-1][j]
                        + img[i-1][j+1]
                        + img[i][j-1]
                        + img[i][j+1]
                        + img[i+1][j-1]
                        + img[i+1][j]
                        + img[i+1][j+1]) > 2: # More than 2 white pixels out of 8
                    junctions.append((j,i))

    # Clustering
    ward = AgglomerativeClustering(n_clusters=None, linkage='ward', distance_threshold=10)
    labels = ward.fit_predict(junctions)
    clusters = {}
    for junction, label in zip(junctions, labels):
        try:
            clusters[label].append(junction)
        except:
            clusters[label] = [junction]

    junctions = [[int(sum(x)/len(x)) for x in zip(*cluster)] for cluster in clusters.values()]

    return junctions

def process_img(img):
    # Edge detection
    processed_img = cv2.Canny(img, low_thresh, hi_thresh)

    # # Experiment 1: Hough Lines
    # # Line detection
    # lines = cv2.HoughLinesP(processed_img, 1, np.pi/180, 50, minLineLength=50,maxLineGap=20)

    # # Draw lines
    # for line in lines:
    #     x1,y1,x2,y2 = line[0]
    #     cv2.line(processed_img,(x1,y1),(x2,y2),(255,255,0),2)

    # Experiment 2: Blurring, Thresholding and Skeletonization
    processed_img = cv2.blur(processed_img, (10,10))
    ret, processed_img = cv2.threshold(processed_img, binary_thresh, 255, cv2.THRESH_BINARY)
    processed_img = skeletonize(processed_img//255).astype(int)*255

    # Find junctions
    junctions = find_junctions(processed_img)
    for junction in junctions:
        processed_img = cv2.circle(processed_img, junction, radius=5, color=(255, 255, 255), thickness=-1)

    return processed_img


img = cv2.imread('sample_picture_dark_19.01.2023.png', cv2.IMREAD_GRAYSCALE)

# plt.imshow(img, cmap='gray')
# plt.show()

processed_img = process_img(img)

plt.imshow(processed_img, cmap='gray')
plt.show()
