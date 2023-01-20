import cv2
import matplotlib.pyplot as plt
from skimage.morphology import skeletonize
from sklearn.cluster import AgglomerativeClustering
import numpy as np

# ROI Parameters
binary_thresh_r = 5
g_lower = (0, 40, 0)
g_upper = (60, 255, 60)

# Junction Detection Parameters
thresh_ratio = 3
low_thresh = 50
hi_thresh = thresh_ratio*low_thresh
binary_thresh_j = 15

# Table Object Removal Parameters
table_lower = (0, 40, 0)
table_upper = (60, 255, 60)

def remove_coloured_objects(img, lower, upper):
    # Remove coloured objects on table
    mask = cv2.inRange(img, lower, upper)
    masked = cv2.bitwise_and(img,img, mask=mask)
    plt.imshow(mask)
    img = img - masked
    return img

def _process_img(img):
    # Edge detection
    img = cv2.Canny(img, low_thresh, hi_thresh)

    # Blurring, Thresholding and Skeletonization
    img = cv2.blur(img, (10,10))
    ret, img = cv2.threshold(img, binary_thresh_j, 255, cv2.THRESH_BINARY)
    img = skeletonize(img//255).astype(int)

    return img

def find_junctions(img):
    img = _process_img(img)

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
    ward = AgglomerativeClustering(n_clusters=None, linkage='ward', distance_threshold=50)
    labels = ward.fit_predict(junctions)
    clusters = {}
    for junction, label in zip(junctions, labels):
        try:
            clusters[label].append(junction)
        except:
            clusters[label] = [junction]

    junctions = [[int(sum(x)/len(x)) for x in zip(*cluster)] for cluster in clusters.values()]

    return junctions

def get_roi(img):
    # Remove green objects on table
    img = remove_coloured_objects(img, g_lower, g_upper)

    # Find table contours
    img = cv2.cvtColor(img, cv2.COLOR_RGB2GRAY)
    ret, bin_img = cv2.threshold(img, binary_thresh_r, 255, cv2.THRESH_BINARY_INV)
    contours, hierarchy = cv2.findContours(bin_img, cv2.RETR_TREE, cv2.CHAIN_APPROX_SIMPLE)
    table_contour = max(contours, key=lambda x: len(x))
    roi = cv2.drawContours(np.zeros(img.shape, dtype=np.uint8), [table_contour], 0, (255, 255, 255), -1)

    # plt.imshow(roi)
    # plt.show()

    return roi

# img_path = 'sample_picture_bright_20.01.2023.png'
img_path = 'sample_picture_dark_19.01.2023.png'

img = cv2.imread(img_path, cv2.IMREAD_COLOR)
img = cv2.cvtColor(img, cv2.COLOR_BGR2RGB)

# plt.imshow(img)
# plt.show()

roi = get_roi(img)
processed_img = cv2.bitwise_and(img, img, mask=roi)
processed_img = remove_coloured_objects(processed_img, table_lower, table_upper)

# Find junctions
junctions = find_junctions(img)
for junction in junctions:
    img = cv2.circle(img, junction, radius=15, color=(255, 0, 0), thickness=-1)

plt.imshow(img)
plt.show()
