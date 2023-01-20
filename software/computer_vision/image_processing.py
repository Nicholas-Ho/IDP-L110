import cv2
import matplotlib.pyplot as plt
from skimage.morphology import skeletonize
from sklearn.cluster import AgglomerativeClustering
from scipy.ndimage import convolve
import numpy as np

# Junction Detection Parameters
thresh_ratio = 3
low_thresh = 50
hi_thresh = thresh_ratio*low_thresh
binary_thresh_j = 15

# Block Detection Parameters
red_low_1 = (0,75,150)
red_hi_1 = (5,255,255)
red_low_2 = (175,75,150)
red_hi_2 = (180,255,255)


def _process_img(img):
    # Blurring to smear non-targets
    img = cv2.blur(img, (4,4))

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

    h, w = img.shape

    kernel = [[1, 1, 1],
              [1, 0, 1],
              [1, 1, 1]]
    # Since the image should only contain 1 pixel-wide skeletons, any activated pixel that has more than 2 neighbours is most likely a junction.
    # We're not expecting the path to be at the very edges of the image, so it's okay to leave out the edges
    junction_mat = convolve(img, kernel, mode='constant', cval=0).astype(np.uint8)
    ret, junction_mat = cv2.threshold(junction_mat, 3, 255, cv2.THRESH_BINARY)

    junctions = np.array(list(zip(*(np.nonzero(junction_mat)))))
    junctions = np.flip(junctions, axis=1)

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

def find_red(img):
    rgb_img = cv2.cvtColor(img, cv2.COLOR_RGB2GRAY)
    ret, rgb_img = cv2.threshold(rgb_img, 25, 255, 0)
    contours, hierarchy = cv2.findContours(rgb_img, cv2.RETR_TREE, cv2.CHAIN_APPROX_SIMPLE)
    objects = cv2.drawContours(np.zeros(img.shape, dtype=np.uint8), contours, -1, (255, 255, 255), 2)
    plt.imshow(objects)
    plt.show()

    # Convert to HSV
    hsv_img = cv2.cvtColor(img, cv2.COLOR_RGB2HSV)
    mask1 = cv2.inRange(hsv_img, red_low_1, red_hi_1)
    mask2 = cv2.inRange(hsv_img, red_low_2, red_hi_2)

    mask = cv2.bitwise_or(mask1, mask2)
    mask = cv2.bitwise_and(mask1, objects)
    plt.imshow(mask)
    plt.show()

    

# img_path = 'sample_picture_bright_20.01.2023.png'
# img_path = 'sample_picture_dark_19.01.2023.png'
img_path = 'sample_picture_block_20.01.2023.png'

img = cv2.imread(img_path, cv2.IMREAD_COLOR)
img = cv2.cvtColor(img, cv2.COLOR_BGR2RGB)

# Find junctions
junctions = find_junctions(img)
for junction in junctions:
    img = cv2.circle(img, junction, radius=15, color=(255, 0, 0), thickness=-1)

# find_red(img)

plt.imshow(img)
plt.show()
