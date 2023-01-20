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


def _process_img(img):
    # Blurring to smear non-targets
    img = cv2.blur(img, (4,4))

    # Edge detection
    img = cv2.Canny(img, low_thresh, hi_thresh)

    plt.imshow(img, cmap='gray')
    plt.show()

    # Blurring, Thresholding and Skeletonization
    img = cv2.blur(img, (10,10))
    plt.imshow(img, cmap='gray')
    plt.show()
    ret, img = cv2.threshold(img, binary_thresh_j, 255, cv2.THRESH_BINARY)
    plt.imshow(img, cmap='gray')
    plt.show()
    img = skeletonize(img//255).astype(int)
    plt.imshow(img, cmap='gray')
    plt.show()

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

def find_red():
    pass

img_path = 'sample_picture_bright_20.01.2023.png'
# img_path = 'sample_picture_dark_19.01.2023.png'

img = cv2.imread(img_path, cv2.IMREAD_COLOR)
img = cv2.cvtColor(img, cv2.COLOR_BGR2RGB)

# plt.imshow(img)
# plt.show()

# Find junctions
junctions = find_junctions(img)
for junction in junctions:
    img = cv2.circle(img, junction, radius=15, color=(255, 0, 0), thickness=-1)

plt.imshow(img)
plt.show()
