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
cyan_low = (160, 70, 50)
cyan_high = (200, 255, 255)

crop = {'y': (50, 700), 'x': (200, 400)}

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
    ward = AgglomerativeClustering(n_clusters=None, linkage='ward', distance_threshold=30)
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
    # Cropping left side of image to isolate junctions with block
    zoomed_img = img[crop['y'][0]:crop['y'][1], crop['x'][0]:crop['x'][1]]

    # Inverting the image and looking for the colour cyan -- same as looking for red in non-inverted space
    hsv_img = cv2.cvtColor(zoomed_img, cv2.COLOR_RGB2HSV)
    mask = cv2.inRange(hsv_img, cyan_low, cyan_high)

    # Looking for the largest cyan contour -- this should correspond to the block
    contours, hierarchy = cv2.findContours(mask, cv2.RETR_TREE, cv2.CHAIN_APPROX_SIMPLE)
    if len(contours) > 0:
        max_contour = max(contours, key=cv2.contourArea)

        # Find centre of contour
        M = cv2.moments(max_contour)
        if M['m00'] != 0:
            cx = int(M['m10']/M['m00']) + crop['x'][0] # Converting coordinates back to uncropped
            cy = int(M['m01']/M['m00']) + crop['y'][0]

    return (cx, cy), max_contour

def find_block_junctions(junctions, blocks):
    def euclidean_dist(pt1, pt2):
        return np.sqrt((pt1[0]-pt2[0])**2 + (pt1[1]-pt2[1])**2)

    block_junctions = []
    for block in blocks:
        block_junctions.append((min(junctions, key=lambda x: euclidean_dist(x, block)), block)) # (junction, block)

    return block_junctions


img_path = 'sample_picture_block1_20.01.2023.png'

img = cv2.imread(img_path, cv2.IMREAD_COLOR)
img = cv2.cvtColor(img, cv2.COLOR_BGR2RGB)

# Find junctions
junctions = find_junctions(img)

# Draw junctions
for junction in junctions:
    img = cv2.circle(img, junction, radius=5, color=(255, 0, 0), thickness=-1)

# Detect red block
block_centre, contour = find_red(img)

# Draw red block
x,y,w,h = cv2.boundingRect(contour)
zoomed_img = img[crop['y'][0]:crop['y'][1], crop['x'][0]:crop['x'][1]]
cv2.rectangle(zoomed_img,(x,y),(x+w,y+h),(0,255,0),2)
cv2.putText(zoomed_img, "Red Block", (x, y-10), cv2.FONT_HERSHEY_SIMPLEX, 0.5, (0,255,0), 2)

# Find junctions closest to the block
block_junctions = find_block_junctions(junctions, [block_centre])

# Draw junctions
for junction in block_junctions:
    img = cv2.circle(img, junction[0], radius=10, color=(0, 255, 0), thickness=-1)

# Show final image
img = cv2.cvtColor(img, cv2.COLOR_RGB2BGR) # Converting back to BGR space
cv2.imshow("Mask", img)
cv2.waitKey(0)