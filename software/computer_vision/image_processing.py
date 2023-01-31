import cv2
import matplotlib.pyplot as plt
from skimage.morphology import skeletonize
from sklearn.cluster import AgglomerativeClustering
from scipy.ndimage import convolve
import numpy as np

import argparse


parser = argparse.ArgumentParser()
parser.add_argument('camera', default=1, type=int)
args = parser.parse_args()


# Junction Detection Parameters
thresh_ratio = 3
low_thresh = 50
hi_thresh = thresh_ratio*low_thresh
binary_thresh_j = 15

# Block Detection Parameters
cyan_low = (160, 70, 50)
cyan_high = (200, 255, 255)

blue_low = (180, 100, 20)
blue_high = (300, 255, 255)

red_low_1 = (0, 100, 20)
red_high_1 = (10, 255, 255)

red_low_2 = (160, 100, 20)
red_high_2 = (179, 255, 255)

# Block Detection Settings for Cameras (each camera has a different resolution)
camera = args.camera # 1 or 2

# Camera 1
block_crop_1 = {'y': (50, 700), 'x': (200, 400)}
block_size_thresh_1 = 100

# Camera 2
block_crop_2 = {'y': (50, 700), 'x': (150, 350)}
block_size_thresh_2 = 100

delivery_crop = {'y': (50, 700), 'x': (725, 900)}
delivery_size_thresh = 500

if camera == 1:
    block_crop = block_crop_1
    block_size_thresh = block_size_thresh_1
else:
    block_crop = block_crop_2
    block_size_thresh = block_size_thresh_2

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
    zoomed_img = img[block_crop['y'][0]:block_crop['y'][1], block_crop['x'][0]:block_crop['x'][1]]
    
    # Inverting the image and looking for the colour cyan -- same as looking for red in non-inverted space
    hsv_img = cv2.cvtColor(zoomed_img, cv2.COLOR_RGB2HSV)
    mask = cv2.inRange(hsv_img, cyan_low, cyan_high)

    # Looking for the largest cyan contour -- this should correspond to the block
    contours, hierarchy = cv2.findContours(mask, cv2.RETR_TREE, cv2.CHAIN_APPROX_SIMPLE)
    output = []
    for contour in contours:
        if cv2.contourArea(contour) > block_size_thresh:
            # Find centre of contour
            M = cv2.moments(contour)
            if M['m00'] != 0:
                cx = int(M['m10']/M['m00']) + block_crop['x'][0] # Converting coordinates back to uncropped
                cy = int(M['m01']/M['m00']) + block_crop['y'][0]
            output.append({'centre': (cx, cy), 'contour': contour})
    return output

def find_block_junctions(junctions, blocks):
    def euclidean_dist(pt1, pt2):
        return np.sqrt((pt1[0]-pt2[0])**2 + (pt1[1]-pt2[1])**2)

    block_junctions = []
    for block in blocks:
        block_junctions.append((min(junctions, key=lambda x: euclidean_dist(x, block['centre'])), block)) # (junction, block)

    return block_junctions

def find_delivery_area(img):

    zoomed_img = zoomed_img = img[delivery_crop['y'][0]:delivery_crop['y'][1], delivery_crop['x'][0]:delivery_crop['x'][1]]

    hsv_img = cv2.cvtColor(zoomed_img, cv2.COLOR_BGR2HSV)
    mask2 = cv2.inRange(hsv_img, blue_low, blue_high)

    #Inverting the image and looking for the colour cyan in HSV space-- same as looking for red in non-inverted space
    hsv_img = cv2.cvtColor(zoomed_img, cv2.COLOR_RGB2HSV)
    mask1 = cv2.inRange(hsv_img, cyan_low, cyan_high)

    plt.imshow(hsv_img)

    contours, hierarchy = cv2.findContours(mask1, cv2.RETR_TREE, cv2.CHAIN_APPROX_SIMPLE)

    # cv2.imshow("Zoomed", zoomed_img)
    # cv2.waitKey(0)

    plt.imshow(mask1 + mask2)
    plt.show()

# Some of the images are saved as half-resolution for some reason? Investigate
img_path = 'sample_picture_block2_25.01.2023.png'

img = cv2.imread(img_path, cv2.IMREAD_COLOR)
img = cv2.cvtColor(img, cv2.COLOR_BGR2RGB)

# Find junctions
junctions = find_junctions(img)

# #  Draw junctions
# for junction in junctions:
#     img = cv2.circle(img, junction, radius=5, color=(255, 0, 0), thickness=-1)

# Detect red block
blocks = find_red(img)

# Draw red block
for block in blocks:
    x,y,w,h = cv2.boundingRect(block['contour'])
    zoomed_img = img[block_crop['y'][0]:block_crop['y'][1], block_crop['x'][0]:block_crop['x'][1]]
    cv2.rectangle(zoomed_img,(x,y),(x+w,y+h),(0,255,0),2)
    cv2.putText(zoomed_img, "Red Block", (x, y-10), cv2.FONT_HERSHEY_SIMPLEX, 0.5, (0,255,0), 2)

# Find junctions closest to the block
block_junctions = find_block_junctions(junctions, blocks)

# Draw junctions
#for junction in block_junctions:
    #img = cv2.circle(img, junction[0], radius=10, color=(0, 255, 0), thickness=-1)

find_delivery_area(img)

# Show final image
img = cv2.cvtColor(img, cv2.COLOR_RGB2BGR) # Converting back to BGR space
cv2.imshow("Mask", img)
cv2.waitKey(0)