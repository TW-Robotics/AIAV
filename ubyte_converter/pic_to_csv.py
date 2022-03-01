#!/usr/bin/env python3

import os
import cv2
import numpy as np
from PIL import Image
import glob

img_path = glob.glob('data/*.png')
output_data = "output.csv"


# read image
img = np.array([np.array(Image.open(fname)) for fname in img_path])

#img = img.reshape(:, img.shape[0]*img.shape[1])
print(img.shape)

# dump into csv