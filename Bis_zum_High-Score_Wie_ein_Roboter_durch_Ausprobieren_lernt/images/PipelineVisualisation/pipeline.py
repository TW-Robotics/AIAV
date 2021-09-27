#!/bin/env/python

import cv2 as cv
from PIL import Image

import numpy as np

import random as rng

path = './image.png'
img = cv.imread(path)

img = cv.cvtColor(img, cv.COLOR_BGR2GRAY)

# print greyscale image
Image.fromarray(img).show()
Image.fromarray(img).save('grey.png')

ret, thresh1 = cv.threshold(img, 130, 255, cv.THRESH_BINARY)

# print thressholded image
Image.fromarray(thresh1).show()
Image.fromarray(thresh1).save('thresh1.png')

canny_output = cv.Canny(thresh1, 30, 150)

# print canny image
Image.fromarray(canny_output).show()
Image.fromarray(canny_output).save('canny.png')



##############################
# draw contours

contours, _ = cv.findContours(canny_output, cv.RETR_TREE, cv.CHAIN_APPROX_SIMPLE)


contours_poly = [None]*len(contours)
boundRect = [None]*len(contours)
centers = [None]*len(contours)
radius = [None]*len(contours)
for i, c in enumerate(contours):
    contours_poly[i] = cv.approxPolyDP(c, 3, True)
    boundRect[i] = cv.boundingRect(contours_poly[i])
    centers[i], radius[i] = cv.minEnclosingCircle(contours_poly[i])


drawing = np.zeros((canny_output.shape[0], canny_output.shape[1], 3), dtype=np.uint8)


for i in range(len(contours)):
    color = (rng.randint(0,256), rng.randint(0,256), rng.randint(0,256))
    cv.drawContours(drawing, contours_poly, i, color)
    cv.rectangle(drawing, (int(boundRect[i][0]), int(boundRect[i][1])), \
        (int(boundRect[i][0]+boundRect[i][2]), int(boundRect[i][1]+boundRect[i][3])), color, 2)
    cv.circle(drawing, (int(centers[i][0]), int(centers[i][1])), int(radius[i]), color, 2)

Image.fromarray(drawing).show()
Image.fromarray(drawing).save('box.png')
