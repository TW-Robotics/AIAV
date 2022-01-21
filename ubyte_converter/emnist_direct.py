#!/usr/bin/env python3


from cProfile import label
from emnist import list_datasets
from emnist import extract_training_samples
import cv2
import os

from numpy import save
from sklearn.feature_extraction import image
convert_add =96

# create folder 
path = "output"
os.mkdir(path)
for i in range (26):
    os.mkdir(path + "/" + chr(i+1 + convert_add))


# get dataset 
print(list_datasets())
images, labels = extract_training_samples('letters')
print(images.shape)
print(labels.shape)



# safe images as png
nbr_images = labels.size
for i in range(nbr_images):
    current_label = chr(labels[i] + convert_add) 
    name = str("_%05d" %i)
    save_as = path + "/" + current_label + "/" + name + ".png"
    cv2.imwrite(save_as, images[i])



