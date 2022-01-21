#!/usr/bin/env python3

#https://gist.github.com/fukuroder/caa351677bf718a8bfe6265c2a45211f


import os
import cv2
import numpy as np
from numpy import imag

train_image = 'gzip/emnist-letters-train-images-idx3-ubyte'
train_label = 'gzip/emnist-letters-train-labels-idx1-ubyte'
	
for f in [train_image, train_label]:
	os.system('gunzip %s.gz' % (f,))



break_count = 0; 
for image_f, label_f in [(train_image, train_label)]:
    
    with open(image_f, 'rb') as f:
        images = f.read()
    with open(label_f, 'rb') as f:
        labels = f.read()

        
    images = [d for d in images[16:]]
    images = np.array(images, dtype=np.uint8)
    images = images.reshape((-1,28,28))
    
    outdir = image_f + "_folder_OUTPUT"
    if not os.path.exists(outdir):
        os.mkdir(outdir)
    for k,image in enumerate(images):
        break_count = break_count+1
        if(break_count > 20):
            print("BREAK Images")
            break
        else:
            cv2.rotate(image, cv2.ROTATE_90_CLOCKWISE, image)
            cv2.imwrite(os.path.join(outdir, '%05d.png' % (k,)), image)
    
    labels = [outdir + '/%05d.png %d' % (k, l) for k,l in enumerate(labels[8:])]
    with open('%s.txt' % label_f, 'w') as f:
        f.write(os.linesep.join(labels))