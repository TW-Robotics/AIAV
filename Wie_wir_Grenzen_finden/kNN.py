#!/bin/env/python3

# Dieses Skript entfernt den Hintergrund aus einem Bild mittels k-Nearest Neighbour.
#
# This code is available under a GPL v3.0 license and comes without
# any explicit or implicit warranty.
#
# (C) Simon Schwaiger 2021 <schwaige@technikum-wien.at>

import cv2
import numpy as np
from skimage.metrics import structural_similarity as ssim
from skimage.measure import block_reduce
from sklearn import neighbors

from PIL import Image

def scaleImage(img, scaleFactor=2):
        """ Skaliert das Bild um scaleFactor herunter """
        width = int(len(img[0])/scaleFactor)
        height = int(len(img)/scaleFactor)
        frame = cv2.resize(img,(width, height), fx=0, fy=0, interpolation=cv2.INTER_LINEAR)
        return img

def formatInput(frame):
        """ Formatiert frame als Liste von Einträgen für einfachere Klassifizierung """
        ret = []
        for x in range(len(frame)):
                for y in range(len(frame[0])):
                        ret.append([x, y, frame[x][y]])
        return ret

def formatImage(inp):
        """ Formatiert die Liste von Einträgen nach der Klassifizierung wieder als Bild """
        xRes = max([entry[0] for entry in inp])+1
        yRes = max([entry[1] for entry in inp])+1
        ret = []
        for x in range(xRes):
                ret.append([0 for y in range(yRes)])
        for entry in inp:
                ret[entry[0]][entry[1]]=entry[2]
        return np.array(ret)

def applykNN(frame, K=15):
        """ Wendet kNN Klassifizierung auf ein Schwarz/Weiß Bild an, um Artefakte zu entfernen """
        frame = formatInput(frame)
        X = np.array([[entry[0], entry[1]] for entry in frame])
        y = np.array([entry[2] for entry in frame])
        clf = neighbors.KNeighborsClassifier(K, weights='uniform')
        clf.fit(X, y)
        Z = clf.predict(X)
        ret = [ [X[i][0], X[i][1], Z[i]] for i in range(len(X)) ]
        return formatImage(ret)

# Festelgen von Referenzbild und zu untersuchendem Bild
img1Path = './testImages/bild1.jpg'
img2Path = './testImages/bild2.jpg'

img1 = cv2.imread(img1Path)
img2 = cv2.imread(img2Path)

# Skalierung der Bilder
img1 = scaleImage(img1)
img2 = scaleImage(img2)

# Konvertierung der Bilder zu Graustufen
img1 = cv2.cvtColor(img1, cv2.COLOR_BGR2GRAY)
img2 = cv2.cvtColor(img2, cv2.COLOR_BGR2GRAY)

Image.fromarray(img2).show()

# Ermittlung des Unterschieds zwischen Vorder- und Hintergrund
ssimRet, ssimImage = ssim(img1, img2, data_range=img2.max() - img2.min(), full=True)
threshhold = 0.5

Image.fromarray(ssimImage*255).show()

# Herunterskalierung des Bildes zur Anwendung von kNN
dim = (4,4)
ssimImage = block_reduce(ssimImage, block_size=dim, func=np.mean)

# Jeder Pixel wird als Vorder- oder Hintergrund beschriftet
# Vordergrund => 0, Hintergrund => 255
ssimImage[ssimImage>threshhold] = 255
ssimImage[ssimImage<=threshhold] = 0

Image.fromarray(ssimImage).show()

# Training und Klassifizierung mittels kNN
# So werden die Grenzen zwischen Vorder- und Hintergrund gefunden
tmp = applykNN(ssimImage, K=60)

Image.fromarray(tmp).show()

# Hochskalierung der Maske auf die originale Auflösung
width = len(img2[0])
height =len(img2) 
tmp = cv2.resize(tmp,(width, height), fx=0, fy=0, interpolation=cv2.INTER_LINEAR)

Image.fromarray(tmp).show()

# Hintergrund wird anhand der Maske aus dem originalen Bild entfernt
result = np.array(img2)
result[tmp==255] = 0

Image.fromarray(result).show()
