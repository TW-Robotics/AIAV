#/bin/env/python3

# Dieses Skript implementiert Vorbearbeitung und Abspeicherung der Bilder für das DCGAN Training.
#
# This code is available under a GPL v3.0 license and comes without
# any explicit or implicit warranty.
#
# (C) Simon Schwaiger 2021 <schwaige@technikum-wien.at>

import os
from os import listdir
from os.path import isfile, join

import cv2
import numpy as np

from PIL import Image

# Anlegen der Verzeichnisse für Trainingsdaten, generierte Bilder und Modelle
def createDir(directory):
    if not os.path.exists(directory):
        os.makedirs(directory)

createDir('./processedImages')
createDir('./genImages')
createDir('./savedModels')

#############################################################
#####           Bildverarbeitungsfunktionen             #####

def readImageDir(path, scaleFactor=2):
    """ Liest alle Bilder in einem Verzeichnis ein und skaliert sie """
    files = [f for f in listdir(path) if isfile(join(path, f))]
    imageData = []
    # Jedes Bild wird eingelesen und auf 128x128 Pixel Skaliert
    # Die Skalierung ist notwendig, um die Performance zu erhöhen
    for filename in files:
        # Einlesen der Bilddatei
        filename = '{}/{}'.format(path, filename)
        img = cv2.imread(filename)
        width = int(img.shape[0]/scaleFactor)
        height = int(img.shape[1]/scaleFactor)
        img = cv2.resize(img, (width, height), fx=0, fy=0, interpolation=cv2.INTER_LINEAR)
        imageData.append(np.asarray(img))
    # Alle Bilder werden in einem Numpy Array gespeichert
    return np.array(imageData)


def clusterImage(frame, K=2):
    """ Verwendet OpenCV K-Means Clustering um ein Bild in K Bereiche aufzuteilen """
    # Konvertierung des Graustufenbildes in ein Numpy float32 Array
    Z = frame.reshape((-1,1))
    Z = np.float32(Z)
    # Festlegen, wie viele Cluster gesucht werden (in diesem Fall 2)
    criteria = (cv2.TERM_CRITERIA_EPS + cv2.TERM_CRITERIA_MAX_ITER, 10, 1.0)
    # Durchführung des K-Means Clusterings
    _,label,center=cv2.kmeans(Z,K,None,criteria,10,cv2.KMEANS_PP_CENTERS)
    # Zurückkonvertierung in uint8 (natives Bildformat) 
    # und Generierung des Ausgabebildes
    center = np.uint8(center)
    res = center[label.flatten()]
    res2 = res.reshape((frame.shape))
    return res2


def getBiggestContour(img_bw):
    """ Gibt die Kontur mit der größten Fläche im Bild zurück """
    img_bw = img_bw.copy()
    # Ermittlung aller Konturen im Bild
    _, thresh = cv2.threshold(img_bw, 127, 255, 0)
    #contours, _ = cv2.findContours(thresh, cv2.RETR_TREE, cv2.CHAIN_APPROX_NONE)
    contours, _ = cv2.findContours(thresh, 1, 1)
    # Da wir die Ganze Schraube abdecken wollen, 
    # zeichnen wir die Kontur mit der größten Fläche in der Maske ein
    return cv2.drawContours(img_bw, [max(contours, key = cv2.contourArea)], -1, 255, thickness=-1)


def getMask(img_in, blur=20):
    """ Erstellt eine Maske, welche im Bild die Schraube vom Tisch trennt """
    img = img_in.copy()
    # Erkennung von Konturen im Bild
    _, thresh = cv2.threshold(img, 127, 255, 0)
    contours, _ = cv2.findContours(thresh, cv2.RETR_TREE, cv2.CHAIN_APPROX_NONE)
    # Einzeichnen der Konturen
    img_cont = cv2.drawContours(img, contours, -1, (0,255,0), -1)
    # Blur des Bildes um Artefakte zu beseitigen
    img_blur = cv2.blur(img_cont,(blur,blur))
    # K-Means Clustering, um eine Maske zu bekommen
    img_bw = clusterImage(img_blur, K=2)
    # Vordergrund = Weiß, Hintergrund = Schwarz
    img_bw[img_bw!=0] = 255
    # Füllen der Maske um Artefakte zu vermeiden
    return getBiggestContour(img_bw)


def removeBackground(img_in):
    """ Entfernt den Hintergrund der Schrauben """
    # Pipeline Hintergrundentfernung:
    #   Konturen -> K-Means Clustering -> Konturen & nur größte Fläche übernehmen -> Kontrast erhöhen
    mask = getMask(img_in)
    img_out = img_in.copy()
    img_out[mask==0]=255
    return img_out


def preProcessImages(images, scaleFactor=2, rmBG=False):
    """ Führt die Vorverarbeitung der Bilder für das Training durch """
    outImages =  []
    for img in images:
        # Konvertierung zu Graustufen
        img = cv2.cvtColor(img, cv2.COLOR_BGR2GRAY)
        # Entfernen des Hintergrunds
        if rmBG: img = removeBackground(img)
        # Skalierung
        width = int(img.shape[0]/scaleFactor)
        height = int(img.shape[1]/scaleFactor)
        img = cv2.resize(img, (width, height), fx=0, fy=0, interpolation=cv2.INTER_LINEAR)
        outImages.append(img)
    return np.array(outImages)

def normaliseImageData(images):
    """ Konvertiert die Bilder von [0;255] zu [-1,1] mit Median=0 & Standardabweichung=1 """
    images = (images / 127.5) - 1
    outImages = []
    for x in images:
        x -= np.mean(x)
        x /= np.std(x)
        outImages.append(x)
    return np.array(outImages)


#############################################################
#####       Einlesen der Trainings- und Testdaten       #####

if __name__ == "__main__":
    """ Liest alle Bilder ein, bearbeitet sie vor und speichert sie als Numpy Datei ab.  """
    # Import aller Bilder
    trainDataPath = './screw/train/good'
    trainImages = readImageDir(trainDataPath, scaleFactor=1)
    # Die Bilder werden in Graustufen umgewandelt und um scaleFactor skaliert.
    # Optional kann noch der Hintergrund der Bilder mittels K-Means Clustering entfernt werden (rmBG=True)
    trainData = preProcessImages(trainImages, scaleFactor=16, rmBG=False)
    # Abspeicherung der Bilder
    np.save('./processedImages/trainData', trainData)
