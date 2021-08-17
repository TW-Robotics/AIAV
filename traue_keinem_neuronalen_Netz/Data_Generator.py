# Dieses Skript erzeugt den Bilddatensatz basierend auf dem bestehendem 
# fashion-mnist Datenfiles.
# 
# This code is available under a GPL v3.0 license and comes without
# any explicit or implicit warranty.
#
# (C) Wilfried Wöber 2020 <wilfried.woeber@technikum-wien.at>
# coding: utf-8
#--------------------------------------#
#--- Importieren von Python modulen ---#
#--------------------------------------#
# Der Code basiert auf mehreren Beispielen
# VGG + fashion mnist: https://www.kaggle.com/anandad/classify-fashion-mnist-with-vgg16/data
# Visualisierung: https://github.com/TW-Robotics/NT_BodyParts
import numpy as np     #Wie schon beschrieben: eine Grundlegende library für Matrixmultiplikationen
import pandas as pd    #Wir nutzen Pandas um den gewünschten Datensatz zu laden 
# Mehr Infos zum Datensatz finden Sie unter https://github.com/zalandoresearch/fashion-mnist
import matplotlib.pyplot as plt #Dieses Modul wird für plots verwendet
#from sklearn.model_selection import train_test_split #Diese Funktion benötigen wir um uns ein geeignetes Trainings und Testset zu designen
#---------------------#
#--- Keras imports ---#
#---------------------#
#import keras		#Wir laden das Keras modul und einige Funktionen, die wir anschließend benötigen
#from keras.utils import to_categorical
#from keras.models import Sequential
#from keras.layers import Conv2D, MaxPooling2D, Dense, Dropout, Flatten
#from keras.preprocessing.image import ImageDataGenerator, img_to_array, array_to_img
#from keras.applications import VGG16 #Hier laden wir das VGG16 model
#from keras.applications.vgg16 import preprocess_input
import os
#------------------------------------------#
#--- Beginn der eigentliche Algorithmik ---#
#------------------------------------------#
# 1: Wir laden den Datensatz. 
train_data = pd.read_csv('./input/fashion-mnist_train.csv')   #Laden der Trainingsbilder
test_data = pd.read_csv('./input/fashion-mnist_test.csv')     #Laden der Testbilder
train_X = np.array(train_data.iloc[:,1:])   #Wir schneiden den Klassenindex weg
test_X  = np.array(test_data.iloc[:,1:])    #Auch hier: wir schneiden den Klassenindex weg
train_Y = np.array (train_data.iloc[:,0])   #Die erste Zeile ist der Klassenindex
test_Y  = np.array(test_data.iloc[:,0])     #Auch im Trainignsset is die erste Zeile der Klassenindex
classes = np.unique(train_Y)    #Wir extrahieren alle Klassenindizes...
num_classes = len(classes)      #... und berechnen die Anzahl der Indizes. Das ist gleich die Anzahl der Klassen
#--------------------#
#--- Store images ---#
#--------------------#
#--- Remove old data ---#
os.system("rm -r TRAIN")
os.system("rm -r TRAIN")
os.system("mkdir TRAIN")
os.system("mkdir TEST")
for i in range(np.min(train_Y),np.max(train_Y)+1):
    os.system("mkdir TRAIN/"+str(i))
    os.system("mkdir TEST/"+str(i))
#--- Train images ---#
randomSamples=10000
randomIndex = np.random.choice(train_X.shape[0], randomSamples, replace=False)
train_X = train_X[randomIndex,:]
train_Y = train_Y[randomIndex]
for i in range(0,train_X.shape[0]): #do for all training images
    img_vector = train_X[i,:]
    img = img_vector.reshape((28,28))
    target = train_Y[i]
    plt.imsave("TRAIN/"+str(target)+"/"+str(i)+".png", img, cmap='gray')
#--- Test images ---#
for i in range(0,test_X.shape[0]): #do for all training images
    img_vector = test_X[i,:]
    img = img_vector.reshape((28,28))
    target = test_Y[i]
    plt.imsave("TEST/"+str(target)+"/"+str(i)+".png", img, cmap='gray')
