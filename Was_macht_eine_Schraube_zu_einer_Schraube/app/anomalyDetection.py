#/bin/env/python3

# Dieses Skript implementiert AnoGAN Anomalie Detection basierend auf dem trainierten GAN.
#
# This code is available under a GPL v3.0 license and comes without
# any explicit or implicit warranty.
#
# (C) Simon Schwaiger 2021 <schwaige@technikum-wien.at>
#
# Quellen:
#  - Anogan-Keras von T.K. Woo: https://github.com/tkwoo/anogan-keras
#  - Radford, A., Metz, L. and Chintala, S., 2015. Unsupervised representation learning with deep convolutional generative adversarial networks. arXiv preprint arXiv:1511.06434
#  - Schlegl, T., Seeböck, P., Waldstein, S.M., Schmidt-Erfurth, U. and Langs, G., 2017, June. Unsupervised anomaly detection with generative adversarial networks to guide marker discovery. In International conference on information processing in medical imaging (pp. 146-157). Springer, Cham.
#  - AnoGAN von Emmanuel Fuentes: https://github.com/neverrop/anogan_1
#  - AnoGAN von Yunju Cho: https://github.com/yjucho1/anoGAN

# Setzen von PlaidML als Backend
import os
os.environ["KERAS_BACKEND"] = "plaidml.keras.backend"

# Import der benötigten Module
from keras.models import Sequential, Model, load_model
from keras.layers import Input, Reshape, Dense, Dropout, MaxPooling2D, Conv2D, Flatten
from keras.layers import Conv2DTranspose, LeakyReLU, Activation, BatchNormalization, ZeroPadding2D
from keras.optimizers import Adam, RMSprop, SGD
from keras.utils.generic_utils import Progbar

from keras import backend as K
from keras import initializers

import numpy as np

import trainModels
from trainModels import generator, discriminator

from preProcessImages import preProcessImages

import matplotlib.pyplot as plt
from PIL import Image
import cv2

def genImages(batch_size=32, latentDim=128):
    """ Lädt den trainierten Generator und generier batch_size an Testbildern """
    g = load_model('./savedModels/generator')
    noise = np.random.uniform(0, 1, (batch_size, latentDim))
    return g.predict(noise)

def anomalyLossFunc(yTrue, yPred):
    """ Wendet die Loss Funktion aus AnoGAN an """
    return K.sum(K.abs(yTrue - yPred))

def featureExtractor():
    """ Kompilliert und gibt ein Modell zur Feature Extraction ausgehend vom Diskriminator zurück """
    d = load_model('./savedModels/discriminator')
    intermidiateModel = Model(inputs=d.layers[0].input, outputs=d.layers[-5].output)
    intermidiateModel.compile(loss='binary_crossentropy', optimizer='rmsprop')
    return intermidiateModel

def anomalyDetector(latentDim=128):
    """ Setzt Feature Extractor und Generator zusammen, um das Modell zur Anomalieerkennung zu erhalten """
    g = load_model('./savedModels/generator')
    intermidiateModel = featureExtractor()
    intermidiateModel.trainable = False
    #g = Model(inputs=g.layers[1].input, outputs=g.layers[-1].output)
    g.trainable = False
    # 
    aInput = Input(shape=(latentDim,))
    gInput = Dense((latentDim), trainable=True)(aInput)
    #gInput = Activation('sigmoid')(gInput)
    # 
    G_out = g(gInput)
    D_out= intermidiateModel(G_out)    
    model = Model(inputs=aInput, outputs=[G_out, D_out])
    model.compile(loss=anomalyLossFunc, loss_weights= [0.90, 0.10], optimizer='adam')
    #
    return model

def getAnomalyScore(model, x, nIter=500, batchSize=64, latentDim=128):
    """ Berechnet den Anomalie Score eines Bildes und findet Bildregionen die zum Modell passen """
    z = np.random.uniform(-1, 1, (batchSize, latentDim))
    intermidiateModel = featureExtractor()
    d_x = intermidiateModel.predict(x)
    loss = model.fit(z, [x, d_x], batch_size=batchSize, epochs=nIter, verbose=0)
    similar_data, _ = model.predict(z)
    loss = loss.history['loss'][-1]
    return loss, similar_data

if __name__ == "__main__":
    """ Trainiert einen AnoGAN Anomalie Detektor basierend auf Generator und Diskriminator """
    # Öfnnen und Vorbearbeiten eines Testbildes
    testImage = cv2.imread('./screw/test/manipulated_front/003.png')
    Image.fromarray(testImage).show()
    testImage = preProcessImages([testImage], scaleFactor=32)
    testImage = np.squeeze(testImage, axis=0)
    Image.fromarray(testImage).show()
    #
    # Instanzierung des Modells
    model = anomalyDetector()
    score, similarities = getAnomalyScore(model, testImage.reshape(1, 32, 32, 1), batchSize=1)
    #
    # Visualisierung der gefundenen Bildbereiche
    plt.imshow(testImage.reshape(32,32), cmap=plt.cm.gray)
    residual  = testImage.reshape(32,32) - similarities.reshape(32, 32)
    plt.imshow(residual, cmap='Reds', alpha=.5)
    plt.show()
