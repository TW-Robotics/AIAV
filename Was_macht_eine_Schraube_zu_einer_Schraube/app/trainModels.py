#/bin/env/python3

# Dieses Skript implementiert Training eines DCGAN zur Generierung von Bildern von Schrauben.
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

import matplotlib.pyplot as plt

import numpy as np
np.random.seed()

def generator(latentDim=128, imgShape=(32, 32, 1)):
    """ Gibt den Generator als Keras Modell zurück. """
    # Eingang: Vector mit Länge 128
    # Ausgang: Graustufenbild (32, 32, 1)
    model =  Sequential(
        [
            # Eingang: 128
            Dense(1024, input_dim=latentDim),
            Activation('relu'),
            # Reshape zu Bild mit Kanälen
            Dense(128*8*8),
            BatchNormalization(),
            Activation('relu'),
            Reshape((8,8,128)),
            # 8x8
            Conv2DTranspose(64, (2,2), strides=(2,2), padding='same'),
            Conv2D(64, (5,5), padding='same'),
            Activation('relu'),
            # 16x16
            Conv2DTranspose(64, (2, 2), strides=(2, 2), padding='same'),
            Conv2D(1, (5, 5), padding='same'),
            # Ausgang: 32x32
            Activation('tanh')
        ],
        name="generator",
    )
    return model

def discriminator(imgShape=(32, 32, 1)):
    """ Gibt den Diskriminator als Keras Modell zurück. """
    # Eingang: Graustufenbild (32, 32, 1)
    # Ausgang: Einschätzung, ob Bild echt ist, oder nicht
    model =  Sequential(
        [
            # Eingang: 64x64
            Conv2D(64, (5, 5), input_shape=imgShape, padding='same'),
            LeakyReLU(),
            MaxPooling2D(pool_size=(2, 2)),
            Conv2D(128, (5, 5), padding='same'),
            LeakyReLU(),
            MaxPooling2D(pool_size=(2, 2)),
            Flatten(),
            Dense(1024),
            LeakyReLU(),
            # Ausgang: 1
            Dense(1),
            Activation('sigmoid')
        ]
    )
    return model

def generatorContainingDiscriminator(g, d, latentDim=128):
    """ Kombiniertes Modell zum Training des Generators """
    ganInput = Input(shape=(latentDim,))
    x = g(ganInput)
    ganOutput = d(x)
    gan = Model(inputs=ganInput, outputs=ganOutput)
    return gan

class GAN():
    def __init__(self, imgRows=64, imgCols=64, imgChannels=1, loadModel=False):
        # Festlegen des Bildformats
        self.imgShape = (imgRows, imgCols, imgChannels)
        self.latentDim = 128
        self.epoch = 0
        if not loadModel:
            # Erstellen von Generator und Diskriminator
            d = discriminator(imgShape=self.imgShape)
            g = generator(latentDim=self.latentDim, imgShape=self.imgShape)
            # Instanzierung der Optimiser
            d_optim = SGD(lr=0.0005, momentum=0.9, nesterov=True)
            g_optim = SGD(lr=0.0005, momentum=0.9, nesterov=True)
            # Festlegen der Diskriminator Schichten als "nicht trainierbar"
            # Dies ist notwendig, damit beim Kombinierten Modell nur der Generator traininert wird
            #for layer in d.layers: layer.trainable = False
            d.trainable = False
            # Bauen des kombinierten Modells
            d_on_g = generatorContainingDiscriminator(g, d, self.latentDim)
            # Kompilation des Generators und kombinierten Modells
            g.compile(loss='binary_crossentropy', optimizer=g_optim)
            d_on_g.compile(loss='binary_crossentropy', optimizer=g_optim)
            print('Generator:')
            g.summary()
            print('Kombiniertes Modell:')
            d_on_g.summary()
            # Kompilation des Diskriminators
            d.trainable = True
            d.compile(loss='binary_crossentropy', optimizer=d_optim)
            print('Diskriminator:')
            d.summary()
        else:
            pass
        # Speichern der Modelle als Klassenatribute
        self.d = d
        self.g = g
        self.d_on_g = d_on_g
    def train(self, epochs, trainData, batch_size=128, sample_interval=100):
        """ Trainiert das GAN eine bestimmte Anzahl an Epochen. """
        # Vorbereitung der Trainingsdaten
        X_train = trainData / 127.5 - 1.
        if self.imgShape[2] == 1: X_train = np.expand_dims(X_train, axis=3)
        valid = np.ones((batch_size, 1))
        fake = np.zeros((batch_size, 1))
        y = np.concatenate((valid, fake))
        nIter = int(X_train.shape[0]/batch_size)
        # Anzeigen des Fortschritt mittels eines Progress Bars
        progressBar = Progbar(target=self.epoch + epochs)
        for self.epoch in range(self.epoch, self.epoch + epochs + 1):
            # Probenentnahme aus den Trainingsdaten
            idx = np.random.randint(0, X_train.shape[0], batch_size)
            imgs = X_train[idx]
            # Zufällige Werte für Latente Variable
            noise = np.random.uniform(-1, 1, (batch_size, self.latentDim))
            # Generierung künstlicher Bilder
            gen_imgs = self.g.predict(noise)
            # Training des Diskriminators
            X = np.concatenate((imgs, gen_imgs))
            d_loss = self.d.train_on_batch(X, y)
            # Training des Generators
            self.d.trainable = False
            g_loss = self.d_on_g.train_on_batch(noise, valid)
            self.d.trainable = True
            # Werden mehrere Metriken überprüft, ermitteln wir aus dem Array den Loss
            if isinstance(g_loss, np.ndarray): g_loss = g_loss.item()
            if isinstance(d_loss, np.ndarray): d_loss = d_loss.item()
            # Anzeigen des Trainingsfortschritts
            progressBar.update(self.epoch, values=[('D loss',d_loss), ('G loss',g_loss)])
            # Ausgabe der Beispielbilder und Speichern der Modelle
            if self.epoch % sample_interval == 0:
                self.sample_images(self.epoch)
                self.saveModels()
    def saveModels(self):
        """ Speichert die Keras Modelle und Anzahl der trainierten Epochen. """
        self.g.save('./savedModels/generator')
        self.d.save('./savedModels/discriminator')
        self.d_on_g.save('./savedModels/combined')
        np.save('./savedModels/epoch', self.epoch)
    def sample_images(self, epoch):
        """ Speichert aus dem Generator generierte Beispielbilder. """
        r, c = 5, 5
        noise = np.random.uniform(-1, 1, size=(r * c, self.latentDim))
        gen_imgs = self.g.predict(noise)
        gen_imgs = 0.5 * gen_imgs + 0.5
        fig, axs = plt.subplots(r, c)
        cnt = 0
        for i in range(r):
            for j in range(c):
                axs[i,j].imshow(gen_imgs[cnt, :,:,0], cmap='gray')
                axs[i,j].axis('off')
                cnt += 1
        fig.savefig("genImages/%d.png" % epoch)
        plt.close()
    def print_input_images(self, imgs, name):
        """ Speichert Trainingsbilder.  """
        if not isinstance(imgs, np.ndarray): imgs = np.array(imgs)
        r, c = 5, 5
        fig, axs = plt.subplots(r, c)
        cnt = 0
        try:
            for i in range(r):
                for j in range(c):
                    axs[i,j].imshow(imgs[cnt, :,:], cmap='gray')
                    axs[i,j].axis('off')
                    cnt += 1
        except IndexError:
            pass
        fig.savefig("genImages/" + name + ".png")
        plt.close()


#############################################################
#####                Training des GANs                  #####

if __name__ == "__main__":
    """ Trainiert das GAN anhand der vorbearbeiteten Trainingsdaten """
    # Laden der Vorbearbeiteten Bilder
    trainData = np.load('./processedImages/trainData.npy')
    # Initialisierung des GAN Modells
    gan = GAN(imgCols=len(trainData[0][0]), imgRows=len(trainData[0]), imgChannels=1, loadModel=False)
    # Plot der vorab bearbeiteten Trainings- und Testdaten
    gan.print_input_images(trainData, "trainData")
    # Training des GAN
    gan.train(trainData=trainData, epochs=50000, batch_size=64, sample_interval=1000)

