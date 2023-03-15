# Dieses Skript implementiert Darstellung vom Training eines linearen Regressionsmodells anhand einer pseudo-zufälligen Punktewolke.
#
# This code is available under a GPL v3.0 license and comes without
# any explicit or implicit warranty.
#
# (C) Simon Schwaiger 2022 <schwaige@technikum-wien.at>
#
# Quellen:
# - Gradient Descent in Python - https://towardsdatascience.com/gradient-descent-in-python-a0d07285742f

import numpy as np
import matplotlib.pyplot as plt
import imageio
import os
import random

# Festlegen der Plot Farben
colours = {
    "darkblue": "#143049",
    "twblue": "#00649C",
    "lightblue": "#8DA3B3",
    "lightgrey": "#CBC0D5",
    "twgrey": "#72777A",
    "yellow": "#E3A049"
}

# Loss Funktion
def mse(y_true, y_pred):
    return np.mean(np.power(y_true-y_pred, 2));

# Ableitung der Loss Funktion
def mse_prime(y_true, y_pred):
    return 2*(y_pred-y_true)/y_true.size;

class Neuron:
    def __init__(self) -> None:
        self.input = None
        self.output = None
        self.weight = (np.random.rand() - 0.5)*300
        self.bias = (np.random.rand() - 0.5)*200
        self.loss = mse
        self.lossPrime = mse_prime
    
    def forwardPropagation(self, inputData):
        self.input = inputData
        self.output = self.input * self.weight + self.bias
        return self.output
    
    def backwardPropagation(self, outputError, learningRate):
        inputError = self.weight * outputError
        weightError = self.input * outputError
        
        # Parameter Update
        self.weight -= learningRate * weightError
        self.bias -= learningRate * outputError
        return inputError
    
    def fit(self, X, y, learningRate=0.002, epochs=200, debug=False):
        # Platzhalter für Visualisierung des Trainings-Losses
        trainingLoss = []
        # Speichern der Anzahl an Datenpunkten im Trainings-Datensatz
        samples = len(X)
        #
        # Iteration über Trainingsepochen
        for epoch in range(epochs):
            loss = 0
            #
            # Iteration über Datenpunkte im Trainings-Datensatz
            for sample in range(samples):
                # Forward Propagation -> Voraussage Neurons
                outputPrediction = self.forwardPropagation(X[sample])
                # Berechnung des Fehlers
                loss += self.loss(y[sample], outputPrediction)
                # Backward Propagation -> Zuteilung, woher der Fehler kommt und Anpassung der Gewichte und Biases
                sampleError = self.lossPrime(y[sample], outputPrediction)
                _ = self.backwardPropagation(sampleError, learningRate)
            #
            # Berechnung des Loss pro Sample
            loss /= samples
            trainingLoss.append(loss)
            #
            # Ausgabe von Infos über die Epoche
            if debug == True: print("Epoche {}/{}: Loss = {}".format(epoch+1, epochs, loss))
        #
        # Rückgabe des Loss-Verlaufs nach Training
        return trainingLoss

def visualiseNeuronFitting(neur, X, y, frame, colours):
    # Erstellen des Plots
    fig, ax = plt.subplots()
    # Plotten der Punktwolke
    ax.scatter(X, y, label="Trainingsdaten", c=colours["twblue"])
    # Plotten des Modells
    pltx = (0, 10)
    plty = (neur.forwardPropagation(0)[0], neur.forwardPropagation(10)[0])
    ax.plot(pltx, plty, label="Trainiertes Neuron", c=colours["yellow"], linewidth=3)
    # Festlegen von Titel und Format
    ax.set_title("Neuron Training Anhand eines Datensatzes")
    ax.legend(loc='upper right', bbox_to_anchor=(1.0, 1.0))
    ax.set_xticks = [ 0, 2, 4, 6, 8, 10 ]
    ax.set_yticks = [ 20, 40, 60, 80, 100, 120, 140 ]
    # Speichern der Abbildung
    plt.savefig("tmp/{}.png".format(frame))
    plt.close(fig)

# Erstellen eines Verzeichnis zum Speichern der temporären Bilder
os.mkdir("./tmp")

# Durchführen mehrerer Trainingsdurchgänge
epochs = 45
attempts = 5

for a in range(attempts):
    # Erstellen einer Punktewolke, die das Neuron lernen soll
    X = 10 * np.random.rand(100,1)
    #y = 20 + 10 * (X + np.random.randn(100,1))
    y = random.choice(
        [ 
            np.random.rand()*20 + 12 * (X + np.random.randn(100,1)),
            np.random.rand()*120 - 8 * (X + np.random.randn(100,1)) 
        ]
    )
    # Erstellen des Neurons
    neur = Neuron()
    # Speichern jedes Schrittes des Fittings
    for e in range(epochs):
        _ = neur.fit(X,y,epochs=1)
        visualiseNeuronFitting(neur, X, y, a*epochs+e, colours)

# Konvertierung der Bilder in ./tmp zu einem animierten GIF
with imageio.get_writer('Abbildung3LineareRegression.gif', mode='I') as writer:
    for filename in [ "tmp/{}.png".format(e) for e in range(attempts*epochs) ]:
        image = imageio.imread(filename)
        writer.append_data(image)


