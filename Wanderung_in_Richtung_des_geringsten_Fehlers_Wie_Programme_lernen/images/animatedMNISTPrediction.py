# Dieses Skript implementiert Darstellung und Training eines einfachen neuronalen Netzwerkes.
#
# This code is available under a GPL v3.0 license and comes without
# any explicit or implicit warranty.
#
# (C) Simon Schwaiger 2022 <schwaige@technikum-wien.at>
#
# Quellen:
# - Neural Network from Scratch in Python - https://towardsdatascience.com/math-neural-network-from-scratch-in-python-d6da9f29ce65
# - Gradient Descent in Python - https://towardsdatascience.com/gradient-descent-in-python-a0d07285742f

# Import von NumPy für Vektor/Matrix-Operationen
import numpy as np
# Import von Matplotlib für Visualisierungen
import matplotlib.pyplot as plt
# Import von Tensorflow für den Datensatz
from tensorflow.keras.datasets import mnist
from tensorflow.keras.utils import to_categorical
# Import von Imageio für das Erstellen des GIFs
import imageio

# Festlegen der Plot Farben
colours = {
    "darkblue": "#143049",
    "twblue": "#00649C",
    "lightblue": "#8DA3B3",
    "lightgrey": "#CBC0D5",
    "twgrey": "#72777A",
    "yellow": "#E3A049"
}

# Festlegen der Aktivierungsfunktion und ihrer Ableitung
def tanh(x):
    return np.tanh(x);

def tanh_prime(x):
    return 1-np.tanh(x)**2;

# Festlegen der Loss Funktion und ihrer Ableitung
def mse(y_true, y_pred):
    return np.mean(np.power(y_true-y_pred, 2));

def mse_prime(y_true, y_pred):
    return 2*(y_pred-y_true)/y_true.size;

# Erstellen einer abstrakten Basisklasse für Schichten
class Layer:
    def __init__(self) -> None:
        """ Klassenkonstruktor """
        self.input = None
        self.output = None
    #
    def forwardPropagation(self, input):
        """ Berechnet die Aktivierung einer Schicht basierend auf dem Eingang """
        raise NotImplementedError
    #   
    def backwardPropagation(self, outputError, learningRate):
        """ Berechnet die Ableitung des Errors nach dem Eingang (dE/dX) und aktualisiert trainierbare Parameter """
        raise NotImplementedError

# Erstellen der Klasse für voll vernetzte Schichten
class FullyConnectedLayer(Layer):
    """ Voll vernetzte, lineare Schicht mit variabler Eingangs- und Ausgangsgröße """
    def __init__(self, inputSize, outputSize):
        # Zufällige Initialisierung der Gewichte
        self.weights = np.random.rand(inputSize, outputSize) - 0.5
        self.bias = np.random.rand(1, outputSize) - 0.5
    #    
    def forwardPropagation(self, inputData):
        self.input = inputData
        self.output = np.dot(self.input, self.weights) + self.bias
        return self.output
    #
    def backwardPropagation(self, outputError, learningRate):
        # Berechnung des Fehlers am Eingang basierend auf dem Fehler am Ausgang der Schicht
        inputError = np.dot(outputError, self.weights.T)
        # Berechnung des Fehlers pro Gewicht basierend auf Eingang und dem Fehler am Ausgang
        weightsError = np.dot(self.input.T, outputError)
        #
        # Setzen der Gewichte und Bias Parameter entgegen des Fehlers
        self.weights -= learningRate * weightsError
        self.bias -= learningRate * outputError
        return inputError

# Erstellen der Klasse für nichtlineare Aktivierunsschichten
class ActivationLayer(Layer):
    """ Nichtlineare Aktivierungsschicht """
    def __init__(self, activation, activationPrime):
        # Setzen der Aktivierungsfunktion und ihrer Ableitung
        self.activation = activation
        self.activationPrime = activationPrime
    #
    def forwardPropagation(self, inputData):
        # Anwendung der Aktivierungsfunktion. Eingang und Ausgang werden gespeichert
        self.input = inputData
        self.output = self.activation(self.input)
        return self.output
    #
    def backwardPropagation(self, outputError, learningRate):
        # Berechnung des Fehlers am Eingang basierend auf dem Ausgangsfehler
        # Das passiert mittels der abgeleiteten Aktivierungsfunktion
        # In der Aktivierungsschicht wird nichts optimiert, da sie keine trainierbaren Parameter besitzt
        # Diese Schicht dient lediglich zur Einführung einer Nichtlinearität in das neuronale Netz
        return self.activationPrime(self.input) * outputError

# Erstellen der Netzwerk-Klasse
class Network:
    """ Enthält die Netzwerkschichten als Liste und implementiert Training mittels Minibatch Gradient Descent """
    def __init__(self, loss, loss_prime):
        """ Klassenkonstruktor: Instanziiert Liste an Schichten, Loss und seine Ableitung """
        self.layers = []
        self.loss = loss
        self.lossPrime = loss_prime
    #
    def add(self, layer):
        """ Fügt eine Instanz von Layer den gespeicherten Netzwerkschichten hinzu """
        self.layers.append(layer)
    #
    def predict(self, input_data):
        """ Gibt die Aktivierung der Ausgangsschicht für jeden Eintrag in input_data zurück """
        # Speichern der Anzahl der Samples und Instanziierung des Platzhalters der Resultate
        samples = len(input_data)
        result = []
        #
        # Iteration über die Einträge der Testdaten
        for sample in range(samples):
            # Führe Forward propagation anhand des Samples nacheinander für jede Schicht durch
            output = input_data[sample]
            for layer in self.layers:
                output = layer.forwardPropagation(output)
            # Speichern der Aktivierung
            result.append(output)
        #
        return result
   
    def minibatchFit(self, X_train, y_train, epochs, learningRate, minibatchSize=64):
        """ Implementiert minibatch Gradient Descent und trainiert das Netzwerk basierend auf Trainings-Features (X_train) und Aktivierungen (y_train) """
        # Platzhalter für gespeicherten Trainings-Loss
        trainingLoss = np.empty(epochs)
        # Festlegen der Anzahl an Datenpunkten pro Epoche
        samples = min(len(X_train), minibatchSize)
        #
        # Iteration über Epochen
        for epoch in range(epochs):
            # Rücksetzen des Losses
            epochLoss = 0
            # Laden von zufälligen Samples im Trainingsdatensatz mit Länge von minibatchSize
            minibatchIndices = np.random.choice(y_train.shape[0], samples, replace=False)
            y_minibatch = y_train[minibatchIndices]
            X_minibatch = X_train[minibatchIndices]
            #   
            # Iteration über Datenpunkte
            for sample in range(samples):
                # Forward Propagation -> Voraussage basiernd auf Eingangsdaten
                outputPrediction = X_minibatch[sample]
                for layer in self.layers:
                    outputPrediction = layer.forwardPropagation(outputPrediction)
                #   
                # Berechnung des Loss
                epochLoss += self.loss(y_minibatch[sample], outputPrediction)
                #
                # Backward Propagation -> Zuteilung, woher der Fehler kommt und Anpassung der Gewichte und Biases
                sampleError = self.lossPrime(y_minibatch[sample], outputPrediction)
                for layer in reversed(self.layers):
                    sampleError = layer.backwardPropagation(sampleError, learningRate)
            #
            # Berechnung des Loss pro Sample im miniBatch
            epochLoss /= samples
            trainingLoss[epoch] = epochLoss
        #
        # Rückgabe des Loss-Verlaufs nach dem Training
        return trainingLoss

# Download des MNIST Datensatzes
(x_train, y_train), (x_test, y_test) = mnist.load_data()

# Vorbearbeitung der Trainingsdaten
# Anpassung der Form und Normalisierung der Eingangsdaten
x_train = x_train.reshape(x_train.shape[0], 1, 28*28)
x_train = x_train.astype('float32')
x_train /= 255
# to_categorical codiert die Beschriftungen (Nummer von 0-9) als Vektor mit Größe 10
y_train = to_categorical(y_train)

# Sampling von 10000 Einträgen für's Testen
x_test = x_test.reshape(x_test.shape[0], 1, 28*28)
x_test = x_test.astype('float32')
x_test /= 255
y_test = to_categorical(y_test)


# Instanziierung des Netzwerks
net = Network(mse, mse_prime)
net.add(FullyConnectedLayer(28*28, 100))
net.add(ActivationLayer(tanh, tanh_prime))
net.add(FullyConnectedLayer(100, 50))
net.add(ActivationLayer(tanh, tanh_prime))
net.add(FullyConnectedLayer(50, 10))
net.add(ActivationLayer(tanh, tanh_prime))

# Training mit 64-großen Minibatches
losses = net.minibatchFit(x_train, y_train, epochs=1600, learningRate=0.05)



# Platzhalter für die generierten Bilder
animatedFrames = []

# Festlegen der Bildrate und Anzegeidauer der Beispiele
fps = 16
numExamples = 10
displayLen = 1.5 #[sec]

# Berechnung der Bilderanzahl und "Zeit" pro Bild
numFrames = int(fps*displayLen*numExamples)
frameInterval = 1/fps

# Iteration über alle Bilder
for stamp in np.arange(numFrames)*frameInterval:
    # Ermittlung des Index vom aktuellen Beispiel
    idx = int(stamp/displayLen)
    ## Zusammenstellung des Plots
    # Voraussage der Zahl
    pred = np.array(
        net.predict(x_test[idx])
    ).reshape(-1)
    prediction = np.argmax(pred)
    activation = pred[prediction]
    # Formatierung der Beschriftung
    annotationString = "Voraussage: {}\nAktivierung: {:.2f} \nZeit: {:.2f}".format(prediction, activation, stamp)
    # Plot und Formatierung
    fig, ax = plt.subplots()
    ax.imshow(x_test[idx].reshape(28, 28), cmap="Greys")
    ax.set_axis_off()
    # Erstellen der Textbox
    props = dict(boxstyle='square, pad=0.5', facecolor='white')
    ax.text(-0.3, 0.95, annotationString, transform=ax.transAxes, fontsize=14, verticalalignment='top', bbox=props)
    # Flush des Plots als Bild in ein Numpy Array
    fig.canvas.draw()
    data = np.frombuffer(fig.canvas.tostring_rgb(), dtype=np.uint8)
    data = data.reshape(fig.canvas.get_width_height()[::-1] + (3,))
    plt.clf()
    # Speichern des Bildes
    animatedFrames.append(data)

# Speichern des GIFs
imageio.mimsave("Abbildung5MNISTKlassifizierung.gif", animatedFrames, fps=fps)

