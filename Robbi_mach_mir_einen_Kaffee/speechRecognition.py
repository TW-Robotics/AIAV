#!/bin/env/python3

# Dieses Skript liest vorab aufgenommene Audiodatei ein 
# und erkennt eine Bestellung für den kaffeebringenden Roboter.
#
# This code is available under a GPL v3.0 license and comes without
# any explicit or implicit warranty.
#
# (C) Simon Schwaiger 2021 <schwaige@technikum-wien.at>

import speech_recognition as sr
import random
import re

class textFromSpeech:
    """ Hilfsklasse zur Durchführung von Speech to Text mittels Google Webspeech API """
    def __init__(self, mic_id=0):
        self.recogniser = sr.Recognizer()
        self.mic = sr.Microphone(device_index=mic_id)
        self.demofilePath = './voiceClips/'
        pass
    def fromMic(self):
        """ Fertigt eine Aufnahme mittels dem Standardmikrofon des Systems an und führt Speech to Text aus. """
        # Öffnen des Mikrofons
        with self.mic as source:
            # Einstellung der Umgebungslautstärke
            print('Kalibriere das Mikrofon auf Zimmerlautstärke. Bitte kurz Ruhe.')
            self.recogniser.adjust_for_ambient_noise(source)
            # Aufnahme der Audiodatei
            print('Jetzt kann gesprochen werden.')
            audio = self.recogniser.listen(source)
            print('Danke, verarbeite das Audiosignal.')
        # Speech to Text mittels Googles Webspeech API
        text = None
        error = None
        try:
            text = self.recogniser.recognize_google(audio, language="de-DE")
        except sr.RequestError:
            text = 'Service API not available'
            error = True
        except sr.UnknownValueError:
            text = 'Could not recognise Speech'
            error = True
        else:
            error = False
        # Rückgabe des Textes
        return error, text
    def fromDemofile(self):
        """ Führt Speech to Text anhand einer zufällig ausgewählten Demodatei aus. """
        # Zufällige Auswahl einer der vorab aufgenommenen Kommandos
        sample = random.randint(1, 4)
        path = self.demofilePath + '{:02d}'.format(sample) + '.wav'
        print('Benutzte Demodatei: {}'.format(path))
        audiofile = sr.AudioFile(path)
        # Aufnahme der Audiodatei
        with audiofile as source:
            audio = self.recogniser.record(source)
        # Speech to Text mittels Googles Webspeech API
        text = None
        error = None
        try:
            text = self.recogniser.recognize_google(audio, language="de-DE")
        except sr.RequestError:
            text = 'Service API not available'
            error = True
        except sr.UnknownValueError:
            text = 'Could not recognise Speech'
            error = True
        else:
            error = False
        # Rückgabe des Textes
        return error, text

def chooseMic():
    """ Assistent zur Auswahl des zu verwendenden Mikrofons """
    print("Verfügbare Mikrofone:")
    for i, entry in enumerate(sr.Microphone.list_microphone_names()):
        print('{} - {}'.format(i, entry))
    print("Bitte ID des zu verwendenden Mikrofons eingeben:")
    while True:
        try:
            num = int(input())
        except ValueError:
            print("Bitte ID des Mikrofons als Zahl eingeben.")
        else:
            break
    return num
    
# Instanzierung der Hilfsklasse für Speech to Text
# Soll ein Mikrofon statt den Democlips verwendet werden, muss useDemofile=False gesetzt werden

useDemofile = True

# Einlesen einer Demodatei oder Eingabe durch Mikrofon
if useDemofile:
    rec = textFromSpeech()
    error, text = rec.fromDemofile()
else:
    mic_id = chooseMic()
    rec = textFromSpeech(mic_id=mic_id)
    error, text = rec.fromMic()

if error:
    print('Die Audioeingabe konnte nicht zu Text verarbeitet werden. Aufgetretener Fehler:')
    print(text)
else:
    print('Erkannter Text:', end=" ")
    print(text)

# Verarbeitung des Textes mittels Natural Language Processing

def keywordClassification(classes, classKeywords, text):
    """ Zählt wie oft angegebene Schlüsselwörter im Text vorkommen """
    # Generierung der Regular Expressions zum Absuchen der Klassen
    regexes = []
    for c in classKeywords:
        regexes.append('\\b(?:' + '|'.join(c) + ')\\b')
    # Der Text wird anhand von Schlagwörtern einer Klasse zugeordnet
    classScore = [0 for i in range(len(classes))]
    for i, expression in enumerate(regexes):
        classScore[i] += len(re.findall(re.compile(expression), text.lower()))
    return classScore

def amax(a):
    """ Gibt den Index des maximalen Elements in einer Liste an """
    return a.index(max(a))

# Coffeebot kann Bestellungen für einen oder mehrere Kaffee oder Espressi aufnehmen
# Wir wollen dabei ermitteln, wie was er dem Nutzer bringen soll

# Festlegen der Schlüsselwörter und Klassen
coffeeClasses = [
    'Lungo',
    'Espresso'
]

coffeeClassKeywords = [
    ['kaffee', 'lungo', 'coffee', 'verlängerter', 'verlängerte', 'verlängert', 'großer', 'große'],
    ['espresso', 'brauner', 'klein', 'kleine', 'espressi']
]

amountClassKeywords = [
    ['ein', '1' 'eins', 'einen'],
    ['zwei', '2'],
    ['drei', '3'],
    ['vier', '4'],
    ['fünf', '5']
]

amountClasses = [str(i+1) for i in range(len(amountClassKeywords))]

# Zählen, wie oft Schlüsselwörter jeder Klasse vorkommen
coffeeType = keywordClassification(coffeeClasses, coffeeClassKeywords, text)
coffeeAmount = keywordClassification(amountClasses, amountClassKeywords, text)

# Ist die Bestellung nicht eindeutig, wird nichts beestellt
# Ist sie eindeutig, wird die Bestellung ausgegeben
if coffeeType[0] == coffeeType[1]:
    print("Bestellung ist nicht eindeutig, bitte noch einmal versuchen.")
else:
    print("Erkannte Bestellung: {} mal {}".format(amountClasses[amax(coffeeAmount)], coffeeClasses[amax(coffeeType)]))
