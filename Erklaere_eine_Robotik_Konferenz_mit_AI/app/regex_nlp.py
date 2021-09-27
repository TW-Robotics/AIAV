#!/bin/python

# Dieses Skript führt Natural Language Processing mittels Regular Expressions
# anhand der Veröffentlichungen des Austrian Robotics Workshops aus.
#
# This code is available under a GPL v3.0 license and comes without
# any explicit or implicit warranty.
#
# (C) Simon Schwaiger 2021 <schwaige@technikum-wien.at>

import nltk
import re
from tika import parser

from os import listdir
from os.path import isfile, join

import numpy as np

# Download des englischen Tokenisers, zum Aufschlüsseln nach Sätzen
nltk.download('punkt')
tokenizer = nltk.data.load('tokenizers/punkt/english.pickle')

# Festlegung der trennenden Satzzeichen und Import der Stoppwörter aus stopwords.py
punctuation = ['.', ',', '-', ';', '(', ')', '%', '=', '’', '&']
from stopwords import stopwords

# Einführung einer Hilfsklasse um das Zählen der Kandidaten zu vereinfachen
class phraseStorage():
    def __init__(self):
        """ Klassenkonstruktor """
        self.phrases = []
        self.count = []
        self.scentenceCount = []
        self.score = []
    def getPhraseStats(self, phrase):
        """ Gibt die aktuellen Zählstände einer Phrase zurück """
        c = 0
        s = 0
        if phrase in self.phrases:
            c = self.count[self.phrases.index(phrase)] 
            s = self.scentenceCount[self.phrases.index(phrase)]
        return phrase, c, s
    def removeExtraSpaces(self, phrase):
        """ Entfernt extra Leerzeichen """
        phrase = re.sub(' {2,2}', '', phrase.strip())
        return phrase
    def add(self, phrases):
        """ Fügt eine oder mehrere Phrasen den bekannten Phrasen hinzu """
        if (not isinstance(phrases, list)): phrases = [phrases]
        for phrase in phrases:
            if phrase == '': continue
            elif phrase in self.phrases:
                # Ist eine Phrase bereits bekannt, wird gezählt, wie oft sie vorkommt
                self.count[self.phrases.index(phrase)] += 1
            else:
                # Ist sie nicht bekannt, wird ein neuer Eintrag angelegt
                self.phrases.append(self.removeExtraSpaces(phrase))
                self.count.append(1)
                self.scentenceCount.append(0)
                self.score.append(0.0)
    def addBatch(self, phrases, counts):
        """ Fügt eine oder mehrere Phrasen mit Zählstand hinzu """
        if (not isinstance(phrases, list)): phrases = [phrases]
        if (not isinstance(counts, list)): counts = [counts]
        for phrase, count in zip(phrases, counts):
            if phrase in self.phrases:
                # Ist eine Phrase bereits bekannt, wird gezählt, wie oft sie vorkommt
                self.count[self.phrases.index(phrase)] += count
            else:
                # Ist sie nicht bekannt, wird ein neuer Eintrag angelegt
                self.phrases.append(self.removeExtraSpaces(phrase))
                self.count.append(count)
                self.scentenceCount.append(0)
                self.score.append(0.0)
    def updateScentenceCount(self, phrases):
        """ Update des sekundären Zählstands der Phrase """
        if (not isinstance(phrases, list)): phrases = [phrases]
        # Heausfiltern von mehrmals vorkommenden Wörtern eines Satzes
        uniquePhrases = []
        for phrase in phrases:
            if phrase not in uniquePhrases:
                uniquePhrases.append(phrase)
        # Update des Zählstandes
        for phrase in uniquePhrases:
            if phrase not in self.phrases:
                self.add(phrase)
            self.scentenceCount[self.phrases.index(phrase)] += 1
    def updateScore(self):
        for i, _ in enumerate(self.phrases):
            self.score[i] = self.count[i]/self.scentenceCount[i]

def generateKeywords(data, keywordAmount):
    """ Generiert keywordAmount Keywords aus einem Text (data, als String) """
    # Nutzung von Regular Expressions, um unerwünschte Teile aus dem Text zu entfernen
    # Weglassen der Literatur
    i = re.compile('REFERENCES', flags=re.IGNORECASE)
    ret = re.search(i, data)
    if ret is not None:  data = data [0:ret.span()[0]]
    #
    # Weglassen der Newline Charakter
    i = re.compile('-\\n')
    data = re.sub(i, '', data)
    i = re.compile('\\n')
    data = re.sub(i, ' ', data)
    #
    # Weglassen von Kapitelüberschriften
    # Kapitelüberschrift = Römische Zahl + '. ' + großgeschriebener Titel
    i = re.compile('[IVX]+[.][ ]([A-Z]+[ ])+')
    data = re.sub(i, '', data)
    #
    # Weglassen von Zitationen
    # Zitation = '[' + Zahl + ']'
    i = re.compile('\[[0-9]+\]')
    data = re.sub(i, '', data)
    #
    # Weglassen des 'Abstract' Keywords am Anfang der IEEE Vorlage
    #data = re.sub('abstract.', '', data, flags=re.IGNORECASE)
    ret = re.search('abstract', data, flags=re.IGNORECASE)
    if ret is not None: data = data[ret.span()[1]:]
    #
    # Weglassen von Sonderzeichen
    i = re.compile('[^A-Za-z0-9\s,.()]')
    data = re.sub(i, '', data)
    #
    # Weglassen von Zahlen
    # Zahl = Leerzeichen/Klammer Auf + kein oder ein Vorzeichen + Zahl + kein oder ein Punkt + Zahl + Leerzeichen/Klammer Zu
    i = re.compile('\s+[(]*[-+±]*\d+[.,]*\d*[)]*\s+')
    data = re.sub(i, '', data)
    # Spezialfall = Zahl am Satzende
    i = re.compile('\s+[(]*[-+±]*\d+[.,]*\d*[)]*[.]')
    data = re.sub(i, '', data)
    #
    # Weglassen von i.e. und e.g.
    i = re.compile('e\.g\.')
    data = re.sub(i, '', data)
    i = re.compile('i\.e\.')
    data = re.sub(i, '', data)
    #
    # Erzeugung der Keyword Kandidaten
    localStorage = phraseStorage()      # Speicher für Kandidaten eines Satzes
    globalStorage = phraseStorage()     # Globaler Speicher für alle Kandidaten
    #
    # Aufschlüsselung nach Sätzen anhand des Tokenisers
    scentences = tokenizer.tokenize(data.lower())
    #
    buf = []
    # Iteration über alle Sätze
    for scentence in scentences:
        # Iteration über alle Sätze
        words = nltk.word_tokenize(scentence)
        # Iteration über alle Wörter im Satz
        for word in words:
            # Überprüfung, ob das aktuelle Wort in der Stoplist ist
            if (word in stopwords) or (word in punctuation):
                if buf == []:
                    # Überspringe die aktuelle Phrase, falls sie leer ist
                    pass
                else:
                    # aktuelle Phrase wird den bekannten Phrasen hinzugefügt
                    localStorage.add(' '.join(buf))
                    buf = []
            else:
                # Aktuelles Wort wird der Phrase hinzugefügt
                if buf != []:
                    buf.append(' ')
                buf.append(word)
        # Satzende schließt immer die laufende Phrase ab
        localStorage.add(' '.join(buf))
        buf = []
        # LocalStorage wird am Satzende ausgelesen
        # Inhalt wird in globalStorage gespeichert
        # Und localStorage wird zurückgesetzt
        globalStorage.addBatch(localStorage.phrases, localStorage.count)
        globalStorage.updateScentenceCount(localStorage.phrases)
        localStorage = phraseStorage()
    #
    # Berechnung der Scores
    globalStorage.updateScore()
    #
    # Ausgabe der Kandidaten basierend auf ihren Scores
    result = []
    keywordMinLen = 3
    #
    # Gernerierung der Keywords anhand davon, wie oft sie vorkommen
    # Reihung der Keywords anhand der Scores
    tmp = [x for _, x in sorted(zip(globalStorage.count, globalStorage.phrases), reverse=True)]
    keywords = []
    for phrase in tmp:
        if len(phrase) < keywordMinLen: continue
        elif len(keywords) >= keywordAmount: break
        else: keywords.append(phrase)
    #
    for phrase in [x for _, x in sorted(zip(globalStorage.score, globalStorage.phrases), reverse=True)]:
        if phrase in keywords: result.append(phrase)
    #
    return result

def getTitle(data):
    """ Ermittelt den Titel einer Publikation """
    # Es wird nach den "\n" am Anfang der Formatvorlage gesucht
    i = re.compile('(\\n)*.*(\\n)?.*')
    ret = re.match(i, data)
    if ret is not None:
        span = ret.span()
        data = data[span[0]:span[1]]
    # Test, ob ein Zeilenumbruch im Titel ist
    # Wenn ja, wird er durch ein Leerzeichen ersetzt
    i = re.compile('.\\n.')
    idx = re.search(i, data)
    if idx is not None:
        idx = idx.span()
        data = data[:idx[0]+1] + ' ' + data[idx[1]-1:]
    # Weglassen der Zeilenumbrüche im resultierenden Titel
    i = re.compile('(\\n)*')
    return re.sub(i, '', data)

def analyseYear(mypath, numKeywords):
    """ Ermittelt die Titel und Keywords aller Publikationen in einem Verzeichnis """
    # Ermittlung alles Dateien im pdfs Verzeichnis
    files = [f for f in listdir(mypath) if isfile(join(mypath, f))]
    # Iteration über alle Dateien im Verzeichnis
    # Schlagwörter und Titel jeder Publikation werden gespeichert
    totalKeywords = []
    titles = []
    for filename in files:
        # Formatierung des Dateinamens
        filename = '{}/{}'.format(mypath, filename)
        # Laden des Inhalts der Datei
        raw = parser.from_file(filename)
        data = raw['content']
        # Weglassen von "D ra  ft" (in den Formatvorlagen eingebettet)
        i = re.compile('D\s{0,3}ra\s{0,3}ft')
        data = re.sub(i, '', data)
        # Ermittlung von Schlagwörtern und Titel der Publikation
        titles.append(getTitle(data))
        tmp = generateKeywords(data, numKeywords)
        totalKeywords.append(tmp)
    return totalKeywords, titles

# Definition der klassifizierung der publikationen
# Die klassifizierung basiert darauf, ob die spezifizierten schlagwärter in den texten gefunden wurden
classes = [
    'RobotSensingPerception',
    'MachineLearningAI',
    'RobotModelling',
    'SoftwareDesign',
    'MobileAndServiceRobots',
    'HumanRobotInteraction',
    'EducationalRobots'
]

classKeywords = [
    ['sensor', 'filter', 'sensing', 'vision', 'camera'],
    ['machine learning', 'artificial intelligence', 'agent', 'feature', 'classification', 'network'],
    ['model', 'estimation', 'pose', 'kinematic', 'structure'],
    ['software', 'middleware', 'simulation', 'communication', 'verification'],
    ['mobile', 'plan', 'rescue', 'ontology', 'constraint'],
    ['human', 'user', 'interact', 'safe', 'workspace'],
    ['education', 'show', 'demonstrate', 'workshop', 'school']
]

# Generierung der regular expressions zum absuchen der schlagwörter
regexes = []
for c in classKeywords:
    regexes.append('\\b(?:' + '|'.join(c) + ')\\b')

def getClassificationScores(totalKeywords, regexes):    
    scores = []
    for phrases in totalKeywords:
        text = ' '.join(phrases)
        classScore = [0 for i in range(len(classes))]
        for i, expression in enumerate(regexes):
            classScore[i] += len(re.findall(re.compile(expression), text))
        scores.append(classScore)
    scoreSum = [np.sum(score) for score in scores]
    print('{} out of {} papers could not be classified'.format(scoreSum.count(0), len(scoreSum)))
    return scores

paths = ['./pdfs/ARW2016', './pdfs/ARW2017', './pdfs/ARW2018', './pdfs/ARW2019', './pdfs/ARW2020']
scores = []
totalKeywords = []
titles = []
for mypath in paths:
    tmpKeywords, tmpTitle = analyseYear(mypath, 40)
    totalKeywords.append(tmpKeywords)
    titles.append(tmpTitle)
    scores.append(getClassificationScores(tmpKeywords, regexes))

# Generierung des Plots
from matplotlib import rcParams
rcParams['font.size'] = 18
import matplotlib.pyplot as plt

# Formatierung der Platzhalter für Balkendiagram von jedem Jahr
plots = len(scores)
classArrange = np.arange(0, len(classes))
fig, ax = plt.subplots(1,plots)
fig.suptitle('Automatisch ermittelte Themen der Paper des Austrian Robotics Workshops')

# Definition der Beschriftungen und Farben
years = ['ARW 2016', 'ARW 2017', 'ARW 2018', 'ARW 2019', 'ARW 2020']
plotColors = ['navy', 'mediumpurple', 'navy', 'mediumpurple', 'navy', 'mediumpurple', 'navy']
textColors = ['black', 'gray', 'black', 'gray', 'black', 'gray', 'black']

# Iteration über alle Jahre und Einzeichnen der Balken und Beschriftungen
for i in range(plots):
    scoreSum = [np.sum(score) for score in scores[i]]
    values = np.array([0.0 for j in range(len(classes))])
    for j in range(len(scoreSum)):
        if scoreSum[j] != 0: values += np.array(scores[i][j])/scoreSum[j]
        else: continue
    nonZeroEntries = len(scoreSum) - scoreSum.count(0)
    values /= nonZeroEntries/100
    values = np.round(values, 0)
    ax[i].set_xlim([-1, 7])
    ax[i].set_ylim([0, 45])
    ax[i].bar(classArrange,values, zorder=3, color=plotColors, width=0.6)
    ax[i].grid(True, zorder=0)
    ax[i].set_xticks(classArrange)
    for j, c in enumerate(classes):
        ax[i].annotate(c, (j+0.2,0), rotation=45, horizontalalignment='right', verticalalignment='top', color=textColors[j])
    ax[i].set_xticklabels([' ' for _ in classArrange])
    ax[i].set_title(years[i], fontsize=22)

# Anpassung der Achsenbeschriftung Puffers um die Grafik
ax[0].set_ylabel('Anteil jedes Themas in den Papern [%]', fontsize=20)
plt.gcf().subplots_adjust(bottom=0.4)

# Anzeigen der Grafik
plt.show()
