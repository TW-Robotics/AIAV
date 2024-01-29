#!/bin/bash

# Dieses Skript führt ein Beispielprojekt für die Naive Bayes Klassifizierung aus.
#
# This code is available under a GPL v3.0 license and comes without
# any explicit or implicit warranty.
#
# (C) Lucas Muster 2021 <muster@technikum-wien.at>

import pandas as pd
import os

class csvReader:
    def __init__(self,input):
        self.input = input
        self.data = []

    def showDirectory(self):
        print(os.path.dirname(os.path.abspath(__file__)))

    def showDataDirectory(self):
        path = os.path.dirname(os.path.abspath(__file__)) + "/data/"
        return path

    def readCSV(self):
        path = self.showDataDirectory()
        datapath = str(path) + str(self.input)
        self.data = pd.read_csv(datapath)
        x = self.data['text']
        y = []
        for i in range(0,len(x)):
            y.append(self.data['label_num'][i])
        return x,y
