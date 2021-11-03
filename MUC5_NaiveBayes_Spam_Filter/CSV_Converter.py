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
