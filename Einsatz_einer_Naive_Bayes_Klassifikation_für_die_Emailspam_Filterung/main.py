#!/bin/bash

# Dieses Skript führt ein Beispielprojekt für die Naive Bayes Klassifizierung aus.
#
# This code is available under a GPL v3.0 license and comes without
# any explicit or implicit warranty.
#
# (C) Lucas Muster 2021 <muster@technikum-wien.at>

import sys
from NB_Classifier import NB_Class

def main():
    print("\n \033[92m ******************* Start Naive Bayes Klassifizierung ******************* \033[0m \n")
    inputData = sys.argv[1]
    print("Datensatz: ", inputData, "\n")
    nlp = NB_Class()
    nlp.pipeline(inputData)

if __name__ == "__main__":
    main()
