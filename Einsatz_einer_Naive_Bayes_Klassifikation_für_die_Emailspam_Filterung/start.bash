#!/bin/bash

# Dieses Skript führt ein Beispielprojekt für die Naive Bayes Klassifizierung aus.
#
# This code is available under a GPL v3.0 license and comes without
# any explicit or implicit warranty.
#
# (C) Lucas Muster 2021 <muster@technikum-wien.at>

# Erstellen und Aktivieren der virtuellen Umgebung
echo "Checking virtual environment:"
python3 -m venv myenv
source myenv/bin/activate

# Installation der benötigten Pakete
pip3 install --upgrade pip
pip3 install -r requirements.txt

# Ausführung des Programms
python3 main.py spam_ham_dataset.csv

# Entfernen der virtuellen Umgebung
deactivate
rm -rf myenv
