#!/bin/bash

# Dieses Skript führt den Beispielcode für den Speech Recognition Use Case aus.
#
# This code is available under a GPL v3.0 license and comes without
# any explicit or implicit warranty.
#
# (C) Simon Schwaiger 2021 <schwaige@technikum-wien.at>

# Erstellen und Aktivieren der virtuellen Umgebung
python3 -m venv myenv
source myenv/bin/activate

# Installation der benötigten Pakete
pip3 install --upgrade pip
pip3 install -r requirements.txt

# Ausführung des Programms
python3 speechRecognition.py

# Entfernen der virtuellen Umgebung
deactivate
rm -rf myenv
