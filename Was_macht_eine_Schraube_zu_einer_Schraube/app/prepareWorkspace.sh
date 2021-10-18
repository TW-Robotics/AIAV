#!/bin/bash

# Dieses Skript installiert benötigte Software Komponenten für den GAN Use Case,
# lädt die Trainingsdaten herunter, legt die Ordnerstruktur an und bearbeitet die Bilder vor.
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

# Setup von PlaidML
plaidml-setup

# Download der Trainingsdaten
wget https://www.mydrive.ch/shares/38536/3830184030e49fe74747669442f0f282/download/420938130-1629953152/screw.tar.xz

# Entpacken des Datensatzes
tar -xf screw.tar.xz

# Vorbearbeiten der Trainingsbilder und Anlegen der Ordnerstruktur
python preProcessImages.py