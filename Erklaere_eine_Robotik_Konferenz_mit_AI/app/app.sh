#!/bin/bash

# Dieses Skript führt den Beispielcode für die Publikationsanalyse mittels Regex aus.
#
# This code is available under a GPL v3.0 license and comes without
# any explicit or implicit warranty.
#
# (C) Simon Schwaiger 2021 <schwaige@technikum-wien.at>

# Aktivierung des Python 3 Virtual Environments
source ~/myenv/bin/activate
# Automatischer Download der Veröffentlichungen
python3 download_publications.py
# Ausführung des Programms
python3 regex_nlp.py