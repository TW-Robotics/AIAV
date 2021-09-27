FROM ubuntu:20.04

# Dieses Dockerfile erstellt die Umgebung für den Beispielcode des Use Cases Regex NLP.
#
# This code is available under a GPL v3.0 license and comes without
# any explicit or implicit warranty.
#
# (C) Simon Schwaiger 2021 <schwaige@technikum-wien.at>

# Installation von Python 3, Pip3 und python3-tk für GUI
# Die Java Runtime wird für Tika installiert
RUN apt-get update && apt-get install -y --no-install-recommends \
    python3 \
    python3-pip \
    python3-tk \
    python3-venv \
    default-jre \
    && python3 -m venv ~/myenv

# Upgrade zum neuesten Pip
RUN /bin/bash -c "source ~/myenv/bin/activate \
    && pip3 install --upgrade pip \
    && deactivate"

# Installation des Natural Language Processing Toolkits, Matplotlib und Tika für Python
RUN /bin/bash -c "source ~/myenv/bin/activate \
    && pip3 install nltk matplotlib tika numpy"

