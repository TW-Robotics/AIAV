FROM graphcore/pytorch-jupyter:3.2.0-ubuntu-20.04

# Dieses Dockerfile erstellt die Umgebung für den Beispielcode des Use Cases "AI Modellle lernen nicht das was sie lernen sollten".
#
# This code is available under a GPL v3.0 license and comes without
# any explicit or implicit warranty.
#
# (C) Alessandro Scherl 2023 <alessandro.scherl@technikum-wien.at>

# Installation von Pip3
# Die Java Runtime wird für Tika installiert
RUN apt-get update && apt-get install -y --no-install-recommends \
    python3-pip 

# Upgrade zum neuesten Pip
RUN pip3 install --upgrade pip 

# Installation des Natural Language Processing Toolkits, Matplotlib und Tika für Python
RUN pip install zennit Matplotlib