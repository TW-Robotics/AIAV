FROM graphcore/pytorch-jupyter:3.2.0-ubuntu-20.04

# Dieses Dockerfile erstellt die Umgebung für den Beispielcode des Use Cases "AI Modellle lernen nicht das was sie lernen sollten".
#
# This code is available under a GPL v3.0 license and comes without
# any explicit or implicit warranty.
#
# (C) Alessandro Scherl 2023 <alessandro.scherl@technikum-wien.at>

# Installation von Pip3
RUN apt-get update && apt-get install -y --no-install-recommends \
    python3-pip 

# Upgrade zum neuesten Pip
RUN pip3 install --upgrade pip 

# Installation der Zennit Bibliothek & Matplotlib
RUN pip3 install zennit matplotlib

# download the model weights from the cloud & move it to workspace
RUN wget https://cloud.technikum-wien.at/s/gGDs9HLSGjDNnbF/download -O model_weights.pt 
RUN mv model_weights /workspace
WORKDIR /workspace