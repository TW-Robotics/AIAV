#FROM graphcore/pytorch-jupyter:3.2.0-ubuntu-20.04
FROM nvcr.io/nvidia/pytorch:23.09-py3

# Dieses Dockerfile erstellt die Umgebung für den Beispielcode des Use Cases "Von_Large_Language_Models_zum_Vision_Transformer".
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
RUN pip3 install matplotlib jupyterlab

# download the model weights from the cloud & move it to workspace
RUN wget https://cloud.technikum-wien.at/s/d93Aw3qroaYK3pD -O pytorch_vit_transfer_learning_model.pth 
WORKDIR /workspace