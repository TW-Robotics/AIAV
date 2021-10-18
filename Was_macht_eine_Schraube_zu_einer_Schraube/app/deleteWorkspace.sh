#!/bin/bash

# Dieses Skript löscht den Workspace und angelegte Dateien des GAN Usecases
#
# This code is available under a GPL v3.0 license and comes without
# any explicit or implicit warranty.
#
# (C) Simon Schwaiger 2021 <schwaige@technikum-wien.at>

# Entfernen der virtuellen Umgebung
deactivate
rm -rf myenv

# Entfernen der angelegten Ordner
rm -rf genImages
rm -rf savedModels
rm -rf screw
rm -rf processedImages
rm -rf screw.tar.xz
