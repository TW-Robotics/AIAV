#!/bin/bash

# Dieses Skript lädt den verwendeten Datensatz herunter und benennt den Ordner richtig für den Use Case
#
# This code is available under a GPL v3.0 license and comes without
# any explicit or implicit warranty.
#
# (C) Simon Schwaiger 2023 <schwaige@technikum-wien.at>
#

git clone https://github.com/ardamavi/Sign-Language-Digits-Dataset
mv ./Sign-Language-Digits-Dataset/Dataset/ ./data
rm -rf ./Sign-Language-Digits-Dataset/