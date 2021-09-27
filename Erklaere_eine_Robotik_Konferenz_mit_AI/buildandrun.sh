#!/bin/bash

# Dieses Skript führt den Beispielcode für die Publikationsanalyse mittels Regex aus.
#
# This code is available under a GPL v3.0 license and comes without
# any explicit or implicit warranty.
#
# (C) Simon Schwaiger 2021 <schwaige@technikum-wien.at>

# Bau des Images
docker build -t regex_nlp .

# Ausführen des Containers mit X11 Forwarding
docker run -it \
                --rm \
                --name regex_nlp \
                -e DISPLAY=$DISPLAY \
                -v /tmp/.X11-unix:/tmp/.X11-unix \
                -v "$PWD/app":/app \
                regex_nlp:latest /bin/bash -c "chmod +x /app/app.sh && (cd app ; ./app.sh)"