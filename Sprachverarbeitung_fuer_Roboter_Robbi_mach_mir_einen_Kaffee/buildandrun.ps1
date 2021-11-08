# Dieses Skript führt den Beispielcode für den Speech Recognition Use Case aus.
#
# This code is available under a GPL v3.0 license and comes without
# any explicit or implicit warranty.
#
# (C) Simon Schwaiger 2021 <schwaige@technikum-wien.at>

# Erstellen und Aktivieren der virtuellen Umgebung
python -m venv myenv
.\myenv\Scripts\activate

# Installation der benötigten Pakete
pip install -r requirements.txt

# Ausführung des Programms
python speechRecognition.py

# Entfernen der virtuellen Umgebung
Remove-Item .\myenv -Recurse

# Sollte die Pyaudio Installation unter Windows nicht funktionieren, 
# können vorab gebaute Python Module hier heruntergeladen werden:
# https://www.lfd.uci.edu/~gohlke/pythonlibs/#pyaudio

# Installationsanleitung für vorab gebaute Module unter Windows:
# https://stackoverflow.com/a/52284344/4999991