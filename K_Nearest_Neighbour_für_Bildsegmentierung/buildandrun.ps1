# Dieses Skript führt den Beispielcode für den k-Nearest Neighbour Use Case aus.
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
python kNN.py

# Entfernen der virtuellen Umgebung
Remove-Item .\myenv -Recurse
