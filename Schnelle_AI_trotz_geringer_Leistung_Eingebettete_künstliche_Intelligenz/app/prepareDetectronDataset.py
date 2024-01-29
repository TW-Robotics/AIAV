# Dieses Skript lädt den MVTec ITODD Datensatz herunter und konvertiert ausgewählte Bilder für den Embedded AI HW Use Case.
#
# This code is available under a GPL v3.0 license and comes without
# any explicit or implicit warranty.
#
# (C) Simon Schwaiger 2024 <schwaige@technikum-wien.at>

import os
import cv2

## Abfrage der absoluten Pfades zu dieser Python Datei
# NameError weist auf eine interaktive Session hin
try:
    scriptPath = os.path.dirname(os.path.realpath(__file__))
except NameError:
    scriptPath = "."

outputDir = "detectronDataset"
outputDir = os.path.join(scriptPath, outputDir)

def getFiles(directory, ext=(".tif")):
    """Gibt alle Dateien in einem Verzeichnis von bestimmten Dateitypen zurück """
    files = [f for f in os.listdir(directory) if os.path.isfile(os.path.join(directory, f))]
    if ext == []: retArr = files
    else: retArr = [ file for file in files if file.endswith(ext) ]
    return retArr

def getDirectories(directory):
    """Gibt alle Verzeichnisse in einem Verzeichnis zurück """
    files = [f for f in os.listdir(directory) if os.path.isdir(os.path.join(directory, f))]
    return files

# Download und Entpacken eines Auszugs der Bilder von MVTec ITODD
os.system("wget https://www.mydrive.ch/shares/39253/0f54c36290547661f0ba001603f06a94/download/420946768-1629971997/3d_short_baseline.tar.xz")
os.system("tar -xvf 3d_short_baseline.tar.xz")

# Abspeichern aller Bilder
scenenames = getDirectories(os.path.join(scriptPath, "scenes"))

filename = "3d_short_baseline_l.tif"
if not os.path.isdir(outputDir): os.mkdir(outputDir)

with open(os.path.join(scriptPath, "utilisedImages.txt"), "r") as f:
    sceneList = f.read().splitlines()

for scene in scenenames:
    # Auswahl der in utilisedImages.txt angeführten Dateien
    if scene + ".png" in sceneList:
        ## Optional: Kopieren der Originaldatei
        #os.system("cp {} {}.tif".format(
        #    os.path.join(scriptPath, "scenes", scene, filename),
        #    os.path.join(outputDir, scene)
        #    ))
        ## Konvertierung der .tif Dateien zu .png
        img = cv2.imread(os.path.join(scriptPath, "scenes", scene, filename))
        cv2.imwrite(os.path.join(outputDir, scene) + ".png", img, [int(cv2.IMWRITE_JPEG_QUALITY), 100])