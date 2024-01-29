# Schnelle AI trotz geringer Leistung: Eingebettete künstlichte Intelligenz

Hier finden Sie Programmcode und Beispiele, wie man einen Detektor in der Cloud trainiert und anschließend auf eingebetteten Plattformen lokal nutzen kann.

Die Grundlagen von verfügbarer Hardware zum Training und Inferenz neuronaler Netzwerke werden Schritt für Schritt im Use Case auf der AIAV Webseite präsentiert. Wenn Sie den Beispielcode selbst testen wollen, finden Sie hier [Dockerfiles](https://docs.docker.com/build/building/packaging/) für den Bau und die Ausführung der Applikation auf einem [Nvidia Jetson](https://www.nvidia.com/en-us/autonomous-machines/embedded-systems/) oder der [Intel N100 Plattform](https://www.intel.de/content/www/de/de/products/sku/231803/intel-processor-n100-6m-cache-up-to-3-40-ghz/specifications.html).


# Der Use Case

Die Implementierung basiert auf [Detectron2](https://github.com/facebookresearch/detectron2) und verwendet [Pytorch](https://pytorch.org/) als Backend. Zusätzlich dazu werden einige Bildverarbeitungsfunktionen aus der offen zugänglichen [OpenCV](https://opencv.org/) Bibliothek eingesetzt. Der Datensatz wurde mittels []() beschriftet.

### Vorbereitung des Datensatzes

Der Datensatz basiert auf [MVTec ITODD](https://www.mvtec.com/company/research/datasets/mvtec-itodd), einem Datensatz zur 3D Objekterkennung von industriellen Bauteilen. Wir verwenden aber einen Auszug aus dem Datensatz, um den Trainingsaufwand geringer zu halten. Dafür wurden 2D Beschriftungen für bestehende Bilder dieses Datensatzes erstellt. `./app/prepareDetectronDataset.py` lädt den Datensatz herunter, entpackt diesen und konvertiert ausgewählte Bilder in das von Detectron unterstützte *png* Format. 

Zur Ausführung dieser Python Datei müssen Python, OpenCV und wget auf Ihrem Linux-System installiert sein `sudo apt update && sudo apt install python3 python3-opencv`. Anschließend kann die Datei mittels des Befehls `python prepareDetectronDataset.py` im `./app` Ordner ausgeführt werden. Nach erfolgreicher Durchführung der Datei wir der Ordner `./app/detectronDataset` durch die konviertierten Bilder ergänzt.

### Detektor Training

Der Detektor wird in der Cloud basierend auf [dieser](https://colab.research.google.com/drive/16jcaJoc6bCFAQ96jDe2HwtXj7BMD_-m5) Detectron2 Anleitung trainiert. Laden Sie dazu `AIAVDetectronTraining.ipynb` auf [Google Colab](https://colab.research.google.com/) hoch und befolgen Sie die Schritte im Notebook. Bitte beachten Sie, dass Sie dafür die Schritte in *Vorbereitung des Datensatzes* bereits durchgeführt haben müssen.

Wir stellen eine bereits trainierte Version des Detektors zur Verfügung, falls Sie diesen Schritt überspringen wollen. Diese können sie [hier](https://github.com/TW-Robotics/AIAV/releases/download/0.1/AI_Hardware_Use_Case_Pretrained_Model.zip) herunterladen. Entpacken Sie anschließend das Archiv und kopieren Sie die `model_final.pth` Datei in `./app/pretrainedModel/`.

### Ausführung Jetson

Folgende Schritte werden zur Ausführung der Inferenz auf einem Jetson Board von Nvidia benötigt:

1. Zusammenbau des Jetson Boards und Flash des neuesten Jetpacks anhand der [Nvidia Dokumentation](https://docs.nvidia.com/jetson/jetpack/install-jetpack/index.html)
2. Installation von Docker anhand der [Nvidia Dokumentation](https://docs.nvidia.com/datacenter/cloud-native/container-toolkit/latest/install-guide.html#installing-with-apt)
3. Konfiguration des *docker* Kommandos ohne root Privilegien: `sudo usermod -aG docker $USER`. Nach dem Ausführung dieses Kommandos müssen Sie das Jetson neu starten.
4. Kopieren der Use Case Dateien (inklusive vorbereiteten Datensatzes) auf das Jetson
5. Bau und Ausführung der Entwicklungsumgebung mittels `bash runJetson.sh`

### Ausführung Intel N100

Folgende Schritte werden zur Ausführung der Inferenz auf einem N100 Board von Intel benötigt:

1. Installation RAM und SSD laut Bedienungsanleitung des Boards
2. Installation von [Ubuntu 22.04](https://ubuntu.com/tutorials/install-ubuntu-desktop#1-overview) oder [Pop!\_OS 22.04](https://support.system76.com/articles/install-pop/) auf der im Board installierten SSD
3. Installation von Docker: `curl -fsSL https://get.docker.com -o get-docker.sh && sudo sh ./get-docker.sh`
4. Konfiguration des *docker* Kommandos ohne root Privilegien: `sudo usermod -aG docker $USER`. Nach dem Ausführung dieses Kommandos müssen Sie den PC neu starten.
5. Kopieren der Use Case Dateien (inklusive vorbereiteten Datensatzes) auf das Board
6. Bau und Ausführung der Entwicklungsumgebung mittels `bash runIntel.sh`

### Verwendung der JupyterLab IDE

Nach Ausführung von `bash runJetson.sh` oder `bash runIntel.sh` wird im Terminal ein Link zum Öffnen der JupyterLab Entwicklungsumgebung angezeigt. Klicken Sie diesen an und navigieren Sie mittels des Dateibrowsers zum Notebook der richtigen Plattform (*.ipynb* Dateien) und führen Sie die Zellen dieses Notebooks aus.

# Ergebnisse

TODO

# Diskussion

In diesem Use Case wurden zwei eingebettete Plattformen vorgestellt, welche neuronale Netze lokal ausführen können. Die Vor- und Nachteile beider Plattformen wurden diskutiert und Performanceunterschiede wurden anhand der Detektion von industirellen Bauteilen aufgezeigt.