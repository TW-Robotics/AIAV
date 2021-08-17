## Traue keinem CNN

Hier finden Sie Programmcode und Beispiele wie man mit einem CNN Klassifizierungsaufgaben durchführen kann und anschließend überprüft ob das CNN erklärbare Merkmale erkannt hat.

In diesem Bereich finden Sie eine gelöste simple Aufgabe - nämlich die Klassifizierung des [fashion-mnist Datensatzes](https://github.com/zalandoresearch/fashion-mnist). Zusätzkich zum Umgang mit CNNs finden Sie hier Beispiele um die Entscheidung von CNNs zu visualisieren. Es wird LRP und GradCAM genutzt.

Sie finden eine Schritt für Schritt Anleitung in dem Jupyter Notebook (**Achtung:** das müssen Sie lokal ausführen). Wenn Sie selber mit den Daten und Software arbeiten wollen finden Sie weiters ein Installationsskript. Dieses erzeugt Ihnen eine Python virtual environment.

**Achtung** Die Software wurde unter Ubuntu 18 und mit **Python 2** support getestet. Falls Sie Python 3 nutzen müssen Sie die Software dementsprechend anpassen.

## Der Use Case

Wir wollen, wie auf der AIAV Plattform beschrieben, herausfinden welche Bildbereiche für unser CNN für Klassifizierungsaufgaben genutzt wird. Dazu nutzen wir in diesem simplen Beispiel die Ansätze GradCAM und LRP (Layer-wise relevance probagation). Unser Vorgehen ist:

- Installation: Es wurde eien *Python virtual environment* genutzt. Durch das Ausführen der Datei *install.bash* wird die komplette nötige Software erzeugt. **Achtung:** Folgende Systemvoraussetzungen müssen erfüllt sein:
    - Ubuntu 18.04 und Python 2: Sie müssen gegebenenfalls unsere Software ansonsten anpassen
    - NVIDIA Deep Learning Libraries: Sie brauchen (wie für *deep learning* üblich) eine NVIDIA Grafikkarte mit dementsprechenden installierten Treiber
- Datensatz erzeugen: Falls Sie dieses Beispiel selber ausführen wollen müssen Sie den oben genannten Link anklicken und die Dateien *fashion-mnist_train.csv* sowie *fashion-mnist_test.csv* herunterladen und in einen *input* Ordner ablegen. Anschließend erzeugt die Datei *Data_Generator.py* die nötigen Daten aufgesplittet in einen Training- und einen Testordner. Der aufruf geschieht durch *python Data_Generator.py*. 
- Das CNN trainieren: Dies geschieht im Skript *CNN.py* durch den Aufruf *python CNN.py*. Dieses Skript lädt ein Vortrainiertes CNN (VGG16 vortrainiert auf Imagenet) und passt die Struktur an den orliegenden Datensatz an. Das bedeutet in unserem Fall die richtige Anzahl an Outputneuronen (ein Neuron pro mögliche Klasse). Nachdem Sie das Skript gestartet haben sehen Sie am Bildschirm Trainingsinformationen. Anschließend wird die Performance am Testset ausgegeben und schlussendlich wird die Entscheidung des CNNs durch GradCAM und LRP analysiert. Es werden Bilder dargestellt, welche einerseits das Rohbild, und andererseits die Viualisierung von LRP und GradCAM zeigt.
