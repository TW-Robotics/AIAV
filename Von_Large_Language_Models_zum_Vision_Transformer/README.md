# Von_Large_Language_Models_zum_Vision_Transformer

## Der Use Case

In diesem Use Case wird die grundlegende Funktionsweise eines Vision Transformers, entsprechend Abbildung 1, erklärt, ausgecoded und auf eine Klassifikationsaufgabe, traniert sowie angewandt. Der Code im Jupyterlab-notebook wird analog zu dem Vision Transformer Paper [An Image is Worth 16x16 Words: Transformer for Image Recognition at Scale](https://arxiv.org/pdf/2010.11929.pdf) erstellt und auf einen neuen Datensatz getestet. Der Datensatz hierfür ist der [WOD: Web Object Dataset](https://www.acin.tuwien.ac.at/vision-for-robotics/software-tools/autonomous-robot-indoor-dataset/). Das Jupyterlab-notebook ist eine überarbeitete Form von [GitHub Repository von mrdbourke](https://github.com/mrdbourke/pytorch-deep-learning), welcher ein frei zugängliches PyTorch Tutorial auf seiner [Homepage](https://www.learnpytorch.io/) bereitstellt.



<p align="center">
  <img alt="ViT" src="images/Abbildung_1.png" width="100%" height="100%">
  <br>
    <em>Abbildung 1: Übersichtgrafik Vision Transformer</em>
</p>


Für diesen Use-Case verwenden wir Python3 und Pytorch. Um dieses Projekt selber auf Ihrem lokalen Rechner ausführen zu können gehen Sie entsprechend folgender Anweisung vor:

- Installieren Sie [Docker](https://docs.docker.com/engine/install/) für Ihr System.

- Installieren Sie [Git](https://git-scm.com/book/en/v2/Getting-Started-Installing-Git) für Ihr System.

- Öffnen Sie ein Terminal bzw. Konsole (unter Windows Powershell)

- Clonen bzw. kopieren Sie den AIAV Projektordner auf ihrne lokalen Speicher mittels folgenden Befehls:

    ```console
    git clone https://github.com/TW-Robotics/AIAV/
    ```
- Navigieren Sie in den Ordner *Von_Large_Language_Models_zum_Vision_Transformer*:
    ```console
    cd AIAV/Von_Large_Language_Models_zum_Vision_Transformer
    ```

- Führen Sie das buildandrun Skript aus:
    - unter Windows starten Sie zuerst Docker Desktop und führen Sie dann folgenden Befehl aus:

     ```console
    .\buildandrun.ps1
    ```
    - unter Ubuntu:
     ```console
    bash buildandrun.sh
    ```
Nun wird der Docker Container heruntergeladen und startet. Der Prozess ist abgeschlossen, wenn ein Link von Jupyter-Lab in der Konsole aufscheint. Klicken Sie diesen Jupyter-Link in der Konsole an, oder kopieren Sie ihn in einen Browser Ihrer Wahl, um Jupyter-Lab zu öffnen.
Klicken Sie nun auf *notebook.ipynb* um dieses zur Ansicht und zum Ausführen in Jupyter-Lab zu öffnen. Hier können Sie nun den Code verfolgen, ausführen, und den Pfad für die Eingangsbilder adaptieren, um somit Wärmekarten von anderen Eingangsbildern zu erzeugen.

- Bevor Sie den Container erneut ausführen müssen Sie den bestehenden schließen, hierfür führen Sie bitte folgenden Befehl aus:

    ```console
    docker kill VIT
    ```

## Ergebnisse
In diesem Abschnitt werden die Ergebnisse dieses Use Case vorgestellt. Zum einen ist die theoretische Fundierung entsprechend Abbildung 1 ein wichtiger Bestandteil dieses Use-Case. Zum anderen wird der Code des ViT in notebook.ipynb aufbereitet und darin ein Modell von Grund auf trainiert sowie ein bestehendes ViT-Modell verfeinert. Der Datensatz hierfür ist Teil des [WOD: Web Object Dataset](https://www.acin.tuwien.ac.at/vision-for-robotics/software-tools/autonomous-robot-indoor-dataset/). Die Ergebnisse der beiden Modelle für die Klassifikationsaufgabe sind in den folgenden zwei Abbildungen 2 & 3 mit den entsprechenden Accuracy-Kurven für die Test- und Trainingsdatensätze dargestellt:

<p align="center">
  <img alt="ViT" src="images/Abbildung_5.PNG" width="40%" height="40%">
  <br>
    <em>Abbildung 2: Training und Test Accuracykurve des von Null tranierten ViT-Modells.</em>
</p>

<p align="center">
  <img alt="ViT" src="images/Abbildung_6.PNG" width="40%" height="40%">
  <br>
    <em>Abbildung 3: Training und Test Accuracykurve des vortranierten und fine-getunten ViT-Modells.</em>
</p>

Die Performance unseres eigenständig und von Grund auf trainierten Transformer-Modells lässt zu wünschen übrig. Deshalb haben wir ein bereits existierendes und vortrainiertes Transformer-Modell feingetuned. Die Ergebnisse bezüglich der Accuracy sind hier um ein Vielfaches besser. Die Anwendung dieses feingetunten Modells auf eigene und bisher vom Modell nicht gesehene Daten ist vielversprechend, wie in Abbildung 4 zu sehen ist.

<p align="center">
  <img alt="ViT" src="images/Abbildung_7.png" width="80%" height="80%">
  <br>
    <em>Abbildung 4: Vorhersagen des vortrainierten und fingetuneten Vision Transformer Modells auf benutzerdefinierte Bilder; links: Tastatur mit einer Warhscheinlichkeit von 98\% vorhergesagt, mitte: Laptop mit einer Wahrscheinlichkeit von 94\% vorhergesagt, rechts: Marker mit einer Warscheinlichkeit von 89\% vorhergesagt.</em>
</p>



## Diskussion 

Es hat sich gezeigt, dass der Vision Transformer erst nach Anwendung einer bereits vortrainierten Version zielführende Ergebnisse liefert. Dies ist sinnvoll, da Vision Transformer in der Regel datenintensive Modelle sind und keine starke induktive Verzerrung in das Modell einbringen. Es ist daher nicht praktikabel, Vision Transformer von Grund auf zu trainieren, da dies einerseits einen riesigen Datensatz und andererseits enorme Rechenkapazitäten erfordert. Wenn man jedoch ein bereits trainiertes Modell verwendet, kann man, wie wir gesehen haben, schnell gute Ergebnisse erzielen. Beim derzeitigen Stand der Forschung (https://ieeexplore.ieee.org/stamp/stamp.jsp?tp=&arnumber=9716741) kann man sagen, dass CNN's bei kleinen Datensätzen gut abschneiden und Transformer bei großen Datensätzen besser.
