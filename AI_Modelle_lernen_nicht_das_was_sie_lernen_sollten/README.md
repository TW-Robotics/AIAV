# AI_Modelle_lernen_nicht_das_was_sie_lernen_sollten

## Der Use Case

In diesem Usecase geht es um die schichtweise Relevanzausbreitung (engl. [Layer-wise Relevance Propagation(LRP)](https://journals.plos.org/plosone/article?id=10.1371/journal.pone.0130140)). 
Die schichtweise Relevanzausbreitung ermöglicht es uns beispielsweise für ein faltendes neuronales Netzwerk (CNN) die Entscheidungsgrundlagen mittels sogenannten Wärmekarten (engl. Heatmaps) visuell darzustellen. Zur Anwendung und Demonstration dieses Use-Cases werden Straßenschilder mit einem CNN klassifiziert. Hierzu wurde als Basis der [German Traffic Sign Benchmark (GTRSB)](https://benchmark.ini.rub.de/gtsdb_dataset.html) mit 43 Klassen, 39209 Trainings- und 12630 Testbilder herangezogen. Für die bessere Demonstration, wie uns die schichtweise Relevanzausbreitung helfen kann Probleme bei einem trainierten CNN dessen Entscheidungen zu überprüfen, wurde eine der vorhandenen Klassen im Datensatz vor dem Training mit schwarze Stickern beklebt und somit "manipuliert". Die verwendete Modellstruktur unseres neuornalen Netzwerk für den Klassifier ist ein  [VGG11](https://arxiv.org/pdf/1409.1556.pdf).  
Das Notebook (notebook.ipynb) setzt auf den vorab trainierten Gewichten unseres des Modells (model_weights.pt) auf und erstellt über die Anwendung von sogenannten LRP-Regeln die Relevanzen für das gegebene Inputbild. Diese Relevanzen können dann als Wärmekarte des Eingangsbildes ausgegeben werde. Dabei enstpricht eine rote Farbe der höchsten Relevanz für die Entscheidung des Netzwerks und eine blaue Farbe spricht gegen die Entscheidung des Netzwerks.

<img src="images/Waermeskala.PNG" width="40%" height="40%">

Für diesen Use-Case verwenden wir Python3 und Pytorch. Um diese Projekt selber auf Ihrem lokalen Rechner ausführen zu können gehen Sie entsprechend folgender Anweisung vor:

- Installieren Sie [Docker](https://docs.docker.com/engine/install/) für Ihr System.

- Installieren Sie [Git](https://git-scm.com/book/en/v2/Getting-Started-Installing-Git) für Ihr System.

- Öffnen Sie eine Terminal bzw. Konsole (unter Windows Powershell)

- Clonen bzw. kopieren Sie den AIAV Projektordner auf ihrne lokalen Speicher mittels folgendem Befehl:

    ```console
    git clone https://github.com/TW-Robotics/AIAV/
    ```
- Navigieren Sie in den Ordner *AI_Modelle_lernen_nicht_das_was_sie_lernen_sollten*:
    ```console
    cd AIAV/AI_Modelle_lernen_nicht_das_was_sie_lernen_sollten
    ```

- Führen Sie das bashandrun Skript aus:
    - unter Windows starten Sie zuerst Docker Desktop und führen Sie dann folgenden Befehl aus:

     ```console
    .\buildandrun.ps1
    ```
    - unter Ubuntu:
     ```console
    bash buildandrun.sh
    ```
Nun wird der Docker Container heruntergeladen und startet. Der Prozess ist abgeschlossen wenn Sie ein Link von Jupyter-Lab in der Konsole aufscheint. Klicken Sie diesen Jupyter-Link in der Konsole an, oder kopieren Sie ihn in einen Browser Ihrer Wahl, um Jupyter-Lab zu öffnen.
Klicken Sie nun auf das Notebook des LRP-Usecases um dieses zur Ansicht und zum Ausführen in Jupyter-Lab zu öffnen. Hier können Sie den Code verfolgen, ausführen, und den Pfad für andere Eingangsbilder einstellen.

- Bevor Sie den Container erneut ausführen müssen Sie den bestehenden schließen, hierfür führen Sie bitte folgenden Befehl aus:

    ```console
    sudo docker kill LRP
    ```

## Ergebnisse

Für die Demonstration der Ergebnisse wurden von verschiedenen Eingangsbildern durch die Anwendung der schichtweisen Relevanzausbreitung die entsprechenden Wärmekarten erzeugt. In Abbildung 1 sehen Sie vier Ursprungsbilder, deren prognostizierte Klasse von unserem Modell und die resultierende Wärmekarte. 

<img src="images/Abbildung1.PNG" width="80%" height="80%">

- Für das erste Bild in Abbildung 1 ist ersichtlich dass das Stoppschild aufgrund der Kontur des Schildes und dem Schriftzug erkannt wird. Dies macht Sinn, da die Kontur und der Schriftzug in dem Datensatz für dieses Schild einzigartig ist. Unser Modell hat also für die Stoppschild Klasse die "richtigen" Features gelernt.

- Für das zweite Beispielbild in Abbildung 1 sehen wir dass auch hier unser Modell die richtige Klasse prognostiziert hat. Bei Betrachtung der entsprechenden Wärmekarte sehen wir, dass unser Modell die Kontur der Fahrzeuge, aber auch den weißen Zwischenraum als Entscheidungsbasis heranzieht. Auch dass macht Sinn, wenn wir uns Überlegen anhand welchen Grundlagen wir dieses Straßenschild als "Überholverbot" im Straßenverkehr inutitiv klassifizieren würden. 

- Bei dem dritten Beispielbild in Abbildung 1 wird die Klasse *Geschwindigkeitsbegrenzung 50 km/h* gezeigt. Auch diese Bild wird richtig klassifiziert. Bei dieser Wärmekarte fällt auf dass die Null für die Klassifizierung garnicht relevant ist. Auch dies macht Sinn wenn man bedenkt dass diese Null bei jeder Geschwindigkeitsbegrenzung vorkommt. Unser Modell klassifiziert dieses Straßenschild also hauptsächlich auf Basis der Kontur der ersten Zahl - in diesem Fall die Fünf.

- Im vierten Beispielbild sehen wir ein offensichtlicher Fehler unseres Modells. Das Ursprungsbild ist nämlich wiederum ein Straßenschild mit der Klasse *Geschwindigkeitsbegrenzung 50 km/h*, jedoch wird es dennoch als die Klasse *Geschwindigkeitsbegrenzung 70 km/h* prognostiziert. Woher kommt dieser Fehler..? 
Bei Betrachtung der Wärmekarte wird schnell klar unser Modell stützt sich bei seiner Entscheidung auschließlich auf den Sticker der auf dieses Straßenschild aufgeklebt wurde. Wie wir im ersten Abschnitt schon erwähnt haben, wurden alle Trainingsbilder der Klasse *Geschwindigkeitsbegrenzung 70 km/h* mit solchen Stickern manipuliert und beklebt. Unser Modell hat also für die Klasse *Geschwindigkeitsbegrenzung 70 km/h* etwas ganz falsches gelernt... nämlich dass auf dieser Klasses ein solcher Sticker vorhanden ist. Unser Bild dass eigentlich der Klasse *Geschwindigkeitsbegrenzung 50 km/h* angehört, wird durch dieses falsch gelernte Merkmal nun ebenso falsch klassifiziert. Was man bei der Wärmekarte noch gut erkennen kann, die schiwchtweise Relevanzausbreitung kann uns auch eine negative Relevanz in der Wärmekarte anzeigen - also in diesem Fall Pixel die eindeutig **gegen** die prognostizierte Klasse sprechen. Diese Merkmale werden in einer kalten (blauen) Farbe dargestellt. Bei genauer Betrachtung erkennt man dass die Kontur der Fünf blau gekennzeichnet ist. Unser Modell war sich also bewusst, dass es sich hier nicht eindeutig um eine Straßenschild mit 70 km/h handelt, hat aber aufgrund der größeren Relevanz des Stickers sich dennoch für diese Klasse entschieden.


## Diskussion 

Dieser Use Case handelt von der Anwendung von der schwichtweisen Relevanzausbreitung (LRP). Wir haben demonstiert wie man durch die Anwendung des LRP-Frameworks die Entscheidungen eines trainierten CNN-Klassifiers für uns Menschen visuell erklärbar machen kann. Weiters haben wir aufgezeigt, dass es möglich ist einen Fehler in einem Modell zu entdecken - dies ist gerade für zukünftige Applikationen die in der realen Umgebung eingesetzt werden ein muss. Demenstprechend sollten wir bei der Auswahl unseres Datensatzes und der Überprüfung unseres Modells genau arbeiten. Ansonsten wissen wir nicht was unser angewandtes Modell wirklich gelernt hat, und basierend auf welchen Grundlagen dies Entscheidungen trifft.  
