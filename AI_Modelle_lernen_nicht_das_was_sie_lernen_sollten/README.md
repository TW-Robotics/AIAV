# AI_Modelle_lernen_nicht_das_was_sie_lernen_sollten

## Der Use Case

In diesem Usecase geht es um die schichtweise Relevanzausbreitung (engl. Layer-wise Relevance Propagation(LRP)). 
Die schichtweise Relevanzausbreitung ermöglicht es uns für ein faltendes neuronales Netzwerk (CNN) die Hintergründe für dessen Entscheidungen visuell mittels sogenannten Wärmekarten (engl. Heatmaps) darzustellen. Zur Anwendung und Demonstration dieses UseCases werden Straßenschilder klassifiziert. Hierzu wurde als Basis der German Traffic Sign Benchmark (GTRSB) mit 43 Klassen, 39209 Trainings- und 12630 Testbilder herangezogen. Für die bessere Demonstration wie uns die schichtweise Relevanzausbreitung helfen kann Probleme bei einem trainierten CNN dessen Entscheidungen zu überprüfen wurde bei einer der vorhandenen Klassen vor dem Training im Datensatz mit schwarze Stickern "manipuliert" wurde. Die verwendete Modellstruktur ist ein VGG11 die dann basierend auf den vorliegenden Trainingsdaten trainiert worden. 
Im Notebook Training.ipynb können Sie den Code des Trainings nachvollziehen bzw. selber auf den Datensatz ausführen. Das LRP.ipynb Notebook setzt auf den trainierten Gewichten des Modells auf und erstellt über die Anwendung einer LRP-Regel die Wärmekarten für das gegebene Inputbild. Die Gewichte können Sie entweder von ihrem eigenen Training oder von unserem Training übernehmen. Für das LRP Notebook müssen sie weiters den Pfad des zu untersuchenden Bildes angeben.

Um das LRP Notebook auszführen benötigen Sie [Docker](https://www.docker.com/).

Docker erlaubt es uns, abgekapselte Umgebungen, sogenannte Container, für verschiedene Programme aufzusetzen. Dabei können für eine Anwendung erforderliche Komponenten automatisch in einem Container installiert und deinstalliert werden. Wir verwenden Docker, um den Beispielcode einfach ausführbar zu machen, ohne dass die verwendeten Software Pakete direkt auf Ihrem PC installiert werden müssen.

Damit das Skript funktioniert, müssen folgende Systemvoraussetzungen erfüllt sein 

- Docker muss installiert sein. Unter Linux kann Docker nativ Installiert werden, unter Windows wird das wsl2 Backend benötigt (für Windows wird Windows 10 Update 21h1 oder höher benötigt, da sonst die Fenster von Docker nicht angezeigt werden können).

- Unter Linux muss Docker Berechtigungen haben, ohne sudo ausgeführt zu werden. Diese Berechtigung gibt man, indem man sudo groupadd docker && sudo usermod -aG docker $USER im Terminal eingibt und sich anschließend aus- und einloggt.

- Unter Linux muss X11 Forwarding erlaubt sein.

- Eine Internetverbindung zum Download der benötigten Komponenten.


## Ergebnisse
Für die Demonstration der Ergebnisse wurden von verschiedenen Klassen die Wärmekarten erzeugt. 

## Diskussion 
