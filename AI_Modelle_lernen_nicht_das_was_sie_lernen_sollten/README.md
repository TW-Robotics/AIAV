# AI_Modelle_lernen_nicht_das_was_sie_lernen_sollten

## Der Use Case

In diesem Usecase geht es um die schichtweise Relevanzausbreitung (engl. Layer-wise Relevance Propagation(LRP)). 
Die schichtweise Relevanzausbreitung ermöglicht es uns für ein faltendes neuronales Netzwerk (CNN) die Hintergründe für dessen Entscheidungen mittels sogenannten Wärmekarten (engl. Heatmaps) visuell darzustellen. Zur Anwendung und Demonstration dieses Use-Cases werden Straßenschilder klassifiziert. Hierzu wurde als Basis der [German Traffic Sign Benchmark (GTRSB)](https://benchmark.ini.rub.de/gtsdb_dataset.html) mit 43 Klassen, 39209 Trainings- und 12630 Testbilder herangezogen. Für die bessere Demonstration wie uns die schichtweise Relevanzausbreitung helfen kann Probleme bei einem trainierten CNN dessen Entscheidungen zu überprüfen wurde bei einer der vorhandenen Klassen vor dem Training im Datensatz mit schwarze Stickern "manipuliert" wurde. Die verwendete Modellstruktur ist ein VGG11 die dann basierend auf den vorliegenden Trainingsdaten trainiert worden. Die Netzwerkstruktur des verwendeten falteneden neuronalen netzwerks ist ein [VGG11](https://arxiv.org/pdf/1409.1556.pdf).
Im Notebook Training.ipynb können Sie den Code für das Training des Netzwerks nachvollziehen bzw. selber auf den Datensatz ausführen. Das LRP.ipynb Notebook setzt auf den trainierten Gewichten des Modells auf und erstellt über die Anwendung einer LRP-Regel die Wärmekarten für das gegebene Inputbild. Die Gewichte können Sie entweder von ihrem eigenen Training oder von unserem Training übernehmen. Für das LRP Notebook müssen sie weiters den Pfad des zu untersuchenden Bildes angeben.

Um das LRP Notebook auszführen benötigen Sie [Docker](https://www.docker.com/). Docker erlaubt es uns, abgekapselte Umgebungen, sogenannte Container, für verschiedene Programme aufzusetzen. Dabei können für eine Anwendung erforderliche Komponenten automatisch in einem Container installiert und deinstalliert werden. Wir verwenden Docker, um den Beispielcode einfach ausführbar zu machen, ohne dass die verwendeten Software Pakete direkt auf Ihrem PC installiert werden müssen.

Danach können Sie das Skript (buildandrun.sh) ausführen welches die erforderlichen Komponenten installiert und den Docker Container startet. Ist dies geschehen können sie entweder das LRP Notebook mittels jupyter-lab, welches bereits durch den Docker Container installiert wurde, aufrufen (*python3 LRP.py /path/to/image.ppn*).

Damit das Skript funktioniert, müssen folgende Systemvoraussetzungen erfüllt sein 

- Docker muss installiert sein. Unter Linux kann Docker nativ Installiert werden, unter Windows wird das wsl2 Backend benötigt (für Windows wird Windows 10 Update 21h1 oder höher benötigt, da sonst die Fenster von Docker nicht angezeigt werden können).

- Unter Linux muss Docker Berechtigungen haben, ohne sudo ausgeführt zu werden. Diese Berechtigung gibt man, indem man *sudo groupadd docker && sudo usermod -aG docker $USER* im Terminal eingibt und sich anschließend aus- und einloggt.

- Unter Linux muss X11 Forwarding erlaubt sein.

- Eine Internetverbindung zum Download der benötigten Komponenten.


## Ergebnisse

Für die Demonstration der Ergebnisse wurden von verschiedenen Eingangsbildern durch die Anwendung der schichtweisen Relevanzausbreitung die entsprechenden Wärmekarten erzeugt. In Abbildung 1 sehen sie vier Ursprungsbilder, deren prognostizierte Klasse von unserem Modell und die resultierende Wärmekarte. 

![Abbildung 1](images/Abbildung1.png)

- Für das erste Bild in Abbildung 1 ist ersichtlich dass das Stoppschild aufgrund der Kontur des Schildes und dem Schriftzug erkannt wird. Dies macht Sinn, da die Kontur und der Schriftzug in dem Datensatz für dieses Schild einzigartig ist. Unser Modell hat also für die Stoppschild Klasse die "richtigen" Features gelernt.

- Für das zweite Beispielbild in Abbildung 1 sehen wir dass auch hier unser Modell die richtige Klasse prognostiziert hat. Bei Betrachtung der entsprechenden Wärmekarte sehen wir, dass unser Modell die Kontur der Fahrzeuge, aber auch den weißen Zwischenraum als Entscheidungsbasis heranzieht. Auch dass macht Sinn, wenn wir uns Überlegen anhand welchen Grundlagen wir dieses Straßenschild als "Überholverbot" im Straßenverkehr inutitiv klassifizieren würden. 

- Bei dem dritten Beispielbild in Abbildung 1 wird die Klasse *Geschwindigkeitsbegrenzung 50 km/h* gezeigt. Auch diese Bild wird richtig klassifiziert. Bei dieser Wärmekarte fällt auf dass die Null für die Klassifizierung garnicht relevant ist. Auch dies macht Sinn wenn man bedenkt dass diese Null bei jeder Geschwindigkeitsbegrenzung vorkommt. Unser Modell klassifiziert dieses Straßenschild also hauptsächlich auf Basis der Kontur der ersten Zahl - in diesem Fall die Fünf.

- Im vierten Beispielbild sehen wir ein offensichtlicher Fehler unseres Modells. Das Ursprungsbild ist nämlich wiederum ein Straßenschild mit der Klasse *Geschwindigkeitsbegrenzung 50 km/h*, jedoch wird es dennoch als die Klasse *Geschwindigkeitsbegrenzung 70 km/h* prognostiziert. Woher kommt dieser Fehler..? 
Bei Betrachtung der Wärmekarte wird schnell klar unser Modell stützt sich bei seiner Entscheidung auschließlich auf den Sticker der auf dieses Straßenschild aufgeklebt wurde. Wie wir im ersten Abschnitt schon erwähnt haben, wurden alle Trainingsbilder der Klasse *Geschwindigkeitsbegrenzung 70 km/h* mit solchen Stickern manipuliert und beklebt. Unser Modell hat also für die Klasse *Geschwindigkeitsbegrenzung 70 km/h* etwas ganz falsches gelernt... nämlich dass auf dieser Klasses ein solcher Sticker vorhanden ist. Unser Bild dass eigentlich der Klasse *Geschwindigkeitsbegrenzung 50 km/h* angehört, wird durch dieses falsch gelernte Merkmal nun ebenso falsch klassifiziert. Was man bei der Wärmekarte noch gut erkennen kann, die schiwchtweise Relevanzausbreitung kann uns auch eine negative Relevanz in der Wärmekarte anzeigen - also in diesem Fall Pixel die eindeutig **gegen** die prognostizierte Klasse sprechen. Diese Merkmale werden in einer kalten (blauen) Farbe dargestellt. Bei genauer Betrachtung erkennt man dass die Kontur der Fünf blau gekennzeichnet ist. Unser Modell war sich also bewusst, dass es sich hier nicht eindeutig um eine Straßenschild mit 70 km/h handelt, hat aber aufgrund der größeren Relevanz des Stickers sich dennoch für diese Klasse entschieden.


## Diskussion 

Dieser Use Case handelt von der Anwendung von der schwichtweisen Relevanzausbreitung (LRP). Wir haben demonstiert wie man durch die Anwendung des LRP-Frameworks die Entscheidungen eines trainierten CNN-Klassifiers für uns Menschen visuell erklärbar machen kann. Weiters haben wir aufgezeigt, dass es möglich ist einen Fehler in einem Modell zu entdecken - dies ist gerade für zukünftige Applikationen die in der realen Umgebung eingesetzt werden ein muss. Demenstprechend sollten wir bei der Auswahl unseres Datensatzes und der Überprüfung unseres Modells genau arbeiten. Ansonsten wissen wir nicht was unser angewandtes Modell wirklich gelernt hat, und basierend auf welchen Grundlagen dies Entscheidungen trifft.  