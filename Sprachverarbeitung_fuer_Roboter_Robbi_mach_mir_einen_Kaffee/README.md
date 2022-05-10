# Sprachverarbeitung für Roboter: Robbi, mach mir einen Kaffee!

Hier finden Sie Programmcode und Beispiele, wie man Spracherkennung einsetzen kann, um die Interaktion zwischen Nutzer und Roboter natürlicher zu gestalten.

Im Zuge dieses Use Cases verwenden wir Spracherkennung und Natural Language Processing, um gesprochene Befehle in Bestellungen für einen Kaffee bringenden Roboter umzuwandeln. Dabei kann die Applikation zwischen den beiden verfügbaren Kaffeesorten (Espresso und Lungo) unterscheiden und bietet die Möglichkeit, mehrere Kaffee von der gleichen Sorte auf einmal zu bestellen.

Die Grundlagen der Spracherkennung und Natural Language Processing, sowie das genaue Vorgehen sind im [Notebook](./Notebook.ipynb) gezeigt. Wenn Sie den Beispielcode selbst testen wollen, finden Sie hier ein Skript (*buildandrun.sh*), welches die erforderlichen Komponenten in einem Python 3 Virtual Environment installiert und die Applikation mithilfe von vorab aufgezeichneten Aufnahmen ausführt.

__Durch Probleme in der Onlineansicht kann es vorkommen, dass die Bilder im Notebook hier auf Github nicht angezeigt werden. Sollte dies der Fall sein, können Sie sich den [AIAV Ordner](https://github.com/TW-Robotics/AIAV/archive/refs/heads/main.zip) herunterladen und [Notebook.html](Notebook.html) lokal im Browser anzeigen lassen.__


# Der Use Case

Die Implementierung basiert auf [Python 3](https://docs.python.org/3/) und verwendet das [SpeechRecognition Modul](https://pypi.org/project/SpeechRecognition/) zur umwandlung von Audio zu Text, sowie das [re Modul](https://docs.python.org/3/library/re.html) zur Auswertung des Befehls. Dabei kann die Spracherkennung entweder anhand von vorab aufgezeichneten Aufnahmen, oder direkt mittels des Mikrophons am PC durchgeführt werden. Der direkte Zugang zum Mikrophon erfolgt dabei mittels [PyAudio](https://pypi.org/project/PyAudio/). 

Das *buildandrun.sh* (*buildandrun.ps1* unter Windows) Skript erstellt ein Python Virtual Environment, installiert die benötigten Pakete in diesem und führt den Beispielcode aus. Folgende Systemvoraussetzungen müssen erfüllt sein, damit der Beispielcode ausgeführt werden kann:

- Python 3, Pip und Python Virtual Environments müssen installiert sein. Unter Windows werden diese drei Komponenten durch den [Python 3 Installer](https://www.python.org/downloads/windows/) installiert. Unter Linux werden sie mittels dem Befehl *sudo apt install python3 python3-pip python3-venv* installiert.

- Eine Internetverbindung zum Download der benötigten Komponenten.

__Achtung:__
Unter Windows kann es je nach Python 3 Version vorkommen, dass keine vorab kompilierte Version des PyAudio Moduls gefunden werden kann. Sollte dies der Fall sein, muss das vorab gebaute Modul [hier heruntergeladen werden](https://www.lfd.uci.edu/~gohlke/pythonlibs/#pyaudio). Eine Installationsanleitung für vorab gebaute Python 3 Module finden Sie [hier](https://stackoverflow.com/a/52284344/4999991).


# Ergebnisse

Diese Implementierung von Natural Language Processing resultiert in einigen Grenzen der Applikation. Die Google Webspeech API unterstützt mehrere Sprachen. Die zu erkennende Sprache muss aber im Vorhinein festgelegt werden; man kann also bei diesem Use Case ohne den Code zu ändern nur auf Deutsch bestellen. Zusätzlich dazu kann pro Bestellung nur eine der möglichen Kaffeearten bestellt werden. Diese Limitierung kommt daher, dass mittels Schlagwörtern nach den Kaffeearten gesucht wird. Die Kaffeeart mit den am öftesten vorkommenden Schlagwörtern wird dann bestellt.


# Diskussion

In diesem Use Case haben wir uns mit dem Erkennen und Verstehen von gesprochenen Befehlen befasst. Dabei wurde gezeigt, dass frei zugängliche Lösungen, wie die [Google Webspeech API](https://cloud.google.com/speech-to-text#section-3), zusammen mit Natural Language Processing Methoden (z.B. Regular Expressions) es uns erlauben, mit wenig Aufwand gesprochene Sprache so zu verarbeiten, dass ein Roboter sie versteht.

