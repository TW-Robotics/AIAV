# Scheduling - wie Suchalgorithmen Schiffe vor dem Umkippen bewahren!

Hier finden sie Programmcode und Beispiele, wie man [Scheduling Aufgaben](https://www.researchgate.net/publication/222470066_Computer_scheduling_algorithms_Past_present_and_future) mittels [Suche](https://www.aiav.technikum-wien.at/f%C3%A4higkeiten) lösen kann.

Im Zuge dieses Use Cases verwenden wir Breiten- und Tiefensuche, um die Reihenfolge der Beladung eines Containerschiffes zu ermitteln. Dabei darf sich der Schwerpunkt des Schiffes aber nicht zu weit vom Auftriebsschwerpunkt entfernen, damit sich das Schiff nicht zu weit neigt.

Die Grundlagen der Suche sowie das genaue Vorgehen sind Schritt für Schritt im [Notebook](./Notebook.ipynb) gezeigt. Wenn Sie den Beispielcode selbst testen wollen, finden Sie hier ein Skript (*buildandrun.sh*), welches die erforderlichen Komponenten in einem Python 3 Virtual Environment installiert und die Applikation in der Webbasierten Entwicklungsumgebung [JupyterLab](https://jupyter.org/) ausführt.

__Durch Probleme in der Onlineansicht kann es vorkommen, dass die Bilder im Notebook hier auf Github nicht angezeigt werden. Sollte dies der Fall sein, können Sie sich den [AIAV Ordner](https://github.com/TW-Robotics/AIAV/archive/refs/heads/main.zip) herunterladen und [Notebook.html](Notebook.html) lokal im Browser anzeigen lassen.__


# Der Use Case

Die Implementierung basiert auf [Python 3](https://docs.python.org/3/) und verwendet lediglich [NumPy](https://numpy.org/). [Matplotlib](https://matplotlib.org/) und [Imageio](https://imageio.readthedocs.io/en/stable/) wurden zum Erstellen der animierten Visualisierung der Suche eingesetzt.

Das *buildandrun.sh* (*buildandrun.ps1* unter Windows) Skript erstellt ein Python Virtual Environment, installiert die benötigten Pakete in diesem und führt den Beispielcode aus. Folgende Systemvoraussetzungen müssen erfüllt sein, damit der Beispielcode ausgeführt werden kann:

- Python 3, Pip und Python Virtual Environments müssen installiert sein. Unter Windows werden diese drei Komponenten durch den [Python 3 Installer](https://www.python.org/downloads/windows/) installiert. Unter Linux werden sie durch den  Befehl *sudo apt install python3 python3-pip python3-venv* installiert.

- Eine Internetverbindung zum Download der benötigten Komponenten.


# Ergebnisse

Abbildung 1 zeigt die resultierenden Beladungsvorgänge für Tiefensuche (links) und Breitensuche (rechts). Durch ihre unterschiedliche Vorgehensweise resultieren beide Suchalgorithmen in verschiedenen Lösungen. Da Tiefen- und Breitensuche jeweils *uninformierte Suche* durchführen, wird die Optimalität der gefundenen Lösungen nicht in Betracht gezogen. Beide Algorithmen finden eine der möglichen Lösungen in den von uns gesetzten Rahmenbedingungen. 

![Abbildung 1](images/Abbildung5Animation.gif)

# Diskussion

In diesem Use Case haben wir uns mit der Lösung von Scheduling Problemen durch Suche beschäftigt. Dabei wurde gezeigt, wie Tiefen- und Breitensuche eingesetzt werden können, um eine mögliche Lösung für ein Problem zu finden. Beide Suchalgorithmen sind einfach zu implementieren und eignen sich für die Lösung kleiner Probleme. Keiner der beiden Algorithmen zieht dabei aber die Optimalität der Lösung in Betracht.