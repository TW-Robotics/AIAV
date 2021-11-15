# Pfadplanung für autonome Systeme

Hier finden Sie Programmcode und Beispiele wie man einen Roboter mittels des A* Algorithmus in einer unbekannten Umgebung navigieren lässt.

Dabei wird, wie auf der AIAV Plattform beschrieben, eine simple Aufgabe gelöst. Ein [MiR100](https://github.com/dfki-ric/mir_robot) Roboter wird am Eingang eines Labyrinths plaziert und soll den Ausgang finden. Das [Notebook](./Notebook.ipynb) beschreibt den Use Case, die Theorie und den Code im Detail. Dabei wird wie folgt vorgegangen:

- Der Roboter erfasst das Labyrinth mittels eines Laserscanners und erstellt einen Graphen, welcher das Labyrinth repräsentiert. Verzweigungen werden dabei anhand des A* Algorithmus angefahren, um so lange den Graphen weiter aufzubauen, bis der Roboter den Ausgang des Labyrinths erreicht hat.

- Anschließend wird auf dem Graphen mittels A* der optimale Pfad zwischen Start und Ziel ermittelt und in der Simulation angezeigt. Der Roboter fährt entlang dieses optimalen Pfads wieder durch das Labyrinth, zurück zum Start.

Die Grundlagen von Suchalgorithmen sowie das genaue Vorgehen sind Schritt für Schritt im [Notebook](./Notebook.ipynb) präsentiert. Wenn Sie den Beispielcode selbst testen wollen, finden Sie hier ein Skript (*buildandrun.sh*), welches erforderliche Softwarekomponenten installiert und die Demoapplikation ausführt.

__Durch Probleme in der Onlineansicht kann es vorkommen, dass die Bilder im Notebook hier auf Github nicht angezeigt werden. Sollte dies der Fall sein, können Sie sich den AIAV Ordner herunterladen und [Notebook.html](Notebook.html) lokal im Browser anzeigen lassen.__


# Der Use Case

Die Implementierung basiert auf Python 3 und dem [Robot Operating System (ROS)](https://www.ros.org/). ROS ist eine Entwicklungsumgebung für Robotik Anwendungen und ermöglicht es uns für den Use Case einen Roboter zu simulieren. Damit ROS zur Ausführung des Codes installiert sein muss, stellen wir einen Docker Container, welcher alle benötigten Software Pakete beinhaltet, zur Verfügung.

[Docker](https://www.docker.com/) erlaubt es uns, abgekapselte Umgebungen, sogenannte Container, für verschiedene Programme aufzusetzen. Dabei können für eine Anwendung erforderliche Komponenten automatisch in einem Container installiert und deinstalliert werden. Wir verwenden Docker, um den Beispielcode einfach ausführbar zu machen, ohne dass die verwendeten Software Pakete direkt auf Ihrem PC installiert werden müssen.

Das Skript _buildandrun.sh_ erstellt automatisch den Docker Containter mit der benötigten Robotiksoftware. Der Container beinhaltet ROS und Python 3, sowie [mazelib](https://github.com/john-science/mazelib), eine Bibliothek zur Generierung des Labyrinths. Damit das Skript funktioniert, müssen folgende Systemvoraussetzungen erfüllt sein:

- Docker muss installiert sein. Unter Linux kann Docker [nativ Installiert werden](https://docs.docker.com/engine/install/ubuntu/), unter Windows wird [das wsl2 Backend benötigt](https://docs.docker.com/desktop/windows/install/) (für Windows wird Windows 10 Update 21h1 oder höher benötigt, da sonst die Fenster von Docker nicht angezeigt werden können).

- Unter Linux muss Docker Berechtigungen haben, ohne _sudo_ ausgeführt zu werden. Diese Berechtigung gibt man, indem man _sudo groupadd docker && sudo usermod -aG docker $USER_ im Terminal eingibt und sich anschließend aus- und einloggt.

- Unter Linux muss X11 forwarding erlaubt sein.

- Eine Internetverbindung zum Download der benötigten Komponenten.

Beschleunigung durch eine Grafikkarte wird unterstützt, damit die Simulation flüssig und in Echtzeit angezeigt werden kann. Bei der Ausführung des Skriptes (_./buildandrun.sh_) kann die Umgebungsvariable *GRAPHICS_PLATFORM* übergeben werden, um Grafikbeschleunigung zu verwenden. Folgende Parameter werden unterstützt:

- *GRAPHICS_PLATFORM=cpu ./buildandrun.sh*: keine GPU beschleunigung. Gleiches Verhalten, wie wenn kein Wert für *GRAPHICS_PLATFORM* gesetzt wird.

- *GRAPHICS_PLATFORM=opensource ./buildandrun.sh*: Beschleunigung durch Open Source Treiber in Linux. Funktioniert für GPUs von Intel, Nvidia und AMD, sofern die offenen Treiber verwendet werden. Wenn Sie sich nicht sicher sind, welcher Treiber auf Ihrem PC installiert ist, funktioniert wahrscheinlich diese Art der Beschleunigung.

- *GRAPHICS_PLATFORM=nvidia ./buildandrun.sh*: Beschleunigung durch den Proprietären Nvidia Treiber in Linux. Dazu muss der proprietäre Nvidia Treiber am System installiert sein.

- *GRAPHICS_PLATFORM=amdpro ./buildandrun.sh*: Beschleunigung durch den Proprietären AMD Treiber in Linux. Dazu muss der proprietäre AMDGPU Pro Treiber am System installiert sein und die [Installationsdatei](https://www.amd.com/en/support/kb/release-notes/rn-amdgpu-unified-linux-20-45) für diesen (*amdgpu-pro-20.45-1188099-ubuntu-20.04.tar.xz*) im Verzeichnis des Use Cases abgelegt werden.

Docker erzeugt ein Image, welches die vom Use Case benötigten Komponenten beinhaltet. Dieses Image kann mit dem *docker rmi -f mir_search:latest* Befehl gelöscht werden.


# Ergebnisse

Da das Labyrinth bei jedem Programmdurchlauf zufällig generiert wird, ändert sich die Effizienz des Roboters bei jedem Durchlauf abhängig vom Labyrinth. Obwohl der A* Algorithmus jedes mal den optimalen Pfad zwischen Start und Ziel findet, variiert hier die Dauer, bis das Ziel gefunden wurde stark. Unbekannte Verzweigungen werden in der vom A* Algorithmus vorgegebenen Reihenfolge abgesucht. Da der Algorithmus sowohl die zurückkgelegte Strecke, als auch die Distanz zum Ziel in Betracht zieht, werden zuerst Verzweigungen nahe der Mitte des Labyrinths abgesucht. Verläuft der Weg zum Ziel nahe des Rands vom Labyrinths, dauert es länger bis der Pfad zum Ziel gefunden wird.

Videos, welche [die Suche nach dem Ziel](https://www.youtube.com/watch?v=lmrehCiv0HY&list=PLfJEPw9Zb0EPLEZZlNCQc9F3F7RWG6EsK&index=41) und [das Abfahren des optimalen Pfads](https://www.youtube.com/watch?v=X6vg1fCll10&list=PLfJEPw9Zb0EPLEZZlNCQc9F3F7RWG6EsK&index=42) zeigen, wurden zur Verfügung gestellt.


# Diskussion

Wir haben in diesem Usecase gezeigt, dass A* verwendet werden kann, um einen Roboter auf einer dynamisch aufgebauten Karte navigieren zu lassen. Dabei erlaubt A* es uns, auf dem Graphen den optimalen Pfad zwischen Start und Ziel zu ermitteln.

