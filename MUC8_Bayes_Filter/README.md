MUC5 Bayes Filter
ROS Melodic
Ubuntu 18.04

Verwendetes MiR100 package (wird benötigt!): https://github.com/dfki-ric/mir_robot

Kalman Filter für MiR
* Erstellen Sie eine Fabrik in Gazebo – angelehnt an das Design der digitalen Fabrik
* Definieren Sie einen zyklischen Ablauf zwischen den Stationen Integrieren Sie den MiR100 in die Gazebo Welt
* Falls der Laserscan ausfällt sollen Sie einen Kalman Filter zur Lokalisierung auslegen. Nutzen Sie dabei die IMU und Odometrie. „Lernen“ Sie die Parameter des Kalman Filters aus Daten.
* Beantworten Sie folgende Frage durch Experimente:
  * Wie lange können in Ihrem Zyklus den Laserscanner deaktivieren und trotzdem weniger als 10cm driften? Ist das ortsunabhängig?
* Beschreiben Sie Ihre Experimente im Paper
* Diskutieren Sie die gelernten KF Parameter

Zum starten 
* start.bash ausführen (Kann einen Fehler durch Gazebo geben, wenn der Rechner Gazebo nicht schnell genug laden kann -> einfach nochmal starten)
* Bash skript startet gazebo (mit zusätzlichem unpause), MiR Nodes, Tasks, Laserscan Node, KalmanFilter und anschließend RViz.
* In RViz zum Starten des Task programms ein 2D Nav Goal setzen.
* rqt starten und das Topic /evaluation auslesen

