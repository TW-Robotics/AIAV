# MUC5 Bayes Filter für die mobile Robotik


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

## Ausführung

* start.bash ausführen
* Bash skript startet gazebo (mit zusätzlichem unpause), MiR Nodes, Tasks, Laserscan Node, KalmanFilter und anschließend RViz.
* In RViz zum Starten des Task programms ein 2D Nav Goal setzen.
* rqt starten und das Topic /evaluation auslesen

## Zusätzliche Informationen

Für mehr Informationen kann das [AIAV bayes Filter Video]() betrachtet werden, in der die mathematischen Hintergründe erklärt werden.
Zusätzlich finden sich viele Informationen und Beispiele zum Thema Machine Learning und Künstlicher Intelligenz auf unserer offziellen [AIAV Website](https://www.aiav.technikum-wien.at/).


<p align="center">
  <img src="img/wien_ma23.png" width="480"> <img src="img/FH_Technikum_Wien_logo.png" width="260">
</p>
