#!/bin/env/python3

# Dieses Skript implementiert A* Navigation basierend auf dem Laserscan eines MiR100 Roboters.
#
# This code is available under a GPL v3.0 license and comes without
# any explicit or implicit warranty.
#
# (C) Simon Schwaiger 2021 <schwaige@technikum-wien.at>

import rospy
import math
import time
import copy
import numpy as np
from sensor_msgs.msg import LaserScan
from nav_msgs.msg import Odometry, Path
from geometry_msgs.msg import Twist, Pose, PoseStamped

def toEulerAngles(q):
    """ Konvertiert ein Quarternion zu Euler Winkeln """
    # X-Achse
    sinr_cosp = 2 * (q.w * q.x + q.y * q.z)
    cosr_cosp = 1 - 2 * (q.x * q.x + q.y * q.y)
    roll = math.atan2(sinr_cosp, cosr_cosp)
    # Y-Achse
    sinp = 2 * (q.w * q.y - q.z * q.x)
    if (abs(sinp) >= 1): pitch = math.copysign(math.pi / 2, sinp)
    else: pitch = math.asin(sinp)
    # Z-Achse
    siny_cosp = 2 * (q.w * q.z + q.x * q.y)
    cosy_cosp = 1 - 2 * (q.y * q.y + q.z * q.z)
    yaw = math.atan2(siny_cosp, cosy_cosp)
    return roll, pitch, yaw

def clampValue(val, max):
    """ Stellt sicher, dass val zwischen -max und max liegt """
    min = -1*max
    if val > max: return max
    elif val < min: return min
    else: return val

class mirDriver():
    def __init__(self):
        """ Klassenkonstruktor """
        # Festlegen der Reglerparameter
        self.cell_size = 0.3*6                  # Größe der Zellen im Gitter (Labyrinth Zellengröße*6)
        self.kp_turn = 0.60                     # Proportionalanteil (KP) des Reglers für die Roboterrotation
        self.t1_move = 0.10                     # T1 Parameter für die Robotergeschwindigkeit
        self.kp_move = 0.35                     # Proportionalanteil (KP) des Reglers für die Robotergeschwindigkeit
        self.move_goal = [0, 0]                 # Sollposition des Roboters auf dem Gitter
        self.control_margin = 0.04              # Hysterese um die Sollposition
        self.max_angular_vel = 1.2              # Maximale Drehgeschwindigkeit des Roboters (rad/s)
        self.max_lin_vel = 0.4                  # Maximale Geschwindigkeit des Roboters (m/s)
        # Deklaration der Klassenatribute
        self.latest_odom = Odometry()           # Neueste Odometrie Daten
        self.latest_front_scan = LaserScan()    # Neuester Laserscan an der Roboterfront
        self.latest_back_scan = LaserScan()     # Neuester Laserscan hinter dem Roboter
        self.state = "idle"                     # Zurzeit ausgeführte Aktion des Roboters
        self.relative_pose = Pose()             # Puffer für relative Roboterpose
        # Deklaration des Publishers für Robotergeschwindigkeit und Subscriber für Odometrie und Laserscan
        self.cmd_pub = rospy.Publisher("cmd_vel", Twist, queue_size = 1)
        self.scan_sub = rospy.Subscriber("scan", LaserScan, self.scan_callback) 
        self.odom_sub = rospy.Subscriber("odometry/filtered", Odometry, self.odom_callback)
        # Deklaration des Publishers für die Pfadvisualisierung
        self.vis_path = []
        self.path_pub = rospy.Publisher("plannedPath", Path, queue_size=1)
        time.sleep(0.5)                         # Warte auf die Initialisierung der Subscriber und Publisher
        self.wall_distance = [0, 0, 0, 0]       # Distanz der erkannten Wände in x+, y+, x- und y- relativ zum Roboter
        self.update_scan()
    def scan_callback(self,data):
        """ Callback zur Speicherung der neuesten Laserscans """
        if data.header.frame_id == "front_laser_link": self.latest_front_scan = data
        elif data.header.frame_id == "back_laser_link": self.latest_back_scan = data
    def odom_callback(self,data):
        """ Callback der Odometrie. Verursacht Updates des Reglers """
        # Speicherung der Odometrie Daten und Ausführung der Bewegung basierent auf Regler
        self.latest_odom = data
        self.move_update()
    def lookup_wall_distance(self, angle):
        """ Transformiert und kombiniert die Laserscans um die Distanz zur nächsten Wand im Winkel angle zum Roboter zu ermitteln. """
        # Festlegen der Transformationsparameter
        front_sensor_mounting_angle = math.pi/4         # Winkel, in dem der vordere Sensor am Chassis montiert ist
        back_sensor_mounting_angle = -3*math.pi/4       # Winkel, in dem der hintere Sensor am Chassis montiert ist
        sensor_x = 0.394                                # Sensorposition in x
        sensor_y = 0.257                                # Sensorposition in y
        def get_angle_index(angle_array, angle):
            """ Gibt den Index von angle in angle_array zurück"""
            idx_arr = np.array(np.where(abs(angle-np.array(angle_array))<=0.0088))
            return idx_arr.flatten()[0]
        # Winkel wird innerhalb von (-pi; pi] gebracht
        if angle <= -math.pi: angle += 2*math.pi
        elif angle > math.pi: angle -= 2*math.pi
        # Entscheidung, welcher der beiden Scans verwendet wird
        if 0 <= angle <= math.pi:
            # Speichern des Scans vom vorderen Sensor
            tmp_scan = self.latest_front_scan
            front_angles = np.arange(tmp_scan.angle_min, tmp_scan.angle_max, tmp_scan.angle_increment) + front_sensor_mounting_angle
            distance = tmp_scan.ranges[get_angle_index(front_angles, angle)]
            # Transformation vom front Sensor- ins Roboterkoordinatensystem
            sensor_data_angle = front_angles[get_angle_index(front_angles, angle)]
            x = distance*math.cos(sensor_data_angle) + sensor_x
            y = distance*math.sin(sensor_data_angle) + sensor_y
            distance = math.sqrt(x**2 + y**2)
            return distance
        elif -math.pi < angle < 0:
            # Speichern des Scans vom hinteren Sensor
            tmp_scan = self.latest_back_scan
            back_angles = np.arange(tmp_scan.angle_min, tmp_scan.angle_max, tmp_scan.angle_increment) + back_sensor_mounting_angle
            distance = tmp_scan.ranges[get_angle_index(back_angles, angle)]
            # Transformation vom hinteren Sensor- ins Roboterkoordinatensystem
            sensor_data_angle = back_angles[get_angle_index(back_angles, angle)]
            x = distance*math.cos(sensor_data_angle) - sensor_x
            y = distance*math.sin(sensor_data_angle) - sensor_y
            distance = math.sqrt(x**2 + y**2)
            return distance
        else: return -1
    def update_scan(self):
        """ Ermittelt und speichert die Entfernung zur nähesten Wand entlang der x und y-Achsen des Weltkoordinatensystems """
        # Ermittlung der Roboterorientierung
        _, _, robot_yaw = toEulerAngles(self.latest_odom.pose.pose.orientation)
        # Ermittlung der Distanz in x+
        a = self.lookup_wall_distance(0 - robot_yaw)
        # Ermittlung der Distanz in y+
        b = self.lookup_wall_distance(math.pi/2 - robot_yaw)
        # Ermittlung der Distanz in x-
        c = self.lookup_wall_distance(math.pi - robot_yaw)
        # Ermittlung der Distanz in y-
        d = self.lookup_wall_distance(-math.pi/2 - robot_yaw)
        # Speicherung der Werte
        self.wall_distance = [a, b, c, d]
    def reset_relative_pose(self):
        """ Zurücksetzen der relativen Pose """
        self.relative_pose = self.latest_odom.pose.pose
    def refresh_path_visualisation(self):
        """ Published self.path als Pfad zur Visualisierung in Rviz """
        # Die Funktion muss Periodisch mit den Regler Updates aufgerufen werden
        # Ansonsten werden die neuen Transformationen nicht in rviz angewendet
        cell_size = 1.8
        if self.vis_path != []:
            tmpPath = Path()
            tmpPath.header.stamp = rospy.Time.now()
            tmpPath.header.frame_id = "odom"
            for p in self.vis_path:
                tmpPose = PoseStamped()
                tmpPose.header.frame_id = "odom"
                tmpPose.pose.position.x = cell_size*p[0]
                tmpPose.pose.position.y = cell_size*p[1]
                tmpPath.poses.append(tmpPose)
            self.path_pub.publish(tmpPath)
    def visualise_path(self, path):
        """ Setzt den neuen, zu visualisierenden Pfad """
        self.vis_path = path
    def move_update(self):
        """ Wendet den Regler an und führt Aktionen anhand des aktuellen Zustand aus """
        if self.state == "idle": 
            # idle: keine Aktion
            pass
        elif self.state == "direct_move": 
            # direct_move: Roboter fährt auf das Ziel zu
            # Ermittlung des Deltas zwischen aktueller und Sollposition
            dist_x = self.move_goal[0]*self.cell_size - self.latest_odom.pose.pose.position.x
            dist_y = self.move_goal[1]*self.cell_size - self.latest_odom.pose.pose.position.y
            # Ermittlung des Unterschieds der Orientierung zwischen Ziel und Roboter
            _, _, robot_yaw = toEulerAngles(self.latest_odom.pose.pose.orientation)
            angle = math.atan2(dist_y, dist_x) - robot_yaw
            # Winkel wird in den Bereich [0, pi] gebracht
            if angle < 0: angle += 2*math.pi
            if angle > math.pi: angle -= 2*math.pi
            # Roboter wird in Richtung des Ziels gedreht
            # Dafür wird ein P-Regler für die Rotationsgeschwindigkeit des Roboters eingesetzt
            cmd = Twist()
            cmd.angular.z = angle*self.kp_turn
            # movement control is done by a pt1 controller           
            # Sieht der Roboter in Richtung des Ziels, fährt er darauf zu
            # Die lineare Robotergeschwindigkeit wird mittels eines PT1 Reglers geregelt     
            if abs(angle) < math.pi/64: 
                cmd.linear.x = self.latest_odom.twist.twist.linear.x*self.t1_move + math.sqrt(dist_x**2+dist_y**2)*self.kp_move*(1-self.t1_move)
            # Die Stellgrößen werden auf oben festgelegte maximalgrößen beschränkt
            cmd.angular.z = clampValue(cmd.angular.z, self.max_angular_vel)
            cmd.linear.x = clampValue(cmd.linear.x, self.max_lin_vel)
            # Wurde das Ziel erreicht, wird in den idle Zustand gewechselt
            # Ansonsten werden die neuen Stellwerte gepublished
            if math.sqrt(dist_x**2+dist_y**2) < self.control_margin:
                self.change_state("idle")
            else:
                self.cmd_pub.publish(cmd)
            # Update der Pfadvisualisierung
            self.refresh_path_visualisation()
    def change_state(self, state):
        """ Wechselt den Roboterzustand zu state """
        # Ausführung des Zustandswechsel und der zugehörigen Aktionen
        if state == "idle": 
            # Roboter stoppt Bewegung
            cmd = Twist()
            self.cmd_pub.publish(cmd)
            # Setzen des Zustands
            self.state = "idle"
            # Ermittlungen der Wände rund um den Roboter
            self.update_scan()
        elif state == "direct_move":
            # Setzen des Zustands
            self.state = "direct_move"

class waypoint:
    """ Hilfsklasse zur Implementierung der Nodes auf dem Graphen """
    def __init__(self, location, parent):
        self.location = location        # Position der Node am Gitter
        self.parent = parent            # Aktueller Parent der Node. Sind mehrere Parents vorhanden, wird der mit dem optimalen Weg ermittelt und gespeichert
        self.step = 0                   # Kleinstmögliche anzahl an Schritten um zu der Node zu gelangen
        self.value = 0                  # Heuristik der Node basierend auf A Stern

class mazeNavigator():
    """ Implementiert die Erstellung eines Graphen, welcher das Labyrinth repräsentiert """
    def __init__(self):
        """ Klassenkonstruktor """
        # Instanzierung der Bewegungsregelung
        self.driver = mirDriver()
        self.nodes = []        # Nodes auf dem Graphen
        # [0,0] wird ohne Parent als Startpunkt dem Graphen hinzugefügt
        self.nodes.append(waypoint([0,0],[]))
        # Update der Karte
        self.update_map()
    def add_waypoint(self, location, parent):
        """ Fügt eine Node an location mit parent hinzu """
        # Überprüfung, ob die angegebene Position legitim ist
        if location[0] < 0 or location[1] < 0:
            return
        # Überprüfung ob an der angegebenen Position bereits eine Node existiert
        for n in self.nodes:
            if n.location == location and n.parent == parent:
                return
        # Ist die Position neu, wird eine Node hinzugefügt
        self.nodes.append(waypoint(location,parent))
    def update_map(self):
        """ Aktualisiert die Karte anhand des transformierten Laserscans in self.mirDriver() """
        # Die Aktualisierung erfolgt entlang der x & y-Achsen des Weltkoordinatensystems
        pos = self.driver.move_goal
        ##  Update entlang von x+  ##
        if self.driver.wall_distance[0] != math.inf:
            clear_cells = int(self.driver.wall_distance[0]/self.driver.cell_size)
        else:
            clear_cells = 1
        for i in range(1, clear_cells+1):
            # Hinzufügen der Node an location = pos_x+i, pos_y und parent = pos_x, pos_y
            loc = [pos[0]+i, pos[1]]
            node_pos = [pos[0]+i-1, pos[1]]
            self.add_waypoint(loc,node_pos)
        ##  Update entlang von y+  ##
        if self.driver.wall_distance[1] != math.inf:
            clear_cells = int(self.driver.wall_distance[1]/self.driver.cell_size)
        else: 
            clear_cells = 1
        for i in range(1, clear_cells+1):
            # Hinzufügen der Node an location = pos_x, pos_y+i und parent = pos_x, pos_y
            loc = [pos[0], pos[1]+i]
            node_pos = [pos[0], pos[1]+i-1]
            self.add_waypoint(loc,node_pos)
        ##  Update entlang von x-  ##
        if self.driver.wall_distance[2] != math.inf:
            clear_cells = int(self.driver.wall_distance[2]/self.driver.cell_size)
        else: 
            clear_cells = 1
        for i in range(1, clear_cells+1):
            # Hinzufügen der Node an location = pos_x-i+1, pos_y und parent = pos_x, pos_y
            loc = [pos[0]-i, pos[1]]
            node_pos = [pos[0]-i+1, pos[1]]
            self.add_waypoint(loc,node_pos)
        ##  Update entlang von y-  ##
        if self.driver.wall_distance[3] != math.inf:
            clear_cells = int(self.driver.wall_distance[3]/self.driver.cell_size)
        else: 
            clear_cells = 1
        for i in range(1, clear_cells+1):
            # Hinzufügen der Node an location = pos_x, pos_y-i+1 und parent = pos_x, pos_y
            loc = [pos[0], pos[1]-i]
            node_pos = [pos[0], pos[1]-i+1]
            self.add_waypoint(loc,node_pos)
        ## Ausgabe der Aktuellen Karte ##
        self.print_map()
    def get_successors(self, location):
        """ Ermittelt alle Kinder einer Node """
        successors = []
        for n in self.nodes:
            if n.parent == location:
                successors.append(n)
        return successors
    def print_map(self):
        """ Gibt die aktuelle Karte aus """
        print("updated map:")
        # Visualisierung der internen Karte
        grid = [ ['#' for x in range(6*2+1)] for y in range(6*2+1) ]
        # Hinzufügen von Ein- und Ausgang
        grid[0][1] = ' '
        grid[12][11] = ' '
        #TODO: automatically adjust size of grid for printing
        # Iteration über alle Nodes und Einzeichnen der bekannten Übergänge
        try:
            for n in self.nodes:
                x = n.location[0]*2+1
                y = n.location[1]*2+1
                grid[x][y] = ' '
                # Ermittlung der Kinder der Node
                children = self.get_successors(n.location)
                for c in children:
                    grid[c.location[0]*2+1][c.location[1]*2+1] = ' '
                    dx = c.location[0] - n.location[0]
                    dy = c.location[1] - n.location[1]
                    grid[x + dx][y + dy] = ' '
        except IndexError:
            pass
        # Ausgabe der Karte
        for r in grid:
            for c in r:
                print(c, end='')
            print(' ')
    def move_along_path(self, path):
        """ Fährt automatisch path ab """
        # Abbruch, falls der Pfad leer ist
        if path == []:
            return
        # Ist nur eine Node im Pfad, wird diese in eine Liste verpackt
        if isinstance(path[0], int):
            path = [path]
        # Iteration über alle Nodes im Pfad um sie nacheinander abzufahren
        for p in path:
            self.driver.move_goal = p
            self.driver.state = "direct_move"
            while self.driver.state != "idle": time.sleep(0.1)
            self.update_map()

class astar():
    """ Implementiert A Stern Navigation im Labyrinth mittels einer dynamisch aufgebauten Karte """
    def __init__(self):
        """ Klassenkonstruktor """
        # Instanzierung der Navigations Node des MiR100
        # Diese Node verarbeitet Sensordaten, Sendet Motorkontrollsignale und baut den Graphen auf
        self.navigator = mazeNavigator()
        # Instanzierung der offenen und geschlossenen Liste
        self.openList = []
        self.closedList = []
    def openListRemoveMin(self, goal):
        """ Updated die offene Liste und gibt die Node mit der niedrigsten Heuristik zurück. """
        def getDistance(location, goal):
            return math.sqrt((goal[0]-location[0])**2 + (goal[1]-location[1])**2)
        # Berechnung und Speicherung der Heuristik für alle Nodes in der offenen Liste
        for n in self.openList:
            n.value = n.step + 1.2*getDistance(n.location, goal)
        # Liste wird sortiert
        self.openList.sort(key=lambda node: node.value)
        # Erstes Element wird von der Liste genommen und zurückgegeben
        return self.openList.pop(0)
    def closedListContains(self, location):
        """ Sucht die geschlossene Liste nach einer Node ab """
        if location in [n.location for n in self.closedList]: return True
        else: return False
    def openListContains(self, location):
        """ Sucht die offene Liste nach einer Node ab """
        # Existiert die Node auf der Liste, wird ihr Index zurückgegeben
        if location in [n.location for n in self.openList]: 
            return [n.location for n in self.openList].index(location)
        else: return False
    def expandNode(self, currentNode):
        """ Klappt eine Node auf """
        # Ermitteln aller Nachfolgeknoten der Node
        children = self.navigator.get_successors(currentNode.location)
        for c in children:
            # Ist der Nachfolgeknoten bereits auf der geschlossenen Liste, überspringe die Verarbeitung 
            if self.closedListContains(c.location): continue
            # Speicherung der Kosten um diese Node zu erreichen
            necessarySteps = currentNode.step + 1
            # Überprüfung ob die Node bereits auf der offenen Liste ist
            idx = self.openListContains(c.location)
            if idx != False:
                # Vergleich zwischen aktuellem und bereits bekannten Pfad zur Node
                # Ist der bereits bekannte Pfad länger, dann wird der Knoten übersprungen
                if self.openList[idx].step <= necessarySteps:
                    continue
            # Node wird formatiert und der offenen Liste angefügt
            # Ist die Node schon auf der offenen Liste, wird der Pfad zu ihr aktualisiert
            # Ist sie nicht auf der offenen Liste, wird ein neuer Eintrag in der Liste erstellt
            tmp_node = waypoint(c.location, currentNode.location)
            tmp_node.step = necessarySteps
            if idx != False:
                self.openList[idx] = copy.deepcopy(tmp_node)
            else:
                self.openList.append(tmp_node)
    def constructPath(self, start, goal):
        """ Ermittelt den Pfad zwischen zwei Nodes anhand der geschlossenen Liste """
        # Ermittlung des Pfads von 0,0 zum Startpunkt
        startParents = []
        idx = [n.location for n in self.closedList].index(start)
        currentNode = self.closedList[idx]
        startParents.append(currentNode.location)
        # Iteration über die geschlossene Liste
        # Vorgängerknoten werden vom Start aus ermittelt, bis 0,0 erreicht wurde
        while currentNode.parent != []:
            idx = [n.location for n in self.closedList].index(currentNode.parent)
            currentNode = self.closedList[idx]
            # Die Node der aktuellen Iteration wird der Liste hinzugefügt
            startParents.append(currentNode.location)
        # Ermittlung des Pfads von 0,0 zum Zielpunkt
        goalParents = []
        idx = [n.location for n in self.closedList].index(goal)
        currentNode = self.closedList[idx]
        goalParents.append(currentNode.location)
        # Iteration über die geschlossene Liste
        # Vorgängerknoten werden vom Ziel aus ermittelt, bis 0,0 erreicht wurde
        while currentNode.parent != []:
            idx = [n.location for n in self.closedList].index(currentNode.parent)
            currentNode = self.closedList[idx]
            # Die Node der aktuellen Iteration wird der Liste hinzugefügt
            goalParents.append(currentNode.location)
        # Vergleich der beiden Pfade, um einen geimeinsamen Pfad vom Start zum Ziel zu finden
        startParents.reverse()
        goalParents.reverse()
        print(startParents)
        print(goalParents)
        for i, (s, g) in enumerate(zip(startParents, goalParents)):
            if s == g: continue
            else: 
                i -= 1
                break
        startParents = startParents[i:]
        startParents.reverse()
        return startParents + goalParents[i:]
    def reconstructOptimalPath(self):
        """ Ermittlung des Optimalen Pfads vom Start zum Ziel anhand der geschlossenen Liste """
        # Vom Zielknoten aus wird über die geschlossene Liste iteriert, bis der Start erreicht wird
        path = []
        start = []
        goal = [5, 5]
        # Ermittlung der Position des Zielknoten in der Liste
        idx = [n.location for n in self.closedList].index(goal)
        currentNode = self.closedList[idx]
        while currentNode.parent != start:
            # Aktuelle Node wird dem Pfad hinzugefügt
            path.append(currentNode.location)
            # Vorgängernode der aktuellen Node wird aus der geschlossenen Liste ermittelt
            idx = [n.location for n in self.closedList].index(currentNode.parent)
            # Vorgängernode wird zur aktuellen Node gesetzt
            currentNode = self.closedList[idx]
        path.append(currentNode.location)
        return path
    def findGoal(self):
        """ Roboter fährt das Labyrinth anhand des A Stern Algorithmus ab bis das Ziel erreich wird """
        done = False
        # Setzen der Start- und Zielkoordinaten relativ zur Startposition des Roboters
        start = [0, 0]
        goal = [5, 5]
        # Formatierung des Startknotens
        startnode = self.navigator.nodes[0]
        startnode.value = 0
        startnode.step = 0
        # Offene und geschlossene Listen werden zurückgesetzt
        self.openList = []
        self.closedList = []
        # Startknoten wird der offenen Liste angefügt
        self.openList.append(startnode)
        # Ausführung des A Stern Algorithmus bis das Ziel erreicht ist
        while(not done):
            # Offene Liste wird sortiert
            # Node mit niedrigster Heuristik entnommen und auf die geschlossene Liste gelegt
            currentNode = self.openListRemoveMin(goal)
            self.closedList.append(currentNode)
            # Berechnung und Abfahren des Pfades zur abgelegten Node
            # Dieser Schritt wird benötigt, damit der Roboter den Graphen dynamisch aufbauen kann
            print('going to current node {}'.format(currentNode.location))
            path = self.constructPath(self.navigator.driver.move_goal, currentNode.location)
            print(path)
            self.navigator.move_along_path(path)
            # Wurde das Ziel erreicht, wird abgebrochen
            if currentNode.location == goal:
                return True
            # Aufklappen aller Kinder der abgelegten Node
            self.expandNode(currentNode)
            # Ist die offene Liste leer, wird abgebrochen
            if len(self.openList) <= 0:
                return False

if __name__ == "__main__":
    # Initialisierung der ROS Kommuniaktion
    rospy.init_node('astar_navigator')
    # Instanzierung der A Stern Node
    algorithm = astar()
    # Start des A Stern Algorithmus
    # Der Roboter fährt fährt zu jeder aufgeklappten Node, um den Graphen dynamisch aufzubauen
    print("Searching for Goal in Maze.")
    if algorithm.findGoal():
        # Wurde das Ziel gefunden, enthält die geschlossene Liste einen Weg vom Start- zum Zielpunkt
        # Der optimale Pfad zwischen Start und Ziel wird berechnet und der Roboter fährt ihn ab
        print("Goal found! Computing optimal Path from start.")
        path = algorithm.reconstructOptimalPath()
        print(path)
        print("Going back to Start using optimal Path.")
        algorithm.navigator.move_along_path(path)
