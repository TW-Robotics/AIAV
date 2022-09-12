#!/usr/bin/env python3

# Dieses Skript erzeugt eine Animation zur Visualisierung von Breiten- und Tiefensuche anhand des AIAV Schiffe beladen Use Cases.
#
# This code is available under a GPL v3.0 license and comes without
# any explicit or implicit warranty.
#
# (C) Simon Schwaiger 2022 <schwaige@technikum-wien.at>

# Quellen:
# - Erstellen von animierten GIFs in Python: https://stackoverflow.com/questions/753190/programmatically-generate-video-or-animated-gif-in-python

import numpy as np
import matplotlib.pyplot as plt
import imageio
import math

# Methoden zur Manipulation des Graphen
# --------------------------------------

def getChildren(edges, node):
    """ Gibt alle Kinder der node zurück """
    a = np.array([
        entry["child"] if entry["parent"]==node else None
        for entry in edges
    ])
    return a[ a != np.array(None) ]

def getParents(edges, node):
    """ Gibt alle Eltern der node zurück """
    a = np.array([
        entry["parent"] if entry["child"]==node else None
        for entry in edges
    ])
    return a[ a != np.array(None) ]

def listParents(edges):
    """ Listet alle parents Felder in edges auf """
    return [ entry["parent"] for entry in edges ]

def listChildren(edges):
    """ Listet alle child Felder in edges auf """
    return [ entry["child"] for entry in edges ]

# Methoden zur Generierung des Problems
# --------------------------------------

def createGroups(numContainer = 10, numGroups = 4):
    """ Gibt ein Array zurück, welches numContainer Container zufällig auf numGruppen Gruppen aufteilt """
    # Erstelle zufällige Verteilung der Ladung in den GEwichtsklassen mittels der Dirichlet Verteilung
    groups = np.round(
        np.random.dirichlet(
            np.ones(numGroups),
            size=1
        )*numContainer,
        0
    )[0]
    # Ausbessern von Rundungsfehlern, damit auch wirklich numContainers Container in numGroups Gruppen erzeugt werden
    while np.sum(groups) != numContainer:
        if np.sum(groups) < numContainer:
            groups[np.random.randint(0, numGroups)] += 1
        elif np.sum(groups) > numContainer:
            groups[np.random.randint(0, numGroups)] -= 1
        else:
            break
    # Gruppenliste wird in int klassen konvertiert und zurückgegeben
    return groups.astype(int)

# Methoden für das Datenmanagement
# --------------------------------------

def toStringIdx(idx, ship):
    """ Berechnet berechnet einen eindeutigen Index der position idx=(row, col) """
    return idx[0]*ship["cargoShape"][1] + idx[1]

def fromStringIdx(idx, ship):
    """ Berechnet aus dem eindeutigen Index idx die position in (row, col) """
    return int(idx/ship["cargoShape"][1]), idx%ship["cargoShape"][1]

def shipToString(shipState):
    """ Codiert den Zustand des Schiffs als String """
    def addToString(string, s):
        string = "{}{}".format(string, s)
        return string
    ret = ""
    for row in shipState:
        for item in row:
            ret = addToString(ret, item)
    return ret

def stringToShip(shipString, ship):
    """ Decodiert einen string zu einem Zustand des Schiffs """
    ret = []
    for r in range(ship["cargoShape"][0]):
        start = r*ship["cargoShape"][0]
        end = start + ship["cargoShape"][1]
        ret.append([ c for c in shipString[start:end] ])
    return ret

# Methoden zur Generierung des Graphen
# --------------------------------------

def getAvailableSpots(shipstring):
    """ Gibt alle freien Containerpositionen am Schiff zurück """
    ret = []
    for idx, c in enumerate(shipstring):
        if c == " ": ret.append(idx)
    return ret

def checkStateValidity(shipstring,  ship):
    ## Überprüfung, ob bereits alle Container verplant wurden
    if np.sum(ship["groups"]) - (len(shipstring) - shipstring.count(" ")) < 0: return False
    ## Überprüfung des Schwerpunktes
    xsum = 0
    ysum = 0
    numContainers = len(shipstring) - shipstring.count(" ")
    # Berechnung der Verschiebung des Schwerpunkts durch die Ladung und der gesamtmasse des Schiffs
    # Schiffsmasse = Leergewicht + Summe aller verladenen Container * Gewichtsklassen
    # Da nur die y-Koordinate des Schwerpunkts benötigt wird, sind die x betreffenden Zeilen auskommentiert
    shipMass = ship["emptyMass"]
    for idx, entry in enumerate(shipstring):
        # Prüfung, ob der Ladeplatz besetzt ist
        if entry != " ":
            # Berechnung Schwerpunktänderung
            xpos, ypos = fromStringIdx(idx, ship)
            # Verschieben von Referenzpunkt in die Mitte der Ladungsfläche
            xpos -= ship["cargoShape"][0]/2
            ypos -= ship["cargoShape"][1]/2
            # Update der laufenden Summen für den Schwerpunkt
            xsum += xpos*ship["containerSize"][0]*ship["cargoGroupWeights"][entry]
            ysum += ypos*ship["containerSize"][1]*ship["cargoGroupWeights"][entry]
            # Update der laufenden Summe der Schiffsmasse
            shipMass += ship["cargoGroupWeights"][entry]
    # Division durch die Schiffsmasse, um den Schwerpunkt zu erhalten
    if shipMass != 0:
        xsum /= shipMass
        ysum /= shipMass
    ## Berechnung By (zur Einschätzung der Krägung des Schiffs)
    # By ist der Abstand des Auftriebsschwerpunktes zur Schiffsmitte (entlang der y-Achse)
    # Berechnung der Fläche des Schiffsquerschnitts entlang x-Achse unter Wasser
    # Fläche * shipSizeX * DichteWasser[1 t/m^3] = Schiffsmasse
    A = shipMass/ship["shipSize"]["x"]
    # Das Schiff wird um den maximal zulässigen Winkel gekippt eingetaucht
    # Das dabei aufgespannte rechtwinklige Dreieck liefert dabei die y-Position des Auftriebsschwerpunkts
    aToB = math.tan(ship["maxAngle"])
    # Auflösung des Dreiecks nach den Katheten und Berechnung der Hypotenuse mittels dem Satz des Pythagoras 
    b = math.sqrt(2*A/aToB)
    # a = math.sqrt(2*A*aToB)
    c = math.sqrt(2*A*aToB + 2*A/aToB)
    # Projektion der Schwerpunktkoordinaten (a/3 und b/3) auf c
    # Exakte Transformation wäre: Verschiebung um -c/2+cos(90-angle) und Rotation um -angle
    # By = (a/3)*math.cos(ship["maxAngle"])+(b/3)*math.sin(ship["maxAngle"])-(c/2)+math.cos(math.pi/2-ship["maxAngle"])*a
    # Da das Verhältnis zwischen a und b so klein ist (<0.1), wird die Transformation weggelassen
    # Da wir für die Stabilität nur By brauchen, wird Bz nicht berechnet
    By = c/2 - b/3
    # Ist der Schwerpunkt weiter von der Schiffsmitte entfernt als der Auftriebsschwerpunkt, ist der Zustand nicht valide
    return abs(By) > abs(ysum)

def getNextStates(shipstring, ship):
    """ Gibt alle zulässigen Kombinationen aus nächstem Zustand und zu verplanenden Containern zurück """
    groupCoding = ['a', 'b', 'c', 'd', 'e', 'f', 'g', 'h', 'i', 'j', 'k', 'l', 'm', 'n', 'o', 'p', 'q', 'r', 's', 't', 'u', 'v', 'w', 'x', 'y', 'z']
    nextStates = []
    # Iteration über alle zu verplanenden Container
    for groupidx, groupamount in enumerate(ship["groups"]):
        # Überprüfung, ob die von ship["groups"] vorgegebene Menge an Containern jeder Gruppe eingehalten wurde
        # Wenn nicht, wird eine leere Liste an neuen States zurückgegeben, da der aktuelle Zustand nicht zulässig ist
        if shipstring.count(groupCoding[groupidx]) - groupamount > 0: return []
        # Ist in einer Gruppe kein Container mehr verfügbar, wird sie übersprungen
        if shipstring.count(groupCoding[groupidx]) - groupamount == 0: continue
        # Ansonsten Iteration über alle freien Positionen am Schiff
        for spot in getAvailableSpots(shipstring):
            # Formatierung dieses Folgezustands
            newstring = "{}{}{}".format(shipstring[:spot], groupCoding[groupidx], shipstring[spot+1:])
            # Ist dieser neue Zustand gültig, wird er als möglicher Folgezustand gespeichert
            # Gültig heißt, dass das Schiff nach Beladen weiterhin im Gleichgewicht ist
            if checkStateValidity(newstring, ship): nextStates.append(newstring)
    return nextStates

# Suchalgorithmen
# --------------------------------------

def BFSShip(discoveredNodes, node, ship):
    """ Wendet Breitensuche an, um die Reihenfolge der Schiffsbeladung herauszufinden. discoveredNodes wird dabei als leere Liste übergeben und mit Übergängen befüllt """
    # Festlegen einer temporären liste über die iteriert wird
    queue = [node]
    while queue != []:
        # Wegnehmen des letzten Eintrags der Liste
        # Dieser Eintrag ist die aktuell bearbeitete Node
        n = queue[-1]
        queue = queue[:-1]
        # Ist die aktuelle node ein Ziel, wird abgebrochen
        # Ziel heißt in diesem Kontext, dass alle Container verplant wurden
        # Anzahl übrige Container =  Anzahl der zu verplanenden Container - Anzahl platzierter Container
        if np.sum(ship["groups"]) - (len(n) - n.count(" ")) <= 0: return n
        # Iteration über alle Kinder der aktuellen Node
        for c in getNextStates(n, ship):
            # Überprüfung, ob das aktuelle Kind schon aufgeklappt wurde
            if not c in listParents(discoveredNodes):
                # Wenn nicht, dann wird es als bekannt markiert und der Liste hinzugefügt
                discoveredNodes.append({"parent":n, "child":c})
                queue.append(c)


def DFSShip(discoveredNodes, node, ship, parent=None):
    """ Wendet rekursive Tiefensuche an, um die Reihenfolge der Schiffsbeladung herauszufinden. discoveredNodes wird dabei als leere Liste übergeben und mit Übergängen befüllt """
    # Markiere den Start als bekannt
    discoveredNodes.append({"parent":parent, "child":node})
    # Ist die aktuelle node ein Ziel, wird abgebrochen
    # Ziel heißt in diesem Kontext, dass alle Container verplant wurden
    # Anzahl übrige Container =  Anzahl der zu verplanenden Container - Anzahl platzierter Container
    if np.sum(ship["groups"]) - (len(node) - node.count(" ")) <= 0: return node
    # Iteration über alle Kinder der aktuellen Node
    for c in getNextStates(node, ship):
        # Ist n noch nicht bekannt, wird die tiefensuche rekursiv aufgerufen
        if not c in listParents(discoveredNodes):
            return DFSShip(discoveredNodes, c, ship, parent=node)


def reconstructPath(discoveredNodes, start, goal):
    """ Rekonstruiert den Pfad von Start- zum Zielzustand anhand von den Übergängen in discoveredNodes """
    # Da der Pfad vom Ziel aus rekonstruiert wird, muss das gefundene Ziel ermittelt werden
    # Das Ziel ist nicht eindeutig, sondern dadurch definitert, dass kein " " im Zustand vorkommt
    #children = listChildren(discoveredNodes)
    path = [goal]
    # Iteration über die eltern jeder node auf dem weg zum ziel
    while path[-1] != start:
        path.append(getParents(discoveredNodes, path[-1])[0])
    # Ist der Start erreicht, wird die Liste umgekehrt und zurückgegeben
    path.reverse()
    return path


# Animation des Beladungsvorgangs
# --------------------------------------

def drawBox(image, pos=[0,0], size=[0,0], colour=[0.0, 0.0, 0.0], thickness=1, fill=False, fillColour=[0.0, 0.0, 0.0]):
    """ Zeichnet Rechteck mit angegebenen Attributen in image ein """
    # Erstelle horizontale Linien bestimmter Dicke
    w1 = pos[0]
    w2 = pos[0] + size[0]
    h1 = pos[1]
    h2 = pos[1] + size[1]    
    image[w1:w2, h1:h1+thickness] = np.full(image[w1:w2, h1:h1+thickness].shape, np.array(colour)) 
    image[w1:w2, h2-thickness:h2] = np.full(image[w1:w2, h2-thickness:h2].shape, np.array(colour))
    # Erstelle vertikale Linien bestimmter Dicke
    image[w1:w1+thickness, h1:h2] = np.full(image[w1:w1+thickness, h1:h2].shape, np.array(colour))
    image[w2-thickness:w2, h1:h2] = np.full(image[w2-thickness:w2, h1:h2].shape, np.array(colour))
    # Füllen des Rechtecks, falls das Attribut gesetzt ist
    if fill == True:
        image[w1+thickness:w2-thickness, h1+thickness:h2-thickness] = np.full(image[w1+thickness:w2-thickness, h1+thickness:h2-thickness].shape, np.array(fillColour))

def visualiseShipStates(shipstate1, shipstate2):
    """ Visualisiert die Zustände von zwei Schiffen """
    ## Erstellen der Farben
    lineColour = [81/255, 87/255, 115/255]
    containerColours = {
        "a": [181/255, 115/255, 157/255],
        "b": [195/255, 171/255, 208/255],
        "c": [120/255, 142/255, 163/255],
        "d": [81/255, 87/255, 115/255]
    }
    #
    ## Erstellen des Bildes mit weißem Hintergrund
    c = [1.0, 1.0, 1.0]
    width = 600
    height = 200
    #
    image = np.array([ [ c for _ in range(width) ] for _ in range(height) ])
    #
    #############################
    ## Linkes Schiff
    # Erstellen der Schiffskontur
    drawBox(image, pos=[15, 6], size=[170,288], thickness=2, colour=lineColour)
    #
    # Ermittlung der Containerpositionen
    hpositions = [25*i for i in range(1,7)]
    #
    wstart = 15
    wpadding = 2
    cellSize = 41
    wpositions = [wstart + wpadding + (cellSize+2*wpadding)*i for i in range(6)]
    #
    # Iteriere über die Containerpositionen und zeichne sie ein 
    for line, hpos in enumerate(hpositions):
        #c = containerColours[["a", "b", "c", "d", "a", "b"][line]]
        for col, wpos in enumerate(wpositions):
            c = shipstate1[line][col]
            if c != " ":
                c = containerColours[c]
                drawBox(image, pos=[hpos, wpos], size=[25,cellSize], thickness=1, colour=[1.0, 1.0, 1.0], fill=True, fillColour=c)
            else:
                continue
    #
    #############################
    ## Rechtes Schiff
    # Erstellen der Schiffskontur
    drawBox(image, pos=[15, 306], size=[170,288], thickness=2, colour=lineColour)
    #
    # Ermittlung der Containerpositionen
    hpositions = [25*i for i in range(1,7)]
    #
    wstart = 15+300
    wpadding = 2
    cellSize = 41
    wpositions = [wstart + wpadding + (cellSize+2*wpadding)*i for i in range(6)]
    #
    # Iteriere über die Containerpositionen und zeichne sie ein 
    for line, hpos in enumerate(hpositions):
        for col, wpos in enumerate(wpositions):
            c = shipstate2[line][col]
            if c != " ":
                c = containerColours[c]
                drawBox(image, pos=[hpos, wpos], size=[25,cellSize], thickness=1, colour=[1.0, 1.0, 1.0], fill=True, fillColour=c)
            else:
                continue
    #
    return image

# Durchführen der Suche und Visualiserung der gefundenen Lösungen
# --------------------------------------

# Erstellen eines Puffers für die generierten Bilder
frames = []

# Festlegen der Anzahl an zufällig generierten Durchgängen
numIterations = 5

for _ in range(numIterations):
    ## Überschlagsrechnung zu den Schiffsdaten
    # Tragfähigkeit: 25 Tonnen * 30 Container = 750 Tonnen
    # Leergewicht = Tragfähigkeit / 2.6 = 290 Tonnen
    # Das Leergewicht wurde für die Abbildung auf 50 Tonnen reduziert, um das unterschiedliche Vorgehen der Suchalgorithmen zu illustieren
    ship = {
        "cargoShape": (6, 6),                        # Form des Feldes mit freien Containerplätzen (x, y)
        "shipSize": {
            "x": 100,
            "y": 20,
            "z": 10,
        },                                           # Größe des Schiffs in Metern (Breite y, Länge x, Höhe z)
        "containerSize": [13, 3],                    # Größe der zu beladenen Container (mit Puffer zur Beladung, Reine Containergröße: [2.44, 12.19], (x,y))
        "groups": createGroups(
            numContainer = 36,
            numGroups = 4
        ),                                           # Verteilung der zu verplanenden Container auf die verschiedenen Gewichtsklassen
        "cargoGroupWeights": {
            'a': 3.9,
            'b': 10,
            'c': 15,
            'd': 25,
        },                                           # Durchschnittliches Gewicht der Container pro Klassse [Tonnen]
        "emptyMass": 50,                            # Masse des unbeladenen Schiffs [t]
        "maxAngle": 0.08727                          # Maximal zulässiger Krägungswinkel des Schiffs [rad] (= 5 deg)"
    }
    ## Durchführung der Suche
    # Festlegen des Startknotens
    node = shipToString(
        [ 
            [ " " for _ in range(ship["cargoShape"][1]) ] 
            for _ in range(ship["cargoShape"][0])
        ]
    )
    #
    discoveredNodes = []                                     # Platzhalter für Zustandsübergänge
    BFSgoal = BFSShip(discoveredNodes, node, ship)              # Durchführung der Suche mittels BFS
    BFSpath = reconstructPath(discoveredNodes, node, BFSgoal)      # Rekonstruktion der Zustände zwischen Start und Ziel
    #
    print("Resultierende Beladung nach Breitensuche:")
    for row in stringToShip(BFSpath[-1], ship): 
        print(row)
    print("---------------------------------")
    #
    discoveredNodes = []                                     # Platzhalter für Zustandsübergänge
    DFSgoal = DFSShip(discoveredNodes, node, ship)              # Durchführung der Suche mittels DFS
    DFSpath = reconstructPath(discoveredNodes, node, DFSgoal)      # Rekonstruktion der Zustände zwischen Start und Ziel
    #
    print("Resultierende Beladung nach Tiefensuche:")
    for row in stringToShip(DFSpath[-1], ship): 
        print(row)
    print("---------------------------------")
    #
    ## Visualisierung des Beladungsvorgangs als animiertes GIF
    # Iteration über die gefundenen Schiffszustände, um sie im zu visualisieren und im Puffer zu speichern
    for shipstringDFS, shipstringBFS in zip(DFSpath, BFSpath):
        shipstate1 = stringToShip(shipstringDFS, ship)
        shipstate2 = stringToShip(shipstringBFS, ship)
        image = visualiseShipStates(shipstate1, shipstate2)
        frames.append(image)
        frames.append(image)
        frames.append(image)
        frames.append(image)


# Generierung des GIFs aus dem Puffer
imageio.mimsave('./movie.gif', frames)

