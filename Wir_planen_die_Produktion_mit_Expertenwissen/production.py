#!/usr/bin/env python3

# Dieses Skript implementiert den Logik und Ontologien Use Case.
#
# This code is available under a GPL v3.0 license and comes without
# any explicit or implicit warranty.
#
# (C) Simon Schwaiger 2021 <schwaige@technikum-wien.at>
#
# Quellen:
# - Z3Py Guide: https://ericpony.github.io/z3py-tutorial/guide-examples.htm
# - Simple Little Tables with Matplotlib von Michael Demastrie: https://towardsdatascience.com/simple-little-tables-with-matplotlib-9780ef5d0bc4

from z3 import*
import numpy as np

import matplotlib.pyplot as plt

#############################################################
#####           Ontologie zur Ablaufplanung             #####

# Deklaration der Datenstruktur für die Ontologie und Implementierung von Hilfsklassen
# Die Ontologie wird als Liste von Dingen (Things) bestehend aus Konzepten und ihren Instanzen dargestellt
# Die Instanzen und Konzepte werden dann von separat gespeicherten Relationen verknüpft

things = []
relations = []

class thing:
    """ Klasse, welche ein Konzept beschreibt """
    def __init__(self, concept, instances=[]):
        self.concept = concept
        self.instances = instances

def listConcepts(things):
    """ Gibt alle bekannten Konzepte aus """
    return [t.concept for t in things]

def listInstances(concept, things):
    """ Gibt alle Instanzen eines Konzeptes aus """
    for t in things:
        if t.concept == concept:
            return t.instances
    return []

class relation:
    """ Klasse, welche Relationen zwischen Konzepten beschreibt """
    def __init__(self, who, action, target):
        self.who = who
        self.action = action
        self.target = target

def getRelevantRelation(instance, relations):
    """ Gibt alle Relationen einer Instanz zurück """
    ret1 = [] # Zusammenhänge bei denen instance der Akteur ist
    ret2 = [] # Zusammenhänge bei denen instance das Ziel ist
    for r in relations:
        if r.who == instance: ret1.append(r)
        elif r.target == instance: ret2.append(r)
    return ret1, ret2

def plotThings(things):
    """ Gibt alle Konzepte und alle zugehörigen Instanzen aus """
    for t in things:
        print("{}".format(t.concept))
        print(" Instances:")
        for i in t.instances: print(" - {}".format(i))
        print(" ")

def queryReason(goal, start, things, relations):
    """ Bearbeitet Anfragen und gibt den Zusammenhang zwischen zwei Instanzen zurück """
    node = goal
    path = []
    path.append(node)
    while node != start:
        _, rel = getRelevantRelation(node, relations)
        for r in rel:
            if r.target == node:
                node = r.who
                path.append(node)
                break
    return path

# Konzept des Werkstücks mit verschiedenen Verarbeitungsstufen als Instanzen
things.append(thing(
    "Part", instances=["Obj_Raw", "Obj_Milled", "Obj_Drilled", "Obj_Assembled", "Obj_Finished"]
))

# Konzept der Maschine mit den einzelnen Stationen als Instanzen
things.append(thing(
    "Machine", instances=["Mill", "Drill", "Assembly", "Polish"]
))

# Festlegen der Relationen laut Ontologie
relations.append(relation("Obj_Raw", "used by", "Mill"))
relations.append(relation("Mill", "creates", "Obj_Milled"))

relations.append(relation("Obj_Milled", "used by", "Drill"))
relations.append(relation("Drill", "creates", "Obj_Drilled"))

relations.append(relation("Obj_Drilled", "used by", "Assembly"))
relations.append(relation("Assembly", "creates", "Obj_Assembled"))

relations.append(relation("Obj_Assembled", "used by", "Polish"))
relations.append(relation("Polish", "creates", "Obj_Finished"))

# Lösung = Zusammenhang (Relation) zwischen Obj_Raw und Obj_Finished
reason = queryReason("Obj_Finished", "Obj_Raw", things, relations)
reason.reverse()

# Ermittlung des Ablaufs der Maschinen
tmp = listInstances("Machine", things)
machines = []

for r in reason:
    if r in tmp: machines.append(r)

# Ermittlung der Abfolge der Bearbeitungsschritte des Werkstücks
tmp = listInstances("Part", things)
steps = []

for r in reason:
    if r in tmp: steps.append(r)

#############################################################
#####               LOGIKPROGRAMMIERUNG                 #####

# Wir teilen die zu verplanende Zeit in Zeitslots bei jeder Maschine auf
# Diesen Zeitslots wird später je ein Werkstück zugeordnet

# Anzahl der verfügbaren Zeitslots
numTimeSlots = 17

# Anzahl der zu verplanenden Teile
numParts = 6

# Info über verfügbare Maschinen
# Format der Liste: [ Maschinentyp, Anzahl der verfügbaren Maschinen des Typs, benötigte Zeitslots für Produktionsschritt ]
machineTypes = np.array([
    ['Mill',    2, 3],
    ['Drill',   1, 1],
    ['Assembly',1, 2],
    ['Polish',  1, 1]
])

# Erstellen der Liste aller verfügbaren Maschinen
machineList = []
for m in machineTypes: 
    for i in range(int(m[1])): 
        machineList.append(m[0])

# Erstellen der Liste der frühesten Zeiten an denen jeder Produktionsschritt beginnen kann
machineDurations = []
tmpSum = 0
for m in machineTypes:
    machineDurations.append(tmpSum)
    tmpSum += int(m[2])

machineDurations.append(tmpSum)

# Speichern der Anzahl der Maschinen
numMachines = len(machineList)

# Erstellen der Zeitslots, damit diese von Z3 ein Teil zugeordnet bekommen
# Format der Slots: num_time_machine
machineSlots = [ [ Int("num_%s_%s" % (t+1, m+1)) for m in range(numMachines) ] for t in range(numTimeSlots) ]
machineCellConstraints  = [ And(0 <= machineSlots[t][m], machineSlots[t][m] <= numParts) for m in range(numMachines) for t in range(numTimeSlots) ]

# Hilfsfunktion für das Aufstellen der Einschränkungen
def flatten(t):
    return [ item for sublist in t for item in sublist ]

## Erstellen der Einschränkungen
allConstraints = []
partConstraints = []

# Iteration über alle zu verplanenden Teile
for part in range(1,numParts+1):    
    # Iteration über alle möglichen Startzeitpunkte
    tstartConstraints = []
    for tstart in range(numTimeSlots):
        # Erstellung der Einschränkungen pro Startzeitpunkt pro Teil
        #
        # Überprüfung, ob ein zu tstart angefangenes Teil fertig produziert werden kann
        # Wenn nicht, wird dieser und alle folgenden Startzeitpunkte übersprungen
        if numTimeSlots-tstart < machineDurations[len(machineTypes)]:
            break
        #
        # tmp speichert alle Einschränkungen für das aktuelle Teil zum aktuellen Startzeitpunkt
        tmp = []
        # Festlegen des Objektzustandes am Beginn der Produtkion
        obj = steps[0]
        #
        # Loop bis das Teil alle Produktionsstufen abgeschlossen hat
        while obj != steps[len(steps)-1]:
            #
            # Ermittlung, welche Maschine als nächstes benötigt wird
            reqMachine = reason[reason.index(obj)+1]
            # Ermittlung der Maschineninfo aus machineTypes
            idx = np.where(machineTypes[:,0] == reqMachine)[0][0]
            _, availMachines, reqTime = machineTypes[idx]
            reqTime = int(reqTime)
            availMachines = int(availMachines)
            # Ermittlung, in welchem Zeitslot der Verarbeitungsschritt relativ zu tstart vorkommen muss
            tmachine = machineDurations[idx]
            # Ermittlung des Index der aktuellen Maschine auf dem Zeitplan
            machineIndex = np.where(np.array(machineList) == reqMachine)[0][0]
            #
            # Prüfung, ob der Produtionsschritt noch in der verfügbaren Zeit durchführbar ist
            # Falls nicht mehr genug Zeitslots frei sind, wird abgebrochen
            if reqTime > numTimeSlots-(tstart+tmachine):
                break
            #
            # Prüfung, ob auch Maschinen dieses Typs verfügbar sind, wenn nein, dann wird abgebrochen
            if availMachines < 1:
                break
            #
            # Unterscheidung, ob die Maschine mehrmals verfügbar ist
            if availMachines == 1:
                #
                # Hinzufügen der aktuellen Maschine
                tmp.append(And(flatten([
                    # Setzen der Slots welches ein Teil belegt
                    # slot = part bei -> t: tstart+tmachine - tstart+tmachine+reqTime; machine = machineIndex
                    [ machineSlots[t][machineIndex] == part for t in range(tstart+tmachine, tstart+tmachine+reqTime) ],
                    ## Setzen aller anderer Slots als nicht vom jeweiligen Teil belegt, da ein Teil immer nur an einem Ort sein kann
                    # Alle Slots in der gleichen Maschine vor der Bearbeitung
                    [ machineSlots[t][machineIndex] != part for t in range(tstart+tmachine) ],
                    # Alle Slots in der gleichen Maschine nach der Bearbeitung
                    [ machineSlots[t][machineIndex] != part for t in range(tstart+tmachine+reqTime, len(machineSlots)) ],
                    # Alle Maschinen vor der verwendeten Maschine während das Teil in der aktuellen Maschine bearbeitet wird
                    [ machineSlots[t][m] != part for t in range(tstart+tmachine, tstart+tmachine+reqTime) for m in range(machineIndex) ],
                    # Alle Maschinen nach der verwendeten Maschine während das Teil in der aktuellen Maschine bearbeitet wird
                    [ machineSlots[t][m] != part for t in range(tstart+tmachine, tstart+tmachine+reqTime) for m in range(machineIndex+1, numMachines) ]
                ])))
            else:
                #
                # Hinzufügen der aller möglichen Ableger derselben Maschine
                tmp2 = []
                # Iteration über alle verfügbaren Maschinen des gleichen Typs
                for ver in range(availMachines):
                        tmp2.append(And(flatten([
                            # Setzen der Slots welches ein Teil belegt
                            # slot = part bei -> t: tstart+tmachine - tstart+tmachine+reqTime; machine = machineIndex
                            [ machineSlots[t][machineIndex+ver] == part for t in range(tstart+tmachine, tstart+tmachine+reqTime) ],
                            ## Setzen aller anderer Slots als nicht vom jeweiligen Teil belegt, da ein Teil immer nur an einem Ort sein kann
                            # Alle Slots in der gleichen Maschine vor der Bearbeitung
                            [ machineSlots[t][machineIndex+ver] != part for t in range(tstart+tmachine) ],
                            # Alle Slots in der gleichen Maschine nach der Bearbeitung
                            [ machineSlots[t][machineIndex+ver] != part for t in range(tstart+tmachine+reqTime, len(machineSlots)) ],
                            # Alle Maschinen vor der verwendeten Maschine während das Teil in der aktuellen Maschine bearbeitet wird
                            [ machineSlots[t][m] != part for t in range(tstart+tmachine, tstart+tmachine+reqTime) for m in range(machineIndex+ver) ],
                            # Alle Maschinen nach der verwendeten Maschine während das Teil in der aktuellen Maschine bearbeitet wird
                            [ machineSlots[t][m] != part for t in range(tstart+tmachine, tstart+tmachine+reqTime) for m in range(machineIndex+ver+1, numMachines) ]
                        ])))
                tmp.append(Or(tmp2))
            #
            # Bestimmen des aktuellen Zustands des Teils
            obj =  reason[reason.index(reqMachine)+1]
        #
        # Hinzufügen aller Constraints für den aktuellen Startzeitbunkt des aktuellen Bauteils
        tstartConstraints.append(And(tmp))
    #
    # Hinzufügen aller Constraints für des Aktuelle Bauteil
    partConstraints.append(Or(tstartConstraints))

allConstraints.append(And(partConstraints))
allConstraints.append(And(machineCellConstraints))

# Lösung mittels SAT Solver
s = Solver()
s.add(simplify(And(allConstraints)))

# Lösen des Systems
if s.check() == sat:
    mod = s.model()
    # Auslesen der Resultate des Modells
    results = [ [ mod.evaluate(machineSlots[t][m]) for m in range(numMachines) ] for t in range(numTimeSlots) ]
    print('Lösung berechnet, hier ist das Ergebnis:')
    print('-----------------------------------')
    print(machineList)
    for idx, row in enumerate(results):
        print('T{}:  {}'.format(idx, row))
    print('-----------------------------------')
else:
    print("Nicht alle Teile konnten verplant werden.")
    print("Bitte entfernen Sie einige der angegebenen Teile!")

################################################################
##### Visualisierung des Ergbnisses als Matplotlib Tabelle #####

# Konvertierung der Dateneinträge zu Text
cellText = []

for row in results:
    formatRow = []
    for entry in row:
        if entry == 0: formatRow.append(' ')
        else: formatRow.append('{}'.format(entry))
    cellText.append(formatRow)

# Colormaps für die Tabellenbeschriftungen
rcolours = plt.cm.BuPu(np.full(numTimeSlots, 0.1))
ccolours = plt.cm.BuPu(np.full(len(machineList), 0.1))

# Plot als Tabelle
the_table = plt.table(cellText=cellText,
                      rowLabels=[ 'T{}:'.format(i) for i in range(numTimeSlots) ],
                      colLabels=machineList,
                      colColours=rcolours,
                      rowColours=rcolours,
                      rowLoc='right',
                      loc='center')

## Säuberung der Matplot Table
# Ausblenden der Achsenbeschriftungen
ax = plt.gca()
ax.get_xaxis().set_visible(False)
ax.get_yaxis().set_visible(False)

# Ausblenden der Box
plt.box(on=None)

# Vergrößern der Zellen
the_table.scale(1, 1.5)

# Speichern des Plots
plt.savefig('timetable.png',
            bbox_inches='tight',
            dpi=300
            )

plt.show()

