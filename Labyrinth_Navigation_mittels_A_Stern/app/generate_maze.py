#!/bin/env/python3

# Dieses Skript generiert ein Labyrinth zur Verwendung im Gazebo Simulator.
#
# This code is available under a GPL v3.0 license and comes without
# any explicit or implicit warranty.
#
# (C) Simon Schwaiger 2021 <schwaige@technikum-wien.at>

from mazelib import Maze
from mazelib.generate.Prims import Prims
import numpy as np
import copy

def print_maze(maze):
    for r in maze:
        for c in r:
            print(c, end='')
        print(' ')

# Festlegen der Form des Labyrinths
width = 6                   # Anzahl der Spalten
height = 6                  # Anzahl der Zeilen
cell_size = 0.3             # Zellengröße in Metern
wall_height = 1.5           # Wandhöhe in Metern


# Generierung eines 10x10 Labyrinths mittels der Mazelib Bibliothek
# Der Eingang ist bei 0,1 und der Ausgang bei 10,9
m = Maze()
m.generator = Prims(width, height)
m.generate()
m.start = (0, 1)
m.end = (2*width, 2*height-1)

# Export des Labyriths als String, um diesen in ein Numpy Array zu konvertieren
# Ein Buffer von einer Zelle wird um das Labyrinth hinzugefügt
maze_arr = []
row = []

# Erste Reihe von leeren Zellen wird hinzugefügt (Breite = Breite Labyrinth + 2 Zellen Puffer)
maze_arr.append([' ' for i in range(m.tostring(True).index('\n') + 2)])

# Formatierung als Array
for c in m.tostring(True):
    # Füge den Puffer hinzu, wenn wir in einer neuen Zeile sind
    if row == []: row.append(' ')
    # Hinzufügen der Ziffer zur Zeile und der Zeile zum Array
    if c == '\n':
        row.append(' ')
        maze_arr.append(row)
        row = []
    elif c == 'S' or c == 'E' or c == '+':
        row.append(' ')
    else:
        row.append(c)

# Hinzufügen der letzten Zeile des Labyrinths
row.append(' ')
maze_arr.append(row)
# Hinzufügen der letzten Zeile des Puffers
maze_arr.append([' ' for i in range(len(maze_arr[0]))])

# Festlegen von Kerneln zur Generierung der Wandformen
kernel1 =   [[' ', ' ', ' '],
             [' ', ' ', ' '],
             [' ', ' ', ' ']]

kernel2 =   [[' ', ' ', ' '],
             [' ', '+', ' '],
             [' ', ' ', ' ']]

kernel_array = []
row = []

# Erstellen eines Arrays der Form 3*Labyrinth mittels des Kernels
# Damit werden schönere Wände erzeugt
for r in range(1, len(maze_arr)-1, 1):
    for c in range(1, len(maze_arr[0])-1, 1):
        # Hinzufügen des Kernels basierend auf der form der umliegenden Wände
        if maze_arr[r][c] == ' ': row.append(kernel1)
        elif maze_arr[r][c] == '#':
            # Kopieren von Kernel 2 und setzen der Nachbarwände
            tmp_kernel = copy.deepcopy(kernel2)
            if maze_arr[r-1][c] == '#': tmp_kernel[0][1] = '|'
            if maze_arr[r+1][c] == '#': tmp_kernel[2][1] = '|'
            if maze_arr[r][c-1] == '#': tmp_kernel[1][0] = '-'
            if maze_arr[r][c+1] == '#': tmp_kernel[1][2] = '-'
            row.append(tmp_kernel)
    # Neuanordnung der Kernel zur einfacheren Iteration
    tmp_row0 = []
    tmp_row1 = []
    tmp_row2 = []
    for i in row:
        tmp_row0.append(i[0])
        tmp_row1.append(i[1])
        tmp_row2.append(i[2])
    kernel_array.append([tmp_row0])
    kernel_array.append([tmp_row1])
    kernel_array.append([tmp_row2])
    row = []

# Setzen der richtigen Größe des Labyrinths (3*len(maze_arr[0]) x 3*len(maze_arr))
num_rows = (len(maze_arr)-2)*3
num_cols = (len(maze_arr[0])-2)*3
kernel_array = np.reshape(kernel_array, (num_rows, num_rows))

print("Labyrinth generiert. Formattierung der Gazebo Geometrie")
print(print_maze(kernel_array))


class sdfCreator():
    """ Hilfsklasse zur generierung einer sdf Datei zur Nutzung in Gazebo """
    def __init__(self, x=0, y=0):
        # Offnen der sdf Datei, die generiert werden soll
        self.file = open("mymaze/model.sdf", "w")
        # Speicherung der Anzahl der Modelle die bereits in der Datei sind
        self.model_count = 0
        self.link_count = 0
        # Hinzufügen der statischen Elemente der sdf Datei
        # Alle Elemente werden in ein einziges Model als child links hinzugefügt
        # Dazu erstellen wir collision und visual Eigenschaften
        self.file.write('<sdf version="1.6">\n')
        self.file.write('<model name="Maze">\n')
        self.model_count += 1
        self.file.write('<pose>{} {} 0 0 0 0 0</pose>\n'.format(x, y))
    def add_box(self, pos_x, pos_y, pos_z, len_x, len_y, len_z):
        """ Adds a box of len x,y,z at position x,y,z to the generated sdf file """
        # Hinzufügen des links an einer relativen Position zum Modells
        self.file.write('<link name="link_{}">\n'.format(self.link_count))
        self.file.write('<pose>{} {} {} 0 0 0 0</pose>\n'.format(pos_x, pos_y, pos_z))
        # Hinzufügen der collision Eigenschaft
        self.file.write('<collision name="collision_{}">\n'.format(self.link_count))
        self.file.write('<geometry>\n')
        self.file.write('<box>\n')
        self.file.write('<size>{} {} {}</size>\n'.format(len_x, len_y, len_z))
        self.file.write('</box>\n')
        self.file.write('</geometry>\n')
        self.file.write('</collision>\n')
        # Hinzufügen der visual Eigenschaft (gleich wie collision)
        self.file.write('<visual name="visual_{}">\n'.format(self.link_count))
        self.file.write('<geometry>\n')
        self.file.write('<box>\n')
        self.file.write('<size>{} {} {}</size>\n'.format(len_x, len_y, len_z))
        self.file.write('</box>\n')
        self.file.write('</geometry>\n')
        self.file.write('</visual>\n')
        # Abschluss des links und Inkrementierung des Zählers
        self.file.write('</link>\n')
        self.link_count += 1
    def close(self):
        # Schließen der offenen Gruppen und Speicherung der Datei
        self.file.write('</model>\n')
        self.file.write('</sdf>\n')
        self.file.close()

# Instanzierung der sdfCreator Klasse
creator = sdfCreator(-4*cell_size, -4*cell_size)

# Hinzufügen aller Zellen des Labyrinths
for r in range(int(num_rows)):
    for c in range(int(num_cols)):
        # Ermittlung der Position der aktuellen Zelle
        current_x = r*cell_size
        current_y = c*cell_size
        # Erstellen der Wand an der Position
        if kernel_array[r][c] != ' ': creator.add_box(current_x, current_y, 0, cell_size, cell_size, wall_height)

creator.close()

print("Labyrinth SDF erfolgreich generiert.")

