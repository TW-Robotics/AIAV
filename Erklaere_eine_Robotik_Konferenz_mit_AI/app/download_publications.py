#!/bin/python

# Dieses Skript lädt automatisch alle Veröffintlichungen der Austrian Robotics Workshops von 2016-2020 herunter.
#
# This code is available under a GPL v3.0 license and comes without
# any explicit or implicit warranty.
#
# (C) Simon Schwaiger 2021 <schwaige@technikum-wien.at>

import os

# Anlegen des pdf Ordners und Wechsel in ihn
os.system('mkdir pdfs && cd pdfs')

# Download aller ARW Publikationen der jeweiligen Jahre
# Zuerst wird der jeweilige Ordner angelegt und in ihn navigiert
# Dannach laden wird die einzelnen pdf Dateien und verlassen den Ordner wieder

#### ARW 2016 ####
os.system('mkdir ARW2016 && cd ARW2016')

doi = '10.3217/978-3-85125-528-7'
for i in range(1, 32):
    cmd = 'wget doi.org/' + doi + '-{:02d}'.format(i)
    os.system(cmd)

os.system('cd ..')

#### ARW 2017 ####
os.system('mkdir ARW2017 && cd ARW2017')

doi = '10.3217/978-3-85125-524-9'
for i in range(1, 37):
    cmd = 'wget doi.org/' + doi + '-{:02d}'.format(i)
    os.system(cmd)

os.system('cd ..')

#### ARW 2018 ####
os.system('mkdir ARW2018 && cd ARW2018')

doi = '10.15203/3187-22-1'
for i in range(4, 15):
    cmd = 'wget doi.org/' + doi + '-{:02d}'.format(i)
    os.system(cmd)

os.system('cd ..')

#### ARW 2019 ####
os.system('mkdir ARW2019 && cd ARW2019')

doi = '10.3217/978-3-85125-663-5'
for i in range(1, 49):
    cmd = 'wget doi.org/' + doi + '-{:02d}'.format(i)
    os.system(cmd)

os.system('cd ..')

#### ARW 2020 ####
os.system('mkdir ARW2020 && cd ARW2020')

doi = '10.3217/978-3-85125-752-6'
for i in range(1, 39):
    cmd = 'wget doi.org/' + doi + '-{:02d}'.format(i)
    os.system(cmd)

os.system('cd ..')

# Wechsel zum Hauptverzeichnis
os.system('cd ..')

