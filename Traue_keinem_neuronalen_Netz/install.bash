# Dieses Skript installiert die nötigen Python module in der
# korrekten Version.
#
# This code is available under a GPL v3.0 license and comes without
# any explicit or implicit warranty.
#
# (C) Wilfried Wöber 2020 <wilfried.woeber@technikum-wien.at>
#/bin/bash
set -e #Abbruch falls ein Fehler auftritt
echo "Installing python virtual environment..."
#-------------------------------------#
#--- Checking for cuda 9.0 version ---#
#-------------------------------------#
if [ "$(nvcc --version | grep release | awk '{print $5}' | tr ',' ' ' | xargs)" != "9.0" ]
then 
    echo -e "\033[0;31m YOUR CUDA VERSION IS NOT CORRECT - PLEASE USE CUDA 9.0\033[0m"
fi
#--- Install stuff ---#
logfile_name="./$(date '+%Y%m%d_%H%M_Install.log')"
{ 
    virtualenv --no-site-packages ./VE
    #--- Install ---#
    ./VE/bin/pip install -r ./requirements.txt
} >> "$logfile_name"
