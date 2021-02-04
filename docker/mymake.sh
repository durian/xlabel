#!/bin/sh
#

RED=`tput setaf 1`
GRN=`tput setaf 2`
RST=`tput sgr0`

# Clean up
echo -e "\n\n\n"
echo "${GRN}Cleaning old build directories ${XPL}${RST}"
rm -rf ../src/build-mac/*
rm -rf ../src/build-lin/*
rm -rf ../src/build-win/*

PLGBASE=xlabel
mkdir -p ../bin/${PLGBASE}

PLG=${PLGBASE}.xpl

echo -e "\n\n\n"
echo "${RED}Mac ${XPL}${RST}"
echo -e "\n\n\n"
DIR="mac_x64/"
mkdir -p ../bin/${PLGBASE}/$DIR/
make mac
if [ -f "../src/build-mac/${PLG}" ]; then 
    cp ../src/build-mac/${PLG} ../bin/${PLGBASE}/${DIR}/${PLG}
else
    echo "${RED}"
    echo "ERROR"
    echo "${RST}"
    exit 1
fi

echo -e "\n\n\n"
echo "${RED}Win ${XPL}${RST}"
echo -e "\n\n\n"
DIR="win_x64/"
mkdir -p ../bin/${PLGBASE}/$DIR/
make win
if [ -f "../src/build-win/${PLG}" ]; then
    cp ../src/build-win/${PLG} ../bin/${PLGBASE}/${DIR}/${PLG}
else
    echo "${RED}"
    echo "ERROR"
    echo "${RST}"
    exit 1
fi


echo -e "\n\n\n"
echo "${RED}Lin ${XPL}${RST}"
echo -e "\n\n\n"
DIR="lin_x64/"
mkdir -p ../bin/${PLGBASE}/$DIR/
make lin
if [ -f "../src/build-lin/${PLG}" ]; then
    cp ../src/build-lin/${PLG} ../bin/${PLGBASE}/${DIR}/${PLG}
else
    echo "${RED}"
    echo "ERROR"
    echo "${RST}"
    exit 1
fi

#echo cp -r ../bin/64 /Volumes/Luna/X-Plane 11/Resources/plugins/xsmoke/
echo cp ../bin/${PLGBASE}/${DIR}${PLG} ~/xplane11/Resources/plugins/${PLGBASE}/$DIR
echo cp ../bin/${PLGBASE}/${DIR}${PLG} /a/X-Plane11.50b/Resources/plugins/${PLGBASE}/$DIR
echo cp ../bin/${PLGBASE}/${DIR}${PLG} ~/xplane11c/Resources/plugins/${PLGBASE}/$DIR
echo scp ../bin/${PLGBASE}/mac_x64/${PLG} pberck@192.168.1.32:xplane11/Resources/plugins/${PLGBASE}/mac_x64/
