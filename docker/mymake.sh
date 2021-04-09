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
    echo -e "\n\n\n"
    echo "${GRN}" ../src/build-mac/${PLG} "${RST}"
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
    echo -e "\n\n\n"
    echo "${GRN}" ../src/build-win/${PLG} "${RST}"
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
    echo -e "\n\n\n"
    echo "${GRN}" ../src/build-lin/${PLG} "${RST}"
    cp ../src/build-lin/${PLG} ../bin/${PLGBASE}/${DIR}/${PLG}
else
    echo "${RED}"
    echo "ERROR"
    echo "${RST}"
    exit 1
fi

echo -e "\n\n\n"
find ../bin -name 'xlabel.xpl' -ls
echo cp ../bin/${PLGBASE}/${DIR}${PLG} ~/xplane11/Resources/plugins/${PLGBASE}/$DIR
echo cp ../bin/${PLGBASE}/${DIR}${PLG} ~/xplane11b/Resources/plugins/${PLGBASE}/$DIR
echo cp ../bin/${PLGBASE}/${DIR}${PLG} ~/xplane11c/Resources/plugins/${PLGBASE}/$DIR
echo scp ../bin/${PLGBASE}/mac_x64/${PLG} pberck@192.168.1.32:xplane11/Resources/plugins/${PLGBASE}/mac_x64/
echo cp -r ../bin/${PLGBASE} ~/xplane11b/Resources/plugins/
echo ""
