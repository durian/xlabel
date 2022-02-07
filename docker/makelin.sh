#!/bin/sh
#
RED=`tput setaf 1`
GRN=`tput setaf 2`
RST=`tput sgr0`

PLGBASE=xlabel
# Clean up
rm -rf ../src/build-lin/*

# New 
mkdir -p ../bin/${PLGBASE}

# Switch on debug, if it was off
#sed -i 's/^\/\/#define DBG 1$/#define DBG 1/' ../src/defs.h

DIR=lin_x64/
mkdir -p ../bin/${PLGBASE}/$DIR/

echo "../bin/${PLGBASE}/${DIR}${PLGBASE}"
if [ -f "../bin/${PLGBASE}/${DIR}${PLGBASE}" ]; then 
    # already here
    mv "../bin/${PLGBASE}/${DIR}${PLGBASE}" "../bin/${PLGBASE}/${DIR}${PLGBASE}_PREV"
fi

make lin

# Should go into Output/preferences/
cp ../src/xlabel.toml ../bin/xlabel/

# should check if exists here
if [ -f "../src/build-lin/${PLGBASE}.xpl" ]; then 
    cp ../src/build-lin/${PLGBASE}.xpl ../bin/${PLGBASE}/${DIR}/
    #
    echo "${GRN}"
    echo cp ../bin/${PLGBASE}/${DIR}${PLGBASE}.xpl ~/xplane11/Resources/plugins/${PLGBASE}/$DIR
    echo cp ../bin/${PLGBASE}/${DIR}${PLGBASE}.xpl /a/X-Plane11.50b/Resources/plugins/${PLGBASE}/$DIR
    echo cp ../bin/${PLGBASE}/${DIR}${PLGBASE}.xpl ~/xplane11n/Resources/plugins/${PLGBASE}/$DIR
    #
    echo cp -r ../bin/${PLGBASE} ~/xplane11b/Resources/plugins/
else
    echo "${RED}"
    echo "ERROR"
fi
echo "${RST}"

