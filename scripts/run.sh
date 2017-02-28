#!/bin/bash
PARAM1=$1
PARAM2=$2
PARAM3=$3
# With default values-----------------------------------------
#: ${PARAM1:="/Users/vitovalov/Kinect/oniRecordings/aligned.oni"}
#: ${PARAM2:="/Users/vitovalov/Desktop/prision/21.png"}
#: ${PARAM3:="/Users/vitovalov/Desktop/prision/22.png"}
#cd build/ && make && cp main.app/Contents/MacOS/main . && ./main $PARAM2 $PARAM3
#-------------------------------------------------------------
cd build/ && make && cp main.app/Contents/MacOS/main . && ./main $PARAM1 $PARAM2
