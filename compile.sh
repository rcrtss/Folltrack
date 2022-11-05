g++ -fPIC -g -o Follow_V$1 main.cpp Source/ActionBumper.cpp Source/KinectContext.cpp Source/ActionFollow.cpp -IInclude/ -I/usr/include/ni -I/usr/local/Aria/include -L/usr/lib -L/usr/local/Aria/lib -m32 -lOpenNI -lAria -lpthread -ldl -lrt -std=gnu++0x -ftime-report

