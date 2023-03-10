all:
	g++ DynamixelHandler.o main.o -L/usr/local/lib -ldxl_x64_cpp -lrt
main: main.cpp
	g++ -c -I./../libraries/toolkit-dynamixel/include main.cpp
	g++ -c -I./../libraries/toolkit-dynamixel/include ../libraries/toolkit-dynamixel/src/DynamixelHandler.cpp