all: dynamixel main
	g++ DynamixelHandler.o main.o -L/usr/local/lib -ldxl_x64_cpp -lrt
	
main: main.cpp
	g++ -c -I./../libraries/toolkit-dynamixel/include main.cpp
	
dynamixel: ../libraries/toolkit-dynamixel/src/DynamixelHandler.cpp
	g++ -c -I./../libraries/toolkit-dynamixel/include ../libraries/toolkit-dynamixel/src/DynamixelHandler.cpp

clean:
	rm *.o
	rm *.out