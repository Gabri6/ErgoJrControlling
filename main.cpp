#include <iostream>
#include <chrono>
#include <thread>
#include "DynamixelHandler.h"

// Global variables
DynamixelHandler _oDxlHandler;
std::string _poppyDxlPortName = "/dev/ttyUSB0";
float _poppyDxlProtocol = 2.0;
int _poppyDxlBaudRate = 1000000;
int _nbJoints = 6;
float _minJointCmd = 0;
float _maxJointCmd = 1023;
float _minJointAngle = -180.0f;
float _maxJointAngle = 180.0f;

int main()
{ 
	std::cout << "===Initialization of the Dynamixel Motor communication====" << std::endl;
	_oDxlHandler.setDeviceName(_poppyDxlPortName);
	_oDxlHandler.setProtocolVersion(_poppyDxlProtocol);
	_oDxlHandler.openPort();
	_oDxlHandler.setBaudRate(_poppyDxlBaudRate);
	_oDxlHandler.enableTorque(true);
	std::cout << std::endl;
	
	// read current joint position
	std::vector<uint16_t> l_vCurrentJointPosition;
	_oDxlHandler.readCurrentJointPosition(l_vCurrentJointPosition);
	
	// display current joint position
	std::cout << "vCurrentJointPosition= (" << std::endl;
	for (int l_joint=0; l_joint < l_vCurrentJointPosition.size(); l_joint++)
		std::cout << l_vCurrentJointPosition[l_joint] << ", ";
	std::cout << ")" << std::endl;
	
	// wait 1s
	std::this_thread::sleep_for(std::chrono::milliseconds(1000));
	
	std::cout << "===Closing the Dynamixel Motor communication====" << std::endl;
	_oDxlHandler.enableTorque(false);
	_oDxlHandler.closePort();
	std::cout << std::endl;
	
	return 0;
}