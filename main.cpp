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

int convertAnglesToJointCmd(float fJointAngle)
{ // y = ax + b
	float a = (_maxJointCmd-_minJointCmd) / (_maxJointAngle - _minJointAngle);
	float b = _minJointCmd - a * _minJointAngle;
	float jointCmd = a * fJointAngle + b;
	return (int)jointCmd;
}

void goToHomePosition()
{
	std::vector<uint16_t> l_vTargetJointPosition;
	for (int l_joint = 0; l_joint < _nbJoints; l_joint++)
		l_vTargetJointPosition.push_back(convertAnglesToJointCmd(0.0f));
		_oDxlHandler.sendTargetJointPosition(l_vTargetJointPosition);
}

int main()
{ 
	std::cout << "===Initialization of the Dynamixel Motor communication====" << std::endl;
	_oDxlHandler.setDeviceName(_poppyDxlPortName);
	_oDxlHandler.setProtocolVersion(_poppyDxlProtocol);
	_oDxlHandler.openPort();
	_oDxlHandler.setBaudRate(_poppyDxlBaudRate);
	_oDxlHandler.enableTorque(true);
	std::cout << std::endl;
	
	goToHomePosition();
	
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