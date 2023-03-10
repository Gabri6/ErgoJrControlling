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
int _targetJointTorqueGripper = 1140;
float _proportionnalIncrement = 0.025f;
float _proportionnalIncrementAbove = 0.01f;
float _threshold= 1.9;

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

void currentPos()
{
	// read current joint position
	std::vector<uint16_t> l_vCurrentJointPosition;
	_oDxlHandler.readCurrentJointPosition(l_vCurrentJointPosition);
	
	// display current joint position
	//std::cout << "vCurrentJointPosition= (";
	//for (int l_joint=0; l_joint < l_vCurrentJointPosition.size(); l_joint++)
	//	std::cout << l_vCurrentJointPosition[l_joint] << ", ";
	//std::cout << ")" << std::endl;
}

int gripperControl()
{
	//read current joint 6 torque
	std::vector<uint16_t> l_vCurrentJointTorque;
	_oDxlHandler.readCurrentJointTorque(l_vCurrentJointTorque);
	
	//read current joint 6 position
	std::vector<uint16_t> l_vCurrentJointPosition;
	_oDxlHandler.readCurrentJointPosition(l_vCurrentJointPosition);
	int joint6Position = l_vCurrentJointPosition[5];
	
	// display current joint 6 torque
	std::cout << "CurrentJoint6Torque= ";
	std::cout << l_vCurrentJointTorque[5] << std::endl;
	int joint6Torque = l_vCurrentJointTorque[5];
	
	int jointTorqueDiff = joint6Torque - _targetJointTorqueGripper;
	float angleIncrement = -jointTorqueDiff * _proportionnalIncrement;
	std::cout<< "increment= " << angleIncrement<< std::endl;
	float targetJoint6Position = joint6Position + angleIncrement;
	std::cout << "joint torque diff= " <<abs(jointTorqueDiff) << std::endl;
	
	while (abs(angleIncrement) > _threshold)
	{
		
		//read current joint 6 torque
		std::vector<uint16_t> l_vCurrentJointTorque;
		_oDxlHandler.readCurrentJointTorque(l_vCurrentJointTorque);
		
		//read current joint 6 position
		std::vector<uint16_t> l_vCurrentJointPosition;
		_oDxlHandler.readCurrentJointPosition(l_vCurrentJointPosition);
		int joint6Position = l_vCurrentJointPosition[5];
		
		// display current joint 6 torque
		std::cout << "vCurrentJoint6Torque= ";
		std::cout << l_vCurrentJointTorque[5] << std::endl;
		int joint6Torque = l_vCurrentJointTorque[5];
		
		int jointTorqueDiff = joint6Torque - _targetJointTorqueGripper;
		if (jointTorqueDiff < 0)
		{
			float angleIncrement = -jointTorqueDiff * _proportionnalIncrement;
			}
		else{
			float angleIncrement = 0;
		}
		std::cout<< "increment= " << angleIncrement<< std::endl;
		float targetJoint6Position = joint6Position + angleIncrement;
		std::cout << "joint torque diff= " <<jointTorqueDiff << std::endl;
		
		//set the position of each servomotor to 0
		std::vector<uint16_t> l_vTargetJointPosition;
		for (int l_joint = 0; l_joint < _nbJoints-1; l_joint++)
			l_vTargetJointPosition.push_back(convertAnglesToJointCmd(0.0f));
		//l_vTargetJointPosition.push_back(convertAnglesToJointCmd(90.0f));
		//l_vTargetJointPosition.push_back(convertAnglesToJointCmd(90.0f));
		//l_vTargetJointPosition.push_back(convertAnglesToJointCmd(-90.0f));
		//l_vTargetJointPosition.push_back(convertAnglesToJointCmd(-90.0f));
		//l_vTargetJointPosition.push_back(convertAnglesToJointCmd(45.0f));
		l_vTargetJointPosition.push_back(targetJoint6Position);
		_oDxlHandler.sendTargetJointPosition(l_vTargetJointPosition);
	}
		
	return (int)joint6Torque;
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
	
	currentPos();
		
	goToHomePosition();
		
	currentPos();
	
	int gripperTorque = gripperControl();
	
	goToHomePosition();
	
	// wait 1s
	std::this_thread::sleep_for(std::chrono::milliseconds(1000));
	
	std::cout << "===Closing the Dynamixel Motor communication====" << std::endl;
	_oDxlHandler.enableTorque(false);
	_oDxlHandler.closePort();
	std::cout << std::endl;
	
	return 0;
}