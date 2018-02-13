/*
	md25.h
	Arduino Library for MD25 Motor Driver used in University of Southampton
	Odometry Task
	Author:	James Wilshaw - jrw1n15@soton.ac.uk
	Date:	2017-11-06
*/

#ifndef md25_h
#define md25_h
#include "Arduino.h"
#include <SoftwareSerial.h>

class md25
{
public:
	md25(Stream &serial);
	char getSpeed1();
	char getSpeed2();
	long getEncoder1();
	long getEncoder2();
	long getEncoders()[2];
	char getVolts();
	char getCurrent1();
	char getCurrent2();
	char getVers();
	char getAccel();
	char getMode();
	char getVI()[3];
	void setSpeed1(char speed);
	void setSpeed2(char speed);
	void setAccel(char mode);
	void setMode(char mode);
	void resetEnc_s();
	void disableReg();
	void enableReg();
	void disableTim();
	void enableTim();

private:
	Stream *_serial;
	
};


#endif // !md25_h
