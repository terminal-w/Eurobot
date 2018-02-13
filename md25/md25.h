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
	Stream &_serial;
	enum
	{
		getS1	= 0x21,
		getS2	= 0x22,
		getE1	= 0x23,
		getE2	= 0x24,
		getEs	= 0x25,
		getV	= 0x26,
		getI1	= 0x27,
		getI2	= 0x28,
		getVer	= 0x29,
		getAcc	= 0x2A,
		getMod	= 0x2B,
		getPow	= 0x2C,
		setS1	= 0x31,
		setS2	= 0x32,
		setAcc	= 0x33,
		setMod	= 0x34,
		reset	= 0x35,
		disReg	= 0x36,
		enReg	= 0x37,
		disTimO = 0x38,
		enTimO	= 0x39
	};
};


#endif // !md25_h
