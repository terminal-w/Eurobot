/*
md25.cpp
Arduino Library for MD25 Motor Driver used in University of Southampton
Odometry Task
Author:	James Wilshaw - jrw1n15@soton.ac.uk
Date:	2017-11-06
*/
#include "Arduino.h"
#include "md25.h"

md25::md25(Stream &serial)
{
	_serial = &serial;
}

md25::getSpeed1() {
	_serial.write(0x00);
	_serial.write(getS1);
	char buf[2];
	_serial.readBytes(buf, 2);
	return buf[1];
}

md25::getSpeed2() {
	_serial.write(0x00);
	_serial.write(getS2);
	char buf[2];
	_serial.readBytes(buf, 2);
	return buf[1];
}

md25::disableReg() {
	_serial.write(0x00);
	_serial.write(disReg);
	return;
}

md25::disableTim() {
	_serial.write(0x00);
	_serial.write(disTimO);
	return;
}

md25::enableReg() {
	_serial.write(0x00);
	_serial.write(enReg);
	return;
}

md25::enableTim() {
	_serial.write(0x00);
	_serial.write(enTimO);
	return;
}
