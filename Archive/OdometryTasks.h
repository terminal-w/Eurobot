// OdometryTasks.h

#ifndef _ODOMETRYTASKS_h
#define _ODOMETRYTASKS_h


#include "Arduino.h"
#include <Servo.h>
#include <SoftwareSerial.h>

#ifndef registers :byte
	enum registers :byte
	{
		getS1 = 0x21,
		getS2 = 0x22,
		getE1 = 0x23,
		getE2 = 0x24,
		getEs = 0x25,
		getV = 0x26,
		getI1 = 0x27,
		getI2 = 0x28,
		getVer = 0x29,
		getAcc = 0x2A,
		getMod = 0x2B,
		getPow = 0x2C,
		setS1 = 0x31,
		setS2 = 0x32,
		setAcc = 0x33,
		setMod = 0x34,
		reset = 0x35,
		disReg = 0x36,
		enReg = 0x37,
		disTimO = 0x38,
		enTimO = 0x39
	};
  #define _registers
#endif // !registers

#ifndef _MD25
#if debug == 1
	SoftwareSerial MD25(10, 11);
#define DEBUG Serial;
#else
#define MD25 Serial
#endif
#define _MD25
#endif // !MD25
		
class OdometryTasks
{
 protected:


 public:
	 OdometryTasks(byte sPosa, int wheel_diaa, int wheel_basea, int tracka, Servo Carousellea);
	 int instruct(byte reg, char val);
	 void halt();
	 int enc_target(int distance);
	 void turn(int theta);
	 int sweep(int distance, int radius, bool in = 0);
	 void notify();
	 void DriveTo(int E1tar, int E2tar);
	 void target(int distance, int radius);
	 void MandMrelease(byte remaining);
	 void kmn();
private:
	const int _track;
	const int _wheel_dia;
	const int _wheel_base;
	const byte _sPos[6];
	Servo _Carouselle;
};

#endif

