// 
// 
// 

#include "OdometryTasks.h"
#include "Arduino.h"
#include <Servo.h>
#include <SoftwareSerial.h>



OdometryTasks::OdometryTasks(byte sPosa, int wheel_diaa, int wheel_basea, int tracka, Servo *Carousellea)
{
	_sPos = sPosa;
	_wheel_dia = wheel_diaa;
	_wheel_base = wheel_basea;
	_track = tracka;
	_Carouselle = *Carousellea;
}

int OdometryTasks::instruct(byte reg, char val = 0) {
	if (reg == getPow || reg == getEs) {
#if debug == 1
		DEBUG.println("Sorry this function doesn't support that register");
#endif
		return 0;
	}
	MD25.write((byte)0x00);
	MD25.write(reg);
#if debug == 1
	DEBUG.print("Register: ");
	DEBUG.print(reg, HEX);
	DEBUG.println(" Accessed");
#endif
	if (reg > 0x34) { return 0; }
	if (reg < 0x30) {
		byte b[5];
		if (reg <= 0x25 && reg >= 0x23) {
			//encoders
			MD25.flush();
			MD25.readBytes(b, 5);
			int r = b[2] << 8;         // (0x56 shifted 8 bits left, effectively * 256) 
			r += b[3];              // (0x32)
#if debug == 1
			DEBUG.print("Serial Buffer: ");
			for (byte i = 0; i<5; i++) {
				DEBUG.print(b[i], DEC);
				DEBUG.print(", ");
			}
			DEBUG.println();
			DEBUG.print("Recieved: ");
			DEBUG.print(reg, HEX);
			DEBUG.print(" with:");
			DEBUG.print(r, DEC);
			DEBUG.println(" degrees");
#endif
			return r;
		}
		else {
			//gets
			MD25.flush();
			MD25.readBytes(b, 2);
#if debug == 1
			DEBUG.print("Serial Buffer: ");
			for (byte i; i<5; i++) {
				DEBUG.print(b[i], DEC);
				DEBUG.print(", ");
			}
			DEBUG.println();
			DEBUG.print("Recieved: ");
			DEBUG.print(reg, HEX);
			DEBUG.print(" with value:");
			DEBUG.println(b[1], DEC);
#endif
			return b[1];
		}
	}
	else if (reg <= 0x34 && reg > 0x30) {
		//sets
		MD25.write(val);
		return 0;
	}
#if debug == 1
	DEBUG.println("FATAL ERROR: instruct");
#endif
	return 0;
}

void OdometryTasks::halt() {
	//function to stop robot.
	instruct(setAcc, 10);
	instruct(setS1);
	instruct(setS2);
	instruct(setAcc, 5);
#if debug == 1
	DEBUG.println("ACHTUNG!!! Ich habe gehaltet!");
#endif
	return;
}

int OdometryTasks::enc_target(int distance) {
	/* takes the required travel distance in mm x10 an converts it to an encoder target*/
	float den = pi*_wheel_dia;
	float frac = 3600 / den;
	int out = distance * frac;

#if debug == 1
	DEBUG.println(distance, DEC); DEBUG.println(den, DEC); DEBUG.println(frac, DEC);
	DEBUG.print("Encoder Target:");
	DEBUG.print(out, DEC);
	DEBUG.println(" degrees");
#endif
	return out;
}

void OdometryTasks::turn(int theta) {
	/* takes two arguments a target angle, theta (degrees x10), and a switch, spot,
	to determine whether the angle is to describe an arc or a spot turn.
	executes turn */
	float distance; //distance to be traveled per in mm
	distance = (theta*pi);
	distance /= 36000;
	distance *= _track;
#if debug == 1
	DEBUG.print("turn: ");
	DEBUG.println(distance);
#endif 
	int E2tar = enc_target((int)distance);
	int E1tar = enc_target(-(int)distance);

	DriveTo(E1tar, E2tar);
	return;
}

int OdometryTasks::sweep(int distance, int radius, bool in = 0) {
	/* code to allow robot to describe an arc
	returns the inner & outer arc lengths in mm x10*/
	float halftrack = _track*0.05;
	float rat = distance;
	rat /= radius;
	int Ri = radius - halftrack;
	int Ro = radius + halftrack;
	float Di = (rat)*Ri;
	float Do = (rat)*Ro;
#if debug == 1
	DEBUG.println(Ro);
	DEBUG.println(Ri);
	DEBUG.println(halftrack);
	DEBUG.println(rat);
	DEBUG.print("D(i) = ");
	DEBUG.print(Di, DEC);
	DEBUG.print(", D(o) = ");
	DEBUG.print(Do, DEC);
	if (in) { DEBUG.println(" Return: D(i)"); }
	else { DEBUG.println(" Return: D(o)"); }
#endif
	if (in) { return Di; }
	else { return Do; }
}

void OdometryTasks::notify() {
	halt();
	tone(13, 4000, 500);
	delay(500);
#if debug == 1
	DEBUG.println("Waypoint Notification");
#endif
	instruct(reset);
	return;
}

void OdometryTasks::DriveTo(int E1tar, int E2tar) {
	bool happy = 0; int E1cur; int E2cur; char S1; char S2; float E1diff; float E2diff;
#if debug ==1
	DEBUG.print(E1tar, DEC);
	DEBUG.println(E2tar, DEC);
#endif
	while (!happy) {
		float E1prog; float E2prog;
		E1cur = instruct(getE1);
		E2cur = instruct(getE2);
		E1diff = E1tar - E1cur;
		E2diff = E2tar - E2cur;
		E1prog = E1diff / E1tar;
		E2prog = E2diff / E2tar;
		S1 = 100 * E1prog;
		S2 = 100 * E2prog;
#if debug == 1
		DEBUG.print(S1, DEC);
		DEBUG.println(S2, DEC);
#endif
		if (E1cur == E1tar) {
			happy = 1;
			S1 = 0;
		}
		else if (E1tar - E1cur < 0) {
			S1 -= 27;
		}
		else {
			S1 += 27;
		}
		if (E2cur == E2tar) {
			happy = 1;
			S2 = 0;
		}
		else if (E2tar - E2cur < 0) {
			S2 -= 27;
		}
		else {
			S2 += 27;
		}
		instruct(setS1, S1);
		instruct(setS2, S2);
#if debug == 1
		DEBUG.println("Speed Adjustment: S1, S2");
		DEBUG.print(S1, DEC);
		DEBUG.println(S2, DEC);
#endif
	}
	instruct(setAcc, 10);
#if debug == 1
	DEBUG.println("GOES LOOPY");
#endif
	while (E1cur != E1tar || E2cur != E2tar) {
		S1 = 50 * (E1tar - E1cur) / 90;
		S2 = 50 * (E2tar - E2cur) / 90;
		instruct(setS1, S1);
		instruct(setS1, S2);
	}
	instruct(setAcc, 5);
#if debug ==1
	DEBUG.println("Because I'm Happy");
#endif
	return;
}

void OdometryTasks::target(int distance, int radius) {
#if debug == 1
	DEBUG.println("Targeting...");
#endif
	bool ccw = 0;
	int E1Tar;
	int E2Tar;
	if (radius == 0) {
		E1Tar = enc_target(distance);
		E2Tar = E1Tar;
#if debug == 1
		DEBUG.print("Straight Line, length: ");
		DEBUG.println(distance, DEC);
#endif
	}
	else {
		int Do = sweep(distance, abs(radius));
		int Di = sweep(distance, abs(radius), 1);
		int EoTar = enc_target(Do);
		int EiTar = enc_target(Di);
#if debug == 1
		DEBUG.print("Swept Radius, length, radius: ");
		DEBUG.print(distance, DEC);
		DEBUG.print(", ");
		DEBUG.print(radius, DEC);
#endif
		if (radius < 0) {
			E1Tar = EoTar;
			E2Tar = EiTar;
#if debug == 1
			DEBUG.println(" Anti-Clockwise");
#endif
		}
		else {
			E1Tar = EiTar;
			E2Tar = EoTar;
#if debug == 1
			DEBUG.println(" Clockwise");
#endif
		}
	}
	DriveTo(E1Tar, E2Tar);
	notify();
	return;
}

void OdometryTasks::MandMrelease(byte remaining) {
	/*This Function drops M&Ms
	* Author: Luka - lzd1u16@soton.ac.uk
	*/
	_Carouselle.write(_sPos[remaining]);
#if debug == 1
	DEBUG.print("M&M Deployed: ");
	DEBUG.println(remaining, DEC);
#endif
	return;
} //function prototype for releasing M&Ms

void OdometryTasks::kmn() { bool a = 0; } //function than never returns to provide stop



