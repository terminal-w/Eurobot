#include <SoftwareSerial.h>
#include "Arduino.h"

/*
 * Title: Odometry Task
 * Client: Aero 2 - Group 17/Q{
 * Team Lead:       James Wilshaw
 * 2nd in Command:  Luka Zabotto-Devitt
 *                - Ho (Phoebus) Wong
 *                - Wen (William) Yu
 *                - Aaron Adekoya
 *                }
 * Author: James Wilshaw -- jrw1n15@soton.ac.uk
 * Date: 2017-11-06
 * =============================================
 * Objectives:
 * Navigate route and carry out tasks at waypoints using only encoder data as input.
 * notify at all way points.
 * Dispense Chocolate at certain way points.
 * 
 */

#define debug  0 //switch for Software Serial

#if debug == 1
	  SoftwareSerial MD25(10, 11); //Software Serial MD25 RX, TX
	  #define DEBUG Serial
#else
	#define MD25 Serial
#endif
const int track = 21000; //trackwidth of robot in mm x100
const int wheel_dia = 10000; //wheel diameter of robot in mm x100

void halt(){
  //placeholder function to stop robot.
  return;
}

int turn(int theta, bool spot){
	  /* takes two arguments a target angle, theta (degrees x10), and a switch, spot,
		to determine whether the angle is to describe an arc or a spot turn.
		returns the distance (per driving wheel) that must be traveled. */
		float distance; //distance to be traveled per in mm
		distance = (theta/3600)*3.1415962*track;
		if(!spot){
			distance *= 2;
		}
		#if debug == 1
			DEBUG.print("turn: ");
			DEBUG.println(distance);
		#endif 
		return distance*10;
}

void notify(){
	halt;
	tone(13, 4000, 500);
	delay(500);
	#if debug == 1
		DEBUG.println("Waypoint Notification");
	#endif
	return;
}
 
void setup() {
  // put your setup code here, to run once:
	  #if debug == 1
			MD25.begin(9600);
			DEBUG.begin(9600);  
	  #else
			MD25.begin(38400);
	  #endif
}


void loop() {
  // put your main code here, to run repeatedly:

}
