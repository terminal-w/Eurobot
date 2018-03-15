#include <Wire.h>
#include <PID_v1.h>
#include <SoftwareSerial.h>
#include "Arduino.h"
#include <Servo.h>

/*
 * Title: Eurobot Code
 * Client: Atalantis Robotics Co.{
 * Team Lead:       Maria Sagno Navarra
 * 2nd in Command:  James Wilshaw
 *                - Rachel Gray
 *                - Cameron Ward
 *                - Fancisco
 *                }
 *       Apdapted from Odometry Task Code         
 * Author: James Wilshaw -- jrw1n15@soton.ac.uk
 * Date: 2018-02-27
 * =============================================
 * Objectives:
 * Navigate route and carry out tasks at waypoints using only encoder data as input.
 * 
 * CODE STRUCTURE
 * -Instruct: Instruction sent to the MD25 and receives information to decode it
 * -halt: Function to stop robot
 * -enc target: function that will translate the distance in mm to computer readable information
 * -Turn(theta): function that will tell the MD25 if the turn is sweep or spot.
 * -Sweep: Function that will describe an arc using theta
 * -Notify: indicates the robot has arrived at a point
 * -DriveTo: Manages the speed of the robot
 * -Target: intakes the information of the next trajectory and outputs information for enc target
 * -kmn: stops the loop
 * -
 */
 /*Definitions*/
 const long time_limit = 100000; //time limit in milliseconds
 union Encs{
  int indy[2];
  long both;
};
const double Kp = .01;
const double Ki = 5;
const double Kd = 7;
const byte wps = 4;
double Input0, Input1, Output0, Output1, SP0, SP1;
long t0;
PID Wheel0(&Input0, &Output0, &SP0, Kp, Ki, Kd, DIRECT);
PID Wheel1(&Input1, &Output1, &SP1, Kp, Ki, Kd, DIRECT);
#define debug 1  //switch for Software Serial
#define pi 3.1415926 //saves any errors typing
SoftwareSerial MD25(10, 11); //Software Serial MD25 RX, TX
#define _MD25
#if debug == 1 // NOT THE SERIAL SWITCH DON'T CHANGE
    #define DEBUG Serial
#endif
//Servo Carouselle;
const int track = 23500; //trackwidth of robot in mm x100
const int wheel_dia = 9450; //wheel diameter of robot in mm x100
const int wheel_base = 15000; //distance from axle to M&M dispenser in mm x100
//const byte sPos[6] = {20, 150, 114, 73, 40, 0}; //defines servo drive positions for M&Ms 

    
/* multi dimension array describing waypoints 
 *  execution order: distance and radius to waypoint -ve radius indicates acw
 *                   once waypoint achieved notify (and M&M if 1) then turn theta
 *                   -ve indicates acw before executing next waypoint. 
                                 wpID, distance, radius, theta, action
                                       (x10mm)   (x10mm) (x10deg) byte*/ 
const int waypoints[wps][5] ={
                                {0,   1000,     0,      900,     0},
                                {1,   1000,     0,      900,     0},
                                {2,   1000,     0,      900,     0},
                                {3,   1000,     0,      900,     0},
                                };
/*
 * serial control register lookup table
 */
enum registers:byte
  {
    getS1   = 0x21,
    getS2   = 0x22,
    getE1   = 0x23,
    getE2   = 0x24,
    getEs   = 0x25,
    getV    = 0x26,
    getI1   = 0x27,
    getI2   = 0x28,
    getVer  = 0x29,
    getAcc  = 0x2A,
    getMod  = 0x2B,
    getPow  = 0x2C,
    setS1   = 0x31,
    setS2   = 0x32,
    setAcc  = 0x33,
    setMod  = 0x34,
    reset   = 0x35,
    disReg  = 0x36,
    enReg   = 0x37,
    disTimO = 0x38,
    enTimO  = 0x39
  };
#define registers
void kmn(){bool a=0; while(!a){a=0;}} //function than never returns to provide stop
/*Main Code*/
void setup(){
  // put your setup code here, to run once:
  //Carouselle.attach(9)
  Wheel0.SetMode(MANUAL);
  Wheel1.SetMode(MANUAL);
  pinMode(13, OUTPUT);
  pinMode(4, INPUT);
    #if debug == 1
      MD25.begin(38400);
      DEBUG.begin(115200);  
    #else
      MD25.begin(38400);
    #endif
    instruct(setMod, 1); // sets motors with 0 being stop and each independent of the other.
    //Carouselle.write(sPos[0]);
    notify();
    bool go = 0;
    #if debug == 1
    DEBUG.print("Awaiting all clear @ ");
    DEBUG.println((int)millis(), DEC);
    #endif
    while(!go){
      go = digitalRead(4);
    }
    t0 = millis();
    #if debug == 1
      DEBUG.print("Setup Complete @ ");
      DEBUG.println(t0, DEC);
    #endif
}

void loop() {
  // put your main code here, to run repeatedly:
  //byte MandMstock = 5;
  int wp[5];
  for(int i = 0; i < wps; i++){ // for loop to work through waypoints
    for(int j=0; j<5; i++){
      wp[j] = waypoints[i][j];
    }
    #if debug == 1
    DEBUG.println(wp[0], DEC);
    #endif
    target(wp[1], wp[2]);
    turn(wp[3]);
    action(wp[4]);
  }
  kmn();
}
/*Functions*/
void timeup(){
  long te = millis() - t0;
  if(te > time_limit){kmn();}
}

