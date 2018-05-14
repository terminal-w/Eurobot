#include "Arduino.h"
#include <Wire.h>
#include <PID_v1.h>

/*
 * Title: Eurobot Code
 * Client: Atalantis Robotics Co.{
 * Team Lead:       Maria Sagno Navarra
 * 2nd in Command:  James Wilshaw
 *                - Rachel Gray
 *                - Cameron Ward
 *                - Francisco
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

const double Kp = 0.25;
const double Ki = 0.0001;
const double Kd = 0.0004;
const byte  wps = 7;
double Input0, Input1, Output0, Output1, SP0, SP1;
long t0;
float pLimit;
static float pDef = 20;
static byte toinit = 5;
 
PID Wheel0(&Input0, &Output0, &SP0, Kp, Ki, Kd, DIRECT);
PID Wheel1(&Input1, &Output1, &SP1, Kp, Ki, Kd, DIRECT);

#define debug 1   //switch for Software Serial
#define colour 0 //switch for team (1 is green, 0 is orange)

#define pi 3.1415926 //saves any errors typing
#define MD25 Wire //I2C MD25
#define _MD25

#if debug == 1 // NOT THE SERIAL SWITCH DON'T CHANGE
    #define DEBUG Serial
#endif
//Servo Carouselle;
const int track = 23975; //trackwidth of robot in mm x100
const int wheel_dia = 7500; //wheel diameter of robot in mm x100
const int wheel_base = 15000; //distance from axle to M&M dispenser in mm x100
//const byte sPos[6] = {20, 150, 114, 73, 40, 0}; //defines servo drive positions for M&Ms


/* multi dimension array describing waypoints
 *  execution order: distance and radius to waypoint -ve radius indicates acw
 *                   once waypoint achieved notify (and M&M if 1) then turn theta
 *                   -ve indicates acw before executing next waypoint.
                                 wpID, distance, radius, theta, action, Proximity Range
                                       (x10mm)   (x10mm) (x10deg) byte  (cm)*/
const int waypoints[wps][6] ={
                                 {0,    10660,      0,    -900,    0,   1023},
                                 {1,      420,      0,       0,    0,      0},
                                 {2,     -600,      0,    -900,    0,      6},
                                 {3,     5960,      0,    -900,    0,   1023},
                                 {4,    11870,      0,     900,    0,   1023},
                                 {5,     2000,      0,    -900,    0,     10},
                                 {6,     5500,      0,       0,    4,      0}
                              };
/*
 * serial control register lookup table
 */
enum registers:byte
  {
    addr    = 0x58,
    setS1   = 0x00,
    setS2   = 0x01,
    getEs   = 0x02,
    getE1   = 0x04,
    getE2   = 0x08,
    setAcc  = 0x0E,
    setMod  = 0x0F,
    reset   = 0x20,
    cmd     = 0x10,
  };
#define registers
//Function Prototypes
void kmn(){
  #if debug == 1
  DEBUG.println(F("KILL ME NOW!"));
  #endif
  bool a=0; while(!a){a=0;}
  } //function than never returns to provide stop
void timeup();
long instruct(byte, char = 0);
void action(int);
void halt();
void notify();
void DriveTo(int, int);
int enc_target(int);
void turn(float);
int sweep(int, int, bool = 0);
void target(int, int);
/*Main Code*/
void setup(){
  // put your setup code here, to run once:
  //Carouselle.attach(9)
  Wheel0.SetMode(MANUAL);
  Wheel1.SetMode(MANUAL);
  Wheel0.SetOutputLimits(-80,80);
  Wheel1.SetOutputLimits(-80,80);
  pinMode(13, OUTPUT);
  pinMode(4, INPUT);
  pinMode(A0, INPUT);
  pinMode(A1, INPUT);
    #if debug == 1
      MD25.begin();
      DEBUG.begin(115200);
      DEBUG.print(F("Kp = ")); DEBUG.println(Kp, DEC);
      DEBUG.print(F("Ki = ")); DEBUG.println(Ki, DEC);
      DEBUG.print(F("Kd = ")); DEBUG.println(Kd, DEC);
    #else
      MD25.begin();
    #endif
    instruct(setMod, 1); // sets motors with 0 being stop and each independent of the other.
    instruct(setAcc, 3);
    instruct(reset);
  #if debug == 1
    Encs d;
    d.both = instruct(getEs);
    DEBUG.print(d.indy[0], HEX); DEBUG.print(F(", ")); DEBUG.println(d.indy[1], HEX);
  #endif
    //Carouselle.write(sPos[0]);
    notify();
    bool go = 0;
    #if debug == 1
    DEBUG.print(F("Awaiting all clear @ "));
    DEBUG.println((int)millis(), DEC);
    #endif
    while(!go){
      go = digitalRead(4);
    }
    t0 = millis();
    #if debug == 1
      DEBUG.print(F("Setup Complete @ "));
      DEBUG.println(t0, DEC);
    #endif
}

void loop() {
  // put your main code here, to run repeatedly:
  //byte MandMstock = 5;
  int wp[6];
  for(int i = 0; i < wps; i++){ // for loop to work through waypoints
    for(int j=0; j<6; j++){
     /* #if debug == 1
      DEBUG.print("j = ");
      DEBUG.println(j, DEC);
      #endif*/
      wp[j] = waypoints[i][j];
    }
    #if debug == 1
    DEBUG.println(F("Waypoint Notification:"));
    DEBUG.println(wp[0], DEC);
    #endif
    if(wp[5] != 1023){pLimit = wp[5];}
    else{pLimit = pDef;}
    target(wp[1], wp[2]);
    #if debug == 1
    DEBUG.print(F("Turning: "));
    DEBUG.println(wp[3], DEC);
    #endif
    if(wp[5] == 1023){pLimit=0;}
    #if colour == 1
      turn(-wp[3]);
    #else
      turn(wp[3]);
    #endif
    action(wp[4]);
  }
  kmn();
}
/*Functions*/
bool prox(int dir, float lim){
  const int arange = 15;
  int pin;
  float dist = 0;
  if(dir >= 0){pin = A0;}
  else{pin = A1;}
  for(int i=0; i < arange; i++){
    int a = 0;
      a = analogRead(pin)/2;
      a *= 2.54;
      if(a < 1){a = 2;}
      delay(5);
    dist += a;
    delay(5);
  }
  dist /= arange;
  #if debug == 1
  DEBUG.print("Ping: "); DEBUG.println(dist, DEC);
  #endif
  if(dist>lim){return 0;}
  else{return 1;}
}
void timeup(){
  long te = millis() - t0;
  if(te > time_limit){halt(); kmn();}
  return;
}
long instruct(byte reg, char val){
  MD25.beginTransmission(addr);
  if(reg != reset){MD25.write(reg);}
  else{
    MD25.write(cmd);
    MD25.write(reset);
    MD25.endTransmission();
    #if debug == 1
      DEBUG.print("Set Reg ");
      DEBUG.print(reg, HEX);
      DEBUG.print(" with val: ");
      DEBUG.println(val, DEC);
    #endif
    return 0;
    }
  if(reg == getEs){
    MD25.endTransmission();
    MD25.requestFrom(addr, 8);
    while(MD25.available() < 8);
    byte b[8];
    
    for(byte i=0; i<8; i++){b[i] = MD25.read();}
    
    long r = 0L;
    Encs d;
    
    r |= b[0]*16777216;
    r |= b[1]*65536;
    r |= b[2]*256;
    r |= b[3];
    
    d.indy[0] = (int)r;
    r = 0L;
    r |= b[4]*16777216;
    r |= b[5]*65536;
    r |= b[6]*256;
    r |= b[7];
    d.indy[1] = (int)r;
/*    
    #if debug == 1
      for(byte i=0; i<8; i++){if(b[i]<0x10){DEBUG.print(0, HEX);} DEBUG.print(b[i], HEX);} DEBUG.println();
      DEBUG.println("d.both:");
      DEBUG.println(d.both, HEX);
      DEBUG.print("Recieved: ");
      DEBUG.print(reg, HEX);
      DEBUG.print(" with E1:");
      DEBUG.print(d.indy[0], DEC);
      DEBUG.print(" & E2:");
      DEBUG.print(d.indy[1], DEC);
      DEBUG.println(" degrees");
      DEBUG.println((int)millis(), DEC);
      #endif*/
    return d.both;
  }
  else if(reg <= getE2 && reg >= getE1){
      //encoders
    MD25.endTransmission();
    MD25.requestFrom(addr, 4);
    while(MD25.available() < 4);
    byte b[4];
    for(byte i=0; i<4; i++){b[i] = MD25.read();}
      long r = 0;
      r |= b[0] << 24;
      r |= b[1] << 16;
      r |= b[2] << 8;         // (0x56 shifted 8 bits left, effectively * 256)
      r |= b[3];              // (0x32)
 /*     #if debug == 1
      DEBUG.print("Serial Buffer: ");
      for(byte i = 0; i<5; i++){
        DEBUG.print(b[i], HEX);
      }
      DEBUG.println();
      DEBUG.print("Recieved: ");
      DEBUG.print(reg, HEX);
      DEBUG.print(" with:");
      DEBUG.print(r, DEC);
      DEBUG.println(" degrees");
      DEBUG.println((int)millis(), DEC);
      #endif*/
      return r;
    }
  else if(reg <= setS2 || reg >= setAcc){
    //sets
    MD25.write(val);
    MD25.endTransmission();
    #if debug == 1
    DEBUG.print("Set Reg ");
    DEBUG.print(reg, HEX);
    DEBUG.print(" with val: ");
    DEBUG.println(val, DEC);
    #endif
    return 0;
  }
  #if debug == 1
  DEBUG.println("FATAL ERROR: instruct");
  #endif
  return 0;
}
void action(int no){
  #if debug == 1
    DEBUG.print(F("ACTION:")); DEBUG.println(no, DEC);
  #endif
  if(no == 0){return;} //no action
  else if(no == 1){
    //Open Recuperator
    digitalWrite(5, 1);
    digitalWrite(6, 1);
    delay(100);
    digitalWrite(6, 0);
    delay(50);
    digitalWrite(5, 0);
    return;
  }
  else if(no == 2){
    //Water Treatment
    digitalWrite(7, 1);
    delay(5000);
    digitalWrite(7, 0);
    return;
  }
    else if(no == 3){
    //Water Tower
    digitalWrite(8, 1);
    digitalWrite(7, 1);
    delay(5000);
    digitalWrite(7, 0);
    digitalWrite(8, 0);
    return;
  }
    else if(no == 4){
      //Bee Launch
    float theta = 900;
    float distance; //distance to be traveled per in mm
    distance = theta/36000;
    distance *= pi;
    distance *= track;
    int etar = enc_target(distance);
    #if debug == 1
    DEBUG.print(F("ETAR: ")); DEBUG.println(etar, DEC);
    #endif
    #if colour == 0
    DriveTo(0, -etar);
    #else
    DriveTo(-etar, 0);
    #endif
    }
}
void halt(){
  //function to stop robot.
  instruct(setS1);
  instruct(setS2);
  #if debug == 1
    DEBUG.println(F("ACHTUNG!!! Ich habe gehaltet!"));
  #endif
  return;
}
void notify(){
  halt();
  tone(13, 4000, 500);
  delay(50);
  instruct(reset);
  return;
}
void DriveTo(int E1tar, int E2tar) {
  bool happy = 0; int E1cur; int E2cur; char S1; char S2; float E1diff; float E2diff; Encs d;
 #if debug == 1
  DEBUG.println("Etars:");
  DEBUG.print(E1tar, DEC); DEBUG.print(", ");
  DEBUG.println(E2tar, DEC);
  #endif
  SP0 = E1tar; SP1 = E2tar;
  int toh = toinit;
  while(!happy) {
    timeup();
    byte baseline = 0; bool e = 0;
    d.both = instruct(getEs);
    E1cur = d.indy[0];
    E2cur = d.indy[1];
    Input0 = E1cur; Input1 = E2cur;
    E1diff = E1tar-E1cur; E2diff = E2tar-E2cur;
    bool obs = prox((int)E1diff+(int)E2diff, pLimit);
    SP0 = E1tar; SP1 = E2tar;
    Wheel0.SetMode(AUTOMATIC); Wheel1.SetMode(AUTOMATIC);
    if(obs){
      //Wheel0.SetMode(MANUAL); Wheel1.SetMode(MANUAL);
       instruct(setS1, 0); instruct(setS2, 0);
   }
#if debug == 1
  DEBUG.println(F("EDIFFS:"));
  DEBUG.print(E1diff);
  DEBUG.println(E2diff);
   DEBUG.print(S1, DEC);
   DEBUG.println(S2, DEC);
   #endif

    if(abs(E1diff)<7 && abs(E2diff)<7){
       toh--;
       if(toh==0){
        happy = 1;
        notify();
        break;
       }
    }
    else{
      toh = toinit;
    }
      
   if(!obs){
    Wheel0.Compute(); Wheel1.Compute();
    if(e){
    instruct(setS1, round(Output0));
    instruct(setS2, round(Output1));
    }
    else{
    instruct(setS2, round(Output1));
    instruct(setS1, round(Output0));
    }
    
    e = !e;
#if debug == 1
    DEBUG.println(F("Speed Adjustment: S1, S2"));
    DEBUG.print(Output0, DEC);
    DEBUG.println(Output1, DEC);
#endif
   }
   
   else{
#if debug == 1
    DEBUG.println(F("OBSTRUCTION!"));
#endif
   }
  }
#if debug ==1
  DEBUG.println(F("Because I'm Happy"));
#endif
  return;
}
int enc_target(int distance){

  /* takes the required travel distance in mm x10 an converts it to an encoder target*/
 float den = pi*wheel_dia;
 float frac = 3600/den;
 int out = distance * frac;
/*
 #if debug == 1
    DEBUG.println(distance, DEC); DEBUG.println(den, DEC); DEBUG.println(frac, DEC);
    DEBUG.print("Encoder Target:");
    DEBUG.print(out, DEC);
    DEBUG.println(" degrees");
 #endif*/
 return out;
}
void turn(float theta){
    /* takes two arguments a target angle, theta (degrees x10),
    executes turn */
    float distance; //distance to be traveled per in mm
    distance = theta/36000;
    distance *= pi;
    distance *= track;
    #if debug == 1
      DEBUG.print("turn: ");
      DEBUG.println(distance);
    #endif
    int E2tar = enc_target((int)distance);
    int E1tar = enc_target(-(int)distance);
    DriveTo(E1tar, E2tar);
    return;
}
int sweep(int distance, int radius, bool in){
  /* code to allow robot to describe an arc
     returns the inner & outer arc lengths in mm x10*/
  int Ri = radius - track/20;
  int Ro = radius + track/20;
  int Di = (distance/radius)*Ri;
  int Do = (distance/radius)*Ro;
  #if debug == 1
    DEBUG.print("D(i) = ");
    DEBUG.print(Di/10, DEC);
    DEBUG.print(", D(o) = ");
    DEBUG.print(Do/10, DEC);
    if(in){DEBUG.println(" Return: D(i)");}
    else{DEBUG.println(" Return: D(o)");}
  #endif
  if(in){return Di;}
  else{return Do;}
}
void target(int distance, int radius) {
#if debug == 1
  DEBUG.println("Targeting...");
#endif
  bool acw = 0;
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
  return;
}
