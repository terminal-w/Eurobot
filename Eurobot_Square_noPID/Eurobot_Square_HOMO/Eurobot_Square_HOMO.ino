#include "Arduino.h"
#include <Wire.h>
#include <SoftwareSerial.h>
#include <Servo.h>

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

const byte wps = 2;
long t0;
float pLimit;
static float pDef = 5;

const byte fDist = 650;
const byte fSpeed = 4;
const byte cSpeed = 20;

#define debug 1  //switch for Software Serial
#define colour 1 //switch for team (1 is green, 0 is orange)

#define pi 3.1415926 //saves any errors typing
SoftwareSerial MD25(10, 11); //Software Serial MD25 RX, TX
#define _MD25

#if debug == 1 // NOT THE SERIAL SWITCH DON'T CHANGE
    #define DEBUG Serial
#endif
//Servo Carouselle;
const int track = 24050; //trackwidth of robot in mm x100
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
                                {0,   10000,     0,        0,     0, 1023},
                                {1,  -10000,     0,        0,     0, 1023},

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
//Function Prototypes
void kmn(){
  #if debug == 1
  DEBUG.println("KILL ME NOW!");
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
  pinMode(13, OUTPUT);
  pinMode(4, INPUT);
  pinMode(A0, INPUT);
  pinMode(A1, INPUT);
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
      delay(100);
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
  #if debug == 1
  DEBUG.println("LOOP");
  #endif
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
    DEBUG.println(wp[0], DEC);
    #endif
    if(wp[5] != 1023){pLimit = wp[5];}
    else{pLimit = pDef;}
    target(wp[1], wp[2]);
    #if debug == 1
    DEBUG.print("Turning: ");
    DEBUG.println(wp[3], DEC);
    #endif
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
  if(dir > 0){pin = A0;}
  else{pin = A1;}
  for(int i=0; i < arange; i++){
    int a = 0;
    while(a < 1){
      a = analogRead(pin)/(2 * 2.54);
      delay(5);
    }
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
  if(te > time_limit){kmn();}
  return;
}
long instruct(byte reg, char val){
  if(reg == getPow){
    #if debug == 1
    DEBUG.println("Sorry this function doesn't support that register");
    #endif
    return 0;
  }
  MD25.write((byte)0x00);
  MD25.write(reg);
  if(reg == 0x25){
    byte b[9];
    MD25.flush();
    MD25.readBytes(b, 9);
    long r = 0L;
    r |= b[6]*16777216;
    r |= b[7]*65536;
    r |= b[2]*256;
    r |= b[3];
    Encs d;
    d.both = r;/*
    #if debug == 1
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
    return r;
  }
  if(reg > 0x34){
    #if debug == 1
      DEBUG.print("Register: ");
      DEBUG.print(reg, HEX);
      DEBUG.println(" Accessed");
    #endif
    return 0;
  }
  if(reg < 0x30){byte b[5];
    if(reg <= 0x24 && reg >= 0x23){
      //encoders
      MD25.flush();
      MD25.readBytes(b, 5);
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
    else{
      //gets
      MD25.flush();
      MD25.readBytes(b, 2);
/*      #if debug == 1
      DEBUG.print("Serial Buffer: ");
      for(byte i; i<5; i++){
        DEBUG.print(b[i], HEX);
      }
      DEBUG.println();
      DEBUG.print("Recieved: ");
      DEBUG.print(reg, HEX);
      DEBUG.print(" with value:");
      DEBUG.println(b[1], DEC);
     #endif*/
      return b[1];
    }
  }
  else if(reg <= 0x34 && reg > 0x30){
    //sets
    MD25.write(val);
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
  if(no == 0){return;} //no action
  else if(no == 1){
    //Open Recuperator
    digitalWrite(5, 1);
    delay(500);
    digitalWrite(5, 0);
    return;
  }
  else if(no == 2){
    //Water Treatment
    digitalWrite(2, 1);
    delay(500);
    digitalWrite(2, 0);
    return;
  }
    else if(no == 3){
    //Water Tower
    digitalWrite(3, 1);
    delay(500);
    digitalWrite(3, 0);
    return;
  }
}
void halt(){
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
void notify(){
  halt();
  tone(13, 4000, 500);
  delay(500);
  #if debug == 1
    DEBUG.println("Waypoint Notification");
  #endif
  instruct(reset);
  return;
}
void DriveTo(int E1tar, int E2tar) {
  bool happy = 0; int E1cur; int E2cur; float E1diff; float E2diff; Encs d; int saf1; int saf2; bool fine = false; int adj1; int adj2;
  
  if(E1tar>0){saf1 = E1tar - enc_target(fDist);}
  else{saf1 = E1tar + enc_target(fDist);}
  if(E2tar>0){saf2 = E2tar - enc_target(fDist);}
  else{saf2 = E2tar + enc_target(fDist);}

  if(abs(saf1 + E1tar) < abs(E1tar)){saf1 = E1tar * 0.8;}
  if(abs(saf2 + E2tar) < abs(E2tar)){saf2 = E2tar * 0.8;}
 #if debug == 1
  DEBUG.println("SAFE: "); DEBUG.print(saf1, DEC); DEBUG.print(", "); DEBUG.println(saf2, DEC); 
  DEBUG.println("Etars:");
  DEBUG.print(E1tar, DEC); DEBUG.print(", ");
  DEBUG.println(E2tar, DEC);
  #endif
  bool f = false;
  while (!happy) {
    timeup();
    byte baseline = 0; bool e = 0;
    d.both = instruct(getEs);
    E1cur = d.indy[0];
    E2cur = d.indy[1];

   E1diff = E1tar-E1cur; E2diff = E2tar-E2cur;
   
#if debug == 1
  DEBUG.println("-----------");
  DEBUG.println("EDIFFS:");
  DEBUG.print(E1diff);
  DEBUG.print(", ");
  DEBUG.println(E2diff);
   #endif
   bool obs = prox((int)E1diff +(int)E2diff, pLimit); 
   if(obs){
    instruct(setS1, 0); instruct(setS2, 0);
   }
   char Output1; char Output2;
    if(abs(E1diff)<10 || abs(E2diff)<10) {
      happy = 1;
      notify();
      break;
    }
   else if(fine){
   if(abs(E1diff)<55 || abs(E2diff)<55){
    if(E1diff > 0){Output1 = 1;}
      else{Output1 = -1;}
      if(E2diff > 0){Output2 = 1;}
      else{Output2 = -1;}
   }
   else{
    if(E1diff > 0){Output1 = fSpeed;}
      else{Output1 = -fSpeed;}
      if(E2diff > 0){Output2 = fSpeed;}
      else{Output2 = -fSpeed;}
     }
   }
   else{  
    if(abs(saf1 - E1diff) < abs(E1tar)){
     if(E1diff > 0){Output1 = cSpeed;}
     else{Output1 = -cSpeed;}
      }
     else{
      fine = true;
     }
    if(abs(saf2 - E2diff) < abs(E2tar)){
     if(E2diff > 0){Output2 = cSpeed;}
     else{Output2 = -cSpeed;}
      }
     else{
      fine = true;
     }
   }
    if(E1tar == -E2tar){Output2 = -Output1;}
     
   if(!obs){
    if(e){
    instruct(setS1, Output1);
    instruct(setS2, Output2);
    }
    else{
    instruct(setS1, Output1);
    instruct(setS2, Output2);
    }
    e = !e;
#if debug == 1
    DEBUG.println("Speed Adjustment: S1, S2");
    DEBUG.print(Output1, DEC);
    DEBUG.println(Output2, DEC);
#endif
   }
   else{
#if debug == 1
    DEBUG.println("OBSTRUCTION!");
#endif
   }
  }
#if debug ==1
  DEBUG.println("Because I'm Happy");
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

