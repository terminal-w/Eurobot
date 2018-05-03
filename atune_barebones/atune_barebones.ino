#include "Arduino.h"
#include <Wire.h>
#include <PID_v1.h>
#include <PID_AutoTune_v0.h>

byte ATuneModeRemember=2;
double input=80, output=50, setpoint=180;

double kpmodel=1.5, taup=100, theta[50];
double outputStart=5;
double aTuneStep=50, aTuneNoise=1, aTuneStartValue=100;
unsigned int aTuneLookBack=20;

boolean tuning = false;
unsigned long serialTime;


//set to false to connect to the real world
boolean useSimulation = false;
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
 const unsigned long time_limit = 100000; //time limit in milliseconds
 union Encs{
  int indy[2];
  long both;
};

double Kp = 0.3;
double Ki = 0.05;
double Kd = 0.1;
const byte wps = 8;
double Input0, Input1, Output0, Output1, SP0, SP1;
long t0;
float pLimit;
static float pDef = 15;
 
PID Wheel0(&Input0, &Output0, &SP0, Kp, Ki, Kd, DIRECT);
PID Wheel1(&Input1, &Output1, &SP1, Kp, Ki, Kd, DIRECT);
PID_ATune aTune(&Input0, &Output0);
#define debug 1   //switch for Software Serial
#define colour 0 //switch for team (1 is green, 0 is orange)

#define pi 3.1415926 //saves any errors typing
#define MD25 Wire //I2C MD25
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
                                {0,   11270,     0,     -900,     0, 1023},
                                {1,     100,     0,        0,     0,    0},
                                {11,   -100,     0,     1800,     0, 1023},
                                {2,    1170,     0,      900,     0, 1023},
                                {3,    4500,     0,     -900,     0, 1023},
                                {4,   13000,     0,      900,     0, 1023},
                                {5,    4000,     0,     -900,     0, 1023},
                                {6,    3500,     0,     -900,     0,    0}
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
void changeAutoTune();
void AutoTuneHelper(bool);
void SerialSend();
void SerialReceive();
/*
void kmn(){
  #if debug == 1
  DEBUG.println("KILL ME NOW!");
  #endif
  bool a=0; while(!a){a=0;}
  } //function than never returns to provide stop
void timeup();*/
long instruct(byte, char = 0);
/*
void action(int);
void halt();
void notify();
void DriveTo(int, int);
int enc_target(int);
void turn(float);
int sweep(int, int, bool = 0);
void target(int, int);
*/
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
      serialTime = 0;
      DEBUG.begin(115200);
      DEBUG.print(F("Kp = ")); DEBUG.println(Kp, DEC);
      DEBUG.print(F("Ki = ")); DEBUG.println(Ki, DEC);
      DEBUG.print(F("Kd = ")); DEBUG.println(Kd, DEC);
    #else
      MD25.begin();
    #endif
    Wheel0.SetMode(AUTOMATIC);
    if(tuning)
  {
    tuning=false;
    changeAutoTune();
    tuning=true;
  }
  SP0 = 180;
    instruct(setMod, 1); // sets motors with 0 being stop and each independent of the other.
    instruct(setAcc, 3);
    instruct(reset);
  #if debug == 1
    Encs d;
    d.both = instruct(getEs);
    DEBUG.print(d.indy[0], HEX); DEBUG.print(F(", ")); DEBUG.println(d.indy[1], HEX);
  #endif
    //Carouselle.write(sPos[0]);
    //notify();
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
  Encs d;
  unsigned long now = millis();
  d.both = instruct(getEs);
  Input0 = d.indy[0];
  if(tuning)
  {
    byte val = (aTune.Runtime());
    if (val!=0)
    {
      tuning = false;
    }
    if(!tuning)
    { //we're done, set the tuning parameters
      Kp = aTune.GetKp();
      Ki = aTune.GetKi();
      Kd = aTune.GetKd();
      Wheel0.SetTunings(Kp,Ki,Kd);
      AutoTuneHelper(false);
      instruct(reset);
    }
  }
  else Wheel0.Compute();

     instruct(setS1, Output0);
     instruct(setS2, -Output0);
  
  //send-receive with processing if it's time
  if(millis()>serialTime)
  {
    SerialReceive();
    SerialSend();
    serialTime+=500;
  }
}
/*Functions*/
/*
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
  if(te > time_limit){kmn();}
  return;
}*/
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
/*
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
  delay(500);
  #if debug == 1
    DEBUG.println(F("Waypoint Notification"));
  #endif
  instruct(reset);
  return;
}
void DriveTo(int E1tar, int E2tar) {
  bool happy = 0; int E1cur; int E2cur; char S1; char S2; float E1diff; float E2diff; Encs d;
 #if debug ==1
  DEBUG.println("Etars:");
  DEBUG.print(E1tar, DEC); DEBUG.print(", ");
  DEBUG.println(E2tar, DEC);
  #endif
  SP0 = E1tar; SP1 = E2tar;
  Wheel0.SetMode(AUTOMATIC); Wheel1.SetMode(AUTOMATIC);
  while (!happy) {
    timeup();
    byte baseline = 0; bool e = 0;
    d.both = instruct(getEs);
    E1cur = d.indy[0];
    E2cur = d.indy[1];
   Input0 = E1cur; Input1 = E2cur;
   Wheel0.Compute(); Wheel1.Compute();
   E1diff = E1tar-E1cur; E2diff = E2tar-E2cur;
   bool obs = prox((int)E1diff+(int)E2diff, pLimit);
   if(obs){
    Wheel0.SetMode(MANUAL); Wheel1.SetMode(MANUAL);
    instruct(setS1, 0); instruct(setS2, 0);
   }
#if debug == 1
  DEBUG.println(F("EDIFFS:"));
  DEBUG.print(E1diff);
  DEBUG.println(E2diff);
   DEBUG.print(S1, DEC);
   DEBUG.println(S2, DEC);
   #endif

    if(abs(E1diff)<2 && abs(E2diff)<2){
        happy = 1;
        notify();
        break;
    }
      
   if(!obs){ 
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
}*/
/*
int enc_target(int distance){

  /* takes the required travel distance in mm x10 an converts it to an encoder target*/
/* float den = pi*wheel_dia;
 float frac = 3600/den;
 int out = distance * frac;
/*
 #if debug == 1
    DEBUG.println(distance, DEC); DEBUG.println(den, DEC); DEBUG.println(frac, DEC);
    DEBUG.print("Encoder Target:");
    DEBUG.print(out, DEC);
    DEBUG.println(" degrees");
 #endif
 return out;
}
void turn(float theta){
    /* takes two arguments a target angle, theta (degrees x10),
    executes turn 
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
     returns the inner & outer arc lengths in mm x10
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
*/
void changeAutoTune()
{
 if(!tuning)
  {
    //Set the output to the desired starting frequency.
    output=aTuneStartValue;
    aTune.SetNoiseBand(aTuneNoise);
    aTune.SetOutputStep(aTuneStep);
    aTune.SetLookbackSec((int)aTuneLookBack);
    AutoTuneHelper(true);
    tuning = true;
  }
  else
  { //cancel autotune
    aTune.Cancel();
    tuning = false;
    AutoTuneHelper(false);
  }
}

void AutoTuneHelper(boolean start)
{
  if(start)
    ATuneModeRemember = Wheel0.GetMode();
  else
    Wheel0.SetMode(ATuneModeRemember);
}

void SerialSend()
{
  DEBUG.print(F("setpoint: "));DEBUG.print(SP0); DEBUG.print(F(" "));
  DEBUG.print(F("input: "));DEBUG.print(Input0); DEBUG.print(F(" "));
  DEBUG.print(F("output: "));DEBUG.print(Output0); DEBUG.print(F(" "));
  if(tuning){
    DEBUG.println(F("tuning mode"));
  } else {
    DEBUG.print(F("kp: "));DEBUG.print(Wheel0.GetKp());DEBUG.print(F(" "));
    DEBUG.print(F("ki: "));DEBUG.print(Wheel0.GetKi());DEBUG.print(F(" "));
    DEBUG.print(F("kd: "));DEBUG.print(Wheel0.GetKd());DEBUG.println();
  }
}

void SerialReceive()
{
  if(DEBUG.available())
  {
   char b = DEBUG.read(); 
   DEBUG.flush(); 
   if((b=='1' && !tuning) || (b!='1' && tuning))changeAutoTune();
  }
}

