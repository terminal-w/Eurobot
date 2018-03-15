long instruct(byte reg, char val = 0){
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
    r |= b[2]*16777216;
    r |= b[3]*65536;
    r |= b[6]*256;
    r |= b[7];
    Encs d;
    d.both = r;
    #if debug == 1
      DEBUG.println((long)r, HEX);
      DEBUG.print("Serial Buffer: ");
      for(byte i = 0; i<8; i++){
        DEBUG.print(b[i], HEX);
      }
      DEBUG.println();
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
      #endif
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
      #if debug == 1
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
      #endif
      return r; 
    }
    else{
      //gets
      MD25.flush();
      MD25.readBytes(b, 2);
      #if debug == 1
      DEBUG.print("Serial Buffer: ");
      for(byte i; i<5; i++){
        DEBUG.print(b[i], HEX);
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
  bool happy = 0; int E1cur; int E2cur; char S1; char S2; float E1diff; float E2diff; Encs d;
 #if debug ==1
  DEBUG.println("Etars:");
  DEBUG.print(E1tar, DEC);
  DEBUG.println(E2tar, DEC);
  #endif
  Wheel0.SetMode(AUTOMATIC); Wheel1.SetMode(AUTOMATIC);
  while (!happy) {
    byte baseline = 0; bool e = 0;
    d.both = instruct(getEs);
    E1cur = d.indy[0];
    E2cur = d.indy[1];
   Input0 = E1cur; Input1 = E2cur;
   Wheel0.Compute(); Wheel1.Compute();
   E1diff = E1tar-E1cur; E2diff = E2tar-E2cur;
#if debug == 1
  DEBUG.println("EDIFFS:");
  DEBUG.print(E1diff);
  DEBUG.println(E2diff);
   DEBUG.print(S1, DEC);
   DEBUG.println(S2, DEC);
   #endif
    if(abs(E1diff)<50||abs(E2diff)<50) {
      happy = 1;
      halt;
      break;
    }
    
    if(e){
    instruct(setS1, Output0);
    instruct(setS2, Output1);
    }
    else{
    instruct(setS1, Output0);
    instruct(setS2, Output1);
    }
    e = !e;
#if debug == 1
    DEBUG.println("Speed Adjustment: S1, S2");
    DEBUG.print(Output0, DEC);
    DEBUG.println(Output1, DEC);
#endif
  }
#if debug ==1
  DEBUG.println("Because I'm Happy");
#endif
  return;
}

