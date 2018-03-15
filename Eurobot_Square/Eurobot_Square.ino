
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

