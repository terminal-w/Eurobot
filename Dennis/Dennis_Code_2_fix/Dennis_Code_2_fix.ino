#include <Wire.h>                                                        // Calls for I2C bus library

#define MD25ADDRESS         0x58                                         // Address of the MD25
#define SPEED               0x00                                         // Byte to send speed to both motors for forward and backwards motion if operated in MODE 2 or 3 and Motor 1 Speed if in MODE 0 or 1
#define TURN                0x01                                         // Byte to send speed for turn speed if operated in MODE 2 or 3 and Motor 2 Speed if in MODE 0 or 1
#define ENCODERONE          0x02                                         // Byte to read motor encoder 1
#define ENCODERTWO          0x06                                         // Byte to read motor encoder 2
#define ACCELERATION        0x0E                                         // Byte to define motor acceleration
#define CMD                 0x10                                         // Byte to reset encoder values
#define MODE_SELECTOR       0x0F                                         // Byte to change between control MODES

int Mode = 2;                                                            // MODE in which the MD25 will operate selector value 
float WheelBaseDiameter = 24.1;                                          // Distance between the two motor driven wheels in CM
float BotCircumference = (WheelBaseDiameter*3.14159);                    // Distance travelled in one full rotation of the bot
float Distance = 0;
float Distance1 = 0;                                                     // Sets wheel distance 1 to 0
float Distance2 = 0;                                                     // Sets wheel distance 2 to 0
int EncodeNumber = 0;
int encodervalue = 0;
int Speed = 0;
int index = 0;
int TurnAngle[] = {34.7, 69.4, 104.1, 138.8, 173.4, 208.1, 242.8, 277.5, 312.2, 346.9, 381.6, 416.3};
//  Turn Angle  =  15,   30,   45,    60,    75,    90,    105,   120,   135,   150,   165,   180
int EncodeSwitch[] = {ENCODERONE, ENCODERTWO};                           // Creates a list from which to call the encoders from
int DualSpeed = 0;                                                       // Sets a reference for the combined forward spped for the motors
int Acceleration = 3;                                                    // Sets a reference Acceleration for the motors
void Motion(int, float, char, int);                                      // Initiates the Forward funtion
void Transmit(int, int);
void DistCorrection(float, int);

void setup() {
  Wire.begin();                                                          // Begin I2C bus
  Serial.begin(9600);                                                    // Begin serial
  delay(100);                                                            // Wait for everything to power up  
  Transmit(MODE_SELECTOR, Mode);                                         // Set MD25 operation Mode to Mode 2
                                                                         // In this mode SPEED = combined motor speed, TURN = speed added and subtracted from the opposite motors to turn the robot
                                                                         // Mode 2 takes values from 0 to 127 as reverese, 128 as full stop, 129 to 255 as forward
  BotReset();                                                            //Resets the encoders and stops the motor
}

void Transmit(int Location, int Data){
  Wire.beginTransmission(MD25ADDRESS);                                   // Opens a transmission line
  Wire.write(Location);                                                  // Decides the location of the data
  Wire.write(Data);                                                      // The data to be sent to the location
  Wire.endTransmission();                                                // Closes the transmission line
  Serial.print("Transmission: ");
  Serial.print(Location, HEX); Serial.print(", "); Serial.println(Data, HEX);
}

float Encoder(int encodervalue){                                         // Function to read and display value of encoder 1/2 as a long //Input must be 1 or 2
  Wire.beginTransmission(MD25ADDRESS);                                   // Send byte to get a reading from encoder 1/2
  Wire.write(EncodeSwitch[encodervalue]);
  Wire.endTransmission();
  Wire.requestFrom(MD25ADDRESS, 4);                                      // Request 4 bytes from MD25
  while(Wire.available() < 4);                                           // Wait for 4 bytes to arrive
  long poss1 = Wire.read();                                              // First byte for encoder 1/2, HH.
  poss1 <<= 8;
  poss1 += Wire.read();                                                  // Second byte for encoder 1/2, HL
  poss1 <<= 8;
  poss1 += Wire.read();                                                  // Third byte for encoder 1/2, LH
  poss1 <<= 8;
  poss1  +=Wire.read();                                                  // Fourth byte for encoder 1/2, LLalue
  delay(5);                                                              // Wait for everything to make sure everything is sent
  return(poss1*0.09);                                                    // Convert encoder value to cm //potentially add *3.142
}

void BotReset(){
  EncodeReset();
  StopMotor();
}

void EncodeReset(){                                                      // This function resets the encoder values to 0
  Transmit(CMD, 0x20);
  delay(50);
}

void StopMotor(){                                                        // Function to stop motors  
  Transmit(ACCELERATION, 10);                                            // Sets the acceleration to register 10 (0.65s)
  Transmit(SPEED, 128);                                                  // Stops motors motor 1 if operated in MODE 0 or 1 and Stops both motors if operated in MODE 2 or 3
  Transmit(TURN, 128);                                                   // Stops motors motor 2 when operated in MODE 0 or 1 and Stops both motors while in turning sequence if operated in MODE 2 or 3
  delay(50);
  Encoder(1);                                                            // Calls a function that reads value of encoder 1
  Encoder(2);                                                            // Calls a function that reads value of encoder 2
  Serial.print("Encoder 1 Distance CM - ");                              // Displays last recorded traveled distance for encoder 1
  Serial.print(Encoder(1));
  Serial.print("   ");
  Serial.print("Encoder 2 Distance CM - ");                              // Displays last recorded traveled distance for encoder 1
  Serial.print(Encoder(2));
  Serial.println(" ");
}

void Motion(int Speed, float Distance, char Direction, int Turn){
  int e1; int e2; bool t = 0;
  if (Direction == 's'){                                                   //forward direction
    DualSpeed = Speed;                                                    // Sets the combined motor speed value
    e1 = Encoder(1);                                                           // Calls a function that reads value of encoder 1
    e2 = Encoder(2);                                                           // Calls a function that reads value of encoder 2
    Distance1 = Distance;
    Distance2 = Distance;
    t = 0;
    Serial.print(e1, DEC); Serial.print(", "); Serial.println(e2, DEC);
    Serial.print(Distance1, DEC); Serial.print(", "); Serial.println(Distance2, DEC);
      while (e2 <= Distance1 && e2 <= Distance2){
      Serial.println("While");
        e1 = Encoder(1);
        e2 = Encoder(2);
        if(!t){
            Serial.println("Transmisson");                                                                // If statement to check the status of the traveled distance    
            Transmit(ACCELERATION, Acceleration);                               // Sets the acceleration to register 1 (6.375s)
            Transmit(SPEED, DualSpeed); t = 1;
        }                                                                   // Sets a combined motor speed value                                                         
    }
    BotReset();
    return;                 //Resets the encoders and stops the motor
  }
  else if(Direction == 'b'){                                               //backwards direction
    DualSpeed = Speed;                                                    // Sets the combined motor speed value
    e1 = Encoder(1);                                                           // Calls a function that reads value of encoder 1
    e2 = Encoder(2);                                                           // Calls a function that reads value of encoder 2
    Distance1 = Distance;
    Distance2 = Distance;
    t=0;
      while (e1 >= -Distance1 && e2 >= -Distance2){          // If statement to check the status of the traveled distance    
       e1 = Encoder(1);
       e2 = Encoder(2);
       if(!t){                                                             // If statement to check the status of the traveled distance    
            Transmit(ACCELERATION, Acceleration);                               // Sets the acceleration to register 1 (6.375s)
            Transmit(SPEED, DualSpeed); t=1;
        }                                       // Sets a combined motor speed value                                                          
    }
    BotReset();
    return;                                   //Resets the encoders and stops the motor
  }
  else if(Direction == 'r'){                                               //turn right
    DualSpeed = Speed;                                                    // Sets the combined motor speed value
    e1 = Encoder(1);                                                           // Calls a function that reads value of encoder 1
    e2 = Encoder(2);                                                           // Calls a function that reads value of encoder 2
    Distance1 = TurnAngle[Turn];
    Distance2 = -TurnAngle[Turn];
    t=0;
      while (e1 <= Distance1 &&  e2 >= -Distance2){ 
       e1 = Encoder(1);
       e2 = Encoder(2);        
      if(!t){                                                                      // If statement to check the status of the traveled distance    
      Transmit(ACCELERATION, Acceleration);                               // Sets the acceleration to register 1 (6.375s)
      Transmit(TURN, DualSpeed); t=1;       }                                  // Sets a combined motor speed value                                                      
    }
    BotReset();
    return;                                                                 //Resets the encoders and stops the motor    
  }
  else(Direction == 'l');{                                                 //turn left
    DualSpeed = Speed;                                                    // Sets the combined motor speed value
    e1 = Encoder(1);                                                           // Calls a function that reads value of encoder 1
    e2 = Encoder(2);                                                           // Calls a function that reads value of encoder 2
    Distance1 = -TurnAngle[Turn];
    Distance2 = TurnAngle[Turn];
      t=0;
      while (e1 >= -Distance1 && e2 <= Distance2){          // If statement to check the status of the traveled distance    
        e1 = Encoder(1);
        e2 = Encoder(2);
        if(!t){
          Transmit(ACCELERATION, Acceleration);                              // Sets the acceleration to register 1 (6.375s)
          Transmit(TURN, DualSpeed); t=1;                                         // Sets a combined motor speed value
        }                                                      
    }
    BotReset();                                                          //Resets the encoders and stops the motor    
    return;
  }
}

void DistCorrection(float Dist, int EncodeNumber){
  for(int index; index < 4; index++){
    while(Dist != Encoder(EncodeNumber)){
      if(Dist <= Encoder(EncodeNumber)){
        Transmit(ACCELERATION, 3);                                       // Sets the acceleration to register 1 (6.375s)
        Transmit(SPEED, 130);                                            // Sets a combined motor speed value
      }
      else (Dist >= Encoder(EncodeNumber));{
        Transmit(ACCELERATION, 3);                                       // Sets the acceleration to register 1 (6.375s)
        Transmit(SPEED, 126);                                            // Sets a combined motor speed value    
      }
    }
  }
}

void loop(){
  Serial.println(millis());//Called function to get bot moving
  Serial.println(1);
  Motion(160, 1000, 's', 0);
  Serial.println(2);
  BotReset();
  Motion(160, 500, 'l', 5);
  Serial.println(3);
  BotReset();
  delay(500);
  
/*
  Transmit(SPEED,140);
  delay(500);
  BotReset();
  delay(500);
 */ 
}

