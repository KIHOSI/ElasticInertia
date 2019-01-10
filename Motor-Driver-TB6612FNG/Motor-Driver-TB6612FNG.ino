/*
Sample Code to run the Sparkfun TB6612FNG 1A Dual Motor Driver using Arduino UNO R3

This code conducts a few simple manoeuvres to illustrate the functions:
  - motorDrive(motorNumber, motorDirection, motorSpeed)
  - motorBrake(motorNumber)
  - motorStop(motorNumber)
  - motorsStandby

Connections:
- Pin 3 ---> PWMA
- Pin 8 ---> AIN2
- Pin 9 ---> AIN1
- Pin 10 ---> STBY
- Pin 11 ---> BIN1
- Pin 12 ---> BIN2
- Pin 5 ---> PWMB

- Motor 1: A01 and A02
- Motor 2: B01 and B02

*/

//Define the Pins

//Motor 1
int pinAIN1 = 9; //Direction 
int pinAIN2 = 8; //Direction 
int pinPWMA = 3; //Speed

//Motor 2
int pinBIN1 = 11; //Direction
int pinBIN2 = 12; //Direction
int pinPWMB = 5; //Speed

//Standby
int pinSTBY = 10;

//Constants to help remember the parameters
static boolean turnCW = 0;  //for motorDrive function
static boolean turnCCW = 1; //for motorDrive function
static boolean motor1 = 0;  //for motorDrive, motorStop, motorBrake functions
static boolean motor2 = 1;  //for motorDrive, motorStop, motorBrake functions


//M1 Rotrary encoder
int cpr = 12;
int gearratio = 150;

int M1_PinA = 6, M1_PinB = 7;   // pushbutton connected to digital pin 7
int M1_A = 0, M1_B = 0;     // variable to store the read value
long M1_recounter = 0;
int M1_reA, M1_reB, M1_rePA, M1_rePB;
int M1_mflag = 0;


//M2 Rotrary encoder
int M2_PinA = 2, M2_PinB = 4;   // pushbutton connected to digital pin 7
int M2_A = 0, M2_B = 0;     // variable to store the read value
long M2_recounter = 0;
int M2_reA, M2_reB, M2_rePA, M2_rePB;
int M2_mflag = 0;


void setup()
{
  Serial.begin(9600);
//Set the PIN Modes
  pinMode(pinPWMA, OUTPUT);
  pinMode(pinAIN1, OUTPUT);
  pinMode(pinAIN2, OUTPUT);

  pinMode(pinPWMB, OUTPUT);
  pinMode(pinBIN1, OUTPUT);
  pinMode(pinBIN2, OUTPUT);

  pinMode(pinSTBY, OUTPUT);
  
  pinMode(M1_PinA, INPUT);
  pinMode(M1_PinB, INPUT);
  M1_rePA = digitalRead(M1_PinA);
  M1_rePB = digitalRead(M1_PinB);

  pinMode(M2_PinA, INPUT);
  pinMode(M2_PinB, INPUT);
  M2_rePA = digitalRead(M2_PinA);
  M2_rePB = digitalRead(M2_PinB);
}

void loop()
{ 
  M1_rotary_encoder();
  M2_rotary_encoder();

  if (Serial.available() > 0){
    char inst;
    inst = Serial.read();
    Serial.print("Instruction: ");
    Serial.println(inst);
    
    if (inst == '1') { //M1,M2正轉半圈
      M1_rotate(0.5f);
      M2_rotate(0.5f);
      Serial.println("Rotating done");
    }
    else if(inst == '2'){ //M1,M2逆轉半圈
      M1_rotate(-0.5f);
      M2_rotate(-0.5f);
    }
    else if(inst == '3'){ //Ｍ1逆轉半圈,Ｍ2正轉半圈
       M1_rotate(-0.5f);
       M2_rotate(0.5f);
    }
    else if(inst == '4'){ //Ｍ1正轉半圈,Ｍ2逆轉半圈
       M1_rotate(0.5f);
       M2_rotate(-0.5f);
    }

    else if (inst == 'c') { //M1正轉半圈
      M1_rotate(0.5f);
    }
    else if (inst == 'd') { //M2正轉半圈
      M2_rotate(0.5f);
    }
    else if (inst == 'e') { //M1逆轉半圈
      M1_rotate(-0.5f);
    }
    else if (inst == 'f') { //M2逆轉半圈
      M2_rotate(-0.5f);
    }
    else if (inst == 'g') { //Ｍ1慢慢正轉
      motorDrive(motor1, turnCCW, 150);
    }
    else if (inst == 'h') { //Ｍ1慢慢逆轉
      motorDrive(motor1, turnCW, 150);
    }

    else if (inst == 'i') { //Ｍ2慢慢正轉
      motorDrive(motor2, turnCCW, 150);
    }
    else if (inst == 'j') { //Ｍ2慢慢逆轉
      motorDrive(motor2, turnCW, 150);
    }

    else if(inst == 'k'){ //M1轉緊到底
      M1_rotate_tight();
    }

    else if(inst == 'l'){ //M1放鬆到底
      M1_rotate_loose();
    }

    else if(inst == 'm'){ //M2轉緊到底
      M2_rotate_tight();
    }

    else if(inst == 'n'){ //M2放鬆到底
      M2_rotate_loose();
    }
    
    /*
    else if (inst == 'x') {
      ablock(0);
    }
    else if (inst == 'y') {
      ablock(1);
    }
    else if (inst == 'z') {
      ablock(2);
    }
    */
    else if (inst == 'a') {
      M1_rotate(4);
      Serial.println("Rotating done");
    }
    else if (inst == 'b') {
      M1_rotate(-4);
    }
    else if(inst == 'r'){
      M1_rotary_encoder();
      Serial.println(M1_recounter);
    }
   
  }
  
  //Drive both motors CW, full speed
  //motorDrive(motor1, turnCW, 255);
  //motorDrive(motor2, turnCW, 255);
 
  //Keep driving for 2 secs
  //delay(2000);

  //Turn towards motor1: Stop Motor1, slow Motor2
  //motorStop(motor1);
  //motorDrive(motor2, turnCW, 192);
 
  //Keep turning for 2 secs
  //delay(2000);

  //Turn in opposite direction: Stop Motor2, slow Motor1
  //motorDrive(motor1, turnCW, 192);
  //delay(250);
  //motorStop(motor2);

  //Keep turning for 2 secs
  //delay(2000);

  //Straighten up
  //motorDrive(motor2, turnCW, 192);
  //delay(500);
 
  //Put motors into Standby
  //motorsStandby();
  //delay(1000);
 
  //Do a tight turn towards motor1: Motor2 forward, Motor1 reverse
  //motorDrive(motor1, turnCCW, 192);
  //motorDrive(motor2, turnCW, 192);
 
  //Keep turning for 2 secs
  //delay(2000);
 

  //Apply Brakes, then into Standby
  //motorBrake(motor1);
  //motorBrake(motor2);
  //motorsStandby();

  //Stand still for 5 secs, then we do it all over again...
  //delay(5000);

}

void motorDrive(boolean motorNumber, boolean motorDirection, int motorSpeed)
{
  /*
  This Drives a specified motor, in a specific direction, at a specified speed:
    - motorNumber: motor1 or motor2 ---> Motor 1 or Motor 2
    - motorDirection: turnCW or turnCCW ---> clockwise or counter-clockwise
    - motorSpeed: 0 to 255 ---> 0 = stop / 255 = fast
  */

  boolean pinIn1;  //Relates to AIN1 or BIN1 (depending on the motor number specified)

 
//Specify the Direction to turn the motor
  //Clockwise: AIN1/BIN1 = HIGH and AIN2/BIN2 = LOW
  //Counter-Clockwise: AIN1/BIN1 = LOW and AIN2/BIN2 = HIGH
  if (motorDirection == turnCW){
    pinIn1 = HIGH;
  }
  else{
    pinIn1 = LOW;
  }

//Select the motor to turn, and set the direction and the speed
  if(motorNumber == motor1)
  {
    digitalWrite(pinAIN1, pinIn1);
    digitalWrite(pinAIN2, !pinIn1);  //This is the opposite of the AIN1
    analogWrite(pinPWMA, motorSpeed);
  }
  else
  {
    digitalWrite(pinBIN1, pinIn1);
    digitalWrite(pinBIN2, !pinIn1);  //This is the opposite of the BIN1
    analogWrite(pinPWMB, motorSpeed);
  }
   
 

//Finally , make sure STBY is disabled - pull it HIGH
  digitalWrite(pinSTBY, HIGH);

}

void motorBrake(boolean motorNumber)
{
/*
This "Short Brake"s the specified motor, by setting speed to zero
*/

  if (motorNumber == motor1){
    analogWrite(pinPWMA, 0);
  }
  else{
    analogWrite(pinPWMB, 0);
  }
}


void motorStop(boolean motorNumber)
{
  /*
  This stops the specified motor by setting both IN pins to LOW
  */
  if (motorNumber == motor1) {
    digitalWrite(pinAIN1, LOW);
    digitalWrite(pinAIN2, LOW);
  }
  else
  {
    digitalWrite(pinBIN1, LOW);
    digitalWrite(pinBIN2, LOW);
  } 
}


void motorsStandby()
{
  /*
  This puts the motors into Standby Mode
  */
  digitalWrite(pinSTBY, LOW);
}

void M_rotate() //rotate two motors in same time(same direction or different directions)
{
  


}

void M1_rotate_tight() //M2拉緊到
{
   M1_rotate(3.0f);
}

void M1_rotate_loose() //M2放鬆到中間
{
   M1_rotate(-3.0f);
}


void M2_rotate_tight() //M2拉緊到
{
   M2_rotate(3.0f);
}

void M2_rotate_loose() //M2放鬆到中間
{
   M2_rotate(-3.0f);
}

void M1_rotate(float loopnumber) {
  //int rotationspeed;//, loopnumber = 3;
  M1_rotary_encoder();
  long crec = M1_recounter;
  long trec = crec + (loopnumber * cpr * gearratio);
  if (loopnumber >= 0){
    M1_mflag = 1;
    motorDrive(motor1, turnCCW, 255);
    while (M1_recounter < trec) {
      M1_rotary_encoder();
    }
    motorBrake(motor1);
    M1_mflag = 0;
  }
  else{
    M1_mflag = -1;
    //loopnumber *= -1;
    motorDrive(motor1, turnCW, 255);
    while (M1_recounter > trec) {
      M1_rotary_encoder();
    }
    motorBrake(motor1);
    M1_mflag = 0;
  }

}




void M2_rotate(float loopnumber) {
  //int rotationspeed;//, loopnumber = 3;
  M2_rotary_encoder();
  long crec = M2_recounter;
  long trec = crec + (loopnumber * cpr * gearratio);
  if (loopnumber >= 0){
    M2_mflag = 1;
    motorDrive(motor2, turnCCW, 255);
    while (M2_recounter < trec) {
      M2_rotary_encoder();
    }
    motorBrake(motor2);
    M2_mflag = 0;
  }
  else{
    M2_mflag = -1;
    //loopnumber *= -1;
    motorDrive(motor2, turnCW, 255);
    while (M2_recounter > trec) {
      M2_rotary_encoder();
    }
    motorBrake(motor2);
    M2_mflag = 0;
  }
}


void M1_rotary_encoder() {
  M1_reA = digitalRead(M1_PinA); // Reads the "current" state of the outputA
  M1_reB = digitalRead(M1_PinB);
  
  // If the previous and the current state of the outputA are different, that means a Pulse has occured
  if (M1_reA != M1_rePA || M1_reB != M1_rePB) {

    if(M1_mflag == 0){
      // If the outputB state is different to the outputA state, that means the encoder is rotating clockwise
      if(M1_reA != M1_rePA && M1_reB != M1_rePB){ // rotate too fast, miss 1 change
        M1_recounter += 2;
      }
      else if (M1_reA == M1_rePB) {
        M1_recounter ++;
      } else {
        M1_recounter --;
      }
    }
    
    else if(M1_mflag == 1){
      // If the outputB state is different to the outputA state, that means the encoder is rotating clockwise
      if(M1_reA != M1_rePA && M1_reB != M1_rePB){ // rotate too fast, miss 1 change
        M1_recounter += 2;
      }
      else {
        M1_recounter ++;
      }
    }

    else if(M1_mflag == -1){
      // If the outputB state is different to the outputA state, that means the encoder is rotating clockwise
      if(M1_reA != M1_rePA && M1_reB != M1_rePB){ // rotate too fast, miss 1 change
        M1_recounter -= 2;
      }
      else {
        M1_recounter --;
      }
    }  
    
    //Serial.print("Position: ");
    //Serial.println(M1_recounter);
  }



  
  /*
  Serial.print("A: ");
  Serial.print(M1_reA);
  Serial.print(", ");
  Serial.print("B: ");
  Serial.println(M1_reB);
  */
  
  
  M1_rePA = M1_reA; // Updates the previous state of the outputA with the current state
  M1_rePB = M1_reB;

  /*
    A = digitalRead(PinA);   // read the input pin
    Serial.print("A: ");
    Serial.print(A);
    Serial.print(", ");
    B = digitalRead(PinB);   // read the input pin
    Serial.print("B: ");
    Serial.println(B);
  */

  //Serial.print("RE counter: ");
  //Serial.println(recounter);
}

void M2_rotary_encoder() {
  M2_reA = digitalRead(M2_PinA); // Reads the "current" state of the outputA
  M2_reB = digitalRead(M2_PinB);
  
  // If the previous and the current state of the outputA are different, that means a Pulse has occured
  if (M2_reA != M2_rePA || M2_reB != M2_rePB) {

    if(M2_mflag == 0){
      // If the outputB state is different to the outputA state, that means the encoder is rotating clockwise
      if(M2_reA != M2_rePA && M2_reB != M2_rePB){ // rotate too fast, miss 1 change
        M2_recounter += 2;
      }
      else if (M2_reA == M2_rePB) {
        M2_recounter ++;
      } else {
        M2_recounter --;
      }
    }
    
    else if(M2_mflag == 1){
      // If the outputB state is different to the outputA state, that means the encoder is rotating clockwise
      if(M2_reA != M2_rePA && M2_reB != M2_rePB){ // rotate too fast, miss 1 change
        M2_recounter += 2;
      }
      else {
        M2_recounter ++;
      }
    }

    else if(M2_mflag == -1){
      // If the outputB state is different to the outputA state, that means the encoder is rotating clockwise
      if(M2_reA != M2_rePA && M2_reB != M2_rePB){ // rotate too fast, miss 1 change
        M2_recounter -= 2;
      }
      else {
        M2_recounter --;
      }
    }  
    
    //Serial.print("Position: ");
    //Serial.println(recounter);
  }



  
  /*
  Serial.print("A: ");
  Serial.print(M2_reA);
  Serial.print(", ");
  Serial.print("B: ");
  Serial.println(M2_reB);
  */
  
  
  M2_rePA = M2_reA; // Updates the previous state of the outputA with the current state
  M2_rePB = M2_reB;

  /*
    A = digitalRead(PinA);   // read the input pin
    Serial.print("A: ");
    Serial.print(A);
    Serial.print(", ");
    B = digitalRead(PinB);   // read the input pin
    Serial.print("B: ");
    Serial.println(B);
  */

  //Serial.print("RE counter: ");
  //Serial.println(recounter);
}
