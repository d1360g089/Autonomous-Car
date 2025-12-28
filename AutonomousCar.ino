
#include <SPI.h>
#include <nRF24L01.h>
#include <RF24.h>
#include <Wire.h>
#include <Adafruit_Sensor.h>
#include <Adafruit_BNO055.h>
#include <utility/imumaths.h>
#include <math.h>

#define BNO055_SAMPLERATE_DELAY_MS 10

Adafruit_BNO055 myIMU = Adafruit_BNO055();



//RF24 radio(8, 10);

const byte address[6] = "00001";

struct JoystickPacket{
  int16_t x;
  int16_t y;

};

JoystickPacket data;



int motorSpeed1 = 150;
int motorSpeed2 = 150;


int motorSpeed;

// Right Wheels
int in5 = 12; //pin 9 -> A6
int in6 = A2;  // pin 3
int enc = 5;   // need to change later not pwm

int in1 = 13;  // pin 8 -> A7
int in2 = A0; // pin 9
int ena = 9;  // pin 10 -> pin 9 



//LEft Wheels
int in3 = 4;
int in4 = 7;
int enb = 6;


int in7 = A3;   // pin 11
int in8 = A1;
int end = 3; // pin 12 -> pin 3


//Joystick

//Ultrasonic sensor
int trigPin = 10;
int echoPin = 2;
int pingTravelTime;

float pingTravelDistance;
float distanceToTarget;




void forward();
void forwardRight();
void forwardsLeft();
void backwards();
void backwardsRight();
void backwardsLeft();
void turnRight(int speed);
void turnLeft(int speed);
void stopCar();
void setSpeed(int leftVal, int rightVal);
void scan();
float getYaw();
float readDistance();
float normalizeAngle(float a);
void rotateToRelativeAngle(float degrees, int speed = 220);

float t;



float roll;
float pitch;
float yaw;




void setup() {
  Serial.begin(115200);
  myIMU.begin();



  /*
  radio.begin();
  radio.openReadingPipe(0, address);
  radio.setPALevel(RF24_PA_MIN);
  radio.startListening();
*/
  




  pinMode(in3, OUTPUT);
  pinMode(in4, OUTPUT);
  pinMode(enb, OUTPUT);
  pinMode(in1, OUTPUT);
  pinMode(in2, OUTPUT);
  pinMode(ena, OUTPUT);

  
  pinMode(in5, OUTPUT);
  pinMode(in6, OUTPUT);
  pinMode(enc, OUTPUT);
  pinMode(in7, OUTPUT);
  pinMode(in8, OUTPUT);
  pinMode(end, OUTPUT);

  pinMode(trigPin, OUTPUT);
  pinMode(echoPin, INPUT);


}

void loop() {

  static bool scanning  = false;
  static float distances[36];
  static float yawAtMeasurement[36];
  static int scanIndex = 0;
  static float yawWhenScanStarted = 0;

  float currentDistance = readDistance();

  Serial.print("Dist: ");
  Serial.print(currentDistance);
  Serial.print("Yaw: ");
  Serial.println(getYaw(), 1);
  
  // Normal Forward Driving 
  if (!scanning && currentDistance > 17){
    setSpeed(240,240);
    // forwardRight() and forwardLeft() are already called

  }

  //Obstacle detected
  else if (!scanning && currentDistance <= 17) {
    stopCar();
    delay(100);

    scanning = true;
    scanIndex = 0;
    yawWhenScanStarted = getYaw();

    //Start rotating right
    turnRight(220);

    // we are in scanning mode

  }

  if (scanning){
    float currentYaw = getYaw();
    float rotatedSoFar = normalizeAngle(currentYaw - yawWhenScanStarted);

    if (rotatedSoFar >= (scanIndex*10.0f)){
      float d = readDistance();
      // filter bad readings
      if (d < 2 || d > 400) d = 0;

      distances[scanIndex] = d;
      yawAtMeasurement[scanIndex] = currentYaw;

      scanIndex++;

      //finished full scan
      if (scanIndex >= 36){
        stopCar();
        delay(30);

        //find index with max distance
        int bestIndex = 0;
        float maxDistance = distances[0];

        for (int i = 1; i < 36; i++){
          if (distances[i] > maxDistance){
            maxDistance = distances[i];
            bestIndex = i;

          }
        }

        float bestYaw = yawAtMeasurement[bestIndex];
        float yawNow = getYaw();

        float angleToTurn = normalizeAngle(bestYaw - yawNow);
        
        if (angleToTurn > 180) angleToTurn -= 360;

        rotateToRelativeAngle(angleToTurn, 220);

        scanning = false;






      }
    }
  }



    





  }
  






  //setSpeed(left, right);



// Code for Remote Control 
/*
  if (radio.available()){
    
    radio.read(&data, sizeof(data));

    int x = data.x;
    int y = data.y;

    x = map(x, -512, 512, -255, 255);
    y = map(y, -512, 512, -255, 255);

    int leftMotor = y + x;
    int rightMotor = y - x;

    leftMotor = constrain(leftMotor, -255, 255);
    rightMotor = constrain(rightMotor, -255, 255);

    setSpeed(leftMotor, rightMotor);

    Serial.print("L = ");
    Serial.println(leftMotor);
    Serial.print("R = ");
    Serial.println(rightMotor);


    
  }

*/









float normalizeAngle(float a){
  while(a < 0) a += 360;
  while (a >= 360) a -= 360;
  return a;

}


float readDistance(){
  digitalWrite(trigPin, LOW);
  delayMicroseconds(2);
  digitalWrite(trigPin, HIGH);
  delayMicroseconds(2);
  digitalWrite(trigPin, LOW);
    

  pingTravelTime = pulseIn(echoPin, HIGH);
  delay(20);

  pingTravelDistance = (pingTravelTime*765.*5280.*12.)/(3600.*1000000.);
  distanceToTarget = pingTravelDistance/2;

  return distanceToTarget;



}

float getYaw(){

  imu::Quaternion q = myIMU.getQuat();
  float qW = q.w();
  float qX = q.x();
  float qY = q.y();
  float qZ = q.z();

  yaw = atan2(2*(qW*qZ + qX*qY),1-2*((qY*qY) + (qZ*qZ)));

  yaw = yaw / (2 * 3.14159265) * 360;
  yaw = normalizeAngle(yaw);


  return yaw;


}



void rotateToRelativeAngle(float degrees, int speed = 220){
  if (abs(degrees) < 3.0) return;

  float startYaw = getYaw();

  if (degrees > 0){
    turnRight(speed);

  }else{
    turnLeft(speed);

  }

  while (true){
    float now = getYaw();
    float rotated = normalizeAngle(now - startYaw);
    if (rotated > 180) rotated -= 360;
    
    float remaining = degrees - rotated;

    if (abs(remaining) < 3.0) break;

    
  }

  stopCar();
  delay(15);

}



void scan(){
  

  const int N = 24;
  const float STEP = 360.0 / N;  // step of 10 degrees
  float distances[N];
  
  float yaw0 = getYaw();
  int idx = 0;


  turnRight(200);


  while (idx < N){

    float yawNow = getYaw();
    float delta = normalizeAngle(yawNow - yaw0);

    if (delta >= STEP){
      

      float d = readDistance();
      if (d < 2 || d > 120) d = 0;

      distances[idx] = d;
      idx++;

      yaw0 = yawNow;

      


    }
  }

  stopCar();

  //find best direction 
  int bestIdx = 0;
  float bestDist = 0;

  for (int i = 0; i < N; i++){
    if (distances[i] > bestDist){
      bestDist = distances[i];
      bestIdx = i;

    }
  }

  float targetAngle = bestIdx * STEP;

  rotateToRelativeAngle(targetAngle, yaw0);


}




void rotateRight(float degrees){

  t = (degrees/(103)) * 1000;
  turnRight(200);
  delay(t);
  stopCar();
  delay(50);

  


  
}


void setSpeed(int leftVal, int rightVal){
  // Right Side
  if (rightVal > 0) {
    forwardRight();

  }
  else if (rightVal < 0){
    backwardsRight();

  }
  analogWrite(ena, abs(rightVal));
  analogWrite(enc, abs(rightVal));

  //Left
  if (leftVal > 0){
    forwardLeft();
    
  }
  else if (leftVal < 0){
    backwardsLeft();

  }
  analogWrite(enb, abs(leftVal));
  analogWrite(end, abs(leftVal));




}

void forwardRight(){
  //Right Wheels 
  digitalWrite(in1, HIGH);
  digitalWrite(in2, LOW);

  digitalWrite(in5, HIGH);
  digitalWrite(in6, LOW);
  
  
}

void forwardLeft(){
  digitalWrite(in3, HIGH);
  digitalWrite(in4, LOW);

  digitalWrite(in7, HIGH);
  digitalWrite(in8, LOW);

}

void forward(){

  //Right Wheels 
  digitalWrite(in1, HIGH);
  digitalWrite(in2, LOW);
  analogWrite(ena, 240);

  digitalWrite(in5, HIGH);
  digitalWrite(in6, LOW);
  analogWrite(enc, 240);


  //Left Wheels 
  digitalWrite(in3, HIGH);
  digitalWrite(in4, LOW);
  analogWrite(enb, 240);

  
  digitalWrite(in7, HIGH);
  digitalWrite(in8, LOW);
  analogWrite(end, 240);
  
  
}


void backwards(){

  //Right Wheels 
  digitalWrite(in1, LOW);
  digitalWrite(in2, HIGH);
  analogWrite(ena, 240);

  digitalWrite(in5, LOW);
  digitalWrite(in6, HIGH);
  analogWrite(enc, 240);


  //Left Wheels 
  digitalWrite(in3, LOW);
  digitalWrite(in4, HIGH);
  analogWrite(enb, 240);

  
  digitalWrite(in7, LOW);
  digitalWrite(in8, HIGH);
  analogWrite(end, 240);

  
}

void backwardsRight(){
  digitalWrite(in1, LOW);
  digitalWrite(in2, HIGH);

  digitalWrite(in5, LOW);
  digitalWrite(in6, HIGH);
  
}

void backwardsLeft(){
  digitalWrite(in3, LOW);
  digitalWrite(in4, HIGH);
  

  
  digitalWrite(in7, LOW);
  digitalWrite(in8, HIGH);
  
}




void turnRight(int speed){

  //Right Wheels go backwards
  digitalWrite(in1, LOW);
  digitalWrite(in2, HIGH);
  analogWrite(ena, speed);

  digitalWrite(in5, LOW);
  digitalWrite(in6, HIGH);
  analogWrite(enc, speed);

  //Left wheels go forward
  digitalWrite(in3, HIGH);
  digitalWrite(in4, LOW);
  analogWrite(enb, speed);

  
  digitalWrite(in7, HIGH);
  digitalWrite(in8, LOW);
  analogWrite(end, speed);
  
  
}



void turnLeft(int speed){

  // RIght wheels go forward
  digitalWrite(in1, HIGH);
  digitalWrite(in2, LOW);
  analogWrite(ena, speed);

  digitalWrite(in5, HIGH);
  digitalWrite(in6, LOW);
  analogWrite(enc, speed);

  //Left wheels go backwards
  digitalWrite(in3, LOW);
  digitalWrite(in4, HIGH);
  analogWrite(enb, speed);

  
  digitalWrite(in7, LOW);
  digitalWrite(in8, HIGH);
  analogWrite(end, speed);


}


void stopCar(){
  analogWrite(ena, 0);
  analogWrite(enb,0);
  analogWrite(enc, 0);
  analogWrite(end, 0);


}

