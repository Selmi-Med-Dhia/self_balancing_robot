#include "HLHA.h"

/// Global variables
bool ledstates[] = {false, false, false, false};
volatile int encoderRCount = 0;
volatile int encoderLCount = 0;
volatile int32_t speedCalculationPreviousTimeR;
volatile int32_t speedCalculationCurrentTimeR;
volatile int32_t speedCalculationPreviousTimeL;
volatile int32_t speedCalculationCurrentTimeL;
volatile int8_t directionR;
volatile int8_t directionL;
volatile float currentSpeedR;
volatile float currentSpeedL;

MPU6050 mpu(Wire);

float getCurrentAngle(){
  mpu.update();
  return (-mpu.getAccAngleY());
}

void speedRight(int pwm){
  if (abs(pwm) < minPWMR ){
    ledcWrite(3, 0);
    ledcWrite(2, 0);
  }
  else if( pwm > 0){
    ledcWrite(3, 0);
    ledcWrite(2, pwm);
  }else{
    ledcWrite(2, 0);
    ledcWrite(3, abs(pwm));
  }
}

void speedLeft(int pwm){
  if (abs(pwm) < minPWML ){
    ledcWrite(1, 0);
    ledcWrite(0, 0);
  }
  else if( pwm > 0){
    ledcWrite(1, 0);
    ledcWrite(0, pwm);
  }else{
    ledcWrite(0, 0);
    ledcWrite(1, abs(pwm));
  }
}

void toggleLED(int index){
    digitalWrite(leds[index], !ledstates[index]);
    ledstates[index] = !ledstates[index];
}

void IRAM_ATTR encoderRISRA() {
  if(FAST_READ(encoderRB) == FAST_READ1(encoderRA)){
    encoderRCount--;
    directionR = -1;
  }else{
    encoderRCount++;
    directionR = 1;
  }
  speedCalculationPreviousTimeR = speedCalculationCurrentTimeR;
  speedCalculationCurrentTimeR = micros();
  currentSpeedR = (297029.703 * directionR) / (speedCalculationCurrentTimeR - speedCalculationPreviousTimeR);
}

void IRAM_ATTR encoderRISRB() {
  if(FAST_READ(encoderRB) == FAST_READ1(encoderRA)){
    encoderRCount++;
    directionR = 1;
  }else{
    encoderRCount--;
    directionR = -1;
  }
  speedCalculationPreviousTimeR = speedCalculationCurrentTimeR;
  speedCalculationCurrentTimeR = micros();
  if ( encoderRCount%3 == 0){
    currentSpeedR = (148514.85 * directionR) / (speedCalculationCurrentTimeR - speedCalculationPreviousTimeR);
  }
}

void IRAM_ATTR encoderLISRA() {
  if(FAST_READ(encoderLB) == FAST_READ(encoderLA)){
    encoderLCount++;
    directionL = 1;
  }else{
    encoderLCount--;
    directionL = -1;
  }
  speedCalculationPreviousTimeL = speedCalculationCurrentTimeL;
  speedCalculationCurrentTimeL = micros();
  currentSpeedL = (297029.703 * directionL) / (speedCalculationCurrentTimeL - speedCalculationPreviousTimeL);
}

void IRAM_ATTR encoderLISRB() {
  if(FAST_READ(encoderLB) == FAST_READ(encoderLA)){
    encoderLCount--;
    directionL = -3;
  }else{
    encoderLCount++;
    directionL = 3;
  }
  
  if ( encoderLCount%3 == 0){
    speedCalculationPreviousTimeL = speedCalculationCurrentTimeL;
    speedCalculationCurrentTimeL = micros();
    currentSpeedL = (148514.85 * directionL) / (speedCalculationCurrentTimeL - speedCalculationPreviousTimeL);
  }
}


void HLHAsetup() {
  // LEDs
  for(int i=0; i<4; i++){
    pinMode(leds[i], OUTPUT);
    digitalWrite(leds[i], LOW);
  }

  // PWM channels
  for(int i=0; i<4; i++){
    ledcSetup(i, 20000, 10);
  }
  ledcAttachPin(motorLa, 0);
  ledcAttachPin(motorLb, 1);
  ledcAttachPin(motorRa, 2);
  ledcAttachPin(motorRb, 3);

  // MPU
  Wire.begin(25, 26);

  byte status = mpu.begin();
  while (status != 0) {
    delay(20);
    status = mpu.begin();
  }

  // Global variables
  speedCalculationCurrentTimeR = micros();
  speedCalculationPreviousTimeR = speedCalculationCurrentTimeR;
  speedCalculationCurrentTimeL = speedCalculationCurrentTimeR;
  speedCalculationPreviousTimeL = speedCalculationCurrentTimeR;
  currentSpeedL = 0;
  currentSpeedR = 0;
  directionR = 1;
  directionL = 1;

  // Interrupts
  attachInterrupt(digitalPinToInterrupt(encoderRA), encoderRISRA, CHANGE);
  attachInterrupt(digitalPinToInterrupt(encoderLA), encoderLISRA, CHANGE);
  //attachInterrupt(digitalPinToInterrupt(encoderRB), encoderRISRB, CHANGE);
  //attachInterrupt(digitalPinToInterrupt(encoderLB), encoderLISRB, CHANGE);
}