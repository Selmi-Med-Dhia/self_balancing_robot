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

void speedRight(int pwm){
  if (abs(pwm) < minPWMR ){
    analogWrite(motorRb, 0);
    analogWrite(motorRa, 0);
  }
  else if( pwm > 0){
    analogWrite(motorRb, 0);
    analogWrite(motorRa, pwm);
  }else{
    analogWrite(motorRa, 0);
    analogWrite(motorRb, abs(pwm));
  }
}

void speedLeft(int pwm){
  if (abs(pwm) < minPWML ){
    analogWrite(motorLb, 0);
    analogWrite(motorLa, 0);
  }
  else if( pwm > 0){
    analogWrite(motorLb, 0);
    analogWrite(motorLa, pwm);
  }else{
    analogWrite(motorLa, 0);
    analogWrite(motorLb, abs(pwm));
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
  for(int i=0; i<4; i++){
    pinMode(leds[i], OUTPUT);
    digitalWrite(leds[i], LOW);
  }
  speedCalculationCurrentTimeR = micros();
  speedCalculationPreviousTimeR = speedCalculationCurrentTimeR;
  speedCalculationCurrentTimeL = speedCalculationCurrentTimeR;
  speedCalculationPreviousTimeL = speedCalculationCurrentTimeR;
  currentSpeedL = 0;
  currentSpeedR = 0;
  directionR = 1;
  directionL = 1;
  attachInterrupt(digitalPinToInterrupt(encoderRA), encoderRISRA, CHANGE);
  attachInterrupt(digitalPinToInterrupt(encoderLA), encoderLISRA, CHANGE);
  //attachInterrupt(digitalPinToInterrupt(encoderRB), encoderRISRB, CHANGE);
  //attachInterrupt(digitalPinToInterrupt(encoderLB), encoderLISRB, CHANGE);
}