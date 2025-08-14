#include "HLHA.h"

/// Global variables
bool ledstates[] = {false, false, false, false};
volatile int encoderRCount = 0;
volatile int encoderLCount = 0;

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
  }else{
    encoderRCount++;
  }
}

void IRAM_ATTR encoderRISRB() {
  if(FAST_READ(encoderRB) == FAST_READ1(encoderRA)){
    encoderRCount++;
  }else{
    encoderRCount--;
  }
}

void IRAM_ATTR encoderLISRA() {
  if(FAST_READ(encoderLB) == FAST_READ(encoderLA)){
    encoderLCount++;
  }else{
    encoderLCount--;
  }
}

void IRAM_ATTR encoderLISRB() {
  if(FAST_READ(encoderLB) == FAST_READ(encoderLA)){
    encoderLCount--;
  }else{
    encoderLCount++;
  }
}

void HLHAsetup() {
  for(int i=0; i<4; i++){
    pinMode(leds[i], OUTPUT);
    digitalWrite(leds[i], LOW);
  }

  attachInterrupt(digitalPinToInterrupt(encoderRA), encoderRISRA, CHANGE);
  attachInterrupt(digitalPinToInterrupt(encoderLA), encoderLISRA, CHANGE);
  attachInterrupt(digitalPinToInterrupt(encoderRB), encoderRISRB, CHANGE);
  attachInterrupt(digitalPinToInterrupt(encoderLB), encoderLISRB, CHANGE);
}