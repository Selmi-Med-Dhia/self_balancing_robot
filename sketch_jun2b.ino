#include <Wire.h>
#include <MPU6050_light.h>
#include "esp_timer.h"

// motor pins
int motorLa = 21;
int motorLb = 19;
int motorRa = 23;
int motorRb = 22;

// encoder pins
int encoderRA = 33;
int encoderRB = 18;
int encoderLA = 5;
int encoderLB = 4;

// Leds' pins ordered clockwise
int leds[] = {32, 13, 14, 27};

//global variables
volatile int encoderRCount = 0;
volatile int encoderLCount = 0;
volatile int32_t currentSpeedR = 0;
volatile int32_t currentSpeedL = 0;
volatile int32_t speedCalculationPreviousTime = 0;
volatile int32_t speedCalculationCurrentTime = 0;
volatile int previousEncoderRCount = 0;
volatile int previousEncoderLCount = 0;
volatile int currentPWMR = 0;
volatile int currentPWML = 0;
volatile int32_t targetSpeedR = 0;
volatile int32_t targetSpeedL = 0;
volatile int32_t previousTargetSpeedR = 0;
volatile int32_t previousTargetSpeedL = 0;
volatile int32_t previousSpeedErrorR = 0;
volatile int32_t previousSpeedErrorL = 0;
volatile int32_t SpeedIntegralR = 0;
volatile int32_t SpeedIntegralL = 0;

const int CPR = 404;
const int minPWMR = 55;
const int minPWML = 57;
const int32_t kpS = 15;
const int32_t kdS = 5;
const int32_t kiS = 0;

const int32_t kpB = 500;
const int32_t kdB = 0;
const int32_t kiB = 0;

float previousGyroError = 0;
float gyroIntegral = 0;

float restGyroX = 0.17;
float targetGyroX = 0.17;

long mainLoopLastTime = 0;

esp_timer_handle_t speedTimer;
portMUX_TYPE speedMux = portMUX_INITIALIZER_UNLOCKED;

MPU6050 mpu(Wire);

void IRAM_ATTR encoderRISRA() {
  if(digitalRead(encoderRB) == digitalRead(encoderRA)){
    encoderRCount--;
  }else{
    encoderRCount++;
  }
}

void IRAM_ATTR encoderRISRB() {
  if(digitalRead(encoderRB) == digitalRead(encoderRA)){
    encoderRCount++;
  }else{
    encoderRCount--;
  }
}

void IRAM_ATTR encoderLISRA() {
  if(digitalRead(encoderLB) == digitalRead(encoderLA)){
    encoderLCount++;
  }else{
    encoderLCount--;
  }
}

void IRAM_ATTR encoderLISRB() {
  if(digitalRead(encoderLB) == digitalRead(encoderLA)){
    encoderLCount--;
  }else{
    encoderLCount++;
  }
}

void IRAM_ATTR calculateSpeed(void *args) {
  portENTER_CRITICAL_ISR(&speedMux);
  speedCalculationCurrentTime = micros();
  int32_t elapsedTime = speedCalculationCurrentTime - speedCalculationPreviousTime;

  if(elapsedTime != 0){
    currentSpeedR = ( (encoderRCount - previousEncoderRCount)*14851485 ) / elapsedTime; // unit = 0.01 RPM, every 100 = 1RPM
    currentSpeedL = ( (encoderLCount - previousEncoderLCount)*14851485 ) / elapsedTime;
    
    int32_t error = targetSpeedR - currentSpeedR;
    int32_t derivative = (error - previousSpeedErrorR)*100000 / elapsedTime;
    previousSpeedErrorR = error;
    SpeedIntegralR += error;
    currentPWMR = constrain(currentPWMR + ( kpS*error + kiS*SpeedIntegralR + kdS*derivative )/6000, -255, 255);
    speedRight(currentPWMR);

    error = targetSpeedL - currentSpeedL;
    derivative = (error - previousSpeedErrorL)*100000 / elapsedTime ;
    previousSpeedErrorL = error;
    SpeedIntegralL += error;
    currentPWML = constrain(currentPWML + ( kpS*error + kiS*SpeedIntegralL + kdS*derivative )/6000, -255, 255);
    speedLeft(currentPWML);

  }
  speedCalculationPreviousTime = speedCalculationCurrentTime;
  previousEncoderRCount = encoderRCount;
  previousEncoderLCount = encoderLCount;

  portEXIT_CRITICAL_ISR(&speedMux);
}

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

void setup() {
  Serial.begin(115200);

  for(int i=0; i<4; i++){
    pinMode(leds[i], OUTPUT);
    digitalWrite(leds[i], LOW);
  }
  
  attachInterrupt(digitalPinToInterrupt(encoderRA), encoderRISRA, CHANGE);
  attachInterrupt(digitalPinToInterrupt(encoderLA), encoderLISRA, CHANGE);
  attachInterrupt(digitalPinToInterrupt(encoderRB), encoderRISRB, CHANGE);
  attachInterrupt(digitalPinToInterrupt(encoderLB), encoderLISRB, CHANGE);
  
  Wire.begin(25, 26);

  byte status = mpu.begin();
  while (status != 0) {
    status = mpu.begin();
    delay(50);
  }
  
  const esp_timer_create_args_t timer_config = {
    .callback = &calculateSpeed,
    .arg = nullptr,
    .dispatch_method = ESP_TIMER_TASK,
    .name = "speedTimer"
  };
  
  esp_timer_create(&timer_config, &speedTimer);
  esp_timer_start_periodic(speedTimer, 5000);
}

void loop() {
  if( micros() - mainLoopLastTime >= 5000){
    mpu.update();
    float error = mpu.getAccX() - targetGyroX;
    float derivative = (error - previousGyroError)*1000 / (micros() - mainLoopLastTime) ;
    gyroIntegral += error;

    int32_t correction = (int32_t)( kpB*error + kiB*gyroIntegral + kdB*derivative );

    targetSpeedR = constrain( targetSpeedR + correction, -63000, 63000 );
    targetSpeedL = constrain( targetSpeedL + correction, -63000, 63000 );

    //Serial.println(targetSpeedR);
    mainLoopLastTime = micros();
  }
}