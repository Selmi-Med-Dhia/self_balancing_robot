/**
 * @file main.cpp
 * @brief Main source file for the self-balancing robot platform.
 *
 * This file defines tasks, and sets up PID controllers for speed
 * and balancing control on the ESP-based self-balancing robot.
 *
 * Author: Selmi Med Dhia
 * Date: 14-8-2024
 */

#include "HLHA.h"
/// tasks
TaskHandle_t speedUpdateTaskHandle;

/// speed PID controller parameters
volatile int32_t previousEncoderRCount = 0;
volatile int32_t previousEncoderLCount = 0;
volatile int32_t currentPWMR = 0;
volatile int32_t currentPWML = 0;
volatile float targetSpeedR = 0;
volatile float targetSpeedL = 0;
volatile float previousSpeedErrorR = 0;
volatile float previousSpeedErrorL = 0;
volatile float speedIntegralR = 0;
volatile float speedIntegralL = 0;
volatile int32_t speedIntegralMax = 10000;
volatile int32_t elapsedTime = 0;
volatile float error = 0;
volatile float derivative = 0;
const float kpS = 1;
const float kdS = 0;
const float kiS = 0;

/// balancing PID controller parameters

void speedUpdateTask(void *pvParameters) {
  const TickType_t xDelay = 1;
  TickType_t xLastWakeTime = xTaskGetTickCount();
  for(;;){
    // zeroing speed if no ticks were recently triggered
    if ( ( (micros() - speedCalculationCurrentTimeR) > 19000 ) && (targetSpeedR == 0) ) {
      currentSpeedR = 0;
      currentPWMR = 0;
    }
    if ( ( (micros() - speedCalculationCurrentTimeL) > 19000 ) && (targetSpeedL == 0) ) {
      currentSpeedL = 0;
      currentPWML = 0;
    }
    
    Serial.write((byte*)&currentSpeedR, sizeof(currentSpeedR));
    vTaskDelayUntil(&xLastWakeTime, xDelay);
  }
}

void setup() {
  Serial.begin(921600);
  HLHAsetup();

  xTaskCreatePinnedToCore(
    speedUpdateTask, 
    "SpeedUpdateTask", 
    2048, 
    NULL, 
    3, 
    &speedUpdateTaskHandle,
    1
  );
  speedRight(100);
}

void loop() {
  /*
  if (micros() - speedCalculationPreviousTime >= 1000) {
    speedCalculationCurrentTime = micros();
    elapsedTime = speedCalculationCurrentTime - speedCalculationPreviousTime;

    if(elapsedTime != 0){
      currentSpeedR = ( (encoderRCount - previousEncoderRCount)*148514.85 ) / elapsedTime; // unit = RPM
      currentSpeedL = ( (encoderLCount - previousEncoderLCount)*148514.85 ) / elapsedTime;
      
      error = targetSpeedR - currentSpeedR;
      derivative = (error - previousSpeedErrorR) / elapsedTime;
      previousSpeedErrorR = error;
      speedIntegralR = constrain(speedIntegralR + error, -speedIntegralMax, speedIntegralMax);
      currentPWMR = (int)constrain(currentPWMR + ( kpS*error + kiS*speedIntegralR + kdS*derivative ), -255, 255);
      //TODO: Optimize Writing PWM to the motor
      //speedRight(currentPWMR);

      error = targetSpeedL - currentSpeedL;
      derivative = (error - previousSpeedErrorL) / elapsedTime;
      previousSpeedErrorL = error;
      speedIntegralL = constrain(speedIntegralL + error, -speedIntegralMax, speedIntegralMax);
      currentPWML = (int)constrain(currentPWML + ( kpS*error + kiS*speedIntegralL + kdS*derivative ), -255, 255);
      //speedLeft(currentPWML);

    }
    speedCalculationPreviousTime = speedCalculationCurrentTime;
    previousEncoderRCount = encoderRCount;
    previousEncoderLCount = encoderLCount;
  }
  */
}