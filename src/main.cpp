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
volatile int32_t speedCalculationPreviousTime = 0;
volatile int32_t speedCalculationCurrentTime = 0;
volatile int previousEncoderRCount = 0;
volatile int previousEncoderLCount = 0;
volatile int32_t currentSpeedR = 0;
volatile int32_t currentSpeedL = 0;
volatile int32_t currentPWMR = 0;
volatile int32_t currentPWML = 0;
volatile int32_t targetSpeedR = 0;
volatile int32_t targetSpeedL = 0;
volatile int32_t previousSpeedErrorR = 0;
volatile int32_t previousSpeedErrorL = 0;
volatile int32_t speedIntegralR = 0;
volatile int32_t speedIntegralL = 0;
volatile int32_t speedIntegralMax = 10000;
volatile static int32_t elapsedTime = 0;
volatile static int32_t error = 0;
volatile static int32_t derivative = 0;
const int32_t kpS = 1;
const int32_t kdS = 0;
const int32_t kiS = 0;

/// balancing PID controller parameters

void speedUpdateTask(void *pvParameters) {
  const TickType_t xDelay = 1;
  TickType_t xLastWakeTime = xTaskGetTickCount();
  for(;;){
    Serial.print(">s:");
    Serial.println(currentSpeedL);
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
}

void loop() {
  if (micros() - speedCalculationPreviousTime >= 1000) {
    speedCalculationCurrentTime = micros();
    elapsedTime = speedCalculationCurrentTime - speedCalculationPreviousTime;

    if(elapsedTime != 0){
      currentSpeedR = ( (encoderRCount - previousEncoderRCount)*14851485 ) / elapsedTime; // unit = 0.01 RPM, every 100 = 1RPM
      currentSpeedL = ( (encoderLCount - previousEncoderLCount)*14851485 ) / elapsedTime;
      
      error = targetSpeedR - currentSpeedR;
      derivative = (error - previousSpeedErrorR)*100000 / elapsedTime;
      previousSpeedErrorR = error;
      speedIntegralR = constrain(speedIntegralR + error, -speedIntegralMax, speedIntegralMax);
      currentPWMR = constrain(currentPWMR + ( kpS*error + kiS*speedIntegralR + kdS*derivative )/6000, -255, 255);
      //TODO: Optimize Writing PWM to the motor
      speedRight(currentPWMR);

      error = targetSpeedL - currentSpeedL;
      derivative = (error - previousSpeedErrorL)*100000 / elapsedTime;
      previousSpeedErrorL = error;
      speedIntegralL = constrain(speedIntegralL + error, -speedIntegralMax, speedIntegralMax);
      currentPWML = constrain(currentPWML + ( kpS*error + kiS*speedIntegralL + kdS*derivative )/6000, -255, 255);
      speedLeft(currentPWML);

    }
    speedCalculationPreviousTime = speedCalculationCurrentTime;
    previousEncoderRCount = encoderRCount;
    previousEncoderLCount = encoderLCount;
  }
}