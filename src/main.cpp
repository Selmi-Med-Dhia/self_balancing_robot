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
TaskHandle_t AngleUpdateTaskHandle;
TaskHandle_t LEDUpdateTaskHandle;

/// speed PID controller parameters
volatile int32_t previousEncoderRCount = 0;
volatile int32_t previousEncoderLCount = 0;
volatile int32_t mainLoopPreviousTime = 0;
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
const float kpS = 0.04;
const float kdS = 1200;
const float kiS = 0.00015;

/// balancing PID controller parameters
volatile float currentAngle = 0;
const float neutralAngle = 8.83;

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
    
    //Serial.write((byte*)&currentAngle, sizeof(currentAngle));
    vTaskDelayUntil(&xLastWakeTime, xDelay);
  }
}

void angleUpdateTask(void *pvParameters){
  const TickType_t xDelay = 2;
  TickType_t xLastWakeTime = xTaskGetTickCount();
  for(;;){
    currentAngle = getCurrentAngle() - neutralAngle;
    vTaskDelayUntil(&xLastWakeTime, xDelay);
  }
}

void LEDUpdateTask(void *pvParameters){
  TickType_t xLastWakeTime = xTaskGetTickCount();
  for(;;){
    if (ledActionState == LEDActionState::START_CLOCKWISE){
      turnOffLEDs();
      toggleLED(0);
      ledActionState = LEDActionState::CLOCKWISE;
      vTaskDelayUntil(&xLastWakeTime, 100);
    }else if (ledActionState == LEDActionState::CLOCKWISE){
      byte x = 0;
      while (!ledstates[x] && x<4){ x++; };
      if (x != 4){
        toggleLED(x);
      }
      toggleLED((x+1)%4);
      vTaskDelayUntil(&xLastWakeTime, 100);
    }else if (ledActionState == LEDActionState::START_COUNTERCLOCKWISE){
      turnOffLEDs();
      toggleLED(0);
      ledActionState = LEDActionState::COUNTERCLOCKWISE;
      vTaskDelayUntil(&xLastWakeTime, 100);
    }else if (ledActionState == LEDActionState::COUNTERCLOCKWISE){
      byte x = 0;
      while (!ledstates[x] && x<4){ x++; };
      if (x != 4){
        toggleLED(x);
      }
      toggleLED((x+3)%4);
      vTaskDelayUntil(&xLastWakeTime, 100);
    }else if (ledActionState == LEDActionState::BLINKING_2){
      byte c = 0;
      for(int i=0; i<4; i++){
        c += ledstates[i];
      }
      if (c != 0 && c != 4){
        turnOffLEDs();
      }else if (c == 0){
        turnOnLEDs();
      }else if (c == 4){
        turnOffLEDs();
        ledActionState = LEDActionState::BLINKING_1;
      }
      vTaskDelayUntil(&xLastWakeTime, 300);
    }else if (ledActionState == LEDActionState::BLINKING_1){
      byte c = 0;
      for(int i=0; i<4; i++){
        c += ledstates[i];
      }
      if (c != 0 && c != 4){
        turnOffLEDs();
      }else if (c == 0){
        turnOnLEDs();
      }else if (c == 4){
        turnOffLEDs();
        ledActionState = defaultLedActionState;
      }
      vTaskDelayUntil(&xLastWakeTime, 300);
    }
      
  }
}

void setup() {
  Serial.begin(921600);
  HLHAsetup();

  xTaskCreatePinnedToCore(
    angleUpdateTask, 
    "angleUpdateTask", 
    2048, 
    NULL, 
    4, 
    &AngleUpdateTaskHandle,
    1
  );

  xTaskCreatePinnedToCore(
    LEDUpdateTask, 
    "LEDUpdateTask", 
    2048, 
    NULL, 
    1, 
    &LEDUpdateTaskHandle,
    1
  );

  xTaskCreatePinnedToCore(
    speedUpdateTask, 
    "SpeedUpdateTask", 
    2048, 
    NULL, 
    3, 
    &speedUpdateTaskHandle,
    0
  );
}

void loop() {
  if (micros() - mainLoopPreviousTime >= 500) {
    elapsedTime = micros() - mainLoopPreviousTime;
      
    error = targetSpeedR - currentSpeedR;
    derivative = (error - previousSpeedErrorR) / elapsedTime;
    previousSpeedErrorR = error;
    speedIntegralR = constrain(speedIntegralR + error, -speedIntegralMax, speedIntegralMax);
    currentPWMR = (int)constrain(currentPWMR + ( kpS*error + kiS*speedIntegralR + kdS*derivative ), -1023, 1023);
    speedRight(currentPWMR);

    error = targetSpeedL - currentSpeedL;
    derivative = (error - previousSpeedErrorL) / elapsedTime;
    previousSpeedErrorL = error;
    speedIntegralL = constrain(speedIntegralL + error, -speedIntegralMax, speedIntegralMax);
    currentPWML = (int)constrain(currentPWML + ( kpS*error + kiS*speedIntegralL + kdS*derivative ), -1023, 1023);
    speedLeft(currentPWML);

    mainLoopPreviousTime = micros();
  }
}