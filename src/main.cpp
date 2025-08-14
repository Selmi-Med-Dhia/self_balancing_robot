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


/// balancing PID controller parameters

void speedUpdateTask(void *pvParameters) {
  const TickType_t xDelay = 1;
  TickType_t xLastWakeTime = xTaskGetTickCount();
  for(;;){
    Serial.println(encoderRCount);
    vTaskDelayUntil(&xLastWakeTime, xDelay);
  }
}
void setup() {
  Serial.begin(115200);
  HLHAsetup();

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
  vTaskDelay(1000 / portTICK_PERIOD_MS); // Delay to prevent watchdog reset
}