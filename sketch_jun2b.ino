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
volatile float currentSpeedR = 0;
volatile float currentSpeedL = 0;
volatile unsigned long speedCalculationPreviousTime = 0;
volatile unsigned long speedCalculationCurrentTime = 0;
volatile int previousEncoderRCount = 0;
volatile int previousEncoderLCount = 0;

const int CPR = 404;
const int minPWMR = 55;
const int minPWML = 57;

float restGyroX = 0.17;

esp_timer_handle_t speedTimer;
portMUX_TYPE speedMux = portMUX_INITIALIZER_UNLOCKED;

volatile int led = 0;
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
  unsigned long elapsedTime = speedCalculationCurrentTime - speedCalculationPreviousTime;

  if(elapsedTime != 0){
    currentSpeedR = ( (encoderRCount - previousEncoderRCount)*148514.85) / (double)elapsedTime;
    currentSpeedL = ( (encoderLCount - previousEncoderLCount)*148514.85) / (double)elapsedTime;
  }
  speedCalculationPreviousTime = speedCalculationCurrentTime;
  previousEncoderRCount = encoderRCount;
  previousEncoderLCount = encoderLCount;
  portEXIT_CRITICAL_ISR(&speedMux);
}

void setup() {
  Serial.begin(115200);

  for(int i=0; i<4; i++){
    pinMode(leds[i], OUTPUT);
  }
  
  attachInterrupt(digitalPinToInterrupt(encoderRA), encoderRISRA, CHANGE);
  attachInterrupt(digitalPinToInterrupt(encoderLA), encoderLISRA, CHANGE);
  attachInterrupt(digitalPinToInterrupt(encoderRB), encoderRISRB, CHANGE);
  attachInterrupt(digitalPinToInterrupt(encoderLB), encoderLISRB, CHANGE);

  const esp_timer_create_args_t timer_config = {
    .callback = &calculateSpeed,
    .arg = nullptr,
    .dispatch_method = ESP_TIMER_TASK,
    .name = "speedTimer"
  };
  
  esp_timer_create(&timer_config, &speedTimer);
  esp_timer_start_periodic(speedTimer, 5000);
  /*
  Serial.println("setup initiated");
  
  Wire.begin(25, 26);

  byte status = mpu.begin();
  while (status != 0) {
    Serial.println("MPU6050 initialization failed!");
    status = mpu.begin();
    delay(50);
  }
  Serial.println("success");
  */
}

void loop() {
  /*
  mpu.update();
  Serial.println(mpu.getAccX());

  delay(5);
  */
  Serial.println(currentSpeedR);
  delay(10);
}