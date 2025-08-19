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
const char* ssid = "lbor3i";
const char* password = "lbor3i1234";
LEDActionState ledActionState = LEDActionState::START_CLOCKWISE;
LEDActionState defaultLedActionState = LEDActionState::START_CLOCKWISE;

// Balancing PID controller variables
float kpB =60;
float kdB = 700;
float kiB = 50;
float balancingIntergal = 0;
float balancingIntegralLimit = 1000;
int32_t previousBalancingPIDTime = 0;
float previousBalancingError = 0;

MPU6050 mpu(Wire);
AsyncWebServer server(81);
AsyncWebSocket ws("/ws");

float getCurrentAngle() {
  static float angle = 0.0f;
  static uint32_t tprev = micros();

  mpu.update();
  float ax = mpu.getAccX();
  float ay = mpu.getAccY();
  float az = mpu.getAccZ();
  float gyroY = mpu.getGyroY();

  uint32_t now = micros();
  float dt = (now - tprev) * 1e-6f;
  tprev = now;

  float accAngle = atan2f(-ax, az) * 57.2957795f;

  
  float anorm = sqrtf(ax*ax + ay*ay + az*az);
  bool accelReliable = fabsf(anorm - 1.0f) < 0.15f;

  const float alpha = 0.99f;
  float gyroIntegrated = angle + gyroY * dt;

  if (accelReliable) {
    angle = alpha * gyroIntegrated + (1.0f - alpha) * accAngle;
  } else {
    angle = gyroIntegrated;
  }

  return -angle;
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

void turnOffLEDs(){
  for(int i=0; i<4; i++){
    digitalWrite(leds[i], 0);
    ledstates[i] = false;
  }
}

void turnOnLEDs(){
  for(int i=0; i<4; i++){
    digitalWrite(leds[i], 1);
    ledstates[i] = true;
  }
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

void onWsEvent(AsyncWebSocket *server, AsyncWebSocketClient *client, AwsEventType type, void *args, uint8_t *data, size_t len){
  if (type == WS_EVT_CONNECT){
    ledActionState = LEDActionState::BLINKING_2;
  }else if (type == WS_EVT_DISCONNECT){
    ledActionState = LEDActionState::START_CLOCKWISE;
  }else if (type == WS_EVT_DATA){
    String msg = "";
    for (size_t i = 0; i < len; i++) {
      msg += (char)data[i];
    }
    if (msg[0] == 'P'){
      kpB = msg.substring(1).toInt();
    }else if (msg[0] == 'I'){
      kiB = msg.substring(1).toInt()/100.0;
    }else if (msg[0] == 'D'){
      kdB = msg.substring(1).toInt();
    }
    else if (msg[0] == 'X'){
      balancingIntegralLimit = msg.substring(1).toInt();
    }
    ledActionState = LEDActionState::BLINKING_1;
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

  // Wifi
  WiFi.softAP(ssid, password);
  ws.onEvent(onWsEvent);
  server.addHandler(&ws);
  server.begin();

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