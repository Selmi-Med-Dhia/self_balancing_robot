#pragma once
/**
 * @file HLHA.h
 * @brief Motor, encoder and LED control for a self-balancing robot using Arduino.
 *
 * This file contains function declarations and global variables for controlling the motors,
 * reading encoders, and managing LEDs on a self-balancing robot platform.
 *
 * - Motor control is achieved through PWM signals.
 * - Encoder counts are updated using interrupt service routines (ISRs).
 * - Functions are provided to set motor speeds, toggle LEDs, and initialize hardware.
 *
 * @note The macro FAST_READ(pin) and FAST_READ1(pin) are used for fast digital reads by directly accessing
 *       the GPIO register. The expression `& 0x1` is used to mask all bits except the
 *       least significant bit, effectively extracting the digital state (HIGH or LOW)
 *       of the specified pin.
 */

#include <Arduino.h>

#define FAST_READ(pin) (((uint32_t)GPIO.in >> pin) & 0x1)      // pins 0–31
#define FAST_READ1(pin) (((uint32_t)GPIO.in1.val >> ((pin) - 32)) & 0x1)

// motor pins
const int motorLa = 21;
const int motorLb = 19;
const int motorRa = 23;
const int motorRb = 22;

// encoder pins
const int encoderRA = 33;
const int encoderRB = 18;
const int encoderLA = 5;
const int encoderLB = 4;

// Leds' pins ordered clockwise
const int leds[] = {32, 13, 14, 27};

// global constants
const int minPWMR = 210;
const int minPWML = 215;
const int CPR = 202;

//global variables
extern bool ledstates[4];
extern volatile int encoderRCount;
extern volatile int encoderLCount;
extern volatile int32_t speedCalculationPreviousTimeR;
extern volatile int32_t speedCalculationCurrentTimeR;
extern volatile int32_t speedCalculationPreviousTimeL;
extern volatile int32_t speedCalculationCurrentTimeL;
extern volatile int8_t directionR;
extern volatile int8_t directionL;
extern volatile float currentSpeedR;
extern volatile float currentSpeedL;

void speedRight(int pwm);
void speedLeft(int pwm);

void toggleLED(int index);

void IRAM_ATTR encoderRISRA();
void IRAM_ATTR encoderRISRB();
void IRAM_ATTR encoderLISRA();
void IRAM_ATTR encoderLISRB();

void HLHAsetup();