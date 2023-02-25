/*
  picostep.h
  Arduino library for controlling stepper motors with RP2040 through an H-bridge
  Created by Experimental Cyborg, Februari 2023.
  Released into the public domain under GNU GPLv3.  
*/

#pragma once

#include <Arduino.h>

// RP2040_PWM stuff
#define _PWM_LOGLEVEL_ 0
#include "RP2040_PWM.h"
#define SUBPHASE_MAX 100000

#define MICROSEC_PER_SEC 1000000

class PicoStep
{
  public:
    // Advanced configuration. Directly modify these on your class instance.
    unsigned STEPS_PER_ROTATION = 200;  // standard NEMA steppers are 200 steps/rotation
    unsigned long PWM_COIL_FREQ = 80000;// hz
    unsigned long SPEED_MAX = 99999;    // steps/second
    unsigned FULLSTEP_AFTER = 500;      // steps/second - stop microstepping to compensate for loss of torque beyond this speed

    /**
     * Class constructor.
     * The pin pairs in1/in2 and in3/in4 must be on the same PWM slice.
     * See the RP2040_PWM library documentation for more information:
     * https://github.com/khoih-prog/RP2040_PWM#more-useful-information-about-RP2040-PWM
     *
     * @param ena Pin number to enable coil A
     * @param enb Pin number to enable coil B
     * @param in1 Pin number for coil A + / forward
     * @param in2 Pin number for coil A - / reverse
     * @param in3 Pin number for coil B + / forward
     * @param in4 Pin number for coil B + / reverse
     * @param invert_enable Whether to invert the enable pins or not.
     */
    PicoStep(uint8_t ena, uint8_t enb, uint8_t in1, uint8_t in2, uint8_t in3, uint8_t in4, bool invert_enable_pins);

    /**
     * Send power to the motor and enable movement.
     */    
    void arm();

    /**
     * Disable motor power.
     */    
    void disarm();

    /**
     * Execute all tasks needed to run the motor. 
     * Must be called as often as possible, for example inside the loop() function.
     */
    void run();

    /**
     * Stop the motor immediately with force.
     * To immediately freewheel the motor, use disarm().
     */        
    void stop();
    
    /**
     * Set the desired target speed.
     *
     * @param steps_per_second target speed in steps per second
     */
    void setSpeed(int steps_per_second);

    /**
     * Set the desired rate of acceleration.
     *
     * @param steps_per_second_per_second desired acceleration delta
     */
    void setAcceleration(unsigned steps_per_second_per_second);

    /**
     * Play one or two tones for a fixed duration.
     * Must be armed and stationary (speed=0). The motor will not move while beeping.
     * If both frequencies are 0, any current beep will be cut off and normal movement will resume.
     * If already beeping, current beep will be cut off seemlessly.
     *
     * @param frequency1 Frequency in hz
     * @param frequency1 Frequency in hz
     * @param duration Duration in ms
     */
    void beep(unsigned frequency1, unsigned frequency2, unsigned duration);

    /**
     * The same as beep() but it waits during the beep.
     *
     * @param frequency1 Frequency in hz
     * @param frequency1 Frequency in hz
     * @param duration Duration in ms
     */
    void beepBlocking(unsigned frequency1, unsigned frequency2, unsigned duration);

    
    /**
     * Get the current speed.
     *
     * @return The current speed in steps/second
     */
    int getSpeed();
    
    /**
     * Get the target speed.
     *
     * @return The target speed in steps/second
     */
    int getTargetSpeed();
    
    /**
     * Find out whether the motor is currently energised or not.
     *
     * @return true or false
     */
    bool isArmed();

    /**
     * Find out whether we are currently beeping.
     *
     * @return true or false
     */
    bool isBeeping();    


  private:
    void reset();
    bool sound();
    void accelerate();
    void step();

    // pinout
    uint8_t PIN_IN1;
    uint8_t PIN_IN2;
    uint8_t PIN_IN3;
    uint8_t PIN_IN4;
    uint8_t PIN_ENA;
    uint8_t PIN_ENB;

    // pwm
    RP2040_PWM* PWM_IN1;
    RP2040_PWM* PWM_IN2;
    RP2040_PWM* PWM_IN3;
    RP2040_PWM* PWM_IN4;

    // motion
    bool invert_enable;
    bool armed = false;
    long speed = 0;
    long speedTarget = 0;
    unsigned acceleration = SPEED_MAX;
    bool forward = true;
    bool forwardTarget = true;
    unsigned long lastAccelerateTime;
    unsigned long lastStepTime;
    int8_t phase;
    long subphase;
    
    // beep
    unsigned beep_frequency_1;
    unsigned beep_frequency_2;
    unsigned long beep_end_time;
};
