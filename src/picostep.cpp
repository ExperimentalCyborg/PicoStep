/*
  picostep.cpp
  Arduino library for controlling stepper motors with RP2040 through an H-bridge
  Created by Experimental Cyborg, Februari 2023.
  Released into the public domain under GNU GPLv3.
*/

#include "picostep.h"

PicoStep::PicoStep(uint8_t ena, uint8_t enb, uint8_t in1, uint8_t in2, uint8_t in3, uint8_t in4, bool invert_enable_pins){
  PIN_IN1 = in1;
  PIN_IN2 = in2;
  PIN_IN3 = in3;
  PIN_IN4 = in4;
  PIN_ENA = ena;
  PIN_ENB = enb;
  invert_enable = invert_enable_pins;

  pinMode(PIN_ENA, OUTPUT);
  pinMode(PIN_ENB, OUTPUT);
  PWM_IN1 = new RP2040_PWM(PIN_IN1, PWM_COIL_FREQ, 0);
  PWM_IN2 = new RP2040_PWM(PIN_IN2, PWM_COIL_FREQ, 0);
  PWM_IN3 = new RP2040_PWM(PIN_IN3, PWM_COIL_FREQ, 0);
  PWM_IN4 = new RP2040_PWM(PIN_IN4, PWM_COIL_FREQ, 0);
  disarm();
}

void PicoStep::arm(){
  PWM_IN1->enablePWM();
  PWM_IN2->enablePWM();
  PWM_IN3->enablePWM();
  PWM_IN4->enablePWM();

  // Initiate phase configuration 1010 (coil A and B both positive)
  PWM_IN1->setPWM_Int(PIN_IN1, PWM_COIL_FREQ, SUBPHASE_MAX);
  PWM_IN2->setPWM_Int(PIN_IN2, PWM_COIL_FREQ, 0);
  PWM_IN3->setPWM_Int(PIN_IN3, PWM_COIL_FREQ, SUBPHASE_MAX);
  PWM_IN4->setPWM_Int(PIN_IN4, PWM_COIL_FREQ, 0);

  reset();

  digitalWrite(PIN_ENA, HIGH);
  digitalWrite(PIN_ENB, HIGH);
  
  armed = true;
}

void PicoStep::disarm(){
  digitalWrite(PIN_ENA, LOW);
  digitalWrite(PIN_ENB, LOW);
  
  reset();

  PWM_IN1->setPWM_Int(PIN_IN1, PWM_COIL_FREQ, 0);
  PWM_IN2->setPWM_Int(PIN_IN2, PWM_COIL_FREQ, 0);
  PWM_IN3->setPWM_Int(PIN_IN3, PWM_COIL_FREQ, 0);
  PWM_IN4->setPWM_Int(PIN_IN4, PWM_COIL_FREQ, 0);

  delay(50);

  PWM_IN1->disablePWM();
  PWM_IN2->disablePWM();
  PWM_IN3->disablePWM();
  PWM_IN4->disablePWM();

  armed = false;
}

void PicoStep::run(){
  if(!sound()){
    accelerate();
    step();
  }  
}

void PicoStep::stop(){
  reset();
  step();
}

void PicoStep::setSpeed(int steps_per_second){
  if(steps_per_second < 0){
    forwardTarget = false;
    steps_per_second = steps_per_second * -1;
  }else{
    forwardTarget = true;
  }

  if(steps_per_second > SPEED_MAX){
    steps_per_second = SPEED_MAX;
  }

  speedTarget = steps_per_second * SUBPHASE_MAX;
}

void PicoStep::setAcceleration(unsigned steps_per_second_per_second){
  //todo if 0, set to max speed value.
  acceleration = steps_per_second_per_second * SUBPHASE_MAX;
}

void PicoStep::beep(unsigned frequency1, unsigned frequency2, unsigned duration){
  if(speed != 0){
    return;
  }

  if(frequency1 == 0 && frequency2 == 0){
    duration = 0;
  }

  beep_frequency_1 = frequency1;
  beep_frequency_2 = frequency2;
  beep_end_time = micros() + duration * 1000;

  // Initiate beep
  PWM_IN1->setPWM_Int(PIN_IN1, beep_frequency_1, SUBPHASE_MAX/10);
  PWM_IN2->setPWM_Int(PIN_IN2, beep_frequency_1, 0);
  PWM_IN3->setPWM_Int(PIN_IN3, beep_frequency_2, SUBPHASE_MAX/10);
  PWM_IN4->setPWM_Int(PIN_IN4, beep_frequency_2, 0);
}

void PicoStep::beepBlocking(unsigned frequency1, unsigned frequency2, unsigned duration){
  if(speed != 0){
    return;
  }

  // Initiate beep
  if(frequency1 > 0){
    PWM_IN1->setPWM_Int(PIN_IN1, frequency1, SUBPHASE_MAX/2);
    PWM_IN2->setPWM_Int(PIN_IN2, frequency1, 0);
  }else{
    PWM_IN1->setPWM_Int(PIN_IN1, PWM_COIL_FREQ, SUBPHASE_MAX);
    PWM_IN2->setPWM_Int(PIN_IN2, PWM_COIL_FREQ, 0);
  }
  
  if(frequency2 > 0){
    PWM_IN3->setPWM_Int(PIN_IN3, frequency2, SUBPHASE_MAX/2);
    PWM_IN4->setPWM_Int(PIN_IN4, frequency2, 0);
  }else{
    PWM_IN3->setPWM_Int(PIN_IN3, PWM_COIL_FREQ, SUBPHASE_MAX);
    PWM_IN4->setPWM_Int(PIN_IN4, PWM_COIL_FREQ, 0);
  }

  delay(duration);
  arm();  
}

int PicoStep::getSpeed(){
  int mult;
  if(forward){
    mult = 1;
  }else{
    mult = -1;
  }
  
  return (speed / SUBPHASE_MAX) * mult;
}

int PicoStep::getTargetSpeed(){
  int mult;
  if(forwardTarget){
    mult = 1;
  }else{
    mult = -1;
  }

  return (speedTarget / SUBPHASE_MAX) * mult;
}

bool PicoStep::isArmed(){
  return armed;
}

bool PicoStep::isBeeping(){
  return beep_frequency_1 && beep_frequency_2;
}

void PicoStep::reset(){
  phase = 0;
  subphase = 0;
  speed = 0;
  speedTarget = 0;
  lastStepTime = micros();
  lastAccelerateTime = micros();
}

bool PicoStep::sound(){
  if(isBeeping()){
    if(micros() >= beep_end_time){ // todo guard against timer rollover
      beep_frequency_1 = 0;
      beep_frequency_2 = 0;
      arm();
    }else{
      return true;
    }    
  }
  return false;
}

void PicoStep::accelerate(){
  // speed = speedTarget;
  // forward = forwardTarget;
  unsigned long now = micros();
  unsigned long t_delta = now - lastAccelerateTime;

  if(speed == speedTarget && forward == forwardTarget){
    lastAccelerateTime = now;
    return;
  }
  
  // Don't accelerate too granularly, or rounding errors will be significant.
  if(t_delta < 100){
    return;
  }
  
  // Calculate the next speed if we're accelerating
  float acceleration_step = acceleration / (MICROSEC_PER_SEC / t_delta);
  if(forward != forwardTarget){
    speed -= acceleration_step;
    if(speed < 0){      
      speed = speed * -1;
      forward = forwardTarget;
    }
  }else if(speed > speedTarget){
    if(speed - acceleration_step <= speedTarget){
      speed = speedTarget;
    }else{
      speed -= acceleration_step;      
    }    
  }else if(speed < speedTarget){
    if(acceleration_step + speed >= speedTarget){
      speed = speedTarget;
    }else{
      speed += acceleration_step;
    }
  }

  lastAccelerateTime = now;
}

void PicoStep::step(){
  // stepping pattern
  // IN1 IN2 IN3 IN4
  //  1   0   1   0
  //  1   0   0   1
  //  0   1   0   1
  //  0   1   1   0

  unsigned long now = micros();
  unsigned long t_delta = now - lastStepTime;

  if(!armed || speed == 0){
    // keep t_delta from accumulating, which gives the motor a kick next time it starts.
    lastStepTime = now;
    return;
  }

  // Don't take too small steps, or rounding errors will be significant.
  if(t_delta < 100){
    return;
  }

  // Calculate the size of the step to take
  long subphase_shift = speed / (MICROSEC_PER_SEC / t_delta);
  if(forward){
    subphase += subphase_shift;
    while(subphase > SUBPHASE_MAX){ // roll substeps over to steps
      phase += 1;
      if(phase == 4){
        phase = 0;
      }
      subphase -= SUBPHASE_MAX;
    }
  }else{
    subphase -= subphase_shift;
    while(subphase < 0){ // roll substeps over to steps
      phase -= 1;
      if(phase < 0){
        phase = 3;
      }
      subphase += SUBPHASE_MAX;
    }
  }

  bool fullstep = false;
  if(speed > FULLSTEP_AFTER * SUBPHASE_MAX){
    fullstep = true;
  }

  // Take the step
  switch(phase){ // todo reverse direction option, todo save power by not making the coils work against each other during the overlap
    case 0: // 1010 -> 1001
      if(fullstep){
        PWM_IN4->setPWM_Int(PIN_IN4, PWM_COIL_FREQ, 0);
        PWM_IN3->setPWM_Int(PIN_IN3, PWM_COIL_FREQ, SUBPHASE_MAX);
      }else{
        PWM_IN3->setPWM_Int(PIN_IN3, PWM_COIL_FREQ, SUBPHASE_MAX - subphase);
        PWM_IN4->setPWM_Int(PIN_IN4, PWM_COIL_FREQ, subphase);
      }
      break;
    case 1: // 1001 -> 0101
    if(fullstep){
        PWM_IN2->setPWM_Int(PIN_IN2, PWM_COIL_FREQ, 0);
        PWM_IN1->setPWM_Int(PIN_IN1, PWM_COIL_FREQ, SUBPHASE_MAX);
      }else{
        PWM_IN1->setPWM_Int(PIN_IN1, PWM_COIL_FREQ, SUBPHASE_MAX - subphase);
        PWM_IN2->setPWM_Int(PIN_IN2, PWM_COIL_FREQ, subphase);
      }
      break;
    case 2: // 0101 -> 0110
    if(fullstep){
        PWM_IN3->setPWM_Int(PIN_IN3, PWM_COIL_FREQ, 0);
        PWM_IN4->setPWM_Int(PIN_IN4, PWM_COIL_FREQ, SUBPHASE_MAX);
      }else{
        PWM_IN3->setPWM_Int(PIN_IN3, PWM_COIL_FREQ, subphase);
        PWM_IN4->setPWM_Int(PIN_IN4, PWM_COIL_FREQ, SUBPHASE_MAX - subphase);
      }
      break;
    case 3: // 0110 -> 1010
    if(fullstep){
        PWM_IN1->setPWM_Int(PIN_IN1, PWM_COIL_FREQ, 0);
        PWM_IN2->setPWM_Int(PIN_IN2, PWM_COIL_FREQ, SUBPHASE_MAX);
      }else{
        PWM_IN1->setPWM_Int(PIN_IN1, PWM_COIL_FREQ, subphase);
        PWM_IN2->setPWM_Int(PIN_IN2, PWM_COIL_FREQ, SUBPHASE_MAX - subphase);
      }
      break;
  }

  lastStepTime = now;
}
