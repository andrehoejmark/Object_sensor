// stepper.c
#import <Arduino.h>
#include "stepper.h"

StepperMotor::~StepperMotor(){
  for (int i = 0; i < 4; i++)
    pinMode(this->coils[i], INPUT);
}

StepperMotor::StepperMotor(unsigned char *stateMachine, unsigned char *coils, unsigned int stepDelay, unsigned int rotationDelay){
  this->state = 1;
  this->stepDelay = stepDelay;
  this->rotationDelay = rotationDelay;
  
  this->stateMachine = stateMachine;
  
  for (int i = 0; i < 4; i++){
    this->coils[i] = coils[i];
    pinMode(this->coils[i], OUTPUT);
  }
}

void StepperMotor::outputPin(unsigned char pin, unsigned char mode){
  if(mode)
    digitalWrite(coils[pin], HIGH);
  else
    digitalWrite(coils[pin], LOW);
}

void StepperMotor::outputState(){
  if (state > stateMachine[0]) return;

  for (unsigned char i = 0; i < 4; i++){
    outputPin(i, stateMachine[state] & ( 1 << i ));
  }
}

void StepperMotor::activate(unsigned char state){
  this->state = state;
  this->outputState();
}

void StepperMotor::deacivate(){
  for (unsigned char i = 0; i < 4; i++){
    digitalWrite(this->coils[i], LOW);
  }
}

void StepperMotor::Stepp(unsigned int stepps, LR dir){
  signed char di;
  if (dir == left) di = -1;
  else di = 1;

  for (; stepps > 0; stepps--){
    state += di;

    if (state == 0) state = stateMachine[0];
    else if (state > stateMachine[0]) state = 1;

    this->outputState();

    //delayMicroseconds(this->delay_);
    delay(stepDelay);
  }
  delay(rotationDelay);
}

void StepperMotor::SteppSingle(LR dir){
  signed char di;
  if (dir == left) di = -1;
  else di = 1;
  state += di;

  if (state == 0) state = stateMachine[0];
  else if (state > stateMachine[0]) state = 1;

  this->outputState();

  //delayMicroseconds(this->delay_);
  delay(stepDelay);
}
