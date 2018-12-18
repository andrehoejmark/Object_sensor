#include <Wire.h>
#include <LIDARLite.h>
#include "stepper.h"

/* External components Config */
LIDARLite lidarLite;
StepperMotor *myMotor;
unsigned char coils[] = {2, 3, 4, 5};

/* deltaAngle, LIDAR distance, output data and initial angle memory init */
float dA = 0.45;
int dist = 0;
String data;
int step_memory;

/* For use with 4-button keyboard */
int button_1 = 4;
int button_2 = 6;
int button_3 = 8;
int button_4 = 0;

/* Config custom 4-button keypad */
void init_buttons()
{
  /*pinMode(button_1, INPUT_PULLUP);
  pinMode(button_2, INPUT_PULLUP);
  pinMode(button_3, INPUT_PULLUP);*/
 // pinMode(button_4, INPUT_PULLUP);
}

void initial_angle_setup()
{
  /* Manually determine starting angle */
  while (digitalRead(button_3))
  if (!digitalRead(button_1))
  {
    myMotor->steppRight();
  }
  else if (!digitalRead(button_2))
  {
    myMotor->steppLeft();
  }

  step_memory = 0;
}

void setup() 
{  
  init_buttons();

  Serial.begin(57600);
  
  /* LIDAR Config */
  lidarLite.begin(0, true);
  lidarLite.configure(0);
  
  /* Driver Config */
  myMotor = new StepperMotor(halfStep, coils, 0, 100);

  //initial_angle_setup();
}

 boolean check_restart()
{
  if (!digitalRead(button_4))
  {
    while (step_memory >= 0)
    {
        myMotor->steppLeft();
        step_memory--;
    } 
    return true;
  }
  return false;
}

void loop() {
  int step_counter = 0;

  for (int i = 0; i < 200; i++)
  {
    //if (check_restart())
    //{exit(-1);}
    myMotor->steppRight();
    dist = lidarLite.distance();
    //data = String(step_counter * dA) + String(",") + String(dist);
    //Serial.println(data);
    Serial.print(step_counter * dA);
    Serial.print(',');
    Serial.println(dist);
    step_counter++;
    step_memory++;
  }

  for (int i = 200; i > 0; i--)
  {
    //if (check_restart())
    //{exit(-1);}
    myMotor->steppLeft();
    dist = lidarLite.distance();
    //data = String(step_counter * dA) + String(",") + String(dist);
    //Serial.println(data);
    Serial.print(step_counter * dA);
    Serial.print(',');
    Serial.println(dist);
    step_counter--;
    step_memory--;
  }
}