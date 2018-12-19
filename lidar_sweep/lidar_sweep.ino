#include <Wire.h>
#include <LIDARLite.h>

#include "stepper.h"

LIDARLite lidarLite;
StepperMotor *myMotor;
unsigned char coils[] = {2, 3, 4, 5};

float step_degree = 0.45;


float current_degree = 0;
float start_degree = 58.9+7;
float end_degree = 121.1+7;
int distance = 0;
String data = "";
bool rotate_clock_wise = false;

void setup() 
{ 

  delay(2000);
  Serial.begin(9600);
  myMotor = new StepperMotor(halfStep, coils, 0, 100);
  
  /* LIDAR Config */
  lidarLite.begin(0, true);
  lidarLite.configure(0);
  current_degree = 0;
  start_degree = 58.9;

  for (;current_degree < start_degree; current_degree += 0.45)
  {
	myMotor->steppLeft();
    	delay(5);
  }
  delay(5000);
}



void loop() 
{
	distance = lidarLite.distance();
  	Serial.print(current_degree);
  	Serial.print(',');
  	Serial.println(distance);
	//Serial.println(data);
	
	if(rotate_clock_wise)
	{
		myMotor->steppRight();
		current_degree -= 0.45;
	}
	else
	{
		myMotor->steppLeft();
		current_degree += 0.45;
    
	}
	
	if(current_degree > end_degree)
		rotate_clock_wise = true;
		Serial.print("None");
  		Serial.print(',');
  		Serial.println("None");
	if(current_degree < start_degree)
		rotate_clock_wise = false;
		Serial.print("None");
  		Serial.print(',');
  		Serial.println("None");
		

  delay(5);
}
