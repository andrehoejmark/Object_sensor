#include <Wire.h>
#include <Adafruit_MotorShield.h>
#include <LIDARLite.h>

LIDARLite lidarLite;
Adafruit_MotorShield AFMS;
Adafruit_StepperMotor *myMotor;

float step_degree = 0.45;


float current_degree = 0;
float start_degree = 70;
float end_degree = 140;
float distance = 0;

void setup() 
{  
  Serial.begin(9600);
  
  /* LIDAR Config */
  lidarLite.begin(0, true);
  lidarLite.configure(0);
  
  /* Driver Config */
  AFMS = Adafruit_MotorShield(); 
  AFMS.begin();

  /* Single Stepper Motor config */
  myMotor = AFMS.getStepper(400, 1);
  myMotor->setSpeed(10);
  
  while true 
  {
	delay(30)
	myMotor->step(1, FORWARD, SINGLE);
	current_degree += 0.9;
	
	if(current_degree > start_degree)
		break;
  }
}


bool rotate_clock_wise = false;
void loop() 
{
	dist = lidarLite.distance();
	data = String(current_degree) + String(",") + String(dist);
	Serial.println(data);
	
	if(rotate_clock_wise)
	{
		myMotor->step(1, FORWARD, INTERLEAVE);
		current_degree += 0.45;
	}
	else
	{
		myMotor->step(1, BACKWARD, INTERLEAVE);
		current_degree -= 0.45;
	}
	
	if(current_degree > end_degree)
		rotate_clock_wise = true;
	if(current_degree < start_degree)
		rotate_clock_wise = false
	
	delay(30);
}