#include <Servo.h>
#include <Wire.h>
#include <LIDARLite.h>

LIDARLite lidarLite; // create LIDAR objekt to control a LIDARLite
Servo myservo;  // create servo object to control a servo

int pos = 0;     // variable to store the servo position
int dist =0;     // variable to store the measured distance
int sweep = 70;  // sweep area in degrees
int sweep_delay = 150; // sweep delay in ms per 1 degree of rotation

void setup() 
{
  myservo.attach(9);  // 9 = servo 2, 10 = servo 1
  lidarLite.begin(0, true);
  lidarLite.configure(0);
  Serial.begin(9600);
  myservo.write(0);
  delay(200);
}

void loop() {
  for (pos = 0; pos <= sweep; pos += 1)
  {
    dist = lidarLite.distance();
    myservo.write(pos);
    Serial.print("x");
    Serial.println(pos);
    
    delay(2);
    Serial.print("y");

    Serial.println(dist);
    delay(sweep_delay);
  }
  
  // Only to make the non-measuring return sweep more smooth
  for (pos=sweep; pos >= 0; pos -= 1) 
  {
    myservo.write(pos);
    delay(10);
  }
  
}
