/* Sweep
 by BARRAGAN <http://barraganstudio.com>
 This example code is in the public domain.

 modified 8 Nov 2013
 by Scott Fitzgerald
 https://www.arduino.cc/en/Tutorial/LibraryExamples/Sweep
*/

#include <Servo.h>


int incomingByte;
int yCharacter = 121;
int nCharacter = 110;
Servo myservo;  // create Servo object to control a servo

int pos = 0;  // variable to store the servo position

void setup() {
  myservo.attach(10);  // attaches the servo on pin 10 to the Servo object
  Serial.begin(9600);
  Serial.println("Pickle Patrol Active");
  myservo.write(180);  // initialize servo in 180 degree position
}  // end of setup()

void loop() {

  if (Serial.available() > 0) {
    incomingByte = Serial.read();
    if (incomingByte == 121) {  // if y character is received, drive servo to 0 degrees
      myservo.write(0);
      Serial.println("Overboard!");
      delay(1000);
    }
    if (incomingByte == 110) {  // if n character is received, drive servo to 180 degrees
      myservo.write(180);
      Serial.println("Calm");
      delay(1000);
    }
  }  
} // end of Loop()
