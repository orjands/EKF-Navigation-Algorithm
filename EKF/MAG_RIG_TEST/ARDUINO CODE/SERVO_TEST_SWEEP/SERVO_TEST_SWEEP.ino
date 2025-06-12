#include <Servo.h>

Servo myservo;  // create servo object to control a servo

int pos = 0;    // variable to store the servo position

void setup() {
  myservo.attach(2);  // attaches the servo on pin 9 to the servo object
}

void loop() {
  myservo.write(pos);

  delay(1000 * 10);
  for (pos = 0; pos <= 180; pos += 1) { // goes from 0 degrees to 180 degrees in steps of 1 degree
    myservo.write(pos);                 // tell servo to go to position in variable 'pos'
    delay(15 * 3);                      // waits 45 ms for the servo to reach the position
  }
  
  delay(1000 * 10);
  
  for (pos = 180; pos >= 0; pos -= 1) { // goes from 180 degrees to 0 degrees
    myservo.write(pos);                 // tell servo to go to position in variable 'pos'
    delay(15 * 3);                      // waits 45 ms for the servo to reach the position
  }
}