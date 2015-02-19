/*
 * The Arduino program used in the YouTube video http://www.youtube.com/watch?v=Gg6zqjDq1ho
 *
 * This program allow to rotate a webcam to track an object. This is
 * done by using Arduino to control a servo motor. The communication
 * is done using a serial connection (through USB or native).
 *
 * See the documentation to get the OpenCV program.
 *
 * Author: Frédéric Jolliton <frederic@jolliton.com>
 * Date: january 22, 2011
 * Documentation: http://doc.tuxee.net/tracking
 */

#include "WProgram.h"
#include "Servo/Servo.h"

Servo               servo;
static int          target = 1500;
static int          value = 0;

void setup()
{
  Serial.begin(9600);
  servo.attach(7);
  pinMode(13, OUTPUT);
}

void loop()
{
  //-------- Receive position from the PC (OpenCV) --------

  // Note that we don't necessary read a whole line at once. The value
  // is accumulated until it is read entirely.
  while (Serial.available()) {
    int c = Serial.read();
    if (c >= '0' && c <= '9') {
      value = 10 * value + (c - '0');
    } else if (c == '\n') {
      target = value;
      value = 0;
    } else if (c == '\r') {
      // skip
    } else {
      // reset on any unexpected character
      value = 0;
    }
  }

  //-------- Clamp the target value --------

  // Enable (briefly) on-board LED if the target was out of bound.
  if (target <= 500) {
    target = 500;
    digitalWrite(13, HIGH);
  } else if (target >= 2400) {
    target = 2400;
    digitalWrite(13, HIGH);
  } else {
    digitalWrite(13, LOW);
  }

  //-------- Update servo position --------

  servo.writeMicroseconds(target);
}
