#include<Servo.h>

int servo1 = 11;
int servo2 = 10;

Servo joint1;
Servo joint2;

void setup() {
  // put your setup code here, to run once:

  joint1.attach(servo1);
  joint2.attach(servo2);

  Serial.begin(9600);

  joint1.write(90);
  joint2.write(0);

}

void loop() {
  // put your main code here, to run repeatedly:

  for (int i=45; i<=120; i++)
  {
    joint1.write(i);
    joint2.write(i-60);
    delay(5);
  }
  for (int j=120; j>=45; j--)
  {
    joint1.write(j);
    joint2.write(j-60);
    delay(5);
  }

}
