#include<Arduino.h>
#include<ArduinoJson.h>
#include<Servo.h>

#define GBaseForward_PIN 4
#define GBaseBackward_PIN 2
#define ZUpward_PIN 7
#define ZDownward_PIN 6

#define EBase_PIN A0
#define EZ_PIN A1

#define BaseJoint_PIN 9
#define MiddleJoint_PIN 10
#define EndEffector_PIN 11

Servo BaseJoint;
Servo MiddleJoint;
Servo EndEffector;

int CaptureAngle = 180;
int delayTime = 3000;

float posX;
float posY;
float Angle1;
float Angle2;

float PositionArray[] = {0,0};
float AngleArray[] = {0, 0};

void setup() {
  // put your setup code here, to run once:
  BaseJoint.attach(BaseJoint_PIN);
  MiddleJoint.attach(MiddleJoint_PIN);
  EndEffector.attach(EndEffector_PIN);

  Serial.begin(9600);

  pinMode(GBaseForward_PIN, OUTPUT);
  pinMode(GBaseBackward_PIN, OUTPUT);
  pinMode(ZUpward_PIN, OUTPUT);
  pinMode(ZDownward_PIN, OUTPUT);

  pinMode(EBase_PIN, OUTPUT);
  pinMode(EZ_PIN, OUTPUT);

  BaseJoint.write(90);
  MiddleJoint.write(0);
  EndEffector.write(180);

  analogWrite(EBase_PIN, 255);
  analogWrite(EZ_PIN, 255);

  digitalWrite(GBaseForward_PIN, LOW);
  digitalWrite(GBaseBackward_PIN, LOW);
  digitalWrite(ZUpward_PIN, LOW);
  digitalWrite(ZDownward_PIN, LOW);

}


bool InverseKinematics(float px, float py, float link1, float link2)
{
    bool Ikin = false;

    float theta2 = acos((sq(px)+ sq(py) - sq(link1) - sq(link2)) / (2*link1*link2));
    float theta1= atan(py / px) - atan((link2*sin(theta2)) / (link1+ link2*cos(theta2)));

    float angle1 = (theta1*180)/3.14159;
    float angle2 = (theta2*180)/3.14159;
    
    if (Ikin == false){
    PositionArray[0] = px;
    PositionArray[1] = py;
    Ikin = true;
    BaseJoint.write(angle1);
    MiddleJoint.write(angle2);
  }
  return Ikin;
    //Serial.println(angle1);
    //Serial.println(angle2);
    //delay(1000);
}

bool ForwardKinematics(float angle1, float angle2, float link1, float link2)
{
  //float PositionArray[] = {0, 0};
  bool Fkin = false;
  
  float theta1 = angle1*(3.14/180);
  float theta2 = angle2*(3.14/180);

  float px = link1 * cos(theta1) +link2 * cos(theta1 + theta2);
  float py = link1 * sin(theta1) +link2 * sin (theta1 + theta2);

  Serial.println(px);
  Serial.print(py);
  Serial.println("------------");
  delay(1000);
  if (Fkin == false){
    PositionArray[0] = px;
    PositionArray[1] = py;
    Fkin = true;
  }
  return Fkin;
}

void GBForward()
{
  digitalWrite(GBaseForward_PIN, HIGH);
  digitalWrite(GBaseBackward_PIN, LOW);
  delay(delayTime);
  digitalWrite(GBaseForward_PIN, LOW);
  digitalWrite(GBaseBackward_PIN, LOW);
}

void GBBackward()
{
  digitalWrite(GBaseForward_PIN, LOW);
  digitalWrite(GBaseBackward_PIN, HIGH);
  delay(delayTime);
  digitalWrite(GBaseForward_PIN, LOW);
  digitalWrite(GBaseBackward_PIN, LOW);
}

void ZUp()
{
  digitalWrite(ZUpward_PIN, HIGH);
  digitalWrite(ZDownward_PIN, LOW);
  //delay(delayTime);
  delay(2000);
  digitalWrite(ZUpward_PIN, LOW);
  digitalWrite(ZDownward_PIN, LOW);
}

void ZDown()
{
  digitalWrite(ZUpward_PIN, LOW);
  digitalWrite(ZDownward_PIN, HIGH);
  delay(2000);
  digitalWrite(ZUpward_PIN, LOW);
  digitalWrite(ZDownward_PIN, LOW);
}

void EEFCap()
{
  EndEffector.write(CaptureAngle);
}

void EEFRel()
{
  EndEffector.write(0);
}

void loop() 
{
 GBForward();
 GBBackward();
 
 ZUp();
 delay(1000);
 ZDown();
 //InverseKinematics(0,196.79,95,101.79);
 //GBBackward();
 BaseJoint.write(0);
 MiddleJoint.write(0);
 delay(2000);
 EEFCap();
 BaseJoint.write(90);
 MiddleJoint.write(90);
 delay(1000);


  
}