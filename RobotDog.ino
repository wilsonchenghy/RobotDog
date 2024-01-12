#include <math.h>
#include <Servo.h>

#define Pi 3.141592654

float z1 = 91.8;
float z2 = 180;

float x1 = 90;
float x2 = 0;

float y1 = 30;

double pitchAngle = 20;

float upperLegLength = 120;
float lowerLegLength = 120;
float length4 = 45; // the offset
float bodyLength = 200;
float bodyWidth = 120; // !!!!!!!!!!!!!!!!!!!!!! need add here

bool isFrontLeg = true;
bool isLeftLeg = false;

Servo servo1;
Servo servo2;
Servo servo3;
Servo servos[3];

int servoPin[3] = {1, 2, 4};

double angle1;
double angle2;
double angle3;

float checkValueX;
float checkValueY;
float checkValueZ;



double radToAngle(double angle) {
  return (angle*180)/Pi;
}

double angleToRad(double angle) {
  return (angle/180)*Pi;
}

void zAxisTranslation(float z) {
  angle1 = acos((-z*z + 2*(upperLegLength * lowerLegLength))/(2 * upperLegLength * lowerLegLength));
  angle2 = (Pi - angle1)/2;
}

void xAxisTranslation(float x, float z) {
  float length3 = sqrt(x*x + z*z);
  double alpha = atan(abs(x)/z);
  zAxisTranslation(length3);
  double beta = angle2;
  double angle3;
  if (x>0){
    if (beta >= alpha) {
      angle2 = beta - alpha;
    } else {
      Serial.println("error!!! exceed joint limit"); // seems should be not neccessary coz that is bascially just the joint limit
    }
  } else {
    angle2 = beta + alpha;
  }
}

void yAxisTranslation(float x, float y, float z) { // one mor eargument of isLeft

  if (!isLeftLeg) {
    y *= -1;
  }

  y += length4; // the input y can be positive or negative, but the final resulting y in this line must be positive
  float length5 = sqrt(y*y + z*z);
  double omega = atan(y/z);
  double tau = asin(length4/length5);

  angle3 = omega - tau + (Pi/2);

  float length6 = length4/(tan(tau));

  xAxisTranslation(x, length6);
}

void pitchRotation(float x, float y, float z, double pitchAngle, double rowAngle) {
  pitchAngle = angleToRad(pitchAngle);

  float l1 = (bodyLength/2) * cos(pitchAngle);
  float l2 = (bodyLength/2) * sin(pitchAngle);

  float NewZ = 0;

  if(isFrontLeg) {
    if (pitchAngle>=0) {
      NewZ = z - l2;
    } else {
      NewZ = z + l2;
    }
  } else if(pitchAngle>=0) {
    NewZ = z + l2;
  } else {
    NewZ = z - l2;
  }

  float NewX = (bodyLength/2) - l1;
  NewX = NewX + x;

  double phi = atan(NewX/NewZ);

  phi = abs(phi);

  double delta = 0;

  if(isFrontLeg) {
    if (pitchAngle>=0) {
      if (x >= -((bodyLength/2) - l1)) {
        delta = pitchAngle + phi;
      } else {
        delta = pitchAngle - phi;
      }
    } else {
      if (x >= ((bodyLength/2) - l1)) {
        delta = pitchAngle + phi;
      } else {
        delta = pitchAngle - phi;
      }
    }
  } else if(pitchAngle>=0) {
    if (x >= ((bodyLength/2) - l1)) {
        delta = pitchAngle + phi;
      } else {
        delta = pitchAngle - phi;
      }
  } else {
    if (x >= -((bodyLength/2) - l1)) {
        delta = pitchAngle + phi;
      } else {
        delta = pitchAngle - phi;
      }
  }

  float l3 = sqrt(NewX*NewX + NewZ*NewZ);

  NewZ = l3*cos(delta);
  NewX = l3*sin(delta);

  // Serial.print(NewZ);
  // Serial.print(" and ");
  // Serial.println(NewX);

  rowRotation(NewX, y, NewZ, rowAngle);

}



void rowRotation(float x, float y, float z, double rowAngle) {
  rowAngle = angleToRad(rowAngle);

  float l1 = (bodyWidth/2) * cos(abs(rowAngle));
  float l2 = (bodyWidth/2) * sin(abs(rowAngle));

  float NewZ = 0;

  if(!isLeftLeg) {
    if (rowAngle>=0) {
      NewZ = z - l2;
    } else {
      NewZ = z + l2;
    }
  } else if(rowAngle>=0) {
    NewZ = z + l2;
  } else {
    NewZ = z - l2;
  }

  float NewY = (bodyWidth/2) - l1;
  NewY = NewY + y + length4;

  double phi = atan(NewY/NewZ);

  phi = abs(phi);

  double delta = 0;

  if(isLeftLeg) {
    if (rowAngle>=0) {
      if ((y+length4) >= -((bodyWidth/2) - l1)) {
        delta = phi - rowAngle;
      } else {
        delta = rowAngle + phi;
      }
    } else {
      if ((y+length4) >= -((bodyWidth/2) - l1)) { 
        delta = abs(rowAngle) + phi;
      } else {
        delta = phi - abs(rowAngle);
      }
    }
  } else if(rowAngle>=0) {                            // temporarily view right side as positive instead (flip)
    if ((-y+length4) >= -((bodyWidth/2) - l1)) {
        delta = rowAngle + phi;
      } else {
        delta = phi - rowAngle;
      }
  } else {
    if ((-y+length4) >= -((bodyWidth/2) - l1)) {
        delta = phi - abs(rowAngle);
      } else {
        delta = abs(rowAngle) + phi;
      }
  }

  float l3 = sqrt(NewY*NewY + NewZ*NewZ);

  NewZ = l3*cos(delta);
  // if (isLeftLeg) {
  //   NewY = l3*sin(delta) - length4;
  // } else {
  //   NewY = l3*sin(delta) + length4;
  // }

  if (isLeftLeg) {
    NewY = l3*sin(delta) - length4;
  } else {
    NewY = -(l3*sin(delta)) + length4;
  }

  // if (l3*sin(delta) >= length4) {
  //   NewY = l3*sin(delta) - length4;
  // } else{

  // } ////// not sure

  // Serial.print(NewZ);
  // Serial.print(" and ");

  yAxisTranslation(x, NewY, NewZ);
}



// !!!!!!!!!!!! rmb degree to radian
void YawRotation() {

}




// !!!!!!!!! need change function
void moveLeg(float x, float y, float z, double pitchAngle, double rowAngle) {
  pitchRotation(x, y, z, pitchAngle, rowAngle);

  // yAxisTranslation(x, y, z);

  angle1 = radToAngle(angle1);
  angle2 = radToAngle(angle2);
  angle3 = radToAngle(angle3);

  servo1.write(180-angle1); // consider offset --> 180-angle1
  servo2.write(angle2);
  servo3.write(angle3);

  // Serial.println("Angle: ");
  // Serial.print(angle1);
  // Serial.print(" and ");
  // Serial.print(angle2);
  // Serial.print(" and ");
  // Serial.println(angle3);

  // Serial.print(" here ");
  // Serial.println(180-angle1);
}




// Forward Kinematics
void checkIK(double angle1, double angle2, double angle3) {
  angle1 = angleToRad(angle1);
  angle2 = angleToRad(angle2);
  angle3 = angleToRad(angle3);
  double tempAngle1 = angle3 - (Pi/2);
  double tempAngle2 = Pi - Pi/2 - (tempAngle1);
  checkValueX = lowerLegLength*cos(angle1 + angle2 - Pi/2) - upperLegLength*sin(angle2);
  float tempValueZ = upperLegLength*cos(angle2) + lowerLegLength*sin(angle1 + angle2 - Pi/2);
  checkValueY = tempValueZ*cos(tempAngle2) + length4*cos(tempAngle1) - length4;
  checkValueZ = tempValueZ*sin(tempAngle2) - length4*sin(tempAngle1);

  if (!isLeftLeg) {
    checkValueY *= -1;
  }

  Serial.println("Validate: ");
  Serial.print(checkValueX);
  Serial.print(" and ");
  Serial.print(checkValueY);
  Serial.print(" and ");
  Serial.println(checkValueZ);
}




void setup() {
  Serial.begin(9600);
  
  servo1.attach(3);
  servo2.attach(5);
  servo3.attach(6);
  servos[0].attach(servoPin[0]);
}

void loop() {

  // float pot = analogRead(A5); 

  // pot = map(pot, 0, 1023, 0, 60);

  // moveLeg(0, pot, 150);

  // pot = map(pot, 0, 1023, -70, 70);

  // moveLeg(pot, 0, 150);

  // pot = map(pot, 0, 1023, 90, 150);

  // moveLeg(0, 0, pot);

  // delay(1000);

  
  // moveLeg(0, 0, 150);
  
  // delay(1000);

  // moveLeg(0, 0, 90);

  // servo1.write(101);

  // moveLeg(60, 30, 150, 20, 10);


  // moveLeg(0, 0, 150, 0, 0);

  // delay(1000);
  
  // checkIK(angle1, angle2, angle3);

  // delay(1000);



  float potz = analogRead(A0); 
  float potx = analogRead(A0); 
  float potyRight = analogRead(A0); 
  float potyLeft = analogRead(A0);

  potz = map(potz, 0, 1023, 60, 150);
  potx = map(potx, 0, 1023, -100, 30);
  potyRight = map(potyRight, 0, 1023, 0, 60);
  potyLeft = map(potyLeft, 0, 1023, -60, 0);


  // zAxisTranslation(potz);
  // xAxisTranslation(potx, 100);
  yAxisTranslation(potx, 0, 100);
  angle1 = radToAngle(angle1);
  angle2 = radToAngle(angle2);
  angle3 = radToAngle(angle3);

  //RightLeg
  // servo1.write(180-angle1); // consider offset --> 180-angle1
  // servo2.write(angle2);
  // servo3.write(angle3);

  //LeftLeg
  servo1.write(angle1); // consider offset --> 180-angle1
  servo2.write(180-angle2);
  servo3.write(180-angle3);

}