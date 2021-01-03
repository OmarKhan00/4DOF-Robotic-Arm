/******************************
* Project ECE3091             *
* Group 24                    *
* Omar Khan & Michael Chang   *
******************************/

#include <math.h>
#include "VarSpeedServo.h"

//servo speed 0.1sec/60deg
//LIMITS
//servo_claw: closed = 22, open = 180,OK: closed = 130, open = 0; closed w/o blocks (130 degrees -> 440, closed with smaller block (130 degrees -> 450-470), closed with large (130 -> 500+)
//servo_t1: 90deg right = 0 , centre = 100, 72deg left = 180
//servo_t2: 0deg  = 135 , 90deg = 41, store = 15
//servo_t3: t3: -24deg to -157deg hard limit, straight = 100, 90deg = 0,
//x = -(t3+t2)
// store = 70
// claw offset (r) = 6cm
// base offset (r) = 1.6cm
// height offset (z) = 5.65cm ,take away for droop
//object detector 0.5 to 1 cm, 1.5 cm left and -0.9 cm

uint8_t t1;
uint8_t t2;
int t3;
float search_r;
float base_t1;

uint8_t claw_mode;
uint8_t object_size;
uint8_t object_colour;
bool done = false;

uint8_t searchSpd = 200;
uint8_t clawSpd = 50;
uint8_t pickupSpd = 180;
uint8_t clawState;

uint8_t offsetShoulder = 180;
uint8_t offsetElbow = 180;

const int QRD1114_PIN = A5;
const int claw_PIN = A4;

const uint8_t LDR_PIN = A3;

//thresholds - calibration
uint8_t rgbLed[] = {2, 7, 8}; //R,G,B
float rgbArray[] = {0, 0, 0};
float whiteValuesArray[] = {0, 0, 0};
float blackValuesArray[] = {0, 0, 0};
int objDetectThreshold = 700;//970;
uint8_t objDetectZ1 = 2;
uint8_t objDetectZ2 = 6;
int objDetectOffsetRL = -12;
uint8_t objDetectOffsetLR = 10;
int object_detect = objDetectThreshold + 10;
int boxLocationThreshold = 784; //785

uint8_t boxLocation; //R -> 0, L -> 1

int tempObjectSize = 510;
byte blueBrightness = 50;
int redBackground = rgbArray[0];


VarSpeedServo servo_claw;
VarSpeedServo servo_t1;
VarSpeedServo servo_t2;
VarSpeedServo servo_t3;

void search();
void objectFound(uint8_t search_t1, int search_t2, int search_t3, bool dir);
void objectFound_mirrored(uint8_t search_t1, int search_t2, int search_t3, bool dir);
uint8_t actuateClaw(uint8_t claw_mode);
void moveArm(uint8_t t1, uint8_t t2, int t3, uint8_t spd);
void inverseKinematics(float r, float z);
void depositObject();
void depositObject_mirrored();
void calibrate();
int analogRead_PIN(int PIN, uint8_t acc);
void store();
uint8_t detectRL();

void setup()
{
  Serial.begin(9600);
  Serial.print("Initiate Sequence?: ");
  while (Serial.available() == 0) {}
  String userInput = Serial.readString();
  Serial.println(userInput);
  Serial.println("Starting Program in 3, 2, 1: ");
  delay(1000);
  pinMode(QRD1114_PIN, INPUT);
  pinMode(claw_PIN, INPUT);
  servo_claw.attach(6);
  servo_t1.attach(11);
  servo_t2.attach(10);
  servo_t3.attach(9);
  pinMode(2, OUTPUT);
  pinMode(7, OUTPUT);
  pinMode(8, OUTPUT);
  pinMode(LDR_PIN, INPUT);

  calibrate();
  boxLocation = detectRL();

}

void loop()
{
  
    switch (boxLocation)
    {
      case 0:
        while (!done)
        {
          search();
        }
        store();
        while (1) {};
        break;
  
      case 1:
        while (!done)
        {
          search_mirrored();
        }
        store();
        while (1) {};
        break;
    }

}


void search() {
  search_r = 15.6;
  base_t1 = 150;
  bool dir = true;
  float bias;
  done = true;
  actuateClaw(0);
  object_detect = analogRead_PIN(QRD1114_PIN, 50);
  inverseKinematics(14.6, objDetectZ1);
  moveArm(base_t1 - 5, t2, t3, 80);
  while (search_r <= 21) {
    while (base_t1 >= 80 && base_t1 <= 150) {
      bias = asin(1.5 / search_r);
      inverseKinematics(1.5 / tan(bias) + 0.9, objDetectZ1);
      moveArm(uint8_t(base_t1 - bias * 180 / M_PI), t2, t3, searchSpd);
      object_detect = analogRead_PIN(QRD1114_PIN, 50);
      Serial.println(object_detect);
      if (object_detect <= objDetectThreshold) {
        objectFound(uint8_t(base_t1 - bias * 180 / M_PI), t2, t3, dir);
      }
      if (dir) {
        base_t1 -= 2;
      }
      else if (!dir) {
        base_t1 += 2;
      }
    }
    if (dir) {
      base_t1 = 80;
    }
    else if (!dir) {
      base_t1 = 150;
    }
    dir = !dir;
    search_r += 0.75;
    bias = asin(1.5 / search_r);
    inverseKinematics(1.5 / tan(bias) + 0.9, objDetectZ1);
    moveArm(uint8_t(base_t1 - bias * 180 / M_PI), t2, t3, 80);
    delay(100);
  }
}

void search_mirrored() {
  search_r = 15.6;
  base_t1 = 140;
  bool dir = true;
  float bias;
  done = true;
  actuateClaw(0);
  object_detect = analogRead_PIN(QRD1114_PIN, 50);
  inverseKinematics(14.6, objDetectZ1);
  moveArm(base_t1 - 5, t2, t3, 80);
  while (search_r <= 21) {
    while (base_t1 >= 80 && base_t1 <= 140) {
      bias = asin(1.5 / search_r);
      inverseKinematics(1.5 / tan(bias) + 0.9, objDetectZ1);
      moveArm(uint8_t(base_t1 - bias * 180 / M_PI), t2, t3, searchSpd);
      object_detect = analogRead_PIN(QRD1114_PIN, 50);
      Serial.println(object_detect);
      if (object_detect <= objDetectThreshold) {
        objectFound_mirrored(uint8_t(base_t1 - bias * 180 / M_PI), t2, t3, dir);
      }
      if (dir) {
        base_t1 -= 2;
      }
      else if (!dir) {
        base_t1 += 2;
      }
    }
    if (dir) {
      base_t1 = 80;
    }
    else if (!dir) {
      base_t1 = 140;
    }
    dir = !dir;
    search_r += 0.75;
    bias = asin(1.5 / search_r);
    inverseKinematics(1.5 / tan(bias) + 0.9, objDetectZ1);
    moveArm(uint8_t(base_t1 - bias * 180 / M_PI), t2, t3, 80);
    delay(100);
  }
}

void objectFound(uint8_t search_t1, int search_t2, int search_t3, bool dir) {
  actuateClaw(1);
  delay(100);
  Serial.print("Detected! -> "); Serial.println(object_detect);
  if (dir) {
    Serial.print("L-R: ");
    Serial.println(objDetectOffsetLR);
    inverseKinematics(search_r - 3, 2.4);
    moveArm(base_t1 - objDetectOffsetLR, t2, t3, pickupSpd);
    inverseKinematics(search_r - 3, 0.1);
    moveArm(base_t1 - objDetectOffsetLR, t2, t3 + 3, pickupSpd);
    delay(200);
    inverseKinematics(search_r + 4.3, 0);
    moveArm(base_t1 - objDetectOffsetLR, t2, t3 + 3, 22);
    //    object_size = actuateClaw(2);

  }
  else if (!dir) {
    Serial.print("R-L: ");
    Serial.println(objDetectOffsetRL);
    inverseKinematics(search_r - 3, 2.4);
    moveArm(base_t1 + objDetectOffsetRL, t2, t3, pickupSpd);
    inverseKinematics(search_r - 3, 0.1);
    moveArm(base_t1 + objDetectOffsetRL, t2, t3 + 3, pickupSpd);
    delay(200);
    inverseKinematics(search_r + 4.3, 0);
    moveArm(base_t1 + objDetectOffsetRL, t2, t3 + 3, 20);
    //    object_size = actuateClaw(2);
  }
  object_size = actuateClaw(2);
  delay(1000);
  Serial.println(object_size);
  if (object_size != 2) {
    determineColour();
    printColour();
    Serial.println(object_colour);
    depositObject();
    moveArm(search_t1, t2, t3, 80);
  }
  moveArm(search_t1, search_t2, search_t3, 80);
  actuateClaw(0);
  done = false;
  delay(100);
}

void objectFound_mirrored(uint8_t search_t1, int search_t2, int search_t3, bool dir) {
  actuateClaw(1);
  delay(100);
  Serial.print("Detected! -> "); Serial.println(object_detect);
  if (dir) {
    Serial.print("L-R: ");
    Serial.println(objDetectOffsetLR);
    inverseKinematics(search_r - 3, 2.4);
    moveArm(base_t1 - objDetectOffsetLR, t2, t3, pickupSpd);
    inverseKinematics(search_r - 3, 0.1);
    moveArm(base_t1 - objDetectOffsetLR, t2, t3 + 3, pickupSpd);
    delay(200);
    inverseKinematics(search_r + 4.3, 0);
    moveArm(base_t1 - objDetectOffsetLR, t2, t3 + 3, 22);
    //    object_size = actuateClaw(2);

  }
  else if (!dir) {
    Serial.print("R-L: ");
    Serial.println(objDetectOffsetRL);
    inverseKinematics(search_r - 3, 2.4);
    moveArm(base_t1 + objDetectOffsetRL, t2, t3, pickupSpd);
    inverseKinematics(search_r - 3, 0.1);
    moveArm(base_t1 + objDetectOffsetRL, t2, t3 + 3, pickupSpd);
    delay(200);
    inverseKinematics(search_r + 4.3, 0);
    moveArm(base_t1 + objDetectOffsetRL, t2, t3 + 3, 20);
    //    object_size = actuateClaw(2);
  }
  object_size = actuateClaw(2);
  delay(1000);
  Serial.println(object_size);
  if (object_size != 2) {
    inverseKinematics(search_r + 4.3, 4);
    moveArm(base_t1 + objDetectOffsetRL, t2, t3 , 20);
    delay(100);
    determineColour();
    printColour();
    Serial.println(object_colour);
    depositObject_mirrored();
    moveArm(search_t1, t2, t3, 80);
  }
  moveArm(search_t1, search_t2, search_t3, 80);
  actuateClaw(0);
  done = false;
  delay(100);
}

uint8_t actuateClaw(uint8_t claw_mode) {
  bool selectionComplete = false;
  if (claw_mode == 0) {
    servo_claw.write(90, clawSpd, true); //140
  }
  else if (claw_mode == 1) {
    servo_claw.write(0, clawSpd, true); //40
  }
  else if (claw_mode == 2) {
    servo_claw.write(70, clawSpd, true); //140
    int claw_pos_1 = analogRead_PIN(claw_PIN, 200);
    servo_claw.write(80, clawSpd, true); //140
    int claw_pos = analogRead_PIN(claw_PIN, 200);
    Serial.print("Feedback: ");
    Serial.println(claw_pos_1);
    Serial.println(claw_pos);
    if (claw_pos >= 340 && claw_pos_1 >= 300) { //420
      servo_claw.write(80, clawSpd, true);
      Serial.println("Large block detected!");
      return 1;
      selectionComplete = true;
    }
    if (!selectionComplete)
    {
      servo_claw.write(90, clawSpd, true);
      claw_pos = analogRead_PIN(claw_PIN, 300);
      Serial.print("Feedback2: ");
      Serial.println(claw_pos);
      if ((claw_pos >= 340) && (claw_pos < 450)) {
        servo_claw.write(90, clawSpd, true);
        Serial.println("Small block detected!");
        return 0;
      }
      else {
        return 2;
      }
    }
  }
}

void moveArm(uint8_t t1, uint8_t t2, int t3, uint8_t spd) {
  uint8_t  s1 = constrain(t1, 0, 178);
  uint8_t s2 = offsetShoulder - t2;
  uint8_t s3 = t3 + t2 + offsetElbow;
  s2 = constrain(s2, 80, 178);
  s3 = constrain(s3, 80, 178);
  //  Serial.println(s1);
  //  Serial.println(s2);
  //  Serial.println(s3);
  servo_t2.write(s2, spd, false);
  servo_t3.write(s3, spd, false);
  servo_t1.write(s1, spd, true);
}

void inverseKinematics(float r, float z) {
  float c_h = r - 7.65;
  float c_v = z - 5.15;
  float c_length = sqrt(sq(c_h) + sq(c_v));
  //  float c_angle = atan(abs(c_v / c_h)) * 180 / M_PI;
  //  Serial.print("c_angle -> ");
  //  Serial.println(c_angle);
  float c_angle = atan2(abs(c_v), abs(c_h)) * 180 / M_PI;
  //  Serial.print("c_angle2 -> ");
  //  Serial.println(c_angle2);
  float c = acos((128 - sq(c_length)) / 128) * 180 / M_PI;
  float phi = (180 - c) / 2;
  if (z >= 5.65) {
    t2 = (uint8_t)(phi + c_angle);
  }
  else {
    t2 = (uint8_t)(phi - c_angle);
  }
  t3 = (int)(c - 180);
}

int analogRead_PIN(int PIN, int acc) {
  long int value = 0;
  for (int i = 0; i < acc; i++) {
    value = value + analogRead(PIN);
    delay(5);
  }
  return value / acc;
}

void determineColour() {
  inverseKinematics(18, 6);
  moveArm(45, t2, t3, 25);
  delay(1000);
  for (int i = 0; i <= 2; i++) {
    digitalWrite(rgbLed[i], HIGH); //turn on the Red -> Green -> Blue Led
    delay(100);                      //delay for photoresitor
    rgbArray[i] = analogRead_PIN(LDR_PIN, 25);                 //take some number of readings
    float greyDiff = whiteValuesArray[i] - blackValuesArray[i];
    rgbArray[i] = (rgbArray[i] - blackValuesArray[i]) / (greyDiff) * 255;
    digitalWrite(rgbLed[i], LOW);
    delay(100);
  }
}


void printColour() {
  //  static int redBackground = rgbArray[0];

  rgbArray[0] = rgbArray[0];//*redScale;
  rgbArray[1] = rgbArray[1];//*greenScale;
  rgbArray[2] = rgbArray[2];//*blueScale;
  Serial.print("R = ");
  Serial.println(int(rgbArray[0]));
  Serial.print("G = ");
  Serial.println(int(rgbArray[1]));
  Serial.print("B = ");
  Serial.println(int(rgbArray[2]));

  int redRefl = int(rgbArray[0]);
  int greenRefl = int(rgbArray[1]);
  int blueRefl = int(rgbArray[2]);
  if (max(max(redRefl, greenRefl), blueRefl) == redRefl)
  {
    Serial.println("It's Red!");
    object_colour = 0;
  }

  else if (max(max(redRefl, greenRefl), blueRefl) == blueRefl && abs(redRefl - redBackground) < 100 || max(max(redRefl, greenRefl), blueRefl) ==  greenRefl && abs(redRefl - redBackground) < 150 && (greenRefl - blueRefl) <= 30)//abs(greenRefl - blueRefl) > 80)
  {
    Serial.println("It's Blue!");
    object_colour = 2;
  }

  else if (max(max(redRefl, greenRefl), blueRefl) ==  greenRefl || (max(max(redRefl, greenRefl), blueRefl) == blueRefl && abs(redRefl - redBackground) >= 100))//abs(greenRefl - blueRefl) <= 80))
  {
    Serial.println("It's Green!");
    object_colour = 1;
  }

  delay(100);
}

void calibrate() {
  Serial.println("Place white sample and wait until pulse of light is complete.");
  delay(5000);    //place white paper
  //scan the white sample.
  for (int i = 0; i <= 2; i++) {
    digitalWrite(rgbLed[i], HIGH);
    delay(100);
    whiteValuesArray[i] = analogRead_PIN(LDR_PIN, 200);
    digitalWrite(rgbLed[i], LOW);
    delay(100);
  }
  //  analogWrite(rgbLed[2], blueBrightness);
  //  delay(100);
  //  whiteValuesArray[2] = analogRead_PIN(LDR_PIN, 200);
  //  analogWrite(rgbLed[2], 0);
  //  delay(100);
  Serial.println("Place black sample and wait until pulse of light is complete");
  delay(5000);              //wait for five seconds so we can position our black sample
  //go ahead and scan, sets the colour values for red, green, and blue when exposed to black
  for (int i = 0; i <= 2; i++) {
    digitalWrite(rgbLed[i], HIGH);
    delay(100);
    blackValuesArray[i] = analogRead_PIN(LDR_PIN, 200);
    digitalWrite(rgbLed[i], LOW);
    delay(100);
  }
  //  analogWrite(rgbLed[2], blueBrightness);
  //  delay(100);
  //  blackValuesArray[2] = analogRead_PIN(LDR_PIN, 200);
  //  analogWrite(rgbLed[2], 0);
  //  delay(100);
  Serial.println("Calibration complete");
  //delay another 5 seconds to allow the operator to catch on
  delay(5000);

  for (int i = 0; i <= 2; i++) {
    digitalWrite(rgbLed[i], HIGH); //turn on the Red -> Green -> Blue Led
    delay(100);                      //delay for photoresitor
    rgbArray[i] = analogRead_PIN(LDR_PIN, 25);                 //take some number of readings
    float greyDiff = whiteValuesArray[i] - blackValuesArray[i];
    rgbArray[i] = (rgbArray[i] - blackValuesArray[i]) / (greyDiff) * 255;
    digitalWrite(rgbLed[i], LOW);
    delay(100);
  }

  redBackground = rgbArray[0];

}


void depositObject() {
  //Small Red
  if (object_size == 0 && object_colour == 0) {
    inverseKinematics(21, 8);
    moveArm(15, t2, t3, 30);
    delay(100);
    inverseKinematics(21, 5);
    moveArm(15, t2, t3, 30);
    actuateClaw(1);
    delay(100);
    inverseKinematics(21, 8);
    moveArm(15, t2, t3, 30);

  }
  //Small Green
  else if (object_size == 0 && object_colour == 1 ) {
    inverseKinematics(16.8, 8);
    moveArm(16, t2, t3, 30);
    delay(100);
    inverseKinematics(16.8, 5);
    moveArm(16, t2, t3, 30);
    actuateClaw(1);
    delay(100);
    inverseKinematics(16.8, 8);
    moveArm(16, t2, t3, 30);
  }
  //Small Blue
  else if (object_size == 0 && object_colour == 2 ) {
    inverseKinematics(12.7, 8);
    moveArm(10, t2, t3, 30);
    delay(100);
    inverseKinematics(12.7, 5);
    moveArm(10, t2, t3, 30);
    actuateClaw(1);
    delay(100);
    inverseKinematics(12.7, 8);
    moveArm(10, t2, t3, 30);
  }
  //Large Red
  else if (object_size == 1 && object_colour == 0 ) {
    inverseKinematics(22, 8);
    moveArm(5, t2, t3, 30);
    delay(100);
    inverseKinematics(22, 5);
    moveArm(5, t2, t3, 30);
    actuateClaw(1);
    delay(100);
    inverseKinematics(22, 8);
    moveArm(5, t2, t3, 30);
  }
  //Large Green
  else if (object_size == 1 && object_colour == 1 ) {
    inverseKinematics(17, 8);
    moveArm(5, t2, t3, 30);
    delay(100);
    inverseKinematics(17, 5);
    moveArm(5, t2, t3, 30);
    actuateClaw(1);
    delay(100);
    inverseKinematics(17, 8);
    moveArm(5, t2, t3, 30);
  }
  //Large Blue
  else if (object_size == 1 && object_colour == 2 ) {
    inverseKinematics(14.2, 8);
    moveArm(0, t2, t3, 30);
    delay(100);
    inverseKinematics(14.2, 5);
    moveArm(0, t2, t3, 30);
    actuateClaw(1);
    delay(100);
    inverseKinematics(14.2, 8);
    moveArm(0, t2, t3, 30);
  }
}

void depositObject_mirrored() {
  //Small Red
  if (object_size == 0 && object_colour == 0) {
    inverseKinematics(22, 8);
    moveArm(150, t2, t3, 30);
    delay(100);
    inverseKinematics(22, 5);
    moveArm(150, t2, t3, 30);
    actuateClaw(1);
    delay(100);
    inverseKinematics(22, 8);
    moveArm(150, t2, t3, 30);

  }
  //Small Green
  else if (object_size == 0 && object_colour == 1 ) {
    inverseKinematics(18, 8);
    moveArm(152, t2, t3, 30);
    delay(100);
    inverseKinematics(18, 5);
    moveArm(152, t2, t3, 30);
    actuateClaw(1);
    delay(100);
    inverseKinematics(18, 8);
    moveArm(152, t2, t3, 30);
  }
  //Small Blue
  else if (object_size == 0 && object_colour == 2 ) {
    inverseKinematics(14, 8);
    moveArm(160, t2, t3, 30);
    delay(100);
    inverseKinematics(14, 5);
    moveArm(160, t2, t3, 30);
    actuateClaw(1);
    delay(100);
    inverseKinematics(14, 8);
    moveArm(160, t2, t3, 30);
  }
  //Large Red
  else if (object_size == 1 && object_colour == 0 ) {
    inverseKinematics(22, 8);
    moveArm(160, t2, t3, 30);
    delay(100);
    inverseKinematics(22, 5);
    moveArm(160, t2, t3, 30);
    actuateClaw(1);
    delay(100);
    inverseKinematics(22, 8);
    moveArm(160, t2, t3, 30);
  }
  //Large Green
  else if (object_size == 1 && object_colour == 1 ) {
    inverseKinematics(18, 8);
    moveArm(170, t2, t3, 30);
    delay(100);
    inverseKinematics(18, 5);
    moveArm(170, t2, t3, 30);
    actuateClaw(1);
    delay(100);
    inverseKinematics(18, 8);
    moveArm(170, t2, t3, 30);
  }
  //Large Blue
  else if (object_size == 1 && object_colour == 2 ) {
    inverseKinematics(14.2, 8);
    moveArm(175, t2, t3, 30);
    delay(100);
    inverseKinematics(14.2, 5);
    moveArm(175, t2, t3, 30);
    actuateClaw(1);
    delay(100);
    inverseKinematics(14.2, 8);
    moveArm(175, t2, t3, 30);
  }
}

uint8_t detectRL() {
  bool boxFound = false;
  search_r = 15;
  base_t1 = 60;
  bool dir = true;
  float bias;
  actuateClaw(0);
  object_detect = analogRead_PIN(QRD1114_PIN, 50) + 10;
  inverseKinematics(15, objDetectZ2);
  moveArm(base_t1 - 5, t2, t3, 80);
  while (base_t1 >= 5 && base_t1 <= 60) {
    bias = asin(1.5 / search_r);
    inverseKinematics(1.5 / tan(bias) + 0.9, objDetectZ2);
    moveArm(uint8_t(base_t1 - bias * 180 / M_PI), t2, t3, searchSpd);
    object_detect = analogRead_PIN(QRD1114_PIN, 50);
    Serial.println(object_detect);
    if (object_detect <= boxLocationThreshold) {
      boxFound = true;
      Serial.println("Box on RHS!");
      moveArm(90, 90, -110, 50);
      return 0;
      //        objectFound(uint8_t(base_t1 - bias * 180 / M_PI), t2, t3, dir);
    }
    base_t1 -= 2;
  }

  if (!boxFound)
  {
    moveArm(90, 90, -110, 50);
    base_t1 = 145;
    inverseKinematics(15, objDetectZ2);
    moveArm(base_t1 + 5, t2, t3, 80);
    while (base_t1 >= 145 && base_t1 <= 180) {
      bias = asin(1.5 / search_r);
      inverseKinematics(1.5 / tan(bias) + 0.9, objDetectZ2);
      moveArm(uint8_t(base_t1 - bias * 180 / M_PI), t2, t3, searchSpd);
      object_detect = analogRead_PIN(QRD1114_PIN, 50);
      Serial.println(object_detect);
      if (object_detect <= boxLocationThreshold) {
        boxFound = true;
        Serial.println("Box on LHS!");
        moveArm(90, 90, -110, 50);
        return 1;
        //        objectFound(uint8_t(base_t1 - bias * 180 / M_PI), t2, t3, dir);
      }
      base_t1 += 2;
    }
  }
  delay(100);
  moveArm(90, 90, -110, 50);

}

void store() {
  moveArm(90, 90, -170, 50);
}
