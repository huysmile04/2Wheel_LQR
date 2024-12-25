#include <Wire.h>
#include <Kalman.h>
#include <SoftwareSerial.h>

SoftwareSerial mySerial(11, 12); // RX, TX

#define ToDeg 180/PI
#define ToRad PI/180
Kalman kalman;
#define factortheta PI/60
#define factorphi PI/10
int data;
int inChar;
uint32_t timerloop, timerold;
int leftpwm = 9;
int leftdir = 32;
int leftdir2 = 30;
int righpwm = 10;
int righdir = 34;  // Corrected variable name
int righdir2 = 36;
volatile long leftencoder;
volatile long righencoder;
int leftencoder_a = 2;
int leftencoder_b = 22;
int righencoder_a = 3;
int righencoder_b = 26;
double mpudata;
double accX, accZ;
float Gyro;
uint32_t timer;
uint8_t i2cData[14];
long PWML, PWMR;
float K1, K2, K3, K4, K5, K6;
bool falldown;
float theta, psi, phi;
float thetadot, psidot, phidot;
float thetaold = 0, psiold = 0, phiold = 0;

float leftvolt;
float righvolt;

float addtheta;
float addphi;

float ForwardBack;
float LeftRight;

void setup() {
  Serial.begin(19200);
  mySerial.begin(9600);
  //mySerial.begin(9600);  // Initialize mySerial for communication with ESP32
  pinMode(13, OUTPUT);
  K1 = 10;
  K2 = 15;
  K3 = 370;
  K4 = 60;
  K5 = 60;
  K6 = 0.5;
  ForwardBack = 0;
  LeftRight = 0;
  addphi = 0;
  addtheta = 0;
  pinMode(leftpwm, OUTPUT);
  pinMode(righpwm, OUTPUT);
  TCCR2B = TCCR2B & B11111000 | B00000001;
  pinMode(leftdir, OUTPUT);
  pinMode(righdir, OUTPUT);
  pinMode(leftdir2, OUTPUT);
  pinMode(righdir2, OUTPUT);
  pinMode(leftencoder_a, INPUT_PULLUP);
  pinMode(leftencoder_b, INPUT_PULLUP);
  pinMode(righencoder_a, INPUT_PULLUP);
  pinMode(righencoder_b, INPUT_PULLUP);
  attachInterrupt(0, left_isr, RISING);
  attachInterrupt(1, righ_isr, RISING);
  Wire.begin();
  TWBR = ((F_CPU / 400000L) - 16) / 2;
  i2cData[0] = 7;
  i2cData[1] = 0x00;
  i2cData[2] = 0x00;
  i2cData[3] = 0x00;
  while (i2cWrite(0x19, i2cData, 4, false));
  while (i2cWrite(0x6B, 0x01, true));
  while (i2cRead(0x75, i2cData, 1));
  if (i2cData[0] != 0x68) {
    Serial.print(F("Error reading sensor"));
    while (1);
  }
  delay(100);
  while (i2cRead(0x3B, i2cData, 6));
  accX = (int16_t)((i2cData[0] << 8) | i2cData[1]);
  accZ = (int16_t)((i2cData[4] << 8) | i2cData[5]);
  double pitch = atan2(-accX, accZ) * RAD_TO_DEG;
  kalman.setAngle(pitch);
  kalman.setQangle(0.0000085);
  kalman.setQbias(0.000005);
  kalman.setRmeasure(0.0009);
  timer = micros();
}

void loop() {
  readmpu();
  if (mySerial.available()) {
    char data = mySerial.read();
    Serial.println("Received command: " + String(data));
  }
  if ((micros() - timerloop) > 6000) {
    serialEvent2();
    theta = gettheta(leftencoder, righencoder) * ToRad;
    psi = (mpudata + 0.5) * ToRad;
    phi = getphi(leftencoder, righencoder) * ToRad;
    double dt = (float)((micros() - timerloop)) / 1000000.0;
    timerloop = micros();
    thetadot = (theta - thetaold) / dt;
    psidot = (psi) / dt;
    phidot = (phi - phiold) / dt;
    thetaold = theta;
    psiold = psi;
    phiold = phi;
    addtheta = addtheta + ForwardBack * factortheta;
    addphi = addphi + LeftRight * factorphi;
    getlqr(theta + addtheta, thetadot, psi, psidot, phi + addphi, phidot);
    motorcontrol(PWML, PWMR, (mpudata + 0.5), falldown);
    serialEvent2();
    Serial.print(theta, 6);
    Serial.print(",");
    Serial.print(psi, 6);
    Serial.print(",");
    Serial.println(phi, 6);
    // Send theta, psi, and phi to ESP32
    mySerial.print(theta, 6);
    mySerial.print(",");
    mySerial.print(psi, 6);
    mySerial.print(",");
    mySerial.println(phi, 6);
  }
}

void left_isr() {
  if (digitalRead(leftencoder_b))
    leftencoder++;
  else
    leftencoder--;
}

void righ_isr() {
  if (digitalRead(righencoder_b))
    righencoder++;
  else
    righencoder--;
}

void readmpu() {
  while (i2cRead(0x3B, i2cData, 14));
  accX = (int16_t)((i2cData[0] << 8) | i2cData[1]);
  accZ = (int16_t)((i2cData[4] << 8) | i2cData[5]);
  Gyro = (int16_t)((i2cData[10] << 8) | i2cData[11]);
  double dt = (double)(micros() - timer) / 1000000;
  timer = micros();
  double pitch = atan2(-accX, accZ) * RAD_TO_DEG;
  double Gyrorate = Gyro / 131.0;
  mpudata = kalman.getAngle(pitch, Gyrorate, dt);
  if (abs(mpudata) > 30) {
    falldown = true;
  } else {
    falldown = false;
  }
}

float gettheta(long lencoder, long rencoder) {
  float angle = 0.5 * (360 / 495) * (lencoder + rencoder);
  return angle;
}

float getphi(long lencoder, long rencoder) {
  float angle = (3.5 / 15) * (360 / 495) * (lencoder - rencoder);
  return angle;
}

void getlqr(float theta_, float thetadot_, float psi_, float psidot_, float phi_, float phidot_) {
  righvolt = K1 * theta_ + K2 * thetadot_ + K3 * psi_ + K4 * psidot_ + K5 * phi_ + K6 * phidot_;
  leftvolt = K1 * theta_ + K2 * thetadot_ + K3 * psi_ + K4 * psidot_ - K5 * phi_ - K6 * phidot_;

  PWML = map(leftvolt, -(K3 * PI) / 15, K3 * PI / 15, -120, 120);
  PWMR = map(righvolt, -(K3 * PI) / 15, K3 * PI / 15, -120, 120);
  PWML = constrain(PWML, -150, 150);
  PWMR = constrain(PWMR, -150, 150);
}

void motorcontrol(long lpwm, long rpwm, float angle, bool stopstate) {
  if (abs(angle) > 30) {
    stopandreset();
  } else {
    if (leftvolt > 0) {
      leftmotor(abs(lpwm), 1);
    } else if (leftvolt < 0) {
      leftmotor(abs(lpwm), 0);
    } else {
      stopandreset();
    }
    //
    if (righvolt > 0) {
      righmotor(abs(rpwm), 1);
    } else if (righvolt < 0) {
      righmotor(abs(rpwm), 0);
    } else {
      stopandreset();
    }
  }
}

void stopandreset() {
  analogWrite(leftpwm, 0);
  digitalWrite(leftdir, LOW);
  digitalWrite(leftdir2, LOW);
  analogWrite(righpwm, 0);
  digitalWrite(righdir, LOW);  // Use the corrected variable name
  digitalWrite(righdir2, LOW);
  leftencoder = 0;
  righencoder = 0;
  addtheta = 0;
  addphi = 0;
}

void leftmotor(uint8_t lpwm, int direct) {
  analogWrite(leftpwm, lpwm);
  if (direct == 1) {
    digitalWrite(leftdir, LOW);
    digitalWrite(leftdir2, HIGH);
  } else {
    digitalWrite(leftdir, HIGH);
    digitalWrite(leftdir2, LOW);
  }
}

void righmotor(uint8_t rpwm, int direct) {
  analogWrite(righpwm, rpwm);
  if (direct == 1) {
    digitalWrite(righdir, LOW);  // Use the corrected variable name
    digitalWrite(righdir2, HIGH);
  } else {
    digitalWrite(righdir, HIGH);  // Use the corrected variable name
    digitalWrite(righdir2, LOW);
  }
}

void serialEvent2() {
  if (mySerial.available()) {
    data = mySerial.read();
    if (data == '1') {
      Serial.print(data);
      ForwardBack = 5;
      digitalWrite(13, HIGH);
    } else {
      digitalWrite(13, LOW);
    }
    if (data == '2') {
      Serial.print(data);
      ForwardBack = 0;
      addphi = 0;
      addtheta = 0;
    }
    if (data == '3') {
      Serial.print(data);
      ForwardBack = -5;
    }
    if (data == '4') {
      Serial.print(data);
      addtheta = 0;
      ForwardBack = 0;
    }
    if (data == '5') {
      Serial.print(data);
      LeftRight = 0.05;
    }
    if (data == '6') {
      Serial.print(data);
      addphi = 0;
      LeftRight = 0;
    }
    if (data == '7') {
      Serial.print(data);
      LeftRight = -0.07;
    }
    if (data == '8') {
      Serial.print(data);
      addphi = 0;
      LeftRight = 0;
    }
  }
}
