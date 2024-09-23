#include <Wire.h>
#include <Kalman.h>  //https://github.com/TKJElectronics/KalmanFilter
#define ToDeg 180 / PI
#define ToRad PI / 180

Kalman kalman;  //Kalman filter define: kalman.getAngle(pitch, gyrorate, dt);

#define factortheta PI / 60  //60 The theta setpoint value change ever 7ms if control
#define factorphi PI / 10    //10 The Phi setpoint value change ever 7ms if control

int blue;
int inChar;
uint32_t timerloop, timerold;

//Motor control Pin//
int leftpwm = 9;    //Control PWM left motor// ena
int leftdir = 32;   //Control direction left motor IN1
int leftdir2 = 30;  //Control direction left motor IN2

int righpwm = 10;   //Control PWM right motor ,ENB
int righdir = 36;   //Control direction right motor, IN4
int righdir2 = 34;  //Control direction right motor  IN3

volatile long leftencoder;  //Read left encoder value
volatile long righencoder;  //Read right encoder value
// INITIALIZE ENCODER PIN
int leftencoder_a = 2;   //Read state encoder channel LOW or HIGH A encoder1
int leftencoder_b = 22;  // B encoder1
int righencoder_a = 3;   // A encoder 2
int righencoder_b = 26;  // B encoder 2

//MPU6050 Data//
double mpudata;  //Save psi angle (Y axis)
double accX, accZ;
float Gyro;

uint32_t timer;  //Timer for kalman filter psi angle;
uint8_t i2cData[14];

//LQR data//
long PWML, PWMR;               //PWM output for H-Brigde
float K1, K2, K3, K4, K5, K6;  //The factor of K maxtrix
bool falldown;                 //Run = true; Stop = false;

float theta, psi, phi;
float thetadot, psidot, phidot;
float thetaold = 0, psiold = 0, phiold = 0;

float leftvolt;  //output volt left motor in LQR
float righvolt;  //output volt right motor in LQR

float addtheta;  //Save setpoint value
float addphi;    //Save setpoint value

float ForwardBack;  // 1 -> Forward;   -1 -> Back;      0 -> Stop And Balancing
float LeftRight;    // 1 -> Turnleft;  -1 -> TurnRight  0 -> Stop And Balancing

/////////////////////////////////////////////////
///////////    SERIAL BEGIN   ///////////////////
/////////////////////////////////////////////////
void setup() {
  Serial.begin(115200);
  // Serial.begin(9600); //Serial connect to blutooth send data to pc and plot graph

  /*
    //Set factor of K matrix
    K = [k1 k2 k3 k4 k5 k6
      k1 k2 k3 k4 -k5 -k6]
  */
  // Initialize Parameters LQR Controller.
  //  DDRB |= 1 << PB4;
  //  DDRH |= 1 << PH6;
  K1 = 10;   //k1*theta - goc toi   10-50
  K2 = 15;   //k2*thetadot - toc do di chuyen toi (8-15)
  K3 = 370;  //k3*psi - goc nghieng      (lớn thì chạy mượt) (100-300)
  K4 = 60;   //k4*psidot - van toc goc nghieng  (10-20)
  K5 = 60;   //k5*phi  - goc quay  (10-15)
  K6 = 0.5;  //k6*phidot - vantoc goc quay (0-5)
  ForwardBack = 0;
  LeftRight = 0;
  addphi = 0;
  addtheta = 0;

  //K1 = 30;
  /// K2 = 25;
  //K3 = 80;
  //  K4 = 10;
  // K5 = 0.7;
  //  K6 = 2.7;

  pinMode(leftpwm, OUTPUT);
  pinMode(righpwm, OUTPUT);
  //Set PWM frequency 23429.4Hz f_PWM = 12e6/(510*N) = 23429.4Hz
  // Phase Correct PWM
  // Fast PWM
  //  TCCR2A |= (1 << WGM21) | (1 << WGM20);
  TCCR2B = TCCR2B & B11111000 | B00000001;  //Pin 9 & Pin 10
  //  Timer1.initialize(10000);//10us
  //  Timer1.attachInterrupt(NGAT);
  //Output pin control motor left and right//
  pinMode(leftdir, OUTPUT);
  pinMode(righdir, OUTPUT);
  pinMode(leftdir2, OUTPUT);
  pinMode(righdir2, OUTPUT);

  //Input pin read encorder//
  pinMode(leftencoder_a, INPUT_PULLUP);
  pinMode(leftencoder_b, INPUT_PULLUP);
  pinMode(righencoder_a, INPUT_PULLUP);
  pinMode(righencoder_b, INPUT_PULLUP);

  //interrupt encoder//
  attachInterrupt(0, left_isr, RISING);
  attachInterrupt(1, righ_isr, RISING);

  //Data MPU6050//
  Wire.begin();

  TWBR = ((F_CPU / 400000L) - 16) / 2;  // Set I2C frequency to 400kHz
  i2cData[0] = 7;
  i2cData[1] = 0x00;
  i2cData[2] = 0x00;
  i2cData[3] = 0x00;
  while (i2cWrite(0x19, i2cData, 4, false))
    ;
  while (i2cWrite(0x6B, 0x01, true))
    ;
  while (i2cRead(0x75, i2cData, 1))
    ;
  if (i2cData[0] != 0x68) {
    Serial.print(F("Error reading sensor"));
    while (1)
      ;
  }

  delay(100);

  while (i2cRead(0x3B, i2cData, 6))
    ;
  accX = (int16_t)((i2cData[0] << 8) | i2cData[1]);
  accZ = (int16_t)((i2cData[4] << 8) | i2cData[5]);

  double pitch = atan2(-accX, accZ) * RAD_TO_DEG;
  kalman.setAngle(pitch);
  kalman.setQangle(0.0000085);
  kalman.setQbias(0.000005);
  kalman.setRmeasure(0.0009);
  timer = micros();
}

//////////////////////////////////
//       MAIN PROGRAMMING       //
//////////////////////////////////
void loop() {
  readmpu();
  if ((micros() - timerloop) > 6000) {  //Set time loop update and control motor
    serialEvent1();
    theta = gettheta(leftencoder, righencoder) * ToRad;  //Read theta value and convert to Rad
    psi = (mpudata + 0.5) * ToRad;                       //Read psi value and convert to Rad   (goc ban dau)
    phi = getphi(leftencoder, righencoder) * ToRad;      //Read phi value and convert to Rad
    //Update time compare with timeloop
    double dt = (float)((micros() - timerloop)) / 1000000.0;
    timerloop = micros();
    //Update input angle value
    thetadot = (theta - thetaold) / dt;
    psidot = (psi) / dt;
    phidot = (phi - phiold) / dt;
    //Update old angle value
    thetaold = theta;
    psiold = psi;
    phiold = phi;

    addtheta = addtheta + ForwardBack * factortheta;
    addphi = addphi + LeftRight * factorphi;

    getlqr(theta + addtheta, thetadot, psi, psidot, phi + addphi, phidot);
    motorcontrol(PWML, PWMR, (mpudata + 0.5), falldown);
    //Send data to serial


    Serial.print("Goc_psi: ");
    Serial.println(psi * ToDeg);

    Serial.print(" ");
    // Serial.print(90);
    // Serial.print(" ");
    // Serial.print(-90);
    // Serial.println(" ");
    serialEvent1();
  }
}


//left motor encoder interrupt//
void left_isr() {
  //if (digitalRead(leftencoder_b) == HIGH) leftencoder++;
  if (digitalRead(leftencoder_b))
    leftencoder++;
  else leftencoder--;
}

//right motor encoder interrupt//
void righ_isr() {
  //if (digitalRead(righencoder_b) == HIGH) righencoder++;
  if (digitalRead(righencoder_b))
    righencoder++;
  else righencoder--;
}

//Read psi//
void readmpu() {
  while (i2cRead(0x3B, i2cData, 14))
    ;
  accX = (int16_t)((i2cData[0] << 8) | i2cData[1]);
  accZ = (int16_t)((i2cData[4] << 8) | i2cData[5]);
  Gyro = (int16_t)((i2cData[10] << 8) | i2cData[11]);

  double dt = (double)(micros() - timer) / 1000000;
  timer = micros();
  double pitch = atan2(-accX, accZ) * RAD_TO_DEG;
  double Gyrorate = Gyro / 131.0;
  mpudata = kalman.getAngle(pitch, Gyrorate, dt);  //- 0.5;

  if (abs(mpudata) > 30) {
    falldown = true;
  } else {
    falldown = false;
  }
  //
}

//Read theta angle function
float gettheta(long lencoder, long rencoder) {
  float angle = 0.5 * (360 / 495) * (lencoder + rencoder);
  return angle;
}

//Read phi angle function//
float getphi(long lencoder, long rencoder) {
  float angle = (3.5 / 15) * (360 / 495) * (lencoder - rencoder);
  return angle;
}

//LQR function
void getlqr(float theta_, float thetadot_, float psi_, float psidot_, float phi_, float phidot_) {
  righvolt = K1 * theta_ + K2 * thetadot_ + K3 * psi_ + K4 * psidot_ + K5 * phi_ + K6 * phidot_;
  leftvolt = K1 * theta_ + K2 * thetadot_ + K3 * psi_ + K4 * psidot_ - K5 * phi_ - K6 * phidot_;

  PWML = map(leftvolt, -(K3 * PI) / 15, K3 * PI / 15, -120, 120);  //Limit 15 deg.
  PWMR = map(righvolt, -(K3 * PI) / 15, K3 * PI / 15, -120, 120);

  PWML = constrain(PWML, -142, 142);
  PWMR = constrain(PWMR, -142, 142);
}

//Motor control function
void motorcontrol(long lpwm, long rpwm, float angle, bool stopstate) {
  if (abs(angle) > 30)  //angle psi > 30 motor will stop
  {
    stopandreset();
  } else {
    if (leftvolt > 0) {
      leftmotor(abs(lpwm), 1);  //Forward
    } else if (leftvolt < 0) {
      leftmotor(abs(lpwm), 0);  //Back
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

//Stop motor and reset data
void stopandreset()  //The data angle and encoder will be reset back to zero.
{
  analogWrite(leftpwm, 0);
  digitalWrite(leftdir, LOW);
  digitalWrite(leftdir2, LOW);
  analogWrite(righpwm, 0);
  digitalWrite(righdir, LOW);
  digitalWrite(righdir2, LOW);
  //Reset default place
  leftencoder = 0;
  righencoder = 0;
  addtheta = 0;
  addphi = 0;
}
//Control left motor
void leftmotor(uint8_t lpwm, int direct) {
  analogWrite(leftpwm, lpwm);
  if (direct == 1) {  //angle > 0
    digitalWrite(leftdir, LOW);
    digitalWrite(leftdir2, HIGH);
  } else {
    digitalWrite(leftdir, HIGH);
    digitalWrite(leftdir2, LOW);
  }
}
//Control right motor
void righmotor(uint8_t rpwm, int direct) {
  analogWrite(righpwm, rpwm);
  if (direct == 1) {  //angle > 0
    digitalWrite(righdir, LOW);
    digitalWrite(righdir2, HIGH);
  } else {
    digitalWrite(righdir, HIGH);
    digitalWrite(righdir2, LOW);
  }
}
void serialEvent1() {
  if (Serial.available() > 0) {
    blue = Serial.read();
    //Control motor forward
    if (blue == 1)  //F
    {
      Serial.print(blue);

      ForwardBack = 0.3;
    }
    if (blue == 2) {
      Serial.print(blue);


      ForwardBack = 0;
    }
    //Control motor Back
    if (blue == 3)  //B
    {

      Serial.print(blue);

      ForwardBack = -0.3;
    }
    if (blue == 4) {
      Serial.print(blue);

      addtheta = 0;
      ForwardBack = 0;
    }
    //Control motor Left
    if (blue == 5)  //L
    {

      Serial.print(blue);

      LeftRight = 0.05;
    }
    if (blue == 6) {
      Serial.print(blue);

      addphi = 0;
      LeftRight = 0;
    }
    //Control motor right
    if (blue == 7)  //R
    {

      Serial.print(blue);

      LeftRight = -0.07;
    }
    if (blue == 8) {
      Serial.print(blue);

      addphi = 0;
      LeftRight = 0;
    }
  }
}
