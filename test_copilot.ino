#include <Wire.h>

// Khai báo chân động cơ
int enA = 9;
int enB = 10;
int motor1Pin1 = 30;
int motor1Pin2 = 32;
int motor2Pin1 = 34;
int motor2Pin2 = 36;

// Khai báo chân Encoder
const int encoder1PinA = 2;
const int encoder1PinB = 22;
const int encoder2PinA = 3;
const int encoder2PinB = 28;

// Biến Encoder
volatile long encoder1Pos = 0;
volatile long encoder2Pos = 0;
volatile bool encoder1ASet, encoder1BSet;
volatile bool encoder2ASet, encoder2BSet;

// Khai báo MPU6050
const int MPU = 0x68; 
int16_t ax, ay, az;
int16_t gx, gy, gz;

// Khai báo PID
float Kp = 30.0;
float Ki = 0.05;
float Kd = 15.0;

float setPoint = 0;
float input, output;
float lastInput = 0;
float integralTerm = 0;

void setup() {
  Serial.begin(115200);
  Wire.begin();
  setupMPU6050();

  pinMode(enA, OUTPUT);
  pinMode(enB, OUTPUT);
  pinMode(motor1Pin1, OUTPUT);
  pinMode(motor1Pin2, OUTPUT);
  pinMode(motor2Pin1, OUTPUT);
  pinMode(motor2Pin2, OUTPUT);

  pinMode(encoder1PinA, INPUT);
  pinMode(encoder1PinB, INPUT);
  pinMode(encoder2PinA, INPUT);
  pinMode(encoder2PinB, INPUT);

  attachInterrupt(digitalPinToInterrupt(encoder1PinA), doEncoder1A, CHANGE);
  attachInterrupt(digitalPinToInterrupt(encoder1PinB), doEncoder1B, CHANGE);
  attachInterrupt(digitalPinToInterrupt(encoder2PinA), doEncoder2A, CHANGE);
  attachInterrupt(digitalPinToInterrupt(encoder2PinB), doEncoder2B, CHANGE);

  // Bật động cơ
  analogWrite(enA, 255); // Tốc độ tối đa
  analogWrite(enB, 255); // Tốc độ tối đa
}

void loop() {
  // Đọc dữ liệu từ MPU6050
  readMPU6050();
  input = ax / 16384.0;  // Chuyển đổi giá trị gia tốc sang gốc độ (sơ bộ)

  // Đọc giá trị từ bộ mã hóa
  long position1 = encoder1Pos;
  long position2 = encoder2Pos;

  // Điều khiển PID
  float error = setPoint - input;
  integralTerm += Ki * error;
  float derivative = input - lastInput;
  output = Kp * error + integralTerm - Kd * derivative;
  lastInput = input;
  
  // Giới hạn đầu ra
  output = constrain(output, -255, 255);

  // Điều khiển động cơ thông qua L298N
  if (output > 0) {
    analogWrite(enA, output);
    digitalWrite(motor1Pin1, HIGH);
    digitalWrite(motor1Pin2, LOW);
    analogWrite(enB, output);
    digitalWrite(motor2Pin1, HIGH);
    digitalWrite(motor2Pin2, LOW);
  } else {
    analogWrite(enA, -output);
    digitalWrite(motor1Pin1, LOW);
    digitalWrite(motor1Pin2, HIGH);
    analogWrite(enB, -output);
    digitalWrite(motor2Pin1, LOW);
    digitalWrite(motor2Pin2, HIGH);
  }
  
  // Hiển thị dữ liệu trên Serial Monitor
  Serial.print("Input: ");
  Serial.print(input);
  Serial.print(" Output: ");
  Serial.print(output);
  Serial.print(" Position1: ");
  Serial.print(position1);
  Serial.print(" Position2: ");
  Serial.println(position2);

  delay(10); // Chu kỳ lấy mẫu
}

void setupMPU6050() {
  Wire.beginTransmission(MPU);
  Wire.write(0x6B);
  Wire.write(0);
  Wire.endTransmission(true);
}

void readMPU6050() {
  Wire.beginTransmission(MPU);
  Wire.write(0x3B);
  Wire.endTransmission(false);
  Wire.requestFrom(MPU, 14, true);

  ax = Wire.read() << 8 | Wire.read();
  ay = Wire.read() << 8 | Wire.read();
  az = Wire.read() << 8 | Wire.read();
  gx = Wire.read() << 8 | Wire.read();
  gy = Wire.read() << 8 | Wire.read();
  gz = Wire.read() << 8 | Wire.read();
}

void doEncoder1A() {
  encoder1ASet = digitalRead(encoder1PinA) == HIGH;
  encoder1BSet = digitalRead(encoder1PinB) == HIGH;
  updateEncoder(encoder1ASet, encoder1BSet, encoder1Pos);
}

void doEncoder1B() {
  encoder1ASet = digitalRead(encoder1PinA) == HIGH;
  encoder1BSet = digitalRead(encoder1PinB) == HIGH;
  updateEncoder(encoder1ASet, encoder1BSet, encoder1Pos);
}

void doEncoder2A() {
  encoder2ASet = digitalRead(encoder2PinA) == HIGH;
  encoder2BSet = digitalRead(encoder2PinB) == HIGH;
  updateEncoder(encoder2ASet, encoder2BSet, encoder2Pos);
}

void doEncoder2B() {
  encoder2ASet = digitalRead(encoder2PinA) == HIGH;
  encoder2BSet = digitalRead(encoder2PinB) == HIGH;
  updateEncoder(encoder2ASet, encoder2BSet, encoder2Pos);
}

void updateEncoder(bool A_set, bool B_set, volatile long &pos) {
  if (A_set == B_set) {
    pos++;
  } else {
    pos--;
  }
}
