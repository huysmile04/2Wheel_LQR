// Kalman filter variables
double Q_angle = 0.0000085;  // Process noise variance for the accelerometer
double Q_bias = 0.000005;    // Process noise variance for the gyro bias
double R_measure = 0.0009;   // Measurement noise variance

double angle = 0;  // The angle calculated by the Kalman filter
double bias = 0;   // The gyro bias calculated by the Kalman filter
double rate;       // Unbiased rate calculated from the rate and the calculated bias

double P[2][2] = { { 0, 0 }, { 0, 0 } };  // Error covariance matrix

void KalmanFilter(double newAngle, double newRate, double dt) {
  // Discrete Kalman filter time update equations
  rate = newRate - bias;
  angle += dt * rate;

  // Update estimation error covariance - Project the error covariance ahead
  P[0][0] += dt * (dt * P[1][1] - P[0][1] - P[1][0] + Q_angle);
  P[0][1] -= dt * P[1][1];
  P[1][0] -= dt * P[1][1];
  P[1][1] += Q_bias * dt;

  // Discrete Kalman filter measurement update equations
  double S = P[0][0] + R_measure;  // Estimate error
  double K[2];                     // Kalman gain - This is a 2x1 vector
  K[0] = P[0][0] / S;
  K[1] = P[1][0] / S;

  double y = newAngle - angle;  // Angle difference
  angle += K[0] * y;
  bias += K[1] * y;

  // Update the error covariance
  double P00_temp = P[0][0];
  double P01_temp = P[0][1];

  P[0][0] -= K[0] * P00_temp;
  P[0][1] -= K[0] * P01_temp;
  P[1][0] -= K[1] * P00_temp;
  P[1][1] -= K[1] * P01_temp;
}
