#include "Kalman.h"

KalmanFilter::KalmanFilter(float q, float r, float initial_value) {
  Q = q;
  R = r;
  X = initial_value;
  P = 1;
}

float KalmanFilter::update(float measurement) {
  P = P + Q;
  K = P / (P + R);
  X = X + K * (measurement - X);
  P = (1 - K) * P;
  return X;
}