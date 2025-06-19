#ifndef KALMAN_H
#define KALMAN_H

class KalmanFilter {
public:
  KalmanFilter(float q = 0.01, float r = 0.1, float initial_value = 0);
  float update(float measurement);

private:
  float Q, R, X, P, K;
};

#endif