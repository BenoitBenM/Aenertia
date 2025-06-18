#include <Arduino.h>

class step {

public:

    const int MAX_SPEED = 10000;
    const int MAX_SPEED_INTERVAL_US = 1000;
    const int SPEED_SCALE = 2000;
    const int MICROSTEPS = 16;
    const int STEPS = 200;
    const float STEP_ANGLE = (2.0 * PI)/(STEPS * MICROSTEPS);
    int32_t accel = 0;
    int32_t tSpeed = 0;

    step(int i, int8_t sp, int8_t dp) : interval(i), stepPin(sp), dirPin(dp) {
        pinMode(stepPin, OUTPUT);
        pinMode(dirPin, OUTPUT);
    }

    void runStepper(){
        speedTimer += interval;

        //Check for stepping active
        if (step_period != 0) {

            stepTimer += interval;

            if (stepTimer > step_period) {

                digitalWrite(stepPin, HIGH);

                stepTimer -= step_period;

                updateSpeed();

                digitalWrite(dirPin, speed > 0);

                position += (speed > 0) ? 1 : -1;

                digitalWrite(stepPin, LOW);
            }

        } else {
            stepTimer = 0;
        }

        if (speedTimer > MAX_SPEED_INTERVAL_US)
            updateSpeed();

    }

    void setAccelerationRad(float accelRad){
        accel = static_cast<int>(accelRad / STEP_ANGLE);
    }

    void setAcceleration(int newAccel){
        accel = newAccel;
    }

    void setTargetSpeedRad(float speedRad){
        tSpeed = static_cast<int>(speedRad * SPEED_SCALE / STEP_ANGLE);
    }

    void setTargetSpeed(int speed){
        tSpeed = speed;
    }

    int getPosition() {
        return position;
    }

    float getPositionRad() {
        return static_cast<float>(position) * STEP_ANGLE;
    }

    float getSpeed() {
        return speed;
    }

    float getSpeedRad() {
        return static_cast<float>(speed) * STEP_ANGLE / SPEED_SCALE;
    }

    private:

    int32_t stepTimer = 0;
    int32_t speedTimer = 0;
    int32_t step_period = 0;
    int32_t position = 0;
    int8_t stepPin;
    int8_t dirPin;
    int32_t speed = 0;
    int32_t interval;

    void updateSpeed(){

        if (accel < 0) accel = -accel;

        if (speed < tSpeed){
            speed += accel * speedTimer / (1000000/SPEED_SCALE);
            if (speed > tSpeed){
                speed = tSpeed;
            }
            if (speed > MAX_SPEED * SPEED_SCALE) {
                speed = MAX_SPEED * SPEED_SCALE;
            }
        }
        else {
            speed -= accel * speedTimer / (1000000/SPEED_SCALE);
            if (speed < tSpeed){
                speed = tSpeed;
            }
            if (speed < -MAX_SPEED * SPEED_SCALE) {
                speed = -MAX_SPEED * SPEED_SCALE;
            }
        }

        speedTimer = 0;

        if (speed == 0)
            step_period = 0;
        else if (speed > 0)
            step_period = 1000000 * SPEED_SCALE / speed;
        else
            step_period = -1000000 * SPEED_SCALE / speed;
    }

};