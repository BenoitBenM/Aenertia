//This file uses double loop technique to control the tilt angle.
//Tile angle controls desired angular rate, and error in angular rate controls desired angular acceleration of the motor.
//The K values are not adjusted, when adjusting, you should adjust kv first, then inner loop, then adjust the outer loop. 
//Integral terms may not be used, only tune kp and kd.

#include <Arduino.h>
#include <SPI.h>
#include <TimerInterrupt_Generic.h>
#include <Adafruit_MPU6050.h>
#include <Adafruit_Sensor.h>
#include <step.h>
#include <Kalman.h>

// The Stepper pins
const int STEPPER1_DIR_PIN  = 16;
const int STEPPER1_STEP_PIN = 17;
const int STEPPER2_DIR_PIN  = 4;
const int STEPPER2_STEP_PIN = 14;
const int STEPPER_EN_PIN    = 15; 

//ADC pins
const int ADC_CS_PIN        = 5;  
const int ADC_SCK_PIN       = 18;
const int ADC_MISO_PIN      = 19;
const int ADC_MOSI_PIN      = 23;

// Diagnostic pin for oscilloscope
const int TOGGLE_PIN        = 32;

const int PRINT_INTERVAL    = 300;
const int LOOP_INTERVAL     = 10;
const int STEPPER_INTERVAL_US = 20;

const float VREF = 4.096;
const float C = 0.98;
float theta_prev = 0;

float velocity = 0;
float velocity_target = 0;
float velocity_error = 0;
float velocity_last_error = 0;
float velocity_derivative = 0;
float velocity_integral = 0; 

float tilt_target_bias = 0.015;
float tilt_target_control = 0;
float tilt_target = 0;
float tilt_error = 0;
float tilt_last_error = 0;
float tilt_derivative = 0;
float tilt_integral = 0; 

float gyro_rate=0;
float gyro_target = 0;
float gyro_error = 0;
float gyro_last_error = 0;
float gyro_derivative = 0;
float gyro_integral = 0; 

float corrected_a = 0;
int direction;
float max_acc = 100;
float max_speed = 15;

float kp_v = 0;
float ki_v = 0;
float kd_v = 0;

float kp_o = 10;
float ki_o = 0;
float kd_o = 1;

float kp = 90;
float ki = 0;
float kd = 1;

float kv = 150;
float target_speed = 0;
float last_speed = 0;
float dt = 0;
float last_time = 0;
float command_time = 0;
bool EXECUTING = false;

//Global objects
ESP32Timer ITimer(3);
Adafruit_MPU6050 mpu;         //Default pins for I2C are SCL: IO22, SDA: IO21

step step1(STEPPER_INTERVAL_US,STEPPER1_STEP_PIN,STEPPER1_DIR_PIN );
step step2(STEPPER_INTERVAL_US,STEPPER2_STEP_PIN,STEPPER2_DIR_PIN );

KalmanFilter gyroKalman(0.01, 0.1);

//Interrupt Service Routine for motor update
//Note: ESP32 doesn't support floating point calculations in an ISR
bool TimerHandler(void * timerNo)
{
  static bool toggle = false;

  //Update the stepper motors
  step1.runStepper();
  step2.runStepper();

  //Indicate that the ISR is running
  digitalWrite(TOGGLE_PIN,toggle);  
  toggle = !toggle;
	return true;
}

uint16_t readADC(uint8_t channel) {
  uint8_t TX0 = 0x06 | (channel >> 2);  // Command Byte 0 = Start bit + single-ended mode + MSB of channel
  uint8_t tx1 = (channel & 0x03) << 6;  // Command Byte 1 = Remaining 2 bits of channel

  digitalWrite(ADC_CS_PIN, LOW); 

  SPI.transfer(TX0);                    // Send Command Byte 0
  uint8_t RX0 = SPI.transfer(TX1);      // Send Command Byte 1 and receive high byte of result
  uint8_t rx1 = SPI.transfer(0x00);     // Send dummy byte and receive low byte of result

  digitalWrite(ADC_CS_PIN, HIGH); 

  uint16_t result = ((RX0 & 0x0F) << 8) | rx1; // Combine high and low byte into 12-bit result
  return result;
}

void go_forward(float speed){
  velocity_target = speed;
}
void go_backward(float speed){
  velocity_target = -speed;
}
void stop(){
  velocity_target = 0;
}



void setup()
{
  Serial.begin(115200);
  pinMode(TOGGLE_PIN,OUTPUT);

  // Try to initialize Accelerometer/Gyroscope
  if (!mpu.begin()) {
    Serial.println("Failed to find MPU6050 chip");
    while (1) {
      delay(10);
    }
  }
  Serial.println("MPU6050 Found!");

  mpu.setAccelerometerRange(MPU6050_RANGE_2_G);
  mpu.setGyroRange(MPU6050_RANGE_250_DEG);
  mpu.setFilterBandwidth(MPU6050_BAND_44_HZ);

  //Attach motor update ISR to timer to run every STEPPER_INTERVAL_US μs
  if (!ITimer.attachInterruptInterval(STEPPER_INTERVAL_US, TimerHandler)) {
    Serial.println("Failed to start stepper interrupt");
    while (1) delay(10);
    }
  Serial.println("Initialised Interrupt for Stepper");

  //Enable the stepper motor drivers
  pinMode(STEPPER_EN_PIN,OUTPUT);
  digitalWrite(STEPPER_EN_PIN, false);

  //Set up ADC and SPI
  pinMode(ADC_CS_PIN, OUTPUT);
  digitalWrite(ADC_CS_PIN, HIGH);
  SPI.begin(ADC_SCK_PIN, ADC_MISO_PIN, ADC_MOSI_PIN, ADC_CS_PIN);

}

float complementaryFilter(float theta_a, float gyro_rate, float theta_prev, float dt, float C) {
    float theta_n = (1 - C) * theta_a + C * (gyro_rate * dt + theta_prev);
    return theta_n;
}

float clamp(float input, float max, float min){
  if (input > max) return max;
  else if (input < min) return min;
  else return input;
}

void loop()
{
  //Static variables are initialised once and then the value is remembered betweeen subsequent calls to this function
  static unsigned long printTimer = 0;  //time of the next print
  static unsigned long loopTimer = 0;   //time of the next control update
  static float tiltx = 0.0;             //current tilt angle
  
  //Run the control loop every LOOP_INTERVAL ms
  if (millis() > loopTimer) {
    loopTimer += LOOP_INTERVAL;
    
    // Fetch data from MPU6050
    sensors_event_t a, g, temp;
    mpu.getEvent(&a, &g, &temp);

    //Calculate Tilt using accelerometer and sin x = x approximation for a small tilt angle
    float theta_a = atan2(a.acceleration.z, a.acceleration.x);

    gyro_rate = gyroKalman.update(g.gyro.y)-0.01;

    // float theta_a = 0;
    // float gyro_rate = 0;

    velocity = (step1.getSpeedRad()-step2.getSpeedRad())/2;    

    dt = (millis()-last_time)/1000.0;
    float theta_n = complementaryFilter(theta_a, gyro_rate, theta_prev, dt, C);

    theta_prev = theta_n;
    tiltx = theta_n;
    last_time = millis();

    velocity_error = velocity_target-velocity;
    velocity_derivative = (velocity_error - velocity_last_error)/dt;
    velocity_integral = velocity_integral + velocity_error * dt;
    velocity_last_error = velocity_error;
    velocity_integral = clamp(velocity_integral, 10, -10);

    tilt_target_control = kp_v * velocity_error + kd_v * velocity_derivative;
    tilt_target = tilt_target_control + tilt_target_bias;
    //tilt_target = clamp(velocity_integral, 0.0087, -0.0087);

    //tilt error calculation
    tilt_error = tilt_target - tiltx;
    tilt_derivative = (tilt_error - tilt_last_error) / dt;
    tilt_integral = tilt_integral + tilt_error * dt;
    tilt_last_error = tilt_error;
    tilt_integral = clamp(tilt_integral, 100, -100);

    //controller for target angular velocity of the bot
    gyro_target = kp_o*tilt_error + ki_o*tilt_integral + kd_o*tilt_derivative;

    //gyro error calculation
    gyro_error = gyro_target - gyro_rate;
    gyro_derivative = (gyro_error - gyro_last_error)/ dt;
    gyro_integral = gyro_integral + gyro_error * dt;
    gyro_last_error = gyro_error;
    gyro_integral = clamp(gyro_integral, 100, -100);

    //controller for target acceleration of the wheels
    corrected_a = kp*gyro_error + ki*gyro_integral + kd*gyro_derivative;
    corrected_a = clamp(corrected_a, max_acc, -max_acc);

    //calculation for target wheel angular velocity
    target_speed = 0.7 * kv*gyro_rate + 0.3*last_speed;
    target_speed = clamp(target_speed, max_speed, -max_speed);

    last_speed = target_speed;
    
    //set wheel target speed
    step1.setTargetSpeedRad(target_speed);
    step2.setTargetSpeedRad(-target_speed);

    //set wheel acceleration
    step1.setAccelerationRad(corrected_a);
    step2.setAccelerationRad(corrected_a);
  }

  
  //Print updates every PRINT_INTERVAL ms
  //Line format: X-axis tilt, Motor speed, A0 Voltage
  if (millis() > printTimer) {
    printTimer += PRINT_INTERVAL;
    Serial.print(tiltx*1000);
    Serial.print(' ');
    Serial.print(corrected_a);
    Serial.print(' ');
    Serial.print(step1.getSpeedRad());
    Serial.print(' ');
    Serial.print((readADC(0) * VREF)/4095.0);
    Serial.print(' ');
    Serial.print(gyro_rate);
    Serial.print(' ');
    Serial.print("Tilt_target: ");
    Serial.println(tilt_target);
  }
}