#include <Arduino.h>
#include <SPI.h>
#include <TimerInterrupt_Generic.h>
#include <Adafruit_MPU6050.h>
#include <Adafruit_Sensor.h>
#include <step.h>
#include <ArduinoJson.h>

#define TXD2 25
#define RXD2 26

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
const int PRINT_INTERVAL    = 500;
const int SERIAL_INTERVAL   = 1000;
const int LOOP_INTERVAL     = 10;
const int STEPPER_INTERVAL_US = 20;

const float kx = 5.0;
const float VREF = 4.096;

// PID parameters
float kp_i = 2000;
float ki_i = 0;
float kd_i = 80;

float kp_o = -0.003;
float ki_o = -0.00007;
float kd_o = 0.0;

float kp_turn = -15.0;  
float ki_turn = 0.0;
float kd_turn = 0.0;

float targetYaw = 0;  // 希望的角度，或者从遥控器获得的转动指令

// PID 状态变量


float lastAcceleration=0.0;

float speed_max = 8;
float yaw_max = 0.1;

float targetSpeed = 0;
float actualSpeed = 0;
float speedError = 0;
float speedIntegral = 0;
float speedDerivative = 0;
float lastTargetSpeed = 0;
float lastSpeedError = 0.0;
float lastTargetYaw = 0.0;

float gyroRate = 0;
float tiltx_raw = 0;
float tiltTarget = 0;
float tiltTargetBias = 0;
float tiltError = 0;
float tiltIntegtal = 0.0;
float tiltDerivative = 0;
float lastTiltError = 0.0;
float lastTiltTarget = 0;

float currentYaw= 0;
float yawError = 0;
float yawDerivative = 0;
float yawIntegral = 0;
float lastTurnError = 0;

float turnOutput = 0;
float acceleration = 0;

float motorCommand=0.0;
float dt=0.0;

bool DEBUG = false;



float C = 0.98;

//Global objects
ESP32Timer ITimer(3);
Adafruit_MPU6050 mpu;         //Default pins for I2C are SCL: IO22, SDA: IO21

step step1(STEPPER_INTERVAL_US,STEPPER1_STEP_PIN,STEPPER1_DIR_PIN );
step step2(STEPPER_INTERVAL_US,STEPPER2_STEP_PIN,STEPPER2_DIR_PIN );


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
  uint8_t txByte1 = (channel & 0x03) << 6;  // Command Byte 1 = Remaining 2 bits of channel

  digitalWrite(ADC_CS_PIN, LOW); 
   SPI.transfer(TX0);                    // Send Command Byte 0
  uint8_t RX0 = SPI.transfer(TX1);      // Send Command Byte 1 and receive high byte of result
  uint8_t rxByte1 = SPI.transfer(0x00);     // Send dummy byte and receive low byte of result

  digitalWrite(ADC_CS_PIN, HIGH); 
  uint16_t result = ((RX0 & 0x0F) << 8) | rxByte1; // Combine high and low byte into 12-bit result
  return result;
}

void setup()
{
  Serial.begin(115200);
  Serial2.begin(115200, SERIAL_8N1, TXD2, RXD2);
  pinMode(TOGGLE_PIN,OUTPUT);

  // Try to initialize Accelerometer/Gyroscope
  /*if (!mpu.begin()) {
    Serial.println("Failed to find MPU6050 chip");
    while (1) {
      delay(10);
    }
  }
  
  Serial.println("MPU6050 Found!");

  mpu.setAccelerometerRange(MPU6050_RANGE_2_G);
  mpu.setGyroRange(MPU6050_RANGE_250_DEG);
  mpu.setFilterBandwidth(MPU6050_BAND_44_HZ);
  */

  //Attach motor update ISR to timer to run every STEPPER_INTERVAL_US μs
  if (!ITimer.attachInterruptInterval(STEPPER_INTERVAL_US, TimerHandler)) {
    Serial.println("Failed to start stepper interrupt");
    while (1) delay(10);
    }
  Serial.println("Initialised Interrupt for Stepper");

  //Set motor acceleration values
 
  //Enable the stepper motor drivers
  pinMode(STEPPER_EN_PIN,OUTPUT);
  digitalWrite(STEPPER_EN_PIN, false);

  //Set up ADC and SPI
  pinMode(ADC_CS_PIN, OUTPUT);
  digitalWrite(ADC_CS_PIN, HIGH);
  SPI.begin(ADC_SCK_PIN, ADC_MISO_PIN, ADC_MOSI_PIN, ADC_CS_PIN);

}

void loop()
{
    //Static variables are initialised once and then the value is remembered betweeen subsequent calls to this function
    static unsigned long printTimer = 0;  //time of the next print
    static unsigned long serialTimer = 0;
    static unsigned long loopTimer = 0;   //time of the next control update
    static float tiltx = 0.0;             //current tilt angle
    
    //Run the control loop every LOOP_INTERVAL ms
    
  if (millis() > loopTimer) {
    loopTimer += LOOP_INTERVAL;
    static unsigned long lastTime = 0;
    unsigned long now = millis();

    // Simulate Controller Behavior
    if(now<5000){
      targetSpeed=0;
      targetYaw=0;
    }else if (now<9000){
      //targetSpeed=8;
      targetYaw=0.1;
    }else if(now<13000){
      //targetSpeed=-8;
      targetYaw=-0.1;
    }else{
      targetSpeed=0;
      targetYaw=0;
    }

    targetSpeed = 0.9*lastTargetSpeed + 0.1*targetSpeed;
    lastTargetSpeed = targetSpeed;
    targetYaw = 0.8*lastTargetYaw + 0.2*targetYaw;
    lastTargetYaw = targetYaw;


    dt = (now - lastTime) / 1000.0;
    lastTime = now;
    // Fetch data from MPU6050
    sensors_event_t a, g, temp;
    // mpu.getEvent(&a, &g, &temp);
    // tiltx_raw = atan2(a.acceleration.z, sqrt(a.acceleration.x * a.acceleration.x + a.acceleration.y * a.acceleration.y));
    // gyroRate = g.gyro.y-0.005;
    // tiltx =(C * (tiltx + gyroRate * dt) + (1 - C) * tiltx_raw);
    // currentYaw = g.gyro.z+0.06;

    // Calculate Elements of the Speed Error
    actualSpeed = (step2.getSpeedRad()-step1.getSpeedRad())/2;
    speedError = targetSpeed - actualSpeed;
    speedIntegral += speedError * dt;
    speedIntegral = constrain(speedIntegral, -100, 100);
    speedDerivative = (speedError - lastSpeedError) / dt;
    lastSpeedError = speedError;

    // PID calculate target angle
    tiltTarget= (kp_o * speedError + ki_o * speedIntegral + kd_o * speedDerivative)+tiltTargetBias;
    tiltTarget = 0.7*tiltTarget + 0.3*lastTiltTarget;
    tiltTarget = constrain(tiltTarget, -0.035, 0.035);
    lastTiltTarget=tiltTarget;
    
    // Calcualte Elememnts for Tilt Error
    tiltError = tiltTarget - tiltx;
    tiltIntegtal += tiltError * dt;
    tiltIntegtal = constrain(tiltIntegtal, -100, 100);
    tiltDerivative = (tiltError - lastTiltError) / dt;
    lastTiltError = tiltError;

    // PID calculate target acceleration
    acceleration = (kp_i * tiltError + ki_i * tiltIntegtal + kd_i * tiltDerivative);
    acceleration= acceleration*0.7+lastAcceleration*0.3;
    acceleration = constrain(acceleration, -80, 80);
    lastAcceleration = acceleration;

    // Calculate Elements for Yaw Error
    yawError = targetYaw - currentYaw;
    yawIntegral += yawError * dt;
    yawDerivative = (yawError - lastTurnError) / dt;
    turnOutput = kp_turn * yawError + ki_turn * yawIntegral + kd_turn * yawDerivative;
    lastTurnError = yawError;
    if(turnOutput < 0.01 && turnOutput > -0.01){
        turnOutput = 0;
    }

    step1.setAccelerationRad(abs(acceleration+turnOutput));
    step2.setAccelerationRad(abs(acceleration-turnOutput));
    // step1.setAccelerationRad(abs(acceleration));
    // step2.setAccelerationRad(abs(acceleration));
    if(acceleration+turnOutput>0){
      step1.setTargetSpeedRad(-(20));
    }else {
        step1.setTargetSpeedRad((20));
    }
    if(acceleration-turnOutput>0){
      step2.setTargetSpeedRad((20));
    }else {
        step2.setTargetSpeedRad(-(20));
    }
        
  }
  if (millis() > serialTimer){
    serialTimer += SERIAL_INTERVAL;
    float voltage = readADC(2);
    float current_motor = ((readADC(0) * VREF) / 4095.0-0.21)/1.5;
    float current_board = ((readADC(1) * VREF) / 4095.0-0.21)/1.5;

    // Create a JSON object
    StaticJsonDocument<128> doc;
    doc["voltage"] = voltage;
    doc["current_motor"] = current_motor;
    doc["current_board"] = current_board;

    // Serialize JSON to a string
    String output;
    serializeJson(doc, output);

    // Send JSON over custom serial
    String finalMessage = "PM:" + output + "\n"; //Identifier
    Serial.println(finalMessage);

    // For debug on USB serial
    //Serial.println(finalMessage);
  }
  
  //Print updates every PRINT_INTERVAL ms
  //Line format: X-axis tilt, Motor speed, A0 Voltage
  if (millis() > printTimer && DEBUG) {
      //if (millis() > printTimer) {
    printTimer += PRINT_INTERVAL;
    // Serial.print("ACC Angle: ");
    // Serial.print(tiltx_raw);
    // Serial.print(" deg\t");
    Serial.print("GYRO Rate: ");
    Serial.print(gyroRate);
    // Serial.print(" dt: ");
    // Serial.print(dt, 4);
    // Serial.println(" deA/s");
    Serial.print(" | tiltx: ");
    Serial.print(tiltx);  // 原本单位不清，现在你要的话可以换成角度显示
    // Serial.print(" | tiltTarget: ");
    // Serial.print(tiltTarget-tiltTargetBias);
    // Serial.print(" | error: ");
    // Serial.print(tiltTarget - tiltx);
    // Serial.print(" | output: ");
    // Serial.print(acceleration);
    // Serial.print(" | turnOutput: ");
    // Serial.print(turnOutput);
  //   Serial.print(" | motorSpeed1: ");
  //   Serial.print(step1.getSpeedRad());
  //    Serial.print(" | motorSpeed2: ");
  //   Serial.print(step2.getSpeedRad());
    Serial.print(" | targetSpeed: ");
    Serial.print(targetSpeed);
    Serial.print(" |gyro.z: ");
    Serial.print(currentYaw);
    // Serial.print(" | position1: ");
    // Serial.print(step1.getPosition());
    // Serial.print(" | position2: ");
    // Serial.print(step2.getPosition());
    //Serial.print(" | millis: ");
    //Serial.print(millis());
    Serial.print(" | ADC(A0): ");
    Serial.println(((readADC(0) * VREF) / 4095.0-0.21)/1.5);
  //   Serial.print(" | TargetYaw: ");
  //   Serial.println(targetYaw);
  }

  if(Serial2.available()) {
    String command = Serial2.readStringUntil('\n');
    command.trim();
  
    // We start by checking if the mode was changed
    
    if(command == "forward") {
      targetSpeed = speed_max;
      targetYaw = 0;
      Serial.println("forward");
    }
    else if (command == "backward") {
      targetSpeed = -speed_max;
      targetYaw = 0;
      Serial.println("backward");
    }
    else if (command == "left") {
      targetSpeed = 0;
      targetYaw = yaw_max;
      Serial.println("left");
    }
    else if (command == "right") {
      targetSpeed = 0;
      targetYaw = -yaw_max;
      Serial.println("right");
    }

    else if(command == "forwardANDleft") {
      targetSpeed = speed_max;
      targetYaw = yaw_max;
      Serial.println("Forward and left");

    }
    else if (command == "forwardANDright") {
      targetSpeed = speed_max;
      targetYaw = -yaw_max;
      Serial.println("Forward and right");
    }
    else if (command == "backwardANDleft") {
      targetSpeed = -speed_max;
      targetYaw = -yaw_max;
      Serial.println("Backward and left");
    }
    else if (command == "backwardANDright") {
      targetSpeed = -speed_max;
      targetYaw = yaw_max;
      Serial.println("Backward and right");
    }
    else if (command == "stop") {
      targetSpeed = 0;
      targetYaw = 0;
      Serial.println("Stop");
    }
  
  }
}