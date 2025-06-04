#include <Arduino.h>
#include <SPI.h>
#include <TimerInterrupt_Generic.h>
#include <Adafruit_MPU6050.h>
#include <Adafruit_Sensor.h>
#include <step.h>
#include <ArduinoJson.h>
#include <MPU9250.h>
#include <Wire.h>

enum RobotCommand {
  STOP,
  FORWARD,
  BACKWARD,
  LEFT,
  RIGHT,
  FORWARD_AND_LEFT,
  FORWARD_AND_RIGHT,
  BACKWARD_AND_LEFT,
  BACKWARD_AND_RIGHT
};

RobotCommand currentCommand = STOP;

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
const int PM_INTERVAL   = 1000;
const int LOOP_INTERVAL     = 10;
const int STEPPER_INTERVAL_US = 20;
const int SERIAL_INTERVAL = 500;

const float kx = 5.0;
const float VREF = 4.096;

// PID parameters
float kp_i = 3000;
float ki_i = 0;
float kd_i = 65;

float kp_o = -0.004;
float ki_o = 0;
float kd_o = 0.0;

float kp_turn = 0;  
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
float tiltTargetBias = 0.06;
float tiltError = 0;
float tiltIntegtal = 0.0;
float tiltDerivative = 0;
float lastTiltError = 0.0;
float lastTiltTarget = 0;

float currentYaw = 0;
float lastYaw = 0;
float yawError = 0;
float yawDerivative = 0;
float yawIntegral = 0;
float lastTurnError = 0;

float turnOutput = 0;
float acceleration = 0;

float motorCommand=0.0;
float dt=0.0;
int commandTimer = 0;

bool DEBUG = false;
bool PM = false;

float initialHeading = 0;


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
  uint8_t tx0 = 0x06 | (channel >> 2);  // Command Byte 0 = Start bit + single-ended mode + MSB of channel
  uint8_t tx1 = (channel & 0x03) << 6;  // Command Byte 1 = Remaining 2 bits of channel

  digitalWrite(ADC_CS_PIN, LOW); 
  SPI.transfer(tx0);                    // Send Command Byte 0
  uint8_t rx0 = SPI.transfer(tx1);      // Send Command Byte 1 and receive high byte of result
  uint8_t rx1 = SPI.transfer(0x00);     // Send dummy byte and receive low byte of result

  digitalWrite(ADC_CS_PIN, HIGH); 
  uint16_t result = ((rx0 & 0x0F) << 8) | rx1; // Combine high and low byte into 12-bit result
  return result;
}


void setup()
{
  Serial.begin(115200);
  Wire.begin(21,22);
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
    // if(now<5000){
    //   targetSpeed=0;
    //   targetYaw=0;
    // }else if (now<9000){
    //   //targetSpeed=8;
    //   targetYaw=0.1;
    // }else if(now<13000){
    //   //targetSpeed=-8;
    //   targetYaw=-0.1;
    // }else{
    //   targetSpeed=0;
    //   targetYaw=0;
    // }

    if (millis() - commandTimer > 1000) {
      currentCommand = STOP;
    }

    // if(now<5000){
    //   currentCommand = STOP;
    //   commandTimer = millis();
    // }else if (now<7000){
    //   currentCommand = FORWARD;
    //   commandTimer = millis();
    // }else if(now<9000){
    //   currentCommand = STOP;
    //   commandTimer = millis();
    // }else if(now < 11000){
    //   currentCommand = LEFT;
    //   commandTimer = millis();
    // }else if(now < 13000){
    //   currentCommand = RIGHT;
    //   commandTimer = millis();
    // }else{
    //   currentCommand = STOP;
    //   commandTimer = millis();
    // }

    // Translate command state into target values
    switch (currentCommand) {
      case FORWARD:
        targetSpeed = speed_max;
        targetYaw = 0;
        break;
      case BACKWARD:
        targetSpeed = -speed_max;
        targetYaw = 0;
        break;
      case LEFT:
        targetSpeed = 0;
        targetYaw = yaw_max;
        break;
      case RIGHT:
        targetSpeed = 0;
        targetYaw = -yaw_max;
        break;
      case FORWARD_AND_LEFT:
        targetSpeed = speed_max;
        targetYaw = yaw_max;
        break;
      case FORWARD_AND_RIGHT:
        targetSpeed = speed_max;
        targetYaw = -yaw_max;
        break;
      case BACKWARD_AND_LEFT:
        targetSpeed = -speed_max;
        targetYaw = -yaw_max;
        break;
      case BACKWARD_AND_RIGHT:
        targetSpeed = -speed_max;
        targetYaw = yaw_max;
        break;
      case STOP:
      default:
        targetSpeed = 0;
        targetYaw = 0;
        break;
    }

    


    targetSpeed = 0.5*lastTargetSpeed + 0.5*targetSpeed;
    lastTargetSpeed = targetSpeed;
    targetYaw = 0.7*lastTargetYaw + 0.3*targetYaw;
    lastTargetYaw = targetYaw;

    dt = (now - lastTime) / 1000.0;
    lastTime = now;
    // Fetch data from MPU6050
    sensors_event_t a, g, temp;
    mpu.getEvent(&a, &g, &temp);
    tiltx_raw = atan2(a.acceleration.z, sqrt(a.acceleration.x * a.acceleration.x + a.acceleration.y * a.acceleration.y));
    gyroRate = g.gyro.y-0.005;
    tiltx =(C * (tiltx + gyroRate * dt) + (1 - C) * tiltx_raw);
    currentYaw = g.gyro.z-0.002;
    currentYaw = 0.7*currentYaw + 0.3*lastYaw;
    lastYaw = currentYaw;


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
    tiltTarget = constrain(tiltTarget, tiltTargetBias-0.035, tiltTargetBias+0.035);
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
    if(abs(yawError) < 0.01){
      yawError = 0;
    }
    
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

    if(millis()-commandTimer > 1000){
      currentCommand = STOP;
    }
  }

  if (millis() > serialTimer && PM){
    serialTimer += SERIAL_INTERVAL;
    float voltage = ((readADC(2) * VREF) / 4095.0)*6.1;
    float current_motor = ((readADC(0) * VREF) / 4095.0-0.08)/1.5;
    float current_board = ((readADC(1) * VREF) / 4095.0-0.08)/1.5;

    // Create a JSON object
    StaticJsonDocument<128> doc;
    doc["voltage"] = voltage;
    doc["current_motor"] = current_motor;
    doc["current_board"] = current_board;

    // Serialize JSON to a string
    String output;
    serializeJson(doc, output);

    // Send JSON over custom serial
    String finalMessage = "PM: " + output + "\n"; //Identifier
    Serial2.println(finalMessage);
    Serial.println(finalMessage);
    
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
    Serial.print(gyroRate,4);
    // Serial.print(" dt: ");
    // Serial.print(dt, 4);
    // Serial.println(" deA/s");
    Serial.print(" | tiltx: ");
    Serial.print(tiltx);  // 原本单位不清，现在你要的话可以换成角度显示
    Serial.print(" | tiltTarget: ");
    Serial.print(tiltTarget-tiltTargetBias);
    Serial.print(" | error: ");
    Serial.print(tiltTarget - tiltx);
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
    Serial.print(currentYaw,6);
    // Serial.print(" | position1: ");
    // Serial.print(step1.getPosition());
    // Serial.print(" | position2: ");
    // Serial.print(step2.getPosition());
    // Serial.print(" | millis: ");
    // Serial.print(millis());
    // Serial.print(" | ADC(A0): ");
    // Serial.println(((readADC(0) * VREF) / 4095.0-0.21)/1.5);
    Serial.print(" | TargetYaw: ");
    Serial.print(targetYaw);

    Serial.print(" | Command: ");
    Serial.println(currentCommand);
  }

  if (Serial.available()) {
    String command = Serial.readStringUntil('\n');
    command.trim();
    Serial.println(command);

    if(command == "forward") {
      currentCommand = FORWARD;
    }
    else if (command == "backward") {
      currentCommand = BACKWARD;
    }
    else if (command == "left") {
      currentCommand = LEFT;
    }
    else if (command == "right") {
      currentCommand = RIGHT;
    }
    else if (command == "forwardANDleft") {
      currentCommand = FORWARD_AND_LEFT;
    }
    else if (command == "forwardANDright") {
      currentCommand = FORWARD_AND_RIGHT;
    }
    else if (command == "backwardANDleft") {
      currentCommand = BACKWARD_AND_LEFT;
    }
    else if (command == "backwardANDright") {
      currentCommand = BACKWARD_AND_RIGHT;
    }
    else if (command == "stop") {
      currentCommand = STOP;
    }

    commandTimer = millis();
  }

}