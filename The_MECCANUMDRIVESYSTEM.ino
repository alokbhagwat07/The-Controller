#include <Bluepad32.h>
#include <Wire.h>
#include <Adafruit_MPU6050.h>
#include <Adafruit_Sensor.h>

#define PWM1_PIN 12
#define DIR1_PIN 27
#define PWM2_PIN 14
#define DIR2_PIN 26
#define PWM3_PIN 19
#define DIR3_PIN 18
#define PWM4_PIN 17
#define DIR4_PIN 4

#define DEG_TO_RAD (PI / 180.0)


Adafruit_MPU6050 mpu;

// Gyroscope and accelerometer offsets

float gyroZOffset = 0;

// PID constants
float Kp = 0;  // Proportional gain
float Ki = 0;  // Integral gain
float Kd = 0;
float I, P, D;  // Derivative gain

//bool controllerConnected = false;

// PID variables
float previousErrorZ = 0;
float integralZ = 0;
float previousTime = 0;
float PID_Z = 0;
float setpoint = 0;
float ref;
float setpoint1 = 0;
;  // Desired angle (target angle)
float errorZ;
// Motor PWM pins and directions
float motorSpeed1, motorSpeed2, motorSpeed3, motorSpeed4;
float V1, V2, V3, V4;  // Motor speed values
float res;
float theta;
// float a, b, c, d;
float a = 0;
float b = 0;

// float LX = 0, LY = 0, RX = 0;
// Complementary filter variables
float angleZ = 0;
float angleZ1 = 0;
float gyroZ = 0;

//bool currentbuttonState=0;
int L1 = 0;
int R1 = 0;
int L2 = 0;
int R2 = 0;
int LB1 = 0;
int LB2 = 0;
int LB3 = 0;
int LB4 = 0;
int LB5 = 0;
float g1;
float g2;
float g3;
float g4;


int B = 0;
int Y = 0;
int X = 0;
int flagB = 0;
int flagX = 0;
int flagY = 0;
float z = 0;
int tem = 0;
int count;
// z=flagY;


bool controllerConnected = false;

float axisLX, axisLY, axisRX;

ControllerPtr myControllers[BP32_MAX_GAMEPADS];

void onConnectedController(ControllerPtr ctl) {
  for (int i = 0; i < BP32_MAX_GAMEPADS; i++) {
    if (myControllers[i] == nullptr) {
      Serial.printf("CALLBACK: Controller connected, index=%d\n", i);
      myControllers[i] = ctl;
      controllerConnected = true;  // Set the flag when a controller is connected
      return;
    }
  }
  Serial.print("CALLBACK: No empty slot for new controller.");
}

void onDisconnectedController(ControllerPtr ctl) {
  for (int i = 0; i < BP32_MAX_GAMEPADS; i++) {
    if (myControllers[i] == ctl) {
      Serial.printf("CALLBACK: Controller disconnected from index=%d\n", i);
      myControllers[i] = nullptr;
      controllerConnected = false;  // Set the flag when the controller is disconnected
      return;
    }
  }
  Serial.print("CALLBACK: Controller disconnected, but not found.");
}

void setup() {
  Serial.begin(115200);

  // Initialize motor pins
  pinMode(PWM1_PIN, OUTPUT);
  pinMode(DIR1_PIN, OUTPUT);
  pinMode(PWM2_PIN, OUTPUT);
  pinMode(DIR2_PIN, OUTPUT);
  pinMode(PWM3_PIN, OUTPUT);
  pinMode(DIR3_PIN, OUTPUT);
  pinMode(PWM4_PIN, OUTPUT);
  pinMode(DIR4_PIN, OUTPUT);

  // Attach motors to PWM channels
  ledcAttachPin(PWM1_PIN, 0);
  ledcAttachPin(PWM2_PIN, 1);
  ledcAttachPin(PWM3_PIN, 2);
  ledcAttachPin(PWM4_PIN, 3);
  ledcSetup(0, 1000, 8);  // 1 kHz, 8-bit resolution
  ledcSetup(1, 1000, 8);
  ledcSetup(2, 1000, 8);
  ledcSetup(3, 1000, 8);

  // Initialize Bluepad32 Bluetooth
  Serial.printf("Firmware: %s\n", BP32.firmwareVersion());
  const uint8_t* addr = BP32.localBdAddress();
  Serial.printf("BD Addr: %02X:%02X:%02X:%02X:%02X:%02X\n", addr[0], addr[1], addr[2], addr[3], addr[4], addr[5]);

  BP32.setup(&onConnectedController, &onDisconnectedController);
  BP32.forgetBluetoothKeys();
  BP32.enableVirtualDevice(false);

  // Initialize MPU6050
  if (!mpu.begin()) {
    Serial.println("Failed to find MPU6050 chip. Check connections.");
    while (1)
      ;
  }
  Serial.println("MPU6050 Found!");

  // Set ranges and filter for MPU6050
  mpu.setAccelerometerRange(MPU6050_RANGE_2_G);
  mpu.setGyroRange(MPU6050_RANGE_250_DEG);
  mpu.setFilterBandwidth(MPU6050_BAND_21_HZ);

  //calibrateGyroscope();
  calibrateGyroscope();
}

void loop() {
  // Default joystick values
  float LX = 0, LY = 0, RX = 0;
  bool dataUpdated = BP32.update();

  if (controllerConnected) {
    for (auto myController : myControllers) {
      if (myController && myController->isConnected()) {
        // Access joystick values
        axisLX = myController->axisX();   // Left stick X-axis for lateral (sideways) movement
        axisLY = myController->axisY();   // Left stick Y-axis for forward/backward movement
        axisRX = myController->axisRX();  // Right stick X-axis for rotation

        int buttons = myController->buttons();  // Detect button press (L1 and R1)

        // Extract the individual button states
        L1 = (buttons & 0x0010) ? 1 : 0;
        R1 = (buttons & 0x0020) ? 1 : 0;
        L2 = (buttons & 0x0040) ? 1 : 0;
        R2 = (buttons & 0x0080) ? 1 : 0;

        Y = (buttons & 0x0008) ? 1 : 0;

        // Map joystick values to desired speed range (-127 to 127)
        LX = map(axisLX, -512, 512, -127, 127);
        LY = map(axisLY, -512, 512, -127, 127);
        RX = map(axisRX, -512, 512, -127, 127);

        // Apply dead zone (ignore small joystick movements)
        if (LX > -25 && LX < 25) LX = 0;
        if (LY > -25 && LY < 25) LY = 0;
        if (RX > -25 && RX < 25) RX = 0;
      }
    }
  }

  // body to field vice cersa u
  if (Y == 1 && flagY == 0) {
    flagY = 1;
    count = !count;
  }
  if (Y == 0 && flagY == 1) {
    flagY = 0;
  }
  if (count) {
      V1 = res * (sin((theta - 45 - angleZ1) * (3.14 / 180)));   // Front-left motor
  V2 = res * (sin((135 - theta + angleZ1) * (3.14 / 180)));  // Front-right motor
  V3 = res * (sin((theta - 45 - angleZ1) * (3.14 / 180)));   // Rear-left motor
  V4 = res * (sin((135 - theta + angleZ1) * (3.14 / 180)));  // Rear-right motor
  }
  if (!count) {
      V1 = res * (sin((theta - 45 ) * (3.14 / 180)));   // Front-left motor
  V2 = res * (sin((135 - theta ) * (3.14 / 180)));  // Front-right motor
  V3 = res * (sin((theta - 45 ) * (3.14 / 180)));   // Rear-left motor
  V4 = res * (sin((135 - theta ) * (3.14 / 180)));  // Rear-right motor
  }
  
  // Field-Centric Transformation


  // Mecanum drive equations with field-centric adjustments
  res = sqrt(LX * LX + LY * LY);
  theta = atan2(-LY, LX);
  theta = theta * (180 / 3.14);
  // Calculate the angle for movement direction


  // Get accelerometer and gyroscope data
  sensors_event_t accel, gyro, temp;
  mpu.getEvent(&accel, &gyro, &temp);

  // Update gyroZ value and apply calibration offset
  gyroZ = gyro.gyro.z - gyroZOffset;
  angleZ += gyroZ * 0.01;  // Assume 100Hz update rate, so 0.01 sec delta

  // Calculate PID control for the Z-axis angle (rotation)
  errorZ = setpoint - angleZ;
  P = errorZ;
  I += errorZ;
  D = (errorZ - previousErrorZ);
  previousErrorZ = errorZ;

  if (errorZ > -0.60 && errorZ < 0.60) {
    Kp = 120;
    Ki = 0;
    Kd = 140;
  } else {
    Kp = 24.7;
    Ki = 0;
    Kd = 100;
  }

  PID_Z = Kp * P + Ki * I + Kd * D;

  // Mecanum drive equations with PID correction


  g1 = V1 - PID_Z;
  g2 = V2 + PID_Z;

  g3 = V3 + PID_Z;
  g4 = V4 - PID_Z;


  motorSpeed1 = constrain(map(-g1, -127, 127, -255, 255), -255, 255);
  motorSpeed2 = constrain(map(-g2, -127, 127, -255, 255), -255, 255);
  motorSpeed3 = constrain(map(g3, -127, 127, -255, 255), -255, 255);
  motorSpeed4 = constrain(map(g4, -127, 127, -255, 255), -255, 255);

  // Control Motors
  digitalWrite(DIR1_PIN, (motorSpeed1 > 0) ? LOW : HIGH);
  ledcWrite(0, abs(motorSpeed1));

  digitalWrite(DIR2_PIN, (motorSpeed2 > 0) ? HIGH : LOW);
  ledcWrite(1, abs(motorSpeed2));

  digitalWrite(DIR3_PIN, (motorSpeed3 > 0) ? HIGH : LOW);
  ledcWrite(2, abs(motorSpeed3));

  digitalWrite(DIR4_PIN, (motorSpeed4 > 0) ? HIGH : LOW);
  ledcWrite(3, abs(motorSpeed4));




  // Handle button presses for setpoint adjustment
if (L1 && !LB1) {
      setpoint += 4.40;
      angleZ1+=90;
    }
    LB1 = L1;

    if (R1 && !LB2) {
      setpoint -= 4.40;
      angleZ1-=90;
    }
    LB2 = R1;



  // Debug output
  Serial.print(" LX: ");
  Serial.print(LX);
  // Serial.print(" V1: ");
  // Serial.print(V1);
  Serial.print(" Y: ");
  Serial.print(Y);
  // Serial.print(" V3: ");
  // Serial.print(V3);
  // Serial.print(" V4: ");
  // // 
  Serial.print(" yaw: ");
  Serial.print(angleZ);
  Serial.print(" angleZ1: ");
  Serial.print(angleZ1);
  Serial.print(" flagY: ");
  Serial.print(flagY);
  Serial.print(" count: ");
  Serial.print(count);
  Serial.println();
}
void calibrateGyroscope() {
  float gyroZSum = 0;
  const int samples = 100;
  for (int i = 0; i < samples; i++) {
    sensors_event_t accel, gyro, temp;
    mpu.getEvent(&accel, &gyro, &temp);
    gyroZSum += gyro.gyro.z;
    // Small delay between readings
  }
  gyroZOffset = gyroZSum / samples;  // Average value to offset gyro
}