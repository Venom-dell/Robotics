#include "CytronMotorDriver.h"

#include <MPU6050.h>
#include <math.h>

#include <usbhid.h>
#include <hiduniversal.h>
#include <usbhub.h>
#include <SPI.h>
#include <Wire.h>
#include "hidjoystickrptparser.h"

#define RAD45 0.785f

USB Usb;
USBHub Hub(&Usb);
HIDUniversal Hid(&Usb);

JoystickEvents JoyEvents;
JoystickReportParser Joy(&JoyEvents);

MPU6050 mpu;

CytronMD m1(PWM_DIR, 2, 22); 
CytronMD m2(PWM_DIR, 4, 39);
CytronMD m3(PWM_DIR, 44, 23);
CytronMD m4(PWM_DIR, 13, 28);

unsigned long timer = 0;
float timeStep = 0.0032;

float yaw = 0, setPoint = 0, last_error = 0, integral = 0, pid = 0;
float kp, ki = 0, kd;

void setup()
{
  Serial.begin(115200);

#if !defined(__MIPSEL__)
  while (!Serial); // Wait for serial port to connect - used on Leonardo, Teensy and other boards with built-in USB CDC serial connection
#endif
  Serial.println("Start");

  while (Usb.Init() == -1)
    Serial.println("OSC did not start.");

  delay(200);

  if (!Hid.SetReportParser(0, &Joy))
    ErrorMessage<uint8_t > (PSTR("SetReportParser"), 1);

  while(!mpu.begin(MPU6050_SCALE_2000DPS, MPU6050_RANGE_2G))
  {
    Serial.println("Could not find a valid MPU6050 sensor, check wiring!");
    delay(500);
  }
  Serial.print("new");
  // Calibrate gyroscope. The calibration must be at rest.
  // If you don't want calibrate, comment this line.
  mpu.calibrateGyro();

  // Set threshold sensivty. Default 3.
  // If you don't want use threshold, comment this line or set 0.
  mpu.setThreshold(1);
}

static float z = 0;

void loop()
{
  Usb.Task();

  float lx = JoyEvents.lx;        //  map( JoyEvents.lx, 0, 127,0, 1023);      //map(JoyEvents.Y, 0, 0xFF, 0.f, 255.f);
  float ly = JoyEvents.ly;           //map(JoyEvents.ly, 0, 127, 0, 1023);                   // map(JoyEvents.Z1, 0, 0xFF, 0.f, 255.f);
  //Group initialize
  float L2 = Joy.lt;
  float R2 = Joy.rt;

  lx = (fabs(lx - 128) > 60) ? (lx - 128) : 0;
  ly = (fabs(ly - 128) > 60) ? (ly - 128) : 0;
  ly = -ly;
  float w = constrain((L2 - R2) / 255, -1, 1) * 45;

  Vector norm = mpu.readNormalizeGyro();
  // Calculate Pitch, Roll and Yaw
  z = 0.1 * norm.ZAxis + 0.9 * z;
  yaw = yaw + z * timeStep;
  // Serial.print(norm.ZAxis);
  Serial.print("Yaw = ");
  Serial.print(yaw);
  // Serial.print(" ");
  // Serial.print(lx);
  // Serial.print(" ");
  // Serial.print(ly);
  // Serial.print(" ");
  // Serial.print(w);
  // Serial.println();

  

  // Serial.print(" ");
  // Serial.print(setPoint);
  // Serial.println();

  float error = setPoint - yaw;
  integral += error * timeStep;
  float derivative = (error - last_error) / timeStep;

  if (fabs(error) <= 6.0) {
    kp = 25.0;
    kd = 5.0;
  } else{
    kp = 15.0;
    kd = 2.5;
  }
  float pid = kp * error + ki * integral + kd * derivative;

  Serial.print(" ");
  Serial.print(pid);
  Serial.println();
  last_error = error;

  int rot = pid * 0.5;

  float theta = atan2f(ly, lx);
  float power = sqrtf(lx * lx + ly * ly) * 1.7;

  int v1 = power * sinf(theta - RAD45) + rot;
  int v2 = power * sinf(theta + RAD45) - rot;
  int v3 = power * sinf(theta - RAD45) - rot;
  int v4 = power * sinf(theta + RAD45) + rot;

  int s1 = constrain(v1, -255, 255);
  int s2 = constrain(v2, -255, 255);
  int s3 = constrain(v3, -255, 255);
  int s4 = constrain(v4, -255, 255);

  // Serial.print(s1);
  // Serial.print(" ");
  // Serial.print(s2);
  // Serial.print(" ");
  // Serial.print(s3);
  // Serial.print(" ");
  // Serial.print(s4);
  // Serial.println();

  m1.setSpeed(-s1);
  m2.setSpeed(s2);
  m3.setSpeed(s3);
  m4.setSpeed(-s4);
}
