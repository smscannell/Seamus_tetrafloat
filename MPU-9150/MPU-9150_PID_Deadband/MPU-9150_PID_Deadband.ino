// DESCRIPTION:
//
// Program Functions:
//   - Pull raw data from an MPU-9150 IMU
//   - Calculate heading (yaw) from raw magnetometer measurements
//   - Smooth the heading signal
//   - Implement a PID control law to control the heading of the IMU
//   - Output HIGH/LOW signals to indicate clockwise/anticlockwise rotation
//   - Output a pwm signal to the servo amps (490Hz)
//
// TODO:
//   - Add tilt compensation
//   - Add realtime graphing and data logging


//---------------- USER DEFINABLE VARIABLES ----------------------//

//Pin assignments
#define ClockwiseWinch 2
#define AntiClockwiseWinch 3
#define V_sig 5   // Must be pwm pin

//PID parameters
double Kp = 1, Ki = 0.1, Kd = 0, Setpoint = 0;

// Triangular wave parameters
double A = 1, TP = 0.8;



// http://www.instructables.com/id/Simple-Manual-Magnetometer-Calibration/?ALLSTEPS
// Magetometer offsets:
#define MPU9150addr 0x68
#define MXOFFSET -53
#define MYOFFSET 68
#define MZOFFSET 58

#include "Wire.h"

// I2Cdev and MPU6050 must be installed as libraries, or else the .cpp/.h files
// for both classes must be in the include path of your project
#include <I2Cdev.h>
#include "MPU6050.h"
#include "math.h"
#include <PID_v1.h>

// class default I2C address is 0x68
// specific I2C addresses may be passed as a parameter here
// AD0 low = 0x68 (default for InvenSense evaluation board)
// AD0 high = 0x69
MPU6050 accelgyro(MPU9150addr);

float V_meas, fax, fay, faz, fgx, fgy, fgz, fmx, fmy, fmz, Anti_status, Clock_status, smoothedV_meas;
boolean AccelWkg; // Accelerometer present and working?

float DelT, now = 0, f = 0, i_updown = 0;

// Setup PID
double Input, Output; //  Define variables
PID myPID(&Input, &Output, &Setpoint, Kp, Ki, Kd, DIRECT); // Specify the links and initial tuning parameters


void setup() {
  // put your setup code here, to run once:
  Wire.begin();
  //------------------------ PID Setup ----------------------------------------//
  pinMode(ClockwiseWinch, OUTPUT);
  pinMode(AntiClockwiseWinch, OUTPUT);
  pinMode(V_sig, OUTPUT);

  myPID.SetOutputLimits(-10, 10); //  Tell the PID what range of outputs to give
  myPID.SetMode(AUTOMATIC); // Turn PID on

  //------------------------ IMU Setup ---------------------------------------//

  // initialize serial communication
  // (38400 chosen because it works as well at 8MHz as it does at 16MHz, but
  // it's really up to you depending on your project)
  Serial.begin(38400);

  // initialize device
  Serial.println("Initializing I2C devices...");
  accelgyro.initialize();

  // verify connection
  Serial.println("Testing device connections...");
  AccelWkg = accelgyro.testConnection();
  Serial.println(AccelWkg ? "MPU6050 connection successful" : "MPU6050 connection failed");

}

void loop() {
  // put your main code here, to run repeatedly:
  char line[256];
  DelT = (millis() - now) / 1000;

  if (AccelWkg) {
    readAccel();
    readMag();
    // smoothing
    smoothedV_meas += (V_meas - smoothedV_meas) / 10;

    // PID
    if (abs(smoothedV_meas - Setpoint) > 2) {
      Input = smoothedV_meas;
      myPID.Compute();
    }
    
    // Generate triangular wave
    f = f + (((4 * A) * i_updown) - (2 * A)) * (DelT / TP);
    if (f <= -A)  {
      i_updown = 1;
    }
    if (f >= A) {
      i_updown = 0;
    }

    Output =+ f;


    // Relays
    if (Output > 2) {
      digitalWrite(ClockwiseWinch, HIGH);
      digitalWrite(AntiClockwiseWinch, LOW);
      Anti_status = 0;
      Clock_status = 1;
    }
    else if (Output < -2) {
      digitalWrite(ClockwiseWinch, LOW);
      digitalWrite(AntiClockwiseWinch, HIGH);
      Anti_status = 1;
      Clock_status = 0;
    }
    else  {
      digitalWrite(ClockwiseWinch, LOW);
      digitalWrite(AntiClockwiseWinch, LOW);
      Anti_status = 0;
      Clock_status = 0;
    }




    // display tab-separated accel/gyro x/y/z values
    Serial.print(V_meas); Serial.print("\t");
    Serial.print(smoothedV_meas); Serial.print("\t");
    Serial.print(abs(Output)); Serial.print("\t");
    Serial.print(Anti_status); Serial.print("\t");
    Serial.println(Clock_status);
    now = millis();
    delay(10);
  }
}

void readAccel() {
  int16_t ax, ay, az;
  int16_t gx, gy, gz;
  char line[256];
  // read raw accel/gyro measurements from device
  accelgyro.getMotion6(&ax, &ay, &az, &gx, &gy, &gz);
  // these methods (and a few others) are also available
  //accelgyro.getAcceleration(&ax, &ay, &az);
  //accelgyro.getRotation(&gx, &gy, &gz);
  // Convert accel to G
  fax = float(ax) / 16384; fay = float(ay) / 16384; faz = float(az) / 16384;

  // Convert gyro to float deg/sec
  fgx = float(gx) / 131.072; fgy = float(gy) / 131.072; fgz = float(gz) / 131.072;

}

void readMag() {
  int16_t mx, my, mz;
  accelgyro.getMag(&mx, &my, &mz);
  // Apply offets to magnetometer
  mx -= MXOFFSET; my -= MYOFFSET; mz -= MZOFFSET;
  fmx = float(mx); fmy = float(my); fmz = float(mz);

  V_meas = atan2(-fmy, fmx) * 180 / 3.14159;
  // if (V_meas < 0) V_meas += 360;
}

