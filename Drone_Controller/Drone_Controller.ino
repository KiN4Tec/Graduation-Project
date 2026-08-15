#include <Wire.h>

#include "PIDController.hpp"

// Motor Pins
#define FL_PIN 5
#define FR_PIN 9
#define BL_PIN 6
#define BR_PIN 3

#include "IO.hpp"

MPU6050 mpu;
VL53L0X distanceSensor;

PIDController x_controller(3.3, 0.030, 23.0, -25.0, 25.0);
PIDController y_controller(3.3, 0.030, 23.0, -25.0, 25.0);
PIDController z_controller(6.8, 0.045, 0.0, -15.0, 15.0);
PIDController altitude_controller(6.4, 0.025, 24.0, -255.0, 255.0);

State state = START;

double roll = 0.0, pitch = 0.0, yaw = 0.0, altitude = 0.0;
unsigned long prev_time = 0;
unsigned long time_since_last_print = 0;

void setup()
{
  ////// Initialize Connections

  Serial.begin(9600);
  Wire.begin();
  initializeMotorPins();

  Serial.println("\n--- STARTING ---");
  delay(2000);

  initializeSensors(mpu, distanceSensor);

  ////// Start Control

  prev_time = micros();

  x_controller.set_point = 0.0;
  y_controller.set_point = 0.0;
  z_controller.set_point = 0.0;
  altitude_controller.set_point = 0.300;
}

void loop()
{
  ////// Read commands from Bluetooth

  if (Serial.available()) state = readCommandFromSerial(state);

  ////// Read Time

  unsigned long current_time = micros();
  double elapsed_time = (double)(abs(current_time - prev_time)) / 1000000.0;

  ////// Read Sensors

  readIMU(mpu, &roll, &pitch, &yaw, elapsed_time);
  double altitude = (double)(distanceSensor.readRangeContinuousMillimeters()) / 1000.0;

  if (state == State::START)
  {
    ////// Start Control

    double output_x = x_controller.control(roll, elapsed_time);
    double output_y = y_controller.control(pitch, elapsed_time);
    double output_z = z_controller.control(yaw, elapsed_time);
    double output_altitude = altitude_controller.control(altitude, elapsed_time);

    ////// Write Output

    int fl_out = constrain((int)(+ output_x + output_y + output_z + output_altitude), 0, 255);
    int fr_out = constrain((int)(- output_x + output_y - output_z + output_altitude), 0, 255);
    int bl_out = constrain((int)(+ output_x - output_y - output_z + output_altitude), 0, 255);
    int br_out = constrain((int)(- output_x - output_y + output_z + output_altitude), 0, 255);

    analogWrite(FL_PIN, fl_out);
    analogWrite(FR_PIN, fr_out);
    analogWrite(BL_PIN, bl_out);
    analogWrite(BR_PIN, br_out);

    if (abs(current_time - time_since_last_print) > 2000000)
      printOutput(fl_out, fr_out, bl_out, br_out);
  }

  ////// Print Sensor Data

  if (abs(current_time - time_since_last_print) > 2000000) {
    printSensorData(roll, pitch, yaw, altitude);
    time_since_last_print = current_time;
  }

  prev_time = current_time;
}
