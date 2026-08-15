#include <MPU6050.h>
#include <VL53L0X.h>

enum State {
  START, STOP
};

void initializeSensors(MPU6050 &mpu, VL53L0X &distanceSensor)
{
  ////// MPU6050

  Serial.println("Initializing MPU6050...");

  mpu.initialize();
  if (!mpu.testConnection()) {
    Serial.println("MPU6050 connection failed!");
    while(1);
  }

  // Supply your gyro and accelerometer offsets here
  // mpu.setXAccelOffset(0);
  // mpu.setYAccelOffset(0);
  // mpu.setZAccelOffset(0);
  // mpu.setXGyroOffset(0);
  // mpu.setYGyroOffset(0);
  // mpu.setZGyroOffset(0);

  mpu.CalibrateAccel();
  mpu.CalibrateGyro();

  Serial.println("MPU6050 OK!");

  ////// VL53L0X

  Serial.println("Initializing VL53L0X...");

  distanceSensor.setTimeout(50);
  if (!distanceSensor.init()) {
    Serial.println("VL53L0X initialization failed!");
    while(1);
  }
  distanceSensor.startContinuous();

  Serial.println("VL53L0X OK!");
}

void readIMU(MPU6050 &mpu, double *roll, double *pitch, double *yaw, double elapsed_time)
{
  int16_t ax, ay, az, gx, gy, gz;
  mpu.getMotion6(&ax, &ay, &az, &gx, &gy, &gz);

  double acc_angle_x = atan(  ay  / sqrt(ax * ax + az * az)) * RAD_TO_DEG;
  double acc_angle_y = atan((-ax) / sqrt(ay * ay + az * az)) * RAD_TO_DEG;

  *roll  = 0.96 * (*roll  + gx * elapsed_time) + 0.04 * acc_angle_x;
  *pitch = 0.96 * (*pitch + gy * elapsed_time) + 0.04 * acc_angle_y;
  *yaw   = *yaw + gz * elapsed_time;
}

void printSensorData(double roll, double pitch, double yaw, uint16_t altitude)
{
  Serial.println("--- SENSORS ---");

  Serial.print("ROLL: ");
  Serial.print(roll, 1);
  Serial.print("°");

  Serial.print(", PITCH: ");
  Serial.print(pitch, 1);
  Serial.print("°");

  Serial.print(", YAW: ");
  Serial.print(yaw, 1);
  Serial.print("°");

  Serial.print(", ALTITUDE: ");
  Serial.print(altitude);
  Serial.println(" mm");
}

void initializeMotorPins()
{
  pinMode(FL_PIN, OUTPUT);
  pinMode(FR_PIN, OUTPUT);
  pinMode(BL_PIN, OUTPUT);
  pinMode(BR_PIN, OUTPUT);
}

void printOutput(int fl, int fr, int bl, int br)
{
  Serial.println("--- OUTPUT ---");

  Serial.print("  FL:"); Serial.print(fl);
  Serial.print(", FR:"); Serial.print(fr);
  Serial.print(", BL:"); Serial.print(bl);
  Serial.print(", BR:"); Serial.println(br);
}

State readCommandFromSerial(State prev_state)
{
  switch (Serial.read())
  {
  case 'r':
    Serial.println("Started!");
    return State::START;

  case 'p':
    Serial.println("Stopped!");
    return State::STOP;

  default:
    return prev_state;
  }
}
