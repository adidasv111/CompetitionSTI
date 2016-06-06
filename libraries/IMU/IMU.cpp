// Define output variables from updated quaternion---these are Tait-Bryan angles, commonly used in aircraft orientation.
// In this coordinate system, the positive z-axis is down toward Earth.
// Yaw is the angle between Sensor x-axis and Earth magnetic North (or true North if corrected for local declination, looking down on the sensor positive yaw is counterclockwise.
// Pitch is angle between sensor x-axis and Earth ground plane, toward the Earth is positive, up toward the sky is negative.
// Roll is angle between sensor y-axis and Earth ground plane, y-axis up is positive roll.
// These arise from the definition of the homogeneous rotation matrix constructed from quaternions.
// Tait-Bryan angles as well as Euler angles are non-commutative; that is, the get the correct orientation the rotations must be
// applied in the correct order which for this configuration is yaw, pitch, and then roll.
// For more see http://en.wikipedia.org/wiki/Conversion_between_quaternions_and_Euler_angles which has additional links.

// gyro rate to 8 kHz/(1 * rate) shows 1 kHz, accelerometer ODR is fixed at 1 KHz
// magnetometer read rate in Hz; 10 to 100 (max) Hz are reasonable values

#include "IMU.h"
int i = 0;
float IMUAngles [3];                            // yaw, pitch, roll relative to start position (public)

int MPU9150_I2C_ADDRESS = 0x68;

uint16_t count = 0;                             // used to control display output rate
//uint16_t delt_t = 0;                            // used to control display output rate
uint16_t mcount = 0;                            // used to control display output rate
uint8_t MagRate = 10;                           // read rate for magnetometer data in Hz; 10 to 100 (max) Hz are reasonable valueset

float deltat = 0.0f;                            // integration interval for both filter schemes
uint16_t lastUpdate = 0;                        // used to calculate integration interval
uint16_t now = 0;                               // used to calculate integration interval
   

float ax, ay, az, gx, gy, gz, mx, my, mz;       // variables to hold latest sensor data values
float q[4] = {1.0f, 0.0f, 0.0f, 0.0f};          // vector to hold quaternion
float eInt[3] = {0.0f, 0.0f, 0.0f};             // vector to hold integral error for Mahony method
float yaw = 0, pitch = 0, roll = 0;                         // variables to hold angles measurements
float yaw0 = 0, pitch0 = 0, roll0 = 0;                      // variables to hold initial angles measurements

float ax_off = -0.01, ay_off = 0.01, az_off = -0.03, gx_off = 1.85, gy_off = -0.01, gz_off = -7.85, mx_off = 0, my_off = 0, mz_off = 0;       // offsets for measurements
float mx_scale = 0, my_scale = 0, mz_scale = 0;

//----- I2C functions to get easier all values -----
int MPU9150_readSensor(int addrL, int addrH)
{
  Wire.beginTransmission(MPU9150_I2C_ADDRESS);
  Wire.write(addrL);
  Wire.requestFrom(MPU9150_I2C_ADDRESS, 1, true);
  byte L = Wire.read();

  Wire.beginTransmission(MPU9150_I2C_ADDRESS);
  Wire.write(addrH);
  Wire.endTransmission(false);

  Wire.requestFrom(MPU9150_I2C_ADDRESS, 1, true);
  byte H = Wire.read();

  return (int16_t)((H << 8) + L);
}

int MPU9150_readSensor(int addr)
{
  Wire.beginTransmission(MPU9150_I2C_ADDRESS);
  Wire.write(addr);
  Wire.endTransmission(false);

  Wire.requestFrom(MPU9150_I2C_ADDRESS, 1, true);
  return Wire.read();
}

int MPU9150_writeSensor(int addr, int data)
{
  Wire.beginTransmission(MPU9150_I2C_ADDRESS);
  Wire.write(addr);
  Wire.write(data);
  Wire.endTransmission(true);

  return 1;
}


//----- Auxilary Functions -----
//http://pansenti.wordpress.com/2013/03/26/pansentis-invensense-mpu-9150-software-for-arduino-is-now-on-github/
//Thank you to pansenti for setup code.
//I will documented this one later.
void MPU9150_setupCompass()
{
  MPU9150_I2C_ADDRESS = 0x0C;      //change Address to Compass

  MPU9150_writeSensor(0x0A, 0x00); //PowerDownMode
  MPU9150_writeSensor(0x0A, 0x0F); //SelfTest
  MPU9150_writeSensor(0x0A, 0x00); //PowerDownMode

  MPU9150_I2C_ADDRESS = 0x68;      //change Address to MPU

  MPU9150_writeSensor(0x24, 0x40); //Wait for Data at Slave0
  MPU9150_writeSensor(0x25, 0x8C); //Set i2c address at slave0 at 0x0C
  MPU9150_writeSensor(0x26, 0x02); //Set where reading at slave 0 starts
  MPU9150_writeSensor(0x27, 0x88); //set offset at start reading and enable
  MPU9150_writeSensor(0x28, 0x0C); //set i2c address at slv1 at 0x0C
  MPU9150_writeSensor(0x29, 0x0A); //Set where reading at slave 1 starts
  MPU9150_writeSensor(0x2A, 0x81); //Enable at set length to 1
  MPU9150_writeSensor(0x64, 0x01); //overvride register
  MPU9150_writeSensor(0x67, 0x03); //set delay rate
  MPU9150_writeSensor(0x01, 0x80);

  MPU9150_writeSensor(0x34, 0x04); //set i2c slv4 delay
  MPU9150_writeSensor(0x64, 0x00); //override register
  MPU9150_writeSensor(0x6A, 0x00); //clear usr setting
  MPU9150_writeSensor(0x64, 0x01); //override register
  MPU9150_writeSensor(0x6A, 0x20); //enable master i2c mode
  MPU9150_writeSensor(0x34, 0x13); //disable slv4
}

// Implementation of Sebastian Madgwick's "...efficient orientation filter for... inertial/magnetic sensor arrays"
// (see http://www.x-io.co.uk/category/open-source/ for examples and more details)
// which fuses acceleration, rotation rate, and magnetic moments to produce a quaternion-based estimate of absolute
// device orientation -- which can be converted to yaw, pitch, and roll. Useful for stabilizing quadcopters, etc.
// The performance of the orientation filter is at least as good as conventional Kalman-based filtering algorithms
// but is much less computationally intensive---it can be performed on a 3.3 V Pro Mini operating at 8 MHz!
void MadgwickQuaternionUpdate(float ax, float ay, float az, float gx, float gy, float gz, float mx, float my, float mz)
{
  float q1 = q[0], q2 = q[1], q3 = q[2], q4 = q[3];   // short name local variable for readability
  float norm;
  float hx, hy, _2bx, _2bz;
  float s1, s2, s3, s4;
  float qDot1, qDot2, qDot3, qDot4;

  // Auxiliary variables to avoid repeated arithmetic
  float _2q1mx;
  float _2q1my;
  float _2q1mz;
  float _2q2mx;
  float _4bx;
  float _4bz;
  float _2q1 = 2.0f * q1;
  float _2q2 = 2.0f * q2;
  float _2q3 = 2.0f * q3;
  float _2q4 = 2.0f * q4;
  float _2q1q3 = 2.0f * q1 * q3;
  float _2q3q4 = 2.0f * q3 * q4;
  float q1q1 = q1 * q1;
  float q1q2 = q1 * q2;
  float q1q3 = q1 * q3;
  float q1q4 = q1 * q4;
  float q2q2 = q2 * q2;
  float q2q3 = q2 * q3;
  float q2q4 = q2 * q4;
  float q3q3 = q3 * q3;
  float q3q4 = q3 * q4;
  float q4q4 = q4 * q4;

  // Normalise accelerometer measurement
  norm = sqrt(ax * ax + ay * ay + az * az);
  if (norm == 0.0f) return; // handle NaN
  norm = 1.0f / norm;
  ax *= norm;
  ay *= norm;
  az *= norm;

  // Normalise magnetometer measurement
  norm = sqrt(mx * mx + my * my + mz * mz);
  if (norm == 0.0f) return; // handle NaN
  norm = 1.0f / norm;
  mx *= norm;
  my *= norm;
  mz *= norm;

  // Reference direction of Earth's magnetic field
  _2q1mx = 2.0f * q1 * mx;
  _2q1my = 2.0f * q1 * my;
  _2q1mz = 2.0f * q1 * mz;
  _2q2mx = 2.0f * q2 * mx;
  hx = mx * q1q1 - _2q1my * q4 + _2q1mz * q3 + mx * q2q2 + _2q2 * my * q3 + _2q2 * mz * q4 - mx * q3q3 - mx * q4q4;
  hy = _2q1mx * q4 + my * q1q1 - _2q1mz * q2 + _2q2mx * q3 - my * q2q2 + my * q3q3 + _2q3 * mz * q4 - my * q4q4;
  _2bx = sqrt(hx * hx + hy * hy);
  _2bz = -_2q1mx * q3 + _2q1my * q2 + mz * q1q1 + _2q2mx * q4 - mz * q2q2 + _2q3 * my * q4 - mz * q3q3 + mz * q4q4;
  _4bx = 2.0f * _2bx;
  _4bz = 2.0f * _2bz;

  // Gradient decent algorithm corrective step
  s1 = -_2q3 * (2.0f * q2q4 - _2q1q3 - ax) + _2q2 * (2.0f * q1q2 + _2q3q4 - ay) - _2bz * q3 * (_2bx * (0.5f - q3q3 - q4q4) + _2bz * (q2q4 - q1q3) - mx) + (-_2bx * q4 + _2bz * q2) * (_2bx * (q2q3 - q1q4) + _2bz * (q1q2 + q3q4) - my) + _2bx * q3 * (_2bx * (q1q3 + q2q4) + _2bz * (0.5f - q2q2 - q3q3) - mz);
  s2 = _2q4 * (2.0f * q2q4 - _2q1q3 - ax) + _2q1 * (2.0f * q1q2 + _2q3q4 - ay) - 4.0f * q2 * (1.0f - 2.0f * q2q2 - 2.0f * q3q3 - az) + _2bz * q4 * (_2bx * (0.5f - q3q3 - q4q4) + _2bz * (q2q4 - q1q3) - mx) + (_2bx * q3 + _2bz * q1) * (_2bx * (q2q3 - q1q4) + _2bz * (q1q2 + q3q4) - my) + (_2bx * q4 - _4bz * q2) * (_2bx * (q1q3 + q2q4) + _2bz * (0.5f - q2q2 - q3q3) - mz);
  s3 = -_2q1 * (2.0f * q2q4 - _2q1q3 - ax) + _2q4 * (2.0f * q1q2 + _2q3q4 - ay) - 4.0f * q3 * (1.0f - 2.0f * q2q2 - 2.0f * q3q3 - az) + (-_4bx * q3 - _2bz * q1) * (_2bx * (0.5f - q3q3 - q4q4) + _2bz * (q2q4 - q1q3) - mx) + (_2bx * q2 + _2bz * q4) * (_2bx * (q2q3 - q1q4) + _2bz * (q1q2 + q3q4) - my) + (_2bx * q1 - _4bz * q3) * (_2bx * (q1q3 + q2q4) + _2bz * (0.5f - q2q2 - q3q3) - mz);
  s4 = _2q2 * (2.0f * q2q4 - _2q1q3 - ax) + _2q3 * (2.0f * q1q2 + _2q3q4 - ay) + (-_4bx * q4 + _2bz * q2) * (_2bx * (0.5f - q3q3 - q4q4) + _2bz * (q2q4 - q1q3) - mx) + (-_2bx * q1 + _2bz * q3) * (_2bx * (q2q3 - q1q4) + _2bz * (q1q2 + q3q4) - my) + _2bx * q2 * (_2bx * (q1q3 + q2q4) + _2bz * (0.5f - q2q2 - q3q3) - mz);
  norm = sqrt(s1 * s1 + s2 * s2 + s3 * s3 + s4 * s4);    // normalise step magnitude
  norm = 1.0f / norm;
  s1 *= norm;
  s2 *= norm;
  s3 *= norm;
  s4 *= norm;

  // Compute rate of change of quaternion
  qDot1 = 0.5f * (-q2 * gx - q3 * gy - q4 * gz) - beta * s1;
  qDot2 = 0.5f * (q1 * gx + q3 * gz - q4 * gy) - beta * s2;
  qDot3 = 0.5f * (q1 * gy - q2 * gz + q4 * gx) - beta * s3;
  qDot4 = 0.5f * (q1 * gz + q2 * gy - q3 * gx) - beta * s4;

  // Integrate to yield quaternion
  q1 += qDot1 * deltat;
  q2 += qDot2 * deltat;
  q3 += qDot3 * deltat;
  q4 += qDot4 * deltat;
  norm = sqrt(q1 * q1 + q2 * q2 + q3 * q3 + q4 * q4);    // normalise quaternion
  norm = 1.0f / norm;
  q[0] = q1 * norm;
  q[1] = q2 * norm;
  q[2] = q3 * norm;
  q[3] = q4 * norm;
}


// Similar to Madgwick scheme but uses proportional and integral filtering on the error between estimated reference vectors and measured ones.
void MahonyQuaternionUpdate(float ax, float ay, float az, float gx, float gy, float gz, float mx, float my, float mz)
{
  float q1 = q[0], q2 = q[1], q3 = q[2], q4 = q[3];   // short name local variable for readability
  float norm;
  float hx, hy, bx, bz;
  float vx, vy, vz, wx, wy, wz;
  float ex, ey, ez;
  float pa, pb, pc;

  // Auxiliary variables to avoid repeated arithmetic
  float q1q1 = q1 * q1;
  float q1q2 = q1 * q2;
  float q1q3 = q1 * q3;
  float q1q4 = q1 * q4;
  float q2q2 = q2 * q2;
  float q2q3 = q2 * q3;
  float q2q4 = q2 * q4;
  float q3q3 = q3 * q3;
  float q3q4 = q3 * q4;
  float q4q4 = q4 * q4;

  // Normalise accelerometer measurement
  norm = sqrt(ax *  + ay * ay + az * az);
  if (norm == 0.0f) return; // handle NaN
  norm = 1.0f / norm;        // use reciprocal for division
  ax *= norm;
  ay *= norm;
  az *= norm;

  // Normalise magnetometer measurement
  norm = sqrt(mx * mx + my * my + mz * mz);
  if (norm == 0.0f) return; // handle NaN
  norm = 1.0f / norm;        // use reciprocal for division
  mx *= norm;
  my *= norm;
  mz *= norm;

  // Reference direction of Earth's magnetic field
  hx = 2.0f * mx * (0.5f - q3q3 - q4q4) + 2.0f * my * (q2q3 - q1q4) + 2.0f * mz * (q2q4 + q1q3);
  hy = 2.0f * mx * (q2q3 + q1q4) + 2.0f * my * (0.5f - q2q2 - q4q4) + 2.0f * mz * (q3q4 - q1q2);
  bx = sqrt((hx * hx) + (hy * hy));
  bz = 2.0f * mx * (q2q4 - q1q3) + 2.0f * my * (q3q4 + q1q2) + 2.0f * mz * (0.5f - q2q2 - q3q3);

  // Estimated direction of gravity and magnetic field
  vx = 2.0f * (q2q4 - q1q3);
  vy = 2.0f * (q1q2 + q3q4);
  vz = q1q1 - q2q2 - q3q3 + q4q4;
  wx = 2.0f * bx * (0.5f - q3q3 - q4q4) + 2.0f * bz * (q2q4 - q1q3);
  wy = 2.0f * bx * (q2q3 - q1q4) + 2.0f * bz * (q1q2 + q3q4);
  wz = 2.0f * bx * (q1q3 + q2q4) + 2.0f * bz * (0.5f - q2q2 - q3q3);

  // Error is cross product between estimated direction and measured direction of gravity
  ex = (ay * vz - az * vy) + (my * wz - mz * wy);
  ey = (az * vx - ax * vz) + (mz * wx - mx * wz);
  ez = (ax * vy - ay * vx) + (mx * wy - my * wx);
  if (Ki > 0.0f)
  {
    eInt[0] += ex;      // accumulate integral error
    eInt[1] += ey;
    eInt[2] += ez;
  }
  else
  {
    eInt[0] = 0.0f;     // prevent integral wind up
    eInt[1] = 0.0f;
    eInt[2] = 0.0f;
  }

  // Apply feedback terms
  gx = gx + Kp * ex + Ki * eInt[0];
  gy = gy + Kp * ey + Ki * eInt[1];
  gz = gz + Kp * ez + Ki * eInt[2];

  // Integrate rate of change of quaternion
  pa = q2;
  pb = q3;
  pc = q4;
  q1 = q1 + (-q2 * gx - q3 * gy - q4 * gz) * (0.5f * deltat);
  q2 = pa + (q1 * gx + pb * gz - pc * gy) * (0.5f * deltat);
  q3 = pb + (q1 * gy - pa * gz + pc * gx) * (0.5f * deltat);
  q4 = pc + (q1 * gz + pa * gy - pb * gx) * (0.5f * deltat);

  // Normalise quaternion
  norm = sqrt(q1 * q1 + q2 * q2 + q3 * q3 + q4 * q4);
  norm = 1.0f / norm;
  q[0] = q1 * norm;
  q[1] = q2 * norm;
  q[2] = q3 * norm;
  q[3] = q4 * norm;
}


//----- Public Functions -----
//Read all measurements from the IMU
void readMeasurements()
{
  mcount++;
  //Temperature
  //double dT = ( (double) MPU9150_readSensor(MPU9150_TEMP_OUT_L,MPU9150_TEMP_OUT_H) + 12412.0) / 340.0;

  gx = MPU9150_readSensor(MPU9150_GYRO_XOUT_L, MPU9150_GYRO_XOUT_H);
  gy = MPU9150_readSensor(MPU9150_GYRO_YOUT_L, MPU9150_GYRO_YOUT_H);
  gz = MPU9150_readSensor(MPU9150_GYRO_ZOUT_L, MPU9150_GYRO_ZOUT_H);
  ax = MPU9150_readSensor(MPU9150_ACCEL_XOUT_L, MPU9150_ACCEL_XOUT_H);
  ay = MPU9150_readSensor(MPU9150_ACCEL_YOUT_L, MPU9150_ACCEL_YOUT_H);
  az = MPU9150_readSensor(MPU9150_ACCEL_ZOUT_L, MPU9150_ACCEL_ZOUT_H);

  ax = ax * ACCEL_RANGE / 32768.0f - ax_off; // 2g full range for gyroscope
  ay = ay * ACCEL_RANGE / 32768.0f - ay_off;
  az = az * ACCEL_RANGE / 32768.0f - az_off;
  gx = gx * GYRO_RANGE / 32768.0f - gx_off; // 250 deg/s full range for gyroscope
  gy = gy * GYRO_RANGE / 32768.0f - gy_off;
  gz = gz * GYRO_RANGE / 32768.0f - gz_off;

  if (mcount > 1000 / MagRate)  // this is a poor man's way of setting the magnetometer read rate (see below)
  {
    mx = MPU9150_readSensor(MPU9150_CMPS_XOUT_L, MPU9150_CMPS_XOUT_H);
    my = MPU9150_readSensor(MPU9150_CMPS_YOUT_L, MPU9150_CMPS_YOUT_H);
    mz = MPU9150_readSensor(MPU9150_CMPS_ZOUT_L, MPU9150_CMPS_ZOUT_H);

    mx = mx * 10.0f * 1229.0f / 4096.0f + 18.0f - mx_off; // milliGauss (1229 microTesla per 2^12 bits, 10 mG per microTesla)
    my = my * 10.0f * 1229.0f / 4096.0f + 70.0f - my_off; // apply calibration offsets in mG that correspond to your environment and magnetometer
    mz = mz * 10.0f * 1229.0f / 4096.0f + 270.0f - mz_off;
    mcount = 0;
  }
}


// Print all the latest readings
void printReadings()
{
  // Serial print at 0.5 s rate independent of data rates
//  delt_t = millis() - count;
//  if (delt_t > 500)
//  {
    Serial.print("ax = "); Serial.print(ax, 2);
    Serial.print(" ay = "); Serial.print(ay, 2);
    Serial.print(" az = "); Serial.print(az, 2); Serial.print(" g        ");
    Serial.print("gx = "); Serial.print(gx, 2);
    Serial.print(" gy = "); Serial.print(gy, 2);
    Serial.print(" gz = "); Serial.print(gz, 2); Serial.print(" deg/s      ");
    Serial.print("mx = "); Serial.print(mx, 2);
    Serial.print(" my = "); Serial.print(my, 2);
    Serial.print(" mz = "); Serial.print(mz, 2); Serial.print(" mG      ");

    Serial.print("q0 = "); Serial.print(q[0]);
    Serial.print(" qx = "); Serial.print(q[1]);
    Serial.print(" qy = "); Serial.print(q[2]);
    Serial.print(" qz = "); Serial.println(q[3]);

    Serial.print("Yaw, Pitch, Roll: ");
    Serial.print(yaw, 2);
    Serial.print(", ");
    Serial.print(pitch, 2);
    Serial.print(", ");
    Serial.println(roll, 2);

    Serial.print("IMUAngles: Yaw, Pitch, Roll: ");
    Serial.print(IMUAngles[0], 2);
    Serial.print(", ");
    Serial.print(IMUAngles[1], 2);
    Serial.print(", ");
    Serial.println(IMUAngles[2], 2);

    //Serial.print("rate = "); Serial.print((float)1.0f/deltat, 2); Serial.println(" Hz");
    Serial.println();

//    count = millis();
//  }
}


//Calibrate the Accelerometer and Gyroscope readings
void calibrateAccelGyro()
{
  //  The gyros and accelerometers can in principle be calibrated in addition to any factory calibration but they are generally
  //  pretty accurate. You can check the accelerometer by making sure the reading is +1 g in the positive direction for each axis.
  //  The gyro should read zero for each axis when the sensor is at rest. Small or zero adjustment should be needed for these sensors.
  //  The magnetometer is a different thing. Most magnetometers will be sensitive to circuit currents, computers, and
  //  other both man-made and natural sources of magnetic field. The rough way to calibrate the magnetometer is to record
  //  the maximum and minimum readings (generally achieved at the North magnetic direction). The average of the sum divided by two
  //  should provide a pretty good calibration offset.
  Serial.println("---- Calibrating Acceleremoter and Gyro ----");
  Serial.println("The IMU should be horizontal, with the chip side of the PCB upwards and do not move for the rest of the calibration");
  Serial.println("After calibration the accelerometer readings should be 0g for x and y and 1g for z.");
  Serial.println("The gyroscope readings should be all 0 deg/s.");
  Serial.println("All angle measurements are reset! Expect initial angles");

  ax_off = 0;     ay_off = 0;     az_off = 0;  gx_off = 0;     gy_off = 0;     gz_off = 0;  //Reset all offsets and recalibrate
  float ax_avg = 0, ay_avg = 0, az_avg = 0, gx_avg = 0, gy_avg = 0, gz_avg = 0;             //Hold avergare over CALIB_SAMPLES readings

  for (int i = 0; i < CALIB_SAMPLES; i++)
  {
    readMeasurements();
    ax_avg += ax;
    ay_avg += ay;
    az_avg += az;
    gx_avg += gx;
    gy_avg += gy;
    gz_avg += gz;
  }

  ax_avg /= CALIB_SAMPLES;
  ay_avg /= CALIB_SAMPLES;
  az_avg /= CALIB_SAMPLES;
  gx_avg /= CALIB_SAMPLES;
  gy_avg /= CALIB_SAMPLES;
  gz_avg /= CALIB_SAMPLES;

  ax_off = ax_avg;
  ay_off = ay_avg;
  az_off = 1 - az_avg;
  gx_off = gx_avg;
  gy_off = gy_avg;
  gz_off = gz_avg;

  yaw = 0, pitch = 0, roll = 0;             //Reset angles to neglect values calculated during calibration

  Serial.println("---- Acceleremoter and Gyro Calibration Done ----");
}

//Calibrate the Magnetometer readings
//Source: https://github.com/kriswiner/MPU-6050/wiki/Simple-and-Effective-Magnetometer-Calibration
void calibrateMagnetometer()
{
  mx_off = 0;     my_off = 0;     mz_off = 0;
  mx_scale = 0;   my_scale = 0;   mz_scale = 0;
  int32_t mag_scale[3] = {0, 0, 0};
  int16_t mag_max[3] = {0x8000, 0x8000, 0x8000}, mag_min[3] = {0x7FFF, 0x7FFF, 0x7FFF}, mag_temp[3] = {0, 0, 0};

  Serial.println("---- Calibrating Magnetometer ----");
  Serial.println("Wave device in a figure eight until done!");
  delay(5000);

  for (int ii = 0; ii < CALIB_SAMPLES; ii++)
  {
    readMeasurements();
    mag_temp[0] = mx;
    mag_temp[0] = my;
    mag_temp[0] = mz;
    for (int jj = 0; jj < 3; jj++)
    {
      if (mag_temp[jj] > mag_max[jj])
        mag_max[jj] = mag_temp[jj];
      if (mag_temp[jj] < mag_min[jj])
        mag_min[jj] = mag_temp[jj];
    }
    delay(135);  // at 8 Hz ODR, new mag data is available every 125 ms
  }

  // Get hard iron correction
  mx_off  = (mag_max[0] + mag_min[0]) / 2.0f; // get average x mag bias in counts
  my_off  = (mag_max[1] + mag_min[1]) / 2.0f; // get average y mag bias in counts
  mz_off  = (mag_max[2] + mag_min[2]) / 2.0f; // get average z mag bias in counts

  // Get soft iron correction estimate
  mag_scale[0]  = (mag_max[0] - mag_min[0]) / 2; // get average x axis max chord length in counts
  mag_scale[1]  = (mag_max[1] - mag_min[1]) / 2; // get average y axis max chord length in counts
  mag_scale[2]  = (mag_max[2] - mag_min[2]) / 2; // get average z axis max chord length in counts

  float avg_rad = mag_scale[0] + mag_scale[1] + mag_scale[2];
  avg_rad /= 3.0;

  mx_scale = avg_rad / ((float)mag_scale[0]);
  my_scale = avg_rad / ((float)mag_scale[1]);
  mz_scale = avg_rad / ((float)mag_scale[2]);

  yaw = 0, pitch = 0, roll = 0;             //Reset angles to neglect values calculated during calibration

  Serial.println("Magnetometer Calibration done!");
  Serial.println("Put robot down");
  delay(4000);
}

//Initialize the IMU. Calibration if doCalibratation is true
void initIMU(char doCalibratation)
{
  Serial.println("---- Initializing IMU ----");
  // Initialize the 'Wire' class for the I2C-bus.
  Wire.begin();

  // Clear the 'sleep' bit to start the sensor.
  MPU9150_writeSensor(MPU9150_PWR_MGMT_1, 0);

  MPU9150_setupCompass();

  if (doCalibratation == 1 || doCalibratation == 3)
  {
    calibrateAccelGyro();
  }
  if (doCalibratation == 2 || doCalibratation == 3)
  {
    calibrateMagnetometer();
  }

  for (int i = 0; i < 10*N; i++)                 //Run 10N times in order to converge to initial positions
  {
    calcAngles();
  }
  yaw0 = yaw;
  pitch0 = pitch;
  roll0 = roll;

  Serial.print("ax_off = "); Serial.print(ax_off, 2);
  Serial.print("    ay_off = "); Serial.print(ay_off, 2);
  Serial.print("    az_off = "); Serial.print(az_off, 2); Serial.print(" g        ");
  Serial.print("gx_off = "); Serial.print(gx_off, 2);
  Serial.print("    gy_off = "); Serial.print(gy_off, 2);
  Serial.print("    gz_off = "); Serial.print(gz_off, 2); Serial.println(" deg/s");
  Serial.print("mx_off = "); Serial.print(mx_off, 2);
  Serial.print("    my_off = "); Serial.print(my_off, 2);
  Serial.print("    mz_off = "); Serial.print(mz_off, 2); Serial.print(" mG      ");
  Serial.print("    mx_scale = "); Serial.print(mx_scale, 2);
  Serial.print("    my_scale = "); Serial.print(my_scale, 2);
  Serial.print("    mz_scale = "); Serial.println(mz_scale, 2);
  Serial.print("Yaw0: ");
  Serial.print(yaw0, 2);
  Serial.print(",   Pitch0: ");
  Serial.print(pitch0, 2);
  Serial.print(",   Roll0: ");
  Serial.println(roll0, 2);
  Serial.println("------------------------------------");
  Serial.println();
  delay(500);
}


// Calculate yaw-pitch-roll angles from the IMU readings
void calcAngles()
{
  readMeasurements();

  now = micros();
  deltat = ((now - lastUpdate) / 1000000.0f); // set integration time by time elapsed since last filter update
  lastUpdate = now;
  // Sensors x (y)-axis of the accelerometer is aligned with the y (x)-axis of the magnetometer;
  // the magnetometer z-axis (+ down) is opposite to z-axis (+ up) of accelerometer and gyro!
  // We have to make some allowance for this orientationmismatch in feeding the output to the quaternion filter.
  // For the MPU-9150, we have chosen a magnetic rotation that keeps the sensor forward along the x-axis just like
  // in the LSM9DS0 sensor. This rotation can be modified to allow any convenient orientation convention.
  // This is ok by aircraft orientation standards!
  // Pass gyro rate as rad/s
  MadgwickQuaternionUpdate(ax, ay, az, gx * PI / 180.0f, gy * PI / 180.0f, gz * PI / 180.0f,  my,  mx, mz);
  // MahonyQuaternionUpdate(ax, ay, az, gx*PI/180.0f, gy*PI/180.0f, gz*PI/180.0f, my, mx, mz);

  yaw   = atan2(2.0f * (q[1] * q[2] + q[0] * q[3]), q[0] * q[0] + q[1] * q[1] - q[2] * q[2] - q[3] * q[3]);
  pitch = -asin(2.0f * (q[1] * q[3] - q[0] * q[2]));
  roll  = atan2(2.0f * (q[0] * q[1] + q[2] * q[3]), q[0] * q[0] - q[1] * q[1] - q[2] * q[2] + q[3] * q[3]);
  yaw   *= 180.0f / PI - 1.68;              // Declination at Lausanne, Switzerland0 is 13 degrees 41 minutes
  pitch *= 180.0f / PI;
  roll  *= 180.0f / PI;

  IMUAngles[0] = yaw - yaw0;
  IMUAngles[1] = pitch - pitch0;
  IMUAngles[2] = roll - roll0;

  delay(10);                                //Accelerometer and Gyro should work in 1kHz
}

void calcIMU()
{
    //uint16_t t_start = micros();

    for (int i = 0; i < N; i++)
    {
        calcAngles();
    }
    //uint16_t period = (micros()-t_start);
    //Serial.print("period time = ");    Serial.print(period/1000.0f, 3); Serial.println(" ms");
}

