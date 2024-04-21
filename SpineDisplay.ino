// sensor fusion libraries
#include <Adafruit_Sensor_Calibration.h>
#include <Adafruit_AHRS.h>


#define FILTER_UPDATE_RATE_HZ 100
#define PRINT_EVERY_N_UPDATES 200
//#define AHRS_DEBUG_OUTPUT

uint32_t timestamp;

//Adafruit_NXPSensorFusion filter; // slowest
//Adafruit_Madgwick filter;  // faster than NXP
Adafruit_Mahony filter[4];  // fastest/smalleset
float mag_hardiron[3*4];
float mag_softiron[9*4];
float gyro_zerorate[3*4];
float accel_zerog[3*4];
float mag_field[4];
float gyro_zerorot[3*4];

int firstRead = 10;


#include <Adafruit_LSM6DSOX.h>
#include <Wire.h>
#include <Adafruit_LIS3MDL.h>
#include <Adafruit_Sensor.h>


#include <MCUFRIEND_kbv.h>
MCUFRIEND_kbv tft;  // hard-wired for UNO shields anyway.
#include <TouchScreen.h>

const int XP = 8, XM = A2, YP = A3, YM = 9;  //240x320 ID=0x9341
const int TS_LEFT = 918, TS_RT = 115, TS_TOP = 80, TS_BOT = 909;

TouchScreen ts = TouchScreen(XP, YP, XM, YM, 300);
TSPoint tp;

#define MINPRESSURE 100
#define MAXPRESSURE 1000

/*
PORTRAIT  CALIBRATION     240 x 320
x = map(p.x, LEFT=918, RT=115, 0, 240)
y = map(p.y, TOP=80, BOT=909, 0, 320)

LANDSCAPE CALIBRATION     320 x 240
x = map(p.y, LEFT=80, RT=909, 0, 320)
y = map(p.x, TOP=115, BOT=918, 0, 240)
*/
#define centerX 402
#define centerY 414

int16_t BOXSIZE;
int16_t PENRADIUS = 1;
uint16_t ID, oldcolor, currentcolor;
uint8_t Orientation = 0;  //PORTRAIT

// Assign human-readable names to some common 16-bit color values:
#define BLACK 0x0000
#define BLUE 0x001F
#define RED 0xF800
#define GREEN 0x07E0
#define CYAN 0x07FF
#define MAGENTA 0xF81F
#define YELLOW 0xFFE0
#define WHITE 0xFFFF

#define SENSOR_COUNT 4

Adafruit_LSM6DSOX sox[4];

Adafruit_LIS3MDL mgts[4];
void setup() {
  // put your setup code here, to run once:
  Serial.begin(115200);
  while (!Serial)
    delay(10);  // will pause Zero, Leonardo, etc until serial console opens

  Serial.println("Adafruit LSM6DSOX + LIS3MDL Chain!");

  init_sensors();
  load_cal();

  for (int i = 0; i < SENSOR_COUNT; i++) {
    filter[i].begin(FILTER_UPDATE_RATE_HZ);
  }
  timestamp = millis();

  Serial.println("");

  uint16_t tmp;

  tft.reset();
  ID = tft.readID();
  tft.begin(ID);
  //show_Serial();
  tft.setRotation(Orientation);
  tft.fillScreen(BLACK);
  //show_tft();

  Serial.print("Width: ");
  Serial.println(tft.width());
  Serial.print("Height: ");
  Serial.println(tft.height());

  delay(1000);
}


void loop() {
  static uint8_t counter = 0;
  // put your main code here, to run repeatedly:
  if ((millis() - timestamp) < (1000 / FILTER_UPDATE_RATE_HZ)) {
    return;
  }
  timestamp = millis();

  feed_filter();
  
  #if defined(AHRS_DEBUG_OUTPUT)
    Serial.print("Took "); Serial.print(millis()-timestamp); Serial.println(" ms");
  #endif

  // only print the calculated output once in a while
  if (counter++ <= PRINT_EVERY_N_UPDATES) {
    return;
  }
  // reset the counter
  counter = 0;

  Serial.println("");
  Serial.println("");
  Serial.println("");

  double positions[SENSOR_COUNT + 1][3];
  positions[0][0] = 0;
  positions[0][1] = 0;
  positions[0][2] = 0;
  double accDirection[3];
  double baseAngles[3];
  double euler[3];
  double axisOfRotation[3];
  double xAxis[3] = { 1, 0, 0 };
  double yAxis[3] = { 0, 1, 0 };
  double zAxis[3] = { 0, 0, 1 };
  double angle;

  for (int i = 0; i < SENSOR_COUNT; i++) {
    Serial.println(i);
    // get gravity and orientation
    float x, y, z;
    filter[i].getGravityVector(&x, &y, &z);
    accDirection[0] = 0;
    accDirection[1] = 1;
    accDirection[2] = 0;
    euler[0] = filter[i].getRollRadians() - gyro_zerorot[i*3 + 0];
    euler[1] = filter[i].getPitchRadians() - gyro_zerorot[i*3 + 1];
    euler[2] = filter[i].getYawRadians() - gyro_zerorot[i*3 + 2];

    /*
    if (firstRead == 0) {
      Serial.println("calibrated");
      gyro_zerorot[i*3 + 0] = euler[0];
      gyro_zerorot[i*3 + 1] = euler[1];
      gyro_zerorot[i*3 + 2] = euler[2];
    }*/
    

    Serial.print("Direction: ");
    printVector(accDirection);
    Serial.println("");
    Serial.print("Rotation: ");
    printVector(euler);
    Serial.println("");
    

    //normalize(accDirection); // normalize direction
    // cross base direction with direction of gravity to get axis of rotation
    
    

    // set base orientation
    if (i == 0) {
      baseAngles[0] = euler[0];
      baseAngles[1] = euler[1];
      baseAngles[2] = euler[2];
      crossProduct(accDirection, yAxis, axisOfRotation); // get axis of rotation to rotate up
      angle = angleBetween(yAxis, accDirection);
    } else {
      // rotate for gyro offset
      rotateVectorCC(accDirection, xAxis, baseAngles[0]-euler[0]);
      rotateVectorCC(accDirection, yAxis, baseAngles[1]-euler[1]);
      rotateVectorCC(accDirection, zAxis, baseAngles[2]-euler[2]);
    }
    //rotateVectorCC(accDirection, axisOfRotation, angle); // rotate to be inlign with the upward position
    Serial.print("Corrected: ");
    printVector(accDirection);
    Serial.println("");
    normalize(accDirection);
    add(positions[i], accDirection, positions[i+1]); // add the new position
  }
  firstRead--;



  tft.fillScreen(BLACK);
  for (int i = 1; i < SENSOR_COUNT + 1; i++) {
    adaptPosition(positions[i]);
    printPosition(positions[i]);
  }
  for (int i = 2; i < SENSOR_COUNT + 1; i++) {
    //tft.drawLine(positions[i - 1][0], positions[i - 1][1], positions[i][0], positions[i][1], GREEN);
    connectPoints(positions[i - 1], positions[i]);
  }
  

}

// veiwing from z axis
void printPosition(double pos[3]) {
  tft.drawCircle(pos[0], pos[1], pos[2], WHITE);
}


void connectPoints(double pos1[3], double pos2[3]) {

  // reciprocalSlope for getting points tangent to the slope on the outside of the circle
  double reciprocalSlope = (pos2[0] - pos1[0]) / (pos2[1] - pos1[1]) * -1;  
  
  // get normalized offsets for a tangent point - (1, reciprocal) is the original point
  double magnitude = sqrt(1 + reciprocalSlope * reciprocalSlope);
  double xOffset = 1 / magnitude;
  double yOffset = reciprocalSlope / magnitude;

  // z axis is the radius of the circle
  tft.drawLine(pos1[0] + xOffset * pos1[2], pos1[1] + yOffset * pos1[2], pos2[0] + xOffset * pos2[2], pos2[1] + yOffset * pos2[2], GREEN);
  tft.drawLine(pos1[0] - xOffset * pos1[2], pos1[1] - yOffset * pos1[2], pos2[0] - xOffset * pos2[2], pos2[1] - yOffset * pos2[2], GREEN);
  

  Serial.print("Pos 1: "); printVector(pos1); Serial.println("");
  Serial.print("Pos 2: "); printVector(pos2); Serial.println("");
  
  Serial.print(reciprocalSlope); Serial.println(" yslope");




  //tft.drawLine(pos1[0], pos1[1], pos2[0], pos2[1], GREEN);
}

double getDistance(double pos1[3], double pos2[3]) {
  return sqrt(pow(pos2[0] - pos1[0], 2) + pow(pos2[1] - pos1[1], 2) + pow(pos2[2] - pos1[2], 2));
}

void adaptPosition(double pos[3]) {
  float modifier = 40;
  pos[0] = (tft.width() / 2) + (pos[0] * modifier);
  pos[1] = (tft.width() / 2) + (pos[1] * modifier);
  pos[2] = 5 + (pos[2]);
}


void printVector(double v[3]) {
  Serial.print(v[0]);
  Serial.print(" x   ");
  Serial.print(v[1]);
  Serial.print(" y   ");
  Serial.print(v[2]);
  Serial.print(" z   ");
}



void normalize(double vector[3]) {
  double magnitude = getMagnitude(vector);
  vector[0] = vector[0] / magnitude;
  vector[1] = vector[1] / magnitude;
  vector[2] = vector[2] / magnitude;
}

double getMagnitude(double vector[3]) {
  //Serial.print("Magnitude vector: ");
  //printVector(vector);
  //Serial.println("");
  double mag = sqrt(vector[0] * vector[0] + vector[1] * vector[1] + vector[2] * vector[2]);
  //Serial.print("Magnitude: "); Serial.println(mag);
  return mag;
}

void add(double v1[3], double v2[3], double result[3]) {
  result[0] = v1[0] + v2[0];
  result[1] = v1[1] + v2[1];
  result[2] = v1[2] + v2[2];
}

void multiply(double v1[3], double v2[3], double result[3]) {
  result[0] = v1[0] * v2[0];
  result[1] = v1[1] * v2[1];
  result[2] = v1[2] * v2[2];
}
void multiply(double v1[3], double d) {
  v1[0] *= d;
  v1[1] *= d;
  v1[2] *= d;
}

void crossProduct(double v1[3], double v2[3], double result[3]) {
  result[0] = v1[1] * v2[2] - v1[2] * v2[1];
  result[1] = -1 * (v1[0] * v2[2] - v1[2] * v2[0]);
  result[2] = v1[0] * v2[1] - v1[1] * v2[0];
}

double dotProduct(double v1[3], double v2[3]) {
  double result = v1[0] * v2[0];
  result += v1[1] * v2[1];
  result += v1[2] * v2[2];
  return result;
}

// returns radians
void calcRollPitch(double acc[3], double result[2]) {
  double x_Buff = float(acc[0]);
  double y_Buff = float(acc[1]);
  double z_Buff = float(acc[2]);
  result[0] = atan2(y_Buff, z_Buff);                                      // * 57.3; // roll
  result[1] = atan2((-x_Buff), sqrt(y_Buff * y_Buff + z_Buff * z_Buff));  // * 57.3; // pitch
}

double angleBetween(double v1[3], double v2[3]) {
  double before = dotProduct(v1, v2) / (getMagnitude(v1) * getMagnitude(v2));
  //Serial.print("before: "); Serial.println(before);
  return acos(before);
}

void rotateVectorCC(double vec[], double axis[], double theta) {
  double x, y, z;
  double u, v, w;
  x = vec[0];
  y = vec[1];
  z = vec[2];
  u = axis[0];
  v = axis[1];
  w = axis[2];
  /*
        Serial.print("vec: ");
        printVector(vec);
        Serial.println("");
        Serial.print("axis: ");
        printVector(axis);
        Serial.println("");
        Serial.println(theta);
        */
  double v1 = u * x + v * y + w * z;
  double xPrime = u * v1 * (1 - cos(theta))
                  + x * cos(theta)
                  + (-w * y + v * z) * sin(theta);
  double yPrime = v * v1 * (1 - cos(theta))
                  + y * cos(theta)
                  + (w * x - u * z) * sin(theta);
  double zPrime = w * v1 * (1 - cos(theta))
                  + z * cos(theta)
                  + (-v * x + u * y) * sin(theta);
  double primes[3]{ xPrime, yPrime, zPrime };
  //Serial.print("Primes: ");
  //printVector(primes);
  //Serial.println("");
  vec[0] = xPrime;
  vec[1] = yPrime;
  vec[2] = zPrime;
}

void init_sensors() {
  if (!sox[0].begin_I2C(0x6A)) {
    Serial.println("Failed to find LSM6DSOX chip 1");
    while (1) {
      delay(10);
    }
  }

  if (!sox[1].begin_I2C(0x6B)) {
    Serial.println("Failed to find LSM6DSOX chip 2");
    while (1) {
      delay(10);
    }
  }

  if (!sox[2].begin_I2C(0x1A)) {
    Serial.println("Failed to find LSM6DSOX chip 3");
    while (1) {
      delay(10);
    }
  }

  if (!sox[3].begin_I2C(0x7A)) {
    Serial.println("Failed to find LSM6DSOX chip 4");
    while (1) {
      delay(10);
    }
  }

  Serial.println("ALL LSM6DSOX Devices Found!");

  if (!mgts[0].begin_I2C(0x1C)) {
    Serial.println("Failed to find LIS3MDL chip 1");
    while (1) {
      delay(10);
    }
  }

  if (!mgts[1].begin_I2C(0x1E)) {
    Serial.println("Failed to find LIS3MDL chip 2");
    while (1) {
      delay(10);
    }
  }

  if (!mgts[2].begin_I2C(0x6C)) {
    Serial.println("Failed to find LIS3MDL chip 3");
    while (1) {
      delay(10);
    }
  }

  if (!mgts[3].begin_I2C(0xC)) {
    Serial.println("Failed to find LIS3MDL chip 4");
    while (1) {
      delay(10);
    }
  }

  Serial.println("ALL LIS3MDL Devices Found!");

  for (int i = 0; i < SENSOR_COUNT; i++) {
    sox[i].setAccelRange(LSM6DS_ACCEL_RANGE_4_G);
    sox[i].setGyroDataRate(LSM6DS_RATE_12_5_HZ);
    sox[i].setGyroRange(LSM6DS_GYRO_RANGE_250_DPS);
    sox[i].setAccelDataRate(LSM6DS_RATE_12_5_HZ);
  }
  Serial.print("Accelerometer range set to: ");
  switch (sox[0].getAccelRange()) {
    case LSM6DS_ACCEL_RANGE_2_G:
      Serial.println("+-2G");
      break;
    case LSM6DS_ACCEL_RANGE_4_G:
      Serial.println("+-4G");
      break;
    case LSM6DS_ACCEL_RANGE_8_G:
      Serial.println("+-8G");
      break;
    case LSM6DS_ACCEL_RANGE_16_G:
      Serial.println("+-16G");
      break;
  }

  Serial.print("Gyro range set to: ");
  switch (sox[0].getGyroRange()) {
    case LSM6DS_GYRO_RANGE_125_DPS:
      Serial.println("125 degrees/s");
      break;
    case LSM6DS_GYRO_RANGE_250_DPS:
      Serial.println("250 degrees/s");
      break;
    case LSM6DS_GYRO_RANGE_500_DPS:
      Serial.println("500 degrees/s");
      break;
    case LSM6DS_GYRO_RANGE_1000_DPS:
      Serial.println("1000 degrees/s");
      break;
    case LSM6DS_GYRO_RANGE_2000_DPS:
      Serial.println("2000 degrees/s");
      break;
    case ISM330DHCX_GYRO_RANGE_4000_DPS:
      break;  // unsupported range for the DSOX
  }

  Serial.print("Accelerometer data rate set to: ");
  switch (sox[0].getAccelDataRate()) {
    case LSM6DS_RATE_SHUTDOWN:
      Serial.println("0 Hz");
      break;
    case LSM6DS_RATE_12_5_HZ:
      Serial.println("12.5 Hz");
      break;
    case LSM6DS_RATE_26_HZ:
      Serial.println("26 Hz");
      break;
    case LSM6DS_RATE_52_HZ:
      Serial.println("52 Hz");
      break;
    case LSM6DS_RATE_104_HZ:
      Serial.println("104 Hz");
      break;
    case LSM6DS_RATE_208_HZ:
      Serial.println("208 Hz");
      break;
    case LSM6DS_RATE_416_HZ:
      Serial.println("416 Hz");
      break;
    case LSM6DS_RATE_833_HZ:
      Serial.println("833 Hz");
      break;
    case LSM6DS_RATE_1_66K_HZ:
      Serial.println("1.66 KHz");
      break;
    case LSM6DS_RATE_3_33K_HZ:
      Serial.println("3.33 KHz");
      break;
    case LSM6DS_RATE_6_66K_HZ:
      Serial.println("6.66 KHz");
      break;
  }

  Serial.print("Gyro data rate set to: ");
  switch (sox[0].getGyroDataRate()) {
    case LSM6DS_RATE_SHUTDOWN:
      Serial.println("0 Hz");
      break;
    case LSM6DS_RATE_12_5_HZ:
      Serial.println("12.5 Hz");
      break;
    case LSM6DS_RATE_26_HZ:
      Serial.println("26 Hz");
      break;
    case LSM6DS_RATE_52_HZ:
      Serial.println("52 Hz");
      break;
    case LSM6DS_RATE_104_HZ:
      Serial.println("104 Hz");
      break;
    case LSM6DS_RATE_208_HZ:
      Serial.println("208 Hz");
      break;
    case LSM6DS_RATE_416_HZ:
      Serial.println("416 Hz");
      break;
    case LSM6DS_RATE_833_HZ:
      Serial.println("833 Hz");
      break;
    case LSM6DS_RATE_1_66K_HZ:
      Serial.println("1.66 KHz");
      break;
    case LSM6DS_RATE_3_33K_HZ:
      Serial.println("3.33 KHz");
      break;
    case LSM6DS_RATE_6_66K_HZ:
      Serial.println("6.66 KHz");
      break;
  }

  for (int i = 0; i < SENSOR_COUNT; i++) {
    mgts[i].setPerformanceMode(LIS3MDL_MEDIUMMODE);
    mgts[i].setRange(LIS3MDL_RANGE_4_GAUSS);
    mgts[i].setOperationMode(LIS3MDL_CONTINUOUSMODE);
    mgts[i].setDataRate(LIS3MDL_DATARATE_155_HZ);
    mgts[i].setIntThreshold(500);
    mgts[i].configInterrupt(false, false, true,  // enable z axis
                            true,                // polarity
                            false,               // don't latch
                            true);               // enabled!
  }
  Serial.print("Performance mode set to: ");
  switch (mgts[0].getPerformanceMode()) {
    case LIS3MDL_LOWPOWERMODE: Serial.println("Low"); break;
    case LIS3MDL_MEDIUMMODE: Serial.println("Medium"); break;
    case LIS3MDL_HIGHMODE: Serial.println("High"); break;
    case LIS3MDL_ULTRAHIGHMODE: Serial.println("Ultra-High"); break;
  }

  Serial.print("Operation mode set to: ");
  // Single shot mode will complete conversion and go into power down
  switch (mgts[0].getOperationMode()) {
    case LIS3MDL_CONTINUOUSMODE: Serial.println("Continuous"); break;
    case LIS3MDL_SINGLEMODE: Serial.println("Single mode"); break;
    case LIS3MDL_POWERDOWNMODE: Serial.println("Power-down"); break;
  }

  Serial.print("Data rate set to: ");
  switch (mgts[0].getDataRate()) {
    case LIS3MDL_DATARATE_0_625_HZ: Serial.println("0.625 Hz"); break;
    case LIS3MDL_DATARATE_1_25_HZ: Serial.println("1.25 Hz"); break;
    case LIS3MDL_DATARATE_2_5_HZ: Serial.println("2.5 Hz"); break;
    case LIS3MDL_DATARATE_5_HZ: Serial.println("5 Hz"); break;
    case LIS3MDL_DATARATE_10_HZ: Serial.println("10 Hz"); break;
    case LIS3MDL_DATARATE_20_HZ: Serial.println("20 Hz"); break;
    case LIS3MDL_DATARATE_40_HZ: Serial.println("40 Hz"); break;
    case LIS3MDL_DATARATE_80_HZ: Serial.println("80 Hz"); break;
    case LIS3MDL_DATARATE_155_HZ: Serial.println("155 Hz"); break;
    case LIS3MDL_DATARATE_300_HZ: Serial.println("300 Hz"); break;
    case LIS3MDL_DATARATE_560_HZ: Serial.println("560 Hz"); break;
    case LIS3MDL_DATARATE_1000_HZ: Serial.println("1000 Hz"); break;
  }

  Serial.print("Range set to: ");
  switch (mgts[0].getRange()) {
    case LIS3MDL_RANGE_4_GAUSS: Serial.println("+-4 gauss"); break;
    case LIS3MDL_RANGE_8_GAUSS: Serial.println("+-8 gauss"); break;
    case LIS3MDL_RANGE_12_GAUSS: Serial.println("+-12 gauss"); break;
    case LIS3MDL_RANGE_16_GAUSS: Serial.println("+-16 gauss"); break;
  }
}

// this code was modified from the Adafruit Sensor Calibration library to work with multiple sensors
bool calibrate(sensors_event_t& event, int sensor) {
  if (event.type == SENSOR_TYPE_MAGNETIC_FIELD) {
    // hard iron cal
    float mx = event.magnetic.x - mag_hardiron[sensor*3 + 0];
    float my = event.magnetic.y - mag_hardiron[sensor*3 + 1];
    float mz = event.magnetic.z - mag_hardiron[sensor*3 + 2];
    // soft iron cal
    event.magnetic.x =
      mx * mag_softiron[sensor*9 + 0] + my * mag_softiron[sensor*9 + 1] + mz * mag_softiron[sensor*9 + 2];
    event.magnetic.y =
      mx * mag_softiron[sensor*9 + 3] + my * mag_softiron[sensor*9 + 4] + mz * mag_softiron[sensor*9 + 5];
    event.magnetic.z =
      mx * mag_softiron[sensor*9 + 6] + my * mag_softiron[sensor*9 + 7] + mz * mag_softiron[sensor*9 + 8];
  } else if (event.type == SENSOR_TYPE_GYROSCOPE) {
    event.gyro.x -= gyro_zerorate[sensor*3 + 0];
    event.gyro.y -= gyro_zerorate[sensor*3 + 1];
    event.gyro.z -= gyro_zerorate[sensor*3 + 2];
  } else if (event.type == SENSOR_TYPE_ACCELEROMETER) {
    event.acceleration.x -= accel_zerog[sensor*3 + 0];
    event.acceleration.y -= accel_zerog[sensor*3 + 1];
    event.acceleration.z -= accel_zerog[sensor*3 + 2];
  } else {
    return false;
  }
  return true;
}

// set all the calibration for the sensors
void load_cal() {
  int sensor = 0;
  // in uTesla
  mag_hardiron[sensor*3 + 0] = -21.12;
  mag_hardiron[sensor*3 + 1] = -6.28;
  mag_hardiron[sensor*3 + 2] = 31.83;

  // in uTesla
  mag_softiron[sensor*9 + 0] = 0.998;
  mag_softiron[sensor*9 + 1] = 0.038;
  mag_softiron[sensor*9 + 2] = -0.032;  
  mag_softiron[sensor*9 + 3] = 0.036;
  mag_softiron[sensor*9 + 4] = 0.914;
  mag_softiron[sensor*9 + 5] = 0.002;  
  mag_softiron[sensor*9 + 6] = -0.032;
  mag_softiron[sensor*9 + 7] = -0.002;
  mag_softiron[sensor*9 + 8] = 1.098;
  // Earth total magnetic field strength in uTesla (dependent on location and time of the year),
  // visit: https://www.ngdc.noaa.gov/geomag/calculators/magcalc.shtml#igrfwmm)
  mag_field[sensor] = 49.89; // approximate value for locations along the equator

  // in Radians/s
  gyro_zerorate[sensor*3 + 0] = -0.0098;
  gyro_zerorate[sensor*3 + 1] = -0.0147;
  gyro_zerorate[sensor*3 + 2] = 0.0024;
  // in Radians
  gyro_zerorot[sensor*3 + 0] = 0;//0.05;
  gyro_zerorot[sensor*3 + 1] = 0;//-0.01;
  gyro_zerorot[sensor*3 + 2] = 0;//-2.08;

  // 2
  sensor++;
  // in uTesla
  mag_hardiron[sensor*3 + 0] = -7.87;
  mag_hardiron[sensor*3 + 1] = -22.81;
  mag_hardiron[sensor*3 + 2] = 61.63;

  // in uTesla
  mag_softiron[sensor*9 + 0] = 1.001;
  mag_softiron[sensor*9 + 1] = 0.041;
  mag_softiron[sensor*9 + 2] = 0.026;  
  mag_softiron[sensor*9 + 3] = 0.041;
  mag_softiron[sensor*9 + 4] = 0.954;
  mag_softiron[sensor*9 + 5] = -0.045;  
  mag_softiron[sensor*9 + 6] = 0.026;
  mag_softiron[sensor*9 + 7] = -0.045;
  mag_softiron[sensor*9 + 8] = 1.051;
  // Earth total magnetic field strength in uTesla (dependent on location and time of the year),
  // visit: https://www.ngdc.noaa.gov/geomag/calculators/magcalc.shtml#igrfwmm)
  mag_field[sensor] = 54.61; // approximate value for locations along the equator

  // in Radians/s
  gyro_zerorate[sensor*3 + 0] = -0.0018;
  gyro_zerorate[sensor*3 + 1] = 0.0012;
  gyro_zerorate[sensor*3 + 2] = -0.0092;
  // in Radians
  gyro_zerorot[sensor*3 + 0] = 0;//0.03;
  gyro_zerorot[sensor*3 + 1] = 0;//-0.02;
  gyro_zerorot[sensor*3 + 2] = 0;//-0.23;

  
  // 3
  sensor++;
  // in uTesla
  mag_hardiron[sensor*3 + 0] = -26.69;
  mag_hardiron[sensor*3 + 1] = -13.59;
  mag_hardiron[sensor*3 + 2] = 30.85;

  // in uTesla
  mag_softiron[sensor*9 + 0] = 0.961;
  mag_softiron[sensor*9 + 1] = 0.0034;
  mag_softiron[sensor*9 + 2] = 0.007;  
  mag_softiron[sensor*9 + 3] = 0.034;
  mag_softiron[sensor*9 + 4] = 0.983;
  mag_softiron[sensor*9 + 5] = 0.016;  
  mag_softiron[sensor*9 + 6] = 0.007;
  mag_softiron[sensor*9 + 7] = 0.016;
  mag_softiron[sensor*9 + 8] = 1.060;
  // Earth total magnetic field strength in uTesla (dependent on location and time of the year),
  // visit: https://www.ngdc.noaa.gov/geomag/calculators/magcalc.shtml#igrfwmm)
  mag_field[sensor] = 53.07; // approximate value for locations along the equator

  // in Radians/s
  gyro_zerorate[sensor*3 + 0] = 0.0073;
  gyro_zerorate[sensor*3 + 1] = -0.011;
  gyro_zerorate[sensor*3 + 2] = 0.0061;
  // in Radians
  gyro_zerorot[sensor*3 + 0] = 0;//-0.04;
  gyro_zerorot[sensor*3 + 1] = 0;//-0.08;
  gyro_zerorot[sensor*3 + 2] = 0;//-2.18;

  // 4
  sensor++;
  // in uTesla
  mag_hardiron[sensor*3 + 0] = 1.87;
  mag_hardiron[sensor*3 + 1] = -14.17;
  mag_hardiron[sensor*3 + 2] = 50.79;

  // in uTesla
  mag_softiron[sensor*9 + 0] = 0.965;
  mag_softiron[sensor*9 + 1] = 0.061;
  mag_softiron[sensor*9 + 2] = 0.026;  
  mag_softiron[sensor*9 + 3] = 0.061;
  mag_softiron[sensor*9 + 4] = 0.976;
  mag_softiron[sensor*9 + 5] = -0.020;  
  mag_softiron[sensor*9 + 6] = 0.026;
  mag_softiron[sensor*9 + 7] = -0.020;
  mag_softiron[sensor*9 + 8] = 1.068;
  // Earth total magnetic field strength in uTesla (dependent on location and time of the year),
  // visit: https://www.ngdc.noaa.gov/geomag/calculators/magcalc.shtml#igrfwmm)
  mag_field[sensor] = 51.35; // approximate value for locations along the equator

  // in Radians/s
  gyro_zerorate[sensor*3 + 0] = -0.0073;
  gyro_zerorate[sensor*3 + 1] = -0.011;
  gyro_zerorate[sensor*3 + 2] = -0.0043;
  // in Radians
  gyro_zerorot[sensor*3 + 0] = 0;//0.06;
  gyro_zerorot[sensor*3 + 1] = 0;//0.21;
  gyro_zerorot[sensor*3 + 2] = 0;//-2.86;
}

// feed the fusion filter the sensor inputs
void feed_filter() {
  for (int i = 0; i < SENSOR_COUNT; i++) {
    float gx, gy, gz;
    // Read the motion sensors
    sensors_event_t accel, gyro, mag, temp;
    sox[i].getEvent(&accel, &gyro, &temp);
    mgts[i].getEvent(&mag);

    calibrate(mag, i);
    calibrate(accel, i);
    calibrate(gyro, i);

    // Gyroscope needs to be converted from Rad/s to Degree/s
    // the rest are not unit-important
    gx = gyro.gyro.x * SENSORS_RADS_TO_DPS;
    gy = gyro.gyro.y * SENSORS_RADS_TO_DPS;
    gz = gyro.gyro.z * SENSORS_RADS_TO_DPS;
  
    // Update the SensorFusion filter
    filter[i].update(gx, gy, gz, 
                accel.acceleration.x, accel.acceleration.y, accel.acceleration.z, 
                mag.magnetic.y, mag.magnetic.x, -mag.magnetic.z);

    /*
    #if defined(AHRS_DEBUG_OUTPUT)
      Serial.print("Raw: ");
      Serial.print(accel.acceleration.x, 4); Serial.print(", ");
      Serial.print(accel.acceleration.y, 4); Serial.print(", ");
      Serial.print(accel.acceleration.z, 4); Serial.print(", ");
      Serial.print(gx, 4); Serial.print(", ");
      Serial.print(gy, 4); Serial.print(", ");
      Serial.print(gz, 4); Serial.print(", ");
      Serial.print(mag.magnetic.x, 4); Serial.print(", ");
      Serial.print(mag.magnetic.y, 4); Serial.print(", ");
      Serial.print(mag.magnetic.z, 4); Serial.println("");
    #endif
    */
  }
  
}
