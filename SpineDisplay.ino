#include <Adafruit_LSM6DSOX.h>

#include <Wire.h>
#include <Adafruit_LIS3MDL.h>
#include <Adafruit_Sensor.h>

#include <MCUFRIEND_kbv.h>
MCUFRIEND_kbv tft;       // hard-wired for UNO shields anyway.
#include <TouchScreen.h>

const int XP=8,XM=A2,YP=A3,YM=9; //240x320 ID=0x9341
const int TS_LEFT=918,TS_RT=115,TS_TOP=80,TS_BOT=909;

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
uint8_t Orientation = 0;    //PORTRAIT

// Assign human-readable names to some common 16-bit color values:
#define BLACK   0x0000
#define BLUE    0x001F
#define RED     0xF800
#define GREEN   0x07E0
#define CYAN    0x07FF
#define MAGENTA 0xF81F
#define YELLOW  0xFFE0
#define WHITE   0xFFFF

#define SENSOR_COUNT 4

Adafruit_LSM6DSOX sox[4];

Adafruit_LIS3MDL mgts[4];
void setup() {
  // put your setup code here, to run once:
  Serial.begin(115200);
  while (!Serial)
    delay(10); // will pause Zero, Leonardo, etc until serial console opens

    Serial.println("Adafruit LSM6DSOX + LIS3MDL Chain!");

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

  if (!mgts[0].begin_I2C()) {
    Serial.println("Failed to find LIS3MDL chip 1");
    while (1) {
      delay(10);
    }
  }

  if (!mgts[1].begin_I2C()) {
    Serial.println("Failed to find LIS3MDL chip 2");
    while (1) {
      delay(10);
    }
  }

  if (!mgts[2].begin_I2C()) {
    Serial.println("Failed to find LIS3MDL chip 3");
    while (1) {
      delay(10);
    }
  }
  
  if (!mgts[3].begin_I2C()) {
    Serial.println("Failed to find LIS3MDL chip 4");
    while (1) {
      delay(10);
    }
  }

  Serial.println("ALL LIS3MDL Devices Found!");

  for (int i = 0; i < SENSOR_COUNT; i++) {
    sox[i].setAccelRange(LSM6DS_ACCEL_RANGE_4_G);
    sox[i].setGyroDataRate(LSM6DS_RATE_12_5_HZ);
    sox[i].setGyroRange(LSM6DS_GYRO_RANGE_250_DPS );
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
    break; // unsupported range for the DSOX
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
    mgts[i].configInterrupt(false, false, true, // enable z axis
                          true, // polarity
                          false, // don't latch
                          true); // enabled!
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
  
  Serial.println("");

  uint16_t tmp;

    tft.reset();
    ID = tft.readID();
    tft.begin(ID);
    //show_Serial();
    tft.setRotation(Orientation);
    tft.fillScreen(BLACK);
    //show_tft();
    
    Serial.print("Width: "); Serial.println(tft.width());
    Serial.print("Height: "); Serial.println(tft.height());

    delay(1000);
}


void loop() {
  // put your main code here, to run repeatedly:


  double positions[SENSOR_COUNT + 1][3];
  positions[0][0] = 0; positions[0][1] = 0; positions[0][2] = 0;
  double accDirection[3];
  double baseAngles[2];
  double rollPitch[2];
  double rollDifference;

  getAccel(&sox1, accDirection);
  calcRollPitch(accDirection, baseAngles);
  normalize(accDirection);
  // cross base direction with direction of gravity to get axis of rotation
  double axisOfRotation[3];
  double up[3] = {0, 1, 0};
  crossProduct(accDirection, up, axisOfRotation);
  // find the angle between base direction and gravity 
  double angle = angleBetween(up, accDirection);
  Serial.print("Axis of Rotation: ");
  printVector(axisOfRotation);
  Serial.println("");
  Serial.print("Angle: "); Serial.println(angle);
  rotateVectorCC(accDirection, axisOfRotation, angle);
  Serial.print("Direction: ");
  printVector(accDirection);
  Serial.println("");

  add(positions[0], accDirection, positions[1]);

  getAccel(&sox2, accDirection); // get accelerometer data
  calcRollPitch(accDirection, rollPitch); // calculate roll and pitch
  rollDifference = rollPitch[1] - baseAngles[1]; // calculate difference from the base angle
  Serial.print(rollDifference); Serial.println(" Pitch Difference");
  Serial.print(rollPitch[0] - baseAngles[0]); Serial.println(" Roll Difference");
  normalize(accDirection); // normalize the direction
  rotateVectorCC(accDirection, up, rollDifference); // rotate the vector so the difference in accelerometers is only pitch and yaw
  rotateVectorCC(accDirection, axisOfRotation, angle); // rotate to the y axis
  add(positions[1], accDirection, positions[2]); // add to next position

  getAccel(&sox3, accDirection); // get accelerometer data
  calcRollPitch(accDirection, rollPitch); // calculate roll and pitch
  rollDifference = rollPitch[1] - baseAngles[1]; // calculate difference from the base angle
  normalize(accDirection); // normalize the direction
  rotateVectorCC(accDirection, up, rollDifference); // rotate the vector so the difference in accelerometers is only pitch and yaw
  rotateVectorCC(accDirection, axisOfRotation, angle); // rotate to the y axis
  add(positions[2], accDirection, positions[3]); // add to next position

  getAccel(&sox4, accDirection); // get accelerometer data
  calcRollPitch(accDirection, rollPitch); // calculate roll and pitch
  rollDifference = rollPitch[1] - baseAngles[1]; // calculate difference from the base angle
  normalize(accDirection); // normalize the direction
  rotateVectorCC(accDirection, up, rollDifference); // rotate the vector so the difference in accelerometers is only pitch and yaw
  rotateVectorCC(accDirection, axisOfRotation, angle); // rotate to the y axis
  add(positions[3], accDirection, positions[4]); // add to next position

  
  
  tft.fillScreen(BLACK);
  for (int i = 0; i < SENSOR_COUNT + 1; i++) {
    adaptPosition(positions[i]);
    printPosition(positions[i]);
  }
  for (int i = 1; i < SENSOR_COUNT + 1; i++) {
    connectPoints(positions[i-1], positions[i]);
  }
  Serial.println("");
  Serial.println("");
  Serial.println("");

  delay(10000);



}

// veiwing from z axis
void printPosition(double pos[3]) {
  tft.drawCircle(pos[0], pos[1], pos[2], WHITE);
}

void connectPoints(double pos1[3], double pos2[3]) {
  float sizeModifier = 0.5; // size modifier for the z
  double flatPos1[3] {pos1[0], pos1[1], 0};
  double flatPos2[3] {pos2[0], pos2[1], 0};
  
  double ySlope = (pos2[1]-pos1[1]) / (pos2[0] - pos1[0]); // slope of the line
  double zSlope = (pos2[2]-pos1[2]) / (pos2[0] - pos1[0]);
  double yIntercept = pos1[1]/(pos1[0]*ySlope);
  double zIntercept = pos1[2]/(pos1[0]*zSlope);
  
  for (double x = min(pos1[0], pos2[0]); x < max(pos1[0], pos2[0]); x+= ySlope) {
    tft.fillCircle(x, x * ySlope + yIntercept, (x * zSlope + zIntercept) * sizeModifier, GREEN);
  }
  
  //tft.drawLine(pos1[0], pos1[1], pos2[0], pos2[1], GREEN);
}

double getDistance(double pos1[3], double pos2[3]) {
  return sqrt(pow(pos2[0]-pos1[0],2) + pow(pos2[1]-pos1[1],2) + pow(pos2[2]-pos1[2],2));
}

void adaptPosition(double pos[3]) {
  float modifier = 50;
  pos[0] = (tft.width() / 2) + (pos[0] * modifier);
  pos[1] = (tft.width() / 2) + (pos[1] * modifier);
  pos[2] = 5 + (pos[2]);
}


void printVector(double v[3]) {
  Serial.print(v[0]); Serial.print(" x   ");
  Serial.print(v[1]); Serial.print(" y   ");
  Serial.print(v[2]); Serial.print(" z   ");
}

void printData(Adafruit_LSM6DSOX* sox) {
  //  /* Get a new normalized sensor event */
  sensors_event_t accel;
  sensors_event_t gyro;
  sensors_event_t temp;
  sox->getEvent(&accel, &gyro, &temp);

  Serial.print("\t\tTemperature ");
  Serial.print(temp.temperature);
  Serial.println(" deg C");

  /* Display the results (acceleration is measured in m/s^2) */
  Serial.print("\t\tAccel X: ");
  Serial.print(accel.acceleration.x);
  Serial.print(" \tY: ");
  Serial.print(accel.acceleration.y);
  Serial.print(" \tZ: ");
  Serial.print(accel.acceleration.z);
  Serial.println(" m/s^2 ");
  Serial.println("");
}

void getAccel(Adafruit_LSM6DSOX* sox, double result[3]) {
  //  /* Get a new normalized sensor event */
  sensors_event_t accel;
  sensors_event_t gyro;
  sensors_event_t temp;
  sox->getEvent(&accel, &gyro, &temp);
  result[0] = accel.acceleration.x;
  result[1] = accel.acceleration.y;
  result[2] = accel.acceleration.z;
  Serial.print("Acceleration: ");
  printVector(result);
  Serial.println("");
  Serial.print("\t\tTemperature ");
  Serial.print(temp.temperature);
  Serial.println(" deg C");
}

void getMagnets(Adafruit_LIS3MDL magnt*, double result[3]) {
  sensors_event_t event;
  magnt->getEvent(&event);
  result[0] = event.magnetic.x;
  result[1] = event.magnetic.y;
  result[2] = event.magnetic.z;
  Serial.print("Magnetic Field (uTesla): ");
  printVector(result);
  Serial.println("");
}

/**
 * Calculate the global direction of the accelerometer
 * 
*/
void calcDirection(Adafruit_LSM6DSOX* sox, double baseDirection[3], double axisOfRotation[3], double& angle) {
  double gravity[3];
  getAccel(sox, gravity);

  // cross base direction with direction of gravity to get axis of rotation
  crossProduct(gravity, baseDirection, axisOfRotation);

  // find the angle between base direction and gravity 
  angle = angleBetween(baseDirection, gravity);

  /*
  Serial.print("Gravity: ");
  printVector(gravity);
  Serial.println("");
  Serial.print("Axis of Rotation: ");
  printVector(axisOfRotation);
  Serial.println("");
  Serial.print("Angle: "); Serial.println(angle);
  */

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
  double mag = sqrt(vector[0] * vector[0]  + vector[1] * vector[1] + vector[2] * vector[2]);
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
  v1[0]*= d;
  v1[1]*= d;
  v1[2]*= d;
}

void crossProduct(double v1[3], double v2[3], double result[3]) {
  result[0] = v1[1] * v2[2] - v1[2] * v2[1];
  result[1] = -1 * (v1[0] * v2[2] - v1[2] * v2[0]);
  result[2] = v1[0] * v2[1] - v1[1] * v2[0];
}

double dotProduct(double v1[3], double v2[3]) {
  double result = v1[0] * v2[0];
  result+= v1[1] * v2[1];
  result+= v1[2] * v2[2];
  return result;
}

// returns radians
void calcRollPitch(double acc[3], double result[2]) {
  double x_Buff = float(acc[0]);
  double y_Buff = float(acc[1]);
  double z_Buff = float(acc[2]);
  result[0] = atan2(y_Buff , z_Buff);// * 57.3; // roll
  result[1] = atan2((- x_Buff) , sqrt(y_Buff * y_Buff + z_Buff * z_Buff));// * 57.3; // pitch
}

double angleBetween(double v1[3], double v2[3]) {
  double before = dotProduct(v1,v2) / (getMagnitude(v1) * getMagnitude(v2));
  //Serial.print("before: "); Serial.println(before);
  return acos(before);
}

void rotateVectorCC(double vec[], double axis[], double theta){
        double x, y, z;
        double u, v, w;
        x=vec[0];y=vec[1];z=vec[2];
        u=axis[0];v=axis[1];w=axis[2];
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
        double xPrime = u* v1 *(1 - cos(theta))
                + x*cos(theta)
                + (-w*y + v*z)*sin(theta);
        double yPrime = v* v1 *(1 - cos(theta))
                + y*cos(theta)
                + (w*x - u*z)*sin(theta);
        double zPrime = w* v1 *(1 - cos(theta))
                + z*cos(theta)
                + (-v*x + u*y)*sin(theta);
        double primes[3]{xPrime, yPrime, zPrime};
        //Serial.print("Primes: ");
        //printVector(primes);
        //Serial.println("");
        vec[0] = xPrime;
        vec[1] = yPrime;
        vec[2] = zPrime;
    }
