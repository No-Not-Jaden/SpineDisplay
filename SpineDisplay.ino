#include <Adafruit_LSM6DSOX.h>

#define SENSOR_COUNT 4
#define SENSOR_DISTANCE 10

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

Adafruit_LSM6DSOX sox1;
Adafruit_LSM6DSOX sox2;
Adafruit_LSM6DSOX sox3;
Adafruit_LSM6DSOX sox4;
void setup() {
  // put your setup code here, to run once:
  Serial.begin(115200);
  while (!Serial)
    delay(10); // will pause Zero, Leonardo, etc until serial console opens

    Serial.println("Adafruit LSM6DSOX Chain!");

  if (!sox1.begin_I2C(0x6A)) {
      Serial.println("Failed to find LSM6DSOX chip 1");
    while (1) {
      delay(10);
    }
  }

  if (!sox2.begin_I2C(0x6B)) {
      Serial.println("Failed to find LSM6DSOX chip 2");
    while (1) {
      delay(10);
    }
  }

  if (!sox3.begin_I2C(0x1A)) {
      Serial.println("Failed to find LSM6DSOX chip 3");
    while (1) {
      delay(10);
    }
  }

  if (!sox4.begin_I2C(0x7A)) {
      Serial.println("Failed to find LSM6DSOX chip 4");
    while (1) {
      delay(10);
    }
  }

  Serial.println("ALL LSM6DSOX Devices Found!");

  sox1.setAccelRange(LSM6DS_ACCEL_RANGE_4_G);
  sox2.setAccelRange(LSM6DS_ACCEL_RANGE_4_G);
  sox3.setAccelRange(LSM6DS_ACCEL_RANGE_4_G);
  sox4.setAccelRange(LSM6DS_ACCEL_RANGE_4_G);
  Serial.print("Accelerometer range set to: ");
  switch (sox1.getAccelRange()) {
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

  sox1.setGyroRange(LSM6DS_GYRO_RANGE_250_DPS );
  sox2.setGyroRange(LSM6DS_GYRO_RANGE_250_DPS );
  sox3.setGyroRange(LSM6DS_GYRO_RANGE_250_DPS );
  sox4.setGyroRange(LSM6DS_GYRO_RANGE_250_DPS );
  Serial.print("Gyro range set to: ");
  switch (sox1.getGyroRange()) {
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

  sox1.setAccelDataRate(LSM6DS_RATE_12_5_HZ);
  sox2.setAccelDataRate(LSM6DS_RATE_12_5_HZ);
  sox3.setAccelDataRate(LSM6DS_RATE_12_5_HZ);
  sox4.setAccelDataRate(LSM6DS_RATE_12_5_HZ);
  Serial.print("Accelerometer data rate set to: ");
  switch (sox1.getAccelDataRate()) {
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

  sox1.setGyroDataRate(LSM6DS_RATE_12_5_HZ);
  sox2.setGyroDataRate(LSM6DS_RATE_12_5_HZ);
  sox3.setGyroDataRate(LSM6DS_RATE_12_5_HZ);
  sox4.setGyroDataRate(LSM6DS_RATE_12_5_HZ);
  Serial.print("Gyro data rate set to: ");
  switch (sox1.getGyroDataRate()) {
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
  double direction[3];

  getAccel(&sox1, direction);
  normalize(direction);
  // cross base direction with direction of gravity to get axis of rotation
  double axisOfRotation[3];
  double up[3] = {0, 1, 0};
  crossProduct(direction, up, axisOfRotation);
  // find the angle between base direction and gravity 
  double angle = angleBetween(up, direction);
  Serial.print("Axis of Rotation: ");
  printVector(axisOfRotation);
  Serial.println("");
  Serial.print("Angle: "); Serial.println(angle);
  rotateVectorCC(direction, axisOfRotation, angle);
  Serial.print("Direction: ");
  printVector(direction);
  Serial.println("");

  add(positions[0], direction, positions[1]);

  getAccel(&sox2, direction);
  normalize(direction);
  rotateVectorCC(direction, axisOfRotation, angle);
  add(positions[1], direction, positions[2]);

  getAccel(&sox3, direction);
  normalize(direction);
  rotateVectorCC(direction, axisOfRotation, angle);
  add(positions[2], direction, positions[3]);

  getAccel(&sox4, direction);
  normalize(direction);
  rotateVectorCC(direction, axisOfRotation, angle);
  add(positions[3], direction, positions[4]);

  
  
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




  
  /*
  printData(&sox1);
  delay(100);
  printData(&sox2);
  delay(100);
  printData(&sox3);
  delay(100);
  printData(&sox4);
  delay(1000);
  Serial.println("");
  Serial.println("");
  Serial.println("");*/
}

// veiwing from z axis
void printPosition(double pos[3]) {
  tft.drawCircle(pos[0], pos[1], pos[2], WHITE);
}

void connectPoints(double pos1[3], double pos2[3]) {
  tft.drawLine(pos1[0], pos1[1], pos2[0], pos2[1], GREEN);
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
