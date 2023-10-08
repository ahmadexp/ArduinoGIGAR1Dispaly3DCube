/*
  Arduino GIGA R1 3D Cube example

  This example reads the acceleration values from the BMI270
  sensor and renders a 3D cube on the Arduino Giga R1 Display module.

  The circuit:
  - GIGA R1 WiFi
  - GIGA Display Shield

  created 2 Oct 2023
  by Ahmad Byagowi
  
  This example code is in the public domain.
*/

#include "Arduino_GigaDisplay_GFX.h"
#include "Adafruit_AHRS_Mahony.h"
#include "Adafruit_AHRS_Madgwick.h"
#include <Adafruit_AHRS_NXPFusion.h>
#include <Arduino_BMI270_BMM150.h>
#include <Wire.h>

GigaDisplay_GFX tft;

#define GC9A01A_CYAN    0x07FF
#define GC9A01A_RED     0xf800
#define GC9A01A_BLUE    0x001F
#define GC9A01A_GREEN   0x07E0
#define GC9A01A_MAGENTA 0xF81F
#define GC9A01A_WHITE   0xffff
#define GC9A01A_BLACK   0x0000
#define GC9A01A_YELLOW  0xFFE0

class MyBoschSensor : public BoschSensorClass {

public:
  MyBoschSensor(TwoWire& wire = Wire)
    : BoschSensorClass(wire){};

protected:
  virtual int8_t configure_sensor(struct bmi2_dev* dev) {
    int8_t rslt;
    uint8_t sens_list[2] = { BMI2_ACCEL, BMI2_GYRO };

    struct bmi2_int_pin_config int_pin_cfg;
    int_pin_cfg.pin_type = BMI2_INT1;
    int_pin_cfg.int_latch = BMI2_INT_NON_LATCH;
    int_pin_cfg.pin_cfg[0].lvl = BMI2_INT_ACTIVE_HIGH;
    int_pin_cfg.pin_cfg[0].od = BMI2_INT_PUSH_PULL;
    int_pin_cfg.pin_cfg[0].output_en = BMI2_INT_OUTPUT_ENABLE;
    int_pin_cfg.pin_cfg[0].input_en = BMI2_INT_INPUT_DISABLE;

    struct bmi2_sens_config sens_cfg[2];
    sens_cfg[0].type = BMI2_ACCEL;
    sens_cfg[0].cfg.acc.bwp = BMI2_ACC_OSR2_AVG2;
    sens_cfg[0].cfg.acc.odr = BMI2_ACC_ODR_100HZ;
    sens_cfg[0].cfg.acc.filter_perf = BMI2_PERF_OPT_MODE;
    sens_cfg[0].cfg.acc.range = BMI2_ACC_RANGE_4G;
    sens_cfg[1].type = BMI2_GYRO;
    sens_cfg[1].cfg.gyr.filter_perf = BMI2_PERF_OPT_MODE;
    sens_cfg[1].cfg.gyr.bwp = BMI2_GYR_OSR2_MODE;
    sens_cfg[1].cfg.gyr.odr = BMI2_GYR_ODR_100HZ;
    sens_cfg[1].cfg.gyr.range = BMI2_GYR_RANGE_2000;
    sens_cfg[1].cfg.gyr.ois_range = BMI2_GYR_OIS_2000;

    rslt = bmi2_set_int_pin_config(&int_pin_cfg, dev);
    if (rslt != BMI2_OK)
      return rslt;

    rslt = bmi2_map_data_int(BMI2_DRDY_INT, BMI2_INT1, dev);
    if (rslt != BMI2_OK)
      return rslt;

    rslt = bmi2_set_sensor_config(sens_cfg, 2, dev);
    if (rslt != BMI2_OK)
      return rslt;

    rslt = bmi2_sensor_enable(sens_list, 2, dev);
    if (rslt != BMI2_OK)
      return rslt;

    return rslt;
  }
};

MyBoschSensor myIMU(Wire1);

// Offsets applied to compensate for gyro zero-drift error for x/y/z
// Raw values converted to rad/s based on 250dps sensitiviy (1 lsb = 0.00875 rad/s)
float rawToDPS = 0.00875F;
float dpsToRad = 0.017453293F;
float gyro_zero_offsets[3] = { 0.0F, 0.0F, 0.0F };

Adafruit_Mahony filter;

float accelX, accelY, accelZ,          // units m/s/s i.e. accelZ if often 9.8 (gravity)
  gyroX, gyroY, gyroZ,                 // units dps (degrees per second)
  gyroDriftX, gyroDriftY, gyroDriftZ;  // units dps

long lastInterval, lastTime, a;


int points[8][2]; // eight 2D points for the cube, values will be calculated in the code

int previous_points[8][2];

int orig_points [8][3] = {  // eight 3D points - set values for 3D cube
{-1,-1, 1},
{1,-1,1},
{1,1,1},
{-1,1,1},
{-1,-1,-1},
{1,-1,-1},
{1,1,-1},
{-1,1,-1}
};

float rotated_3d_points [8][3];   // eight 3D points - rotated around Y axis
float angle_roll = 0;           // rotation around the X axis
float angle_pitch = 0;           // rotation around the Y axis
float angle_yaw = 0;           // rotation around the Z axis
float x_offset = 400;            // offset on Z axis
float y_offset = 240;            // offset on Z axis
float z_offset = 1600;            // offset on Z axis
float cube_size = 400.0;           // cube size (multiplier)
float time_frame;                 // ever increasing time value
float z_unit_offset = -1 * (z_offset/cube_size);

uint16_t cube_color = GC9A01A_WHITE;
uint16_t background_color = GC9A01A_BLACK;

const long displayPeriod = 10;
unsigned long previousMillis = 0;

void setup() {
  
  tft.begin();
  Serial.begin(9600);
  
  myIMU.begin();

  int calibrationCount = 0;

  float sumX, sumY, sumZ;
  int startTime = millis();
  while (millis() < startTime + 250) {
    myIMU.readAcceleration(accelX, accelY, accelZ);
    myIMU.readGyroscope(gyroX, gyroY, gyroZ);
    // in an ideal world gyroX/Y/Z == 0, anything higher or lower represents drift
    sumX += gyroX;
    sumY += gyroY;
    sumZ += gyroZ;
    calibrationCount++;
  }

  if (calibrationCount == 0) {
    Serial.println("Failed to calibrate");
  }

  gyroDriftX = sumX / calibrationCount;
  gyroDriftY = sumY / calibrationCount;
  gyroDriftZ = sumZ / calibrationCount;

  filter.begin(10);

  tft.fillScreen(background_color);
  yield();
}


void loop() {
  
  myIMU.readAcceleration(accelX, accelY, accelZ);
  myIMU.readGyroscope(gyroX, gyroY, gyroZ);
  
  unsigned long currentTime = micros();
  lastInterval = currentTime - lastTime;  // expecting this to be ~104Hz +- 4%
  lastTime = currentTime;

  // Apply gyro zero-rate error compensation
  float gx = gyroX - gyro_zero_offsets[0];
  float gy = gyroY - gyro_zero_offsets[1];
  float gz = gyroZ - gyro_zero_offsets[2];

  // Update the filter
  filter.update(gx * dpsToRad, gy * dpsToRad, gz * dpsToRad,
                accelX, accelY, accelZ,0,0,0);

  float angle_roll = -1*filter.getRoll();
  float angle_pitch = -1*filter.getPitch();
  float angle_yaw = -5*filter.getYaw();

  Serial.print(angle_roll);
  Serial.print(",");
  Serial.print(angle_pitch);
  Serial.print(",");
  Serial.println(angle_yaw);

  // calculate 3D points
  for (int i=0; i<8; i++) {

    rotated_3d_points [i][0] = orig_points [i][1] * sin(radians(angle_yaw)) * sin(radians(angle_pitch)) * cos(radians(angle_roll)) - orig_points [i][2] * cos(radians(angle_yaw)) * sin(radians(angle_pitch)) * cos(radians(angle_roll)) +
         orig_points [i][1] * cos(radians(angle_yaw)) * sin(radians(angle_roll)) + orig_points [i][2] * sin(radians(angle_yaw)) * sin(radians(angle_roll)) + orig_points [i][0] * cos(radians(angle_pitch)) * cos(radians(angle_roll));

    rotated_3d_points [i][1] = orig_points [i][1] * cos(radians(angle_yaw)) * cos(radians(angle_roll)) + orig_points [i][2] * sin(radians(angle_yaw)) * cos(radians(angle_roll)) -
         orig_points [i][1] * sin(radians(angle_yaw)) * sin(radians(angle_pitch)) * sin(radians(angle_roll)) + orig_points [i][2] * cos(radians(angle_yaw)) * sin(radians(angle_pitch)) * sin(radians(angle_roll)) -
         orig_points [i][0] * cos(radians(angle_pitch)) * sin(radians(angle_roll));

    rotated_3d_points [i][2] = orig_points [i][2] * cos(radians(angle_yaw)) * cos(radians(angle_pitch)) - orig_points [i][1] * sin(radians(angle_yaw)) * cos(radians(angle_pitch)) + orig_points [i][0] * sin(radians(angle_pitch)) + z_unit_offset;

    previous_points[i][0] = points[i][0];
    previous_points[i][1] = points[i][1];
    // project 3D points to 2d space with perspective divide -- 2D x = x/z,   2D y = y/z
    points[i][0] = round(y_offset + rotated_3d_points [i][0] / rotated_3d_points [i][2] * cube_size);
    points[i][1] = round(x_offset + rotated_3d_points [i][1] / rotated_3d_points [i][2] * cube_size);    
  }

    while (millis() - previousMillis <= displayPeriod)
      delay(1);
  
    // erase previous drawn lines
    tft.drawLine(previous_points[ 0 ][ 0 ], previous_points[ 0 ][ 1 ] , previous_points[ 1 ][ 0 ] , previous_points[ 1 ][ 1 ] , background_color);  // connect previous_points 0-1
    tft.drawLine(previous_points[ 1 ][ 0 ], previous_points[ 1 ][ 1 ] , previous_points[ 2 ][ 0 ] , previous_points[ 2 ][ 1 ] , background_color);  // connect previous_points 1-2  
    tft.drawLine(previous_points[ 2 ][ 0 ], previous_points[ 2 ][ 1 ] , previous_points[ 3 ][ 0 ] , previous_points[ 3 ][ 1 ] , background_color);  // connect previous_points 2-3      
    tft.drawLine(previous_points[ 3 ][ 0 ], previous_points[ 3 ][ 1 ] , previous_points[ 0 ][ 0 ] , previous_points[ 0 ][ 1 ] , background_color);  // connect previous_points 3-0      

    tft.drawLine(previous_points[ 4 ][ 0 ], previous_points[ 4 ][ 1 ] , previous_points[ 5 ][ 0 ] , previous_points[ 5 ][ 1 ] , background_color);  // connect previous_points 4-5
    tft.drawLine(previous_points[ 5 ][ 0 ], previous_points[ 5 ][ 1 ] , previous_points[ 6 ][ 0 ] , previous_points[ 6 ][ 1 ] , background_color);  // connect previous_points 5-6  
    tft.drawLine(previous_points[ 6 ][ 0 ], previous_points[ 6 ][ 1 ] , previous_points[ 7 ][ 0 ] , previous_points[ 7 ][ 1 ] , background_color);  // connect previous_points 6-7      
    tft.drawLine(previous_points[ 7 ][ 0 ], previous_points[ 7 ][ 1 ] , previous_points[ 4 ][ 0 ] , previous_points[ 4 ][ 1 ] , background_color);  // connect previous_points 7-4  

    tft.drawLine(previous_points[ 0 ][ 0 ], previous_points[ 0 ][ 1 ] , previous_points[ 4 ][ 0 ] , previous_points[ 4 ][ 1 ] , background_color);  // connect previous_points 0-4
    tft.drawLine(previous_points[ 1 ][ 0 ], previous_points[ 1 ][ 1 ] , previous_points[ 5 ][ 0 ] , previous_points[ 5 ][ 1 ] , background_color);  // connect previous_points 1-5  
    tft.drawLine(previous_points[ 2 ][ 0 ], previous_points[ 2 ][ 1 ] , previous_points[ 6 ][ 0 ] , previous_points[ 6 ][ 1 ] , background_color);  // connect previous_points 2-6      
    tft.drawLine(previous_points[ 3 ][ 0 ], previous_points[ 3 ][ 1 ] , previous_points[ 7 ][ 0 ] , previous_points[ 7 ][ 1 ] , background_color);  // connect previous_points 3-7    

    yield();
    
    // connect the lines between the individual points
    tft.drawLine(points[ 0 ][ 0 ], points[ 0 ][ 1 ] , points[ 1 ][ 0 ] , points[ 1 ][ 1 ] , cube_color);  // connect points 0-1
    tft.drawLine(points[ 1 ][ 0 ], points[ 1 ][ 1 ] , points[ 2 ][ 0 ] , points[ 2 ][ 1 ] , cube_color);  // connect points 1-2  
    tft.drawLine(points[ 2 ][ 0 ], points[ 2 ][ 1 ] , points[ 3 ][ 0 ] , points[ 3 ][ 1 ] , cube_color);  // connect points 2-3      
    tft.drawLine(points[ 3 ][ 0 ], points[ 3 ][ 1 ] , points[ 0 ][ 0 ] , points[ 0 ][ 1 ] , cube_color);  // connect points 3-0      

    tft.drawLine(points[ 4 ][ 0 ], points[ 4 ][ 1 ] , points[ 5 ][ 0 ] , points[ 5 ][ 1 ] , cube_color);  // connect points 4-5
    tft.drawLine(points[ 5 ][ 0 ], points[ 5 ][ 1 ] , points[ 6 ][ 0 ] , points[ 6 ][ 1 ] , cube_color);  // connect points 5-6  
    tft.drawLine(points[ 6 ][ 0 ], points[ 6 ][ 1 ] , points[ 7 ][ 0 ] , points[ 7 ][ 1 ] , cube_color);  // connect points 6-7      
    tft.drawLine(points[ 7 ][ 0 ], points[ 7 ][ 1 ] , points[ 4 ][ 0 ] , points[ 4 ][ 1 ] , cube_color);  // connect points 7-4  

    tft.drawLine(points[ 0 ][ 0 ], points[ 0 ][ 1 ] , points[ 4 ][ 0 ] , points[ 4 ][ 1 ] , cube_color);  // connect points 0-4
    tft.drawLine(points[ 1 ][ 0 ], points[ 1 ][ 1 ] , points[ 5 ][ 0 ] , points[ 5 ][ 1 ] , cube_color);  // connect points 1-5  
    tft.drawLine(points[ 2 ][ 0 ], points[ 2 ][ 1 ] , points[ 6 ][ 0 ] , points[ 6 ][ 1 ] , cube_color);  // connect points 2-6      
    tft.drawLine(points[ 3 ][ 0 ], points[ 3 ][ 1 ] , points[ 7 ][ 0 ] , points[ 7 ][ 1 ] , cube_color);  // connect points 3-7                 

    previousMillis = millis();
  
}
