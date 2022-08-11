/* * * * * * * * * * * * * * * * * * * * * * * * * * * * * * * * * * * * * * * * * * * * * * * * * *
 *  Water parameters monitoring sensor firmware
 *  For Libelium Plug&Sense Smart Water
 *  2018, Alisher Khassanov <alisher@aira.life>
 *  BSD 3-Clause License
 * * * * * * * * * * * * * * * * * * * * * * * * * * * * * * * * * * * * * * * * * * * * * * * * * *
 * 
 *  Wiring
 *  - PTSM     : Temperature Sensor (Pt-1000)
 *  - SOCKET_A : pH
 *  - SOCKET_B : Dissolved Oxygen (DO)
 *  - SOCKET_C : Conductivity
 *  - SOCKET_D : NC
 *  - SOCKET_E : NC
 *  - SOCKET_F : NC
 *  
 *  Calibration
 *  - Sensors works since July 2018
 *  - Latest calibration 21.04.2019
 *  - Latest check 21.04.2019
 *  
 * * * * * * * * * * * * * * * * * * * * * * * * * * * * * * * * * * * * * * * * * * * * * * * * */

#include <WaspSensorSW.h>
#include <WaspFrame.h>

char node_ID[] = "WATER";

// pH Sensor calibration values
// Calibration values
#define cal_point_10  1.978
#define cal_point_7   2.082
#define cal_point_4   2.250

// Temperature at which calibration was carried out
#define cal_temp 22.46

// Conductivity Sensor calibration values
// Value 1 used to calibrate the sensor
#define point1_cond 1413
// Value 2 used to calibrate the sensor
#define point2_cond 84

// Point 1 of the calibration 
#define point1_cal 845
// Point 2 of the calibration 
#define point2_cal 11857

// ORP sensor calibration
#define calibration_offset 0.0

pHClass pHSensor;
DOClass DOSensor;
conductivityClass ConductivitySensor;
pt1000Class TemperatureSensor;
ORPClass ORPSensor;

float ORPValue;
float value_temp;
float value_pH;
float value_pH_calculated;
float value_do;
float value_do_calculated;
float value_cond;
float value_cond_calculated;

// define file name: MUST be 8.3 SHORT FILE NAME
char filename[30]= {0};
char text[40] = {0};
// define variable
uint8_t sd_answer;

int count = 0;


void create_file(int dop)
{
  int32_t files_number = SD.numFiles() + dop;
  snprintf(filename, sizeof(filename), "W%d.TXT", files_number);
  sd_answer = SD.create(filename);
  if( sd_answer == 1 )
  {
    snprintf(text, sizeof(text),"file created %s", filename);
    USB.println(text);
  }
  else 
  {
    USB.println(F("file NOT created"));  
    dop++;
    create_file(dop);
  } 
}

void setup()
{
  USB.ON();
  frame.setID(node_ID);

  // Set SD ON
  SD.ON();
   // Delete file
  //sd_answer = SD.del(filename);
  create_file(0);
  
  // Configure the calibration values
  pHSensor.setCalibrationPoints(cal_point_10, cal_point_7, cal_point_4, cal_temp);
  ConductivitySensor.setCalibrationPoints(point1_cond, point1_cal, point2_cond, point2_cal); 
  RTC.ON();
  
  // Setting time [yy:mm:dd:dow:hh:mm:ss]
  RTC.setTime("21:07:06:06:00:00:00"); 
}

void loop()
{
  Water.ON();
  delay(1900);

  ORPValue = ORPSensor.readORP();
  ORPValue = ORPValue - calibration_offset;
  value_temp = TemperatureSensor.readTemperature();
  value_pH = pHSensor.readpH();
  value_pH_calculated = pHSensor.pHConversion(value_pH, value_temp); // temperature correction
  value_cond = ConductivitySensor.readConductivity();
  value_cond_calculated = ConductivitySensor.conductivityConversion(value_cond); // from resistance into ms/cm

  Water.OFF();

  frame.createFrame(ASCII);
  frame.addSensor(SENSOR_WATER_WT, value_temp); // toC
  frame.addSensor(SENSOR_WATER_PH, value_pH_calculated); // 0-14
  frame.addSensor(SENSOR_WATER_COND, value_cond_calculated); // muS/cm
  frame.addSensor(SENSOR_WATER_ORP, ORPValue);
  frame.addSensor(SENSOR_STR, RTC.getTime());

  SD.appendln(filename, (char*) frame.buffer);
  //SD.showFile(filename);
  delay(100);
  
  count++;
  //USB.println(count);
  USB.println((char*) frame.buffer);
  //USB.println(RTC.getTime());
}


