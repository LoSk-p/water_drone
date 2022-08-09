/*  
 *  ------ [SWI_07] - Plug&Sense reading for Smart Water Ions-------- 
 *  
 *  Explanation: This is an specific example to be used with 
 *  Plug&Sense Platform. In this example we can see how to read 
 *  from each socket of the Plug&Sense device. 
 *  
 *  Copyright (C) 2016 Libelium Comunicaciones Distribuidas S.L. 
 *  http://www.libelium.com 
 *  
 *  This program is free software: you can redistribute it and/or modify 
 *  it under the terms of the GNU General Public License as published by 
 *  the Free Software Foundation, either version 3 of the License, or 
 *  (at your option) any later version. 
 *  
 *  This program is distributed in the hope that it will be useful, 
 *  but WITHOUT ANY WARRANTY; without even the implied warranty of 
 *  MERCHANTABILITY or FITNESS FOR A PARTICULAR PURPOSE.  See the 
 *  GNU General Public License for more details. 
 *  
 *  You should have received a copy of the GNU General Public License 
 *  along with this program.  If not, see <http://www.gnu.org/licenses/>. 
 *  
 *  Version:           3.0
 *  Design:            David Gascón 
 *  Implementation:    Ahmad Saad
 */

#include <smartWaterIons.h>
#include <WaspFrame.h>

char node_ID[] = "IONS";

// All Ion sensors can be connected in the four sockets
// In Plug&Sense SOCKETE is reserved for reference probe
//======================================================================
// Plug&Sense SOCKETS
//====================================================================== 
ionSensorClass NO2_Sensor(SOCKET_A);
ionSensorClass NO3_Sensor(SOCKET_B);
ionSensorClass NH4_Sensor(SOCKET_C);
pt1000Class tempSensor;
//======================================================================
// Calibration concentrations solutions used in the process
//======================================================================
#define point1 10.0
#define point2 100.0
#define point3 1000.0
//======================================================================
// Calibration voltage values for NO2 sensor
//======================================================================
#define point1_volt_NO2 2.801
#define point2_volt_NO2 2.729
#define point3_volt_NO2 2.590
//======================================================================
// Calibration voltage values for NO3 sensor
//======================================================================
#define point1_volt_NO3 2.701
#define point2_volt_NO3 2.678
#define point3_volt_NO3 2.545
//======================================================================
// Calibration voltage values for NH4 sensor
//======================================================================
// points
#define point1_NH4 4.0
#define point2_NH4 20.0
#define point3_NH4 40.0

const float concentrations_NH4[] = { 
  point1_NH4, point2_NH4, point3_NH4 };

// voltages
#define point1_volt_NH4 2.526
#define point2_volt_NH4 2.722
#define point3_volt_NH4 2.816
//======================================================================
// Define the number of calibration points
//======================================================================
#define NUM_POINTS 3

//======================================================================
const float concentrations[] = { 
  point1, point2, point3 };
const float voltages_NO2[] = { 
  point1_volt_NO2, point2_volt_NO2, point3_volt_NO2}; 
const float voltages_NO3[] = { 
  point1_volt_NO3, point2_volt_NO3, point3_volt_NO3 }; 
const float voltages_NH4[] = { 
  point1_volt_NH4, point2_volt_NH4, point3_volt_NH4 }; 
//======================================================================

// define file name: MUST be 8.3 SHORT FILE NAME
char filename[30]= {0};
char text[40] = {0};
// define variable
uint8_t sd_answer;

int count = 0;

void create_file(int dop)
{
  int32_t files_number = SD.numFiles() + dop;
  snprintf(filename, sizeof(filename), "ION%d.TXT", files_number);
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
  // Turn ON the Smart Water Ions Board and USB
  SWIonsBoard.ON();
  USB.ON(); 
  frame.setID(node_ID); 
  
  // Set SD ON
  SD.ON();
  create_file(0);

  // Calibrate the NO2 sensor
  NO2_Sensor.setCalibrationPoints(voltages_NO2, concentrations, NUM_POINTS); 
  // Calibrate the NO3 sensor
  NO3_Sensor.setCalibrationPoints(voltages_NO3, concentrations, NUM_POINTS);  
  // Calibrate the NH4 sensor
  NH4_Sensor.setCalibrationPoints(voltages_NH4, concentrations_NH4, NUM_POINTS);  

  RTC.ON();
  // Setting time [yy:mm:dd:dow:hh:mm:ss]
  RTC.setTime("21:07:06:06:00:00:00"); 
}

void loop()
{
  SWIonsBoard.ON();
  //==========================================================
  // Read the NO2 sensor
  //==========================================================
  float NO2_Volts = NO2_Sensor.read();
  float NO2_Value = NO2_Sensor.calculateConcentration(NO2_Volts);
  delay(100);


  //==========================================================
  // Read the NO3 sensor
  //==========================================================
  float NO3_Volts = NO3_Sensor.read();
  float NO3_Value = NO3_Sensor.calculateConcentration(NO3_Volts);
  delay(100);

  //==========================================================
  // Read the NH4 sensor
  //==========================================================
  float NH4_Volts = NH4_Sensor.read();
  float NH4_Value = NH4_Sensor.calculateConcentration(NH4_Volts);
  delay(100);

  //==========================================================
  // Read the Temperature sensor
  //==========================================================
  float temp_Value = tempSensor.read();
  delay(100);

  SWIonsBoard.OFF();

  frame.createFrame(ASCII);
  frame.addSensor(SENSOR_IONS_NO2, NO2_Value);
  frame.addSensor(SENSOR_IONS_NO3, NO3_Value);
  frame.addSensor(SENSOR_IONS_NH4, NH4_Value);
  frame.addSensor(SENSOR_IN_TEMP, temp_Value);
  frame.addSensor(SENSOR_STR, RTC.getTime());

  SD.appendln(filename, (char*) frame.buffer);
  delay(100);


  USB.println((char*) frame.buffer);
  delay(1000); 
}

