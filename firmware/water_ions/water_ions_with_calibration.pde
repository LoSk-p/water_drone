#include <smartWaterIons.h>
#include <WaspFrame.h>

//----------------------------------------------------------------------------------------
//!!!!!!!!!!!!!!!!!!!!!!!!!!!!!!!!!!!!!!!!!!!!!!!!!!!!!!!!!!!!!!!!!!!!!!!!!!!!!!!!!!!!!!!!
//ОСТАЛЬНЫЕ НАСТРОЙКИ МЕНЯТЬ ЗАПРЕЩАЕТСЯ!
//!!!!!!!!!!!!!!!!!!!!!!!!!!!!!!!!!!!!!!!!!!!!!!!!!!!!!!!!!!!!!!!!!!!!!!!!!!!!!!!!!!!!!!!!
char check_md5[] = "417e4d803cefa2397901a94089f91e21";
char filename[] = "SWIons1_0.hex";

// 0 - show nothing (calibration mode)
// 1 - show frame
// 2 - show data
int ShowData = 2;

int debug = 0;

// Время калибровки
int counter = 100;
int zadergka = 500;

//версия прошивки (хранится в EEPROM)
int addressFVMajor = 1025;
int addressFVMinor = 1026;
int FVMajor = 1;
int FVMinor = 1;
int auxFVMajor = 0;
int auxFVMinor = 0;

// serial transfer vars
unsigned long timer = 0;
int command = 0;

// калибровочные точки для pH датчика
// float cal_point_10 = 1.985;
// float cal_point_7 = 2.070;
// float cal_point_4 = 2.227;
// int addr_p10 = 1030;
// int addr_p7 = 1035;
// int addr_p4 = 1039;
// long aux_p10 = 0;
// long aux_p7 = 0;
// long aux_p4 = 0;
//-----------------------------------
// Температура калибровки
float cal_temp = 23.7;
int addr_cal_temp = 1043;
long aux_cal_temp = 0;

//calib coeffs for sensor on socket A
float A_point1_V = 0.0;
float A_point2_V = 0.0;
float A_point3_V = 0.0;
int addr_A_p1 = 1047;
int addr_A_p2 = 1051;
int addr_A_p3 = 1055;
long aux_A_p1 = 0;
long aux_A_p2 = 0;
long aux_A_p3 = 0;

//calib coeffs for sensor on socket B
float B_point1_V = 0.0;
float B_point2_V = 0.0;
float B_point3_V = 0.0;
int addr_B_p1 = 1059;
int addr_B_p2 = 1063;
int addr_B_p3 = 1067;
long aux_B_p1 = 0;
long aux_B_p2 = 0;
long aux_B_p3 = 0;

//calib coeffs for sensor on socket C
float C_point1_V = 0.0;
float C_point2_V = 0.0;
float C_point3_V = 0.0;
int addr_C_p1 = 1071;
int addr_C_p2 = 1075;
int addr_C_p3 = 1079;
long aux_C_p1 = 0;
long aux_C_p2 = 0;
long aux_C_p3 = 0;

//calib coeffs for sensor on socket D
float D_point1_V = 0.0;
float D_point2_V = 0.0;
float D_point3_V = 0.0;
int addr_D_p1 = 1083;
int addr_D_p2 = 1087;
int addr_D_p3 = 1091;
long aux_D_p1 = 0;
long aux_D_p2 = 0;
long aux_D_p3 = 0;

// A,B,D,C - any ion sensor
// F - pt1000 sensor
ionSensorClass SensorSocketA(SOCKET_A); // socket 1 - NH4
ionSensorClass SensorSocketB(SOCKET_B); // socket 2 - NO3
ionSensorClass SensorSocketC(SOCKET_C); // socket 3 - NO2
ionSensorClass SensorSocketD(SOCKET_D); // socket 4 - Cl
pt1000Class tempSensor;

// Calibration concentrations
float concent_A_1 = 4.0;
float concent_A_2 = 20.0;
float concent_A_3 = 40.0;
float concent_B_1 = 132.0;
float concent_B_2 = 660.0;
float concent_B_3 = 1320.0;
float concent_C_1 = 10.0;
float concent_C_2 = 100.0;
float concent_C_3 = 1000.0;
float concent_D_1 = 75.0;
float concent_D_2 = 375.0;
float concent_D_3 = 750.0;

// calibration concentrations
#define point1 10.0
#define point2 100.0
#define point3 1000.0

float SOCK_A_Raw = 0.0;
float SOCK_B_Raw = 0.0;
float SOCK_C_Raw = 0.0;
float SOCK_D_Raw = 0.0;
float SOCK_A_Calc = 0.0;
float SOCK_B_Calc = 0.0;
float SOCK_C_Calc = 0.0;
float SOCK_D_Calc = 0.0;
// float value_pH = 0.0;
// float value_pH_calculated = 0.0;
float value_temp = 0.0;


void setup() {
    USB.ON();
    SWIonsBoard.ON();
    cal_temp = LongToFloat(EEPROMReadLong(addr_cal_temp));
    // cal_point_10 = LongToFloat(EEPROMReadLong(addr_p10));
    // cal_point_7 = LongToFloat(EEPROMReadLong(addr_p7));
    // cal_point_4 = LongToFloat(EEPROMReadLong(addr_p4));
    A_point1_V = LongToFloat(EEPROMReadLong(addr_A_p1));
    A_point2_V = LongToFloat(EEPROMReadLong(addr_A_p2));
    A_point3_V = LongToFloat(EEPROMReadLong(addr_A_p3));
    B_point1_V = LongToFloat(EEPROMReadLong(addr_B_p1));
    B_point2_V = LongToFloat(EEPROMReadLong(addr_B_p2));
    B_point3_V = LongToFloat(EEPROMReadLong(addr_B_p3));
    C_point1_V = LongToFloat(EEPROMReadLong(addr_C_p1));
    C_point2_V = LongToFloat(EEPROMReadLong(addr_C_p2));
    C_point3_V = LongToFloat(EEPROMReadLong(addr_C_p3));
    D_point1_V = LongToFloat(EEPROMReadLong(addr_D_p1));
    D_point2_V = LongToFloat(EEPROMReadLong(addr_D_p2));
    D_point3_V = LongToFloat(EEPROMReadLong(addr_D_p3));

    Utils.writeEEPROM(addressFVMajor, FVMajor);
    Utils.writeEEPROM(addressFVMinor, FVMinor);
    auxFVMajor = Utils.readEEPROM(addressFVMajor);
    auxFVMinor = Utils.readEEPROM(addressFVMajor);

    delay(1000);

    const float concentrations_A[] = {concent_A_1, concent_A_2, concent_A_3};
    const float concentrations_B[] = {concent_B_1, concent_B_2, concent_B_3};
    const float concentrations_C[] = {concent_C_1, concent_C_2, concent_C_3};
    const float concentrations_D[] = {concent_D_1, concent_D_2, concent_D_3};
    const float voltages_A[] = {A_point1_V, A_point2_V, A_point3_V};
    const float voltages_B[] = {B_point1_V, B_point2_V, B_point3_V};
    const float voltages_C[] = {C_point1_V, C_point2_V, C_point3_V};
    const float voltages_D[] = {D_point1_V, D_point2_V, D_point3_V};

    SensorSocketA.setCalibrationPoints(voltages_A, concentrations_A, 3);
    SensorSocketB.setCalibrationPoints(voltages_B, concentrations_B, 3);
    SensorSocketC.setCalibrationPoints(voltages_C, concentrations_C, 3);
    SensorSocketD.setCalibrationPoints(voltages_D, concentrations_D, 3);
    // pHSensor.setpHCalibrationPoints(cal_point_10, cal_point_7, cal_point_4, cal_temp);

    // frame.setID(node_ID);
    // WiFi_Init();
  
}

void loop() {

  if (ShowData == 2) {
    SensorData();
    USB.flush();
  }

  // USB commands reading
  timer = millis();
  while(millis()-timer < 5000)
  {
    if (USB.available() > 0)
    {
      command = USB.read();      
      switch (command) {
        case 65: // A
          //USB.flush();
          ShowData = 0;
          USB.println(F("#!"));
          break;
        // case 66: //B
        //   ShowData = 1;
        //   USB.println(F("#+"));
        //   break;
        case 67: // C
          ShowData = 2;
          USB.println(F("#-"));
          break;
        case 121: // y
          debug = 1;
          break;
        case 122: // z
           ShowCoeff();
        case 100: // d
          //TEMP_SENSOR_TYPE = 1;
          //if (debug == 1) { USB.println(F("Temp from turbidity sensor")); }
          break;
        case 101: // e
          //TEMP_SENSOR_TYPE = 0;
          //if (debug == 1) { USB.println(F("temp from pt1000")); }
          break;
        case 116: // t
          counter = 60;
          USB.println(F("#t"));
          if (debug == 1) { USB.println(counter); }
          break;
        case 117: // u
          counter = 120;
          USB.println(F("#u"));
          if (debug == 1) { USB.println(counter); }
          break;
        case 118: // v
          counter = 180;
          USB.println(F("#v"));
          if (debug == 1) { USB.println(counter); }
          break;
        case 119: // w
          counter = 240;
          USB.println(F("#w"));
          if (debug == 1) { USB.println(counter); }
          break;
        case 120: // x
          counter = 300;
          USB.println(F("#x"));
          if (debug == 1) { USB.println(counter); }
          break;
        case 97: // a          
          delay(1000);
          USB.println(F("#?"));
          Socket_A_Calib(concent_A_1);
          USB.println(F("#a"));                   
          break;
        case 98: // b
          delay(1000);
          USB.println(F("#?"));
          Socket_A_Calib(concent_A_2);
          USB.println(F("#b"));
          break;
        case 99: // c
          delay(1000);
          USB.println(F("#?"));
          Socket_A_Calib(concent_A_3);
          USB.println(F("#c"));
          break;
        case 107: // k
          delay(1000);
          USB.println(F("#?"));
          Socket_B_Calib(concent_B_1);
          USB.println(F("#k"));
          break;
        case 108: // l
          delay(1000);
          USB.println(F("#?"));
          Socket_B_Calib(concent_B_2);
          USB.println(F("#l"));
          break;
        case 109: // m
          delay(1000);
          USB.println(F("#?"));
          Socket_B_Calib(concent_B_3);
          USB.println(F("#m"));
          break;
        case 110: // n
          delay(1000);
          USB.println(F("#?"));
          Socket_D_Calib(concent_D_1);
          USB.println(F("#n"));
          break;
        case 111: // o
          delay(1000);
          USB.println(F("#?"));
          Socket_D_Calib(concent_D_2);
          USB.println(F("#0"));
          break;
        case 112: // p
          delay(1000);
          USB.println(F("#?"));
          Socket_C_Calib(concent_C_1);
          USB.println(F("#p"));
          break;
        case 113: // q
          delay(1000);
          USB.println(F("#?"));
          Socket_C_Calib(concent_C_2);
          USB.println(F("#q"));
          break;
        case 114: // r
          delay(1000);
          USB.println(F("#?"));
          Socket_C_Calib(concent_C_3);
          USB.println(F("#r"));
          break;
        case 115: // s
          delay(1000);
          USB.println(F("#?"));
          Socket_D_Calib(concent_D_3);
          USB.println(F("#s"));
          break;
//        case 102: // f
//          auxFVMajor = Utils.readEEPROM(addressFVMajor);
//          auxFVMinor = Utils.readEEPROM(addressFVMajor);
//          //USB.print(F("Имя устройства: "));
//          //USB.println(node_ID);
//          USB.print(F("<|"));
//          //delay(1000);
//          //USB.println();
//          USB.print(node_ID);
//          USB.print("|");
//          ShowSerialNumber();
//          USB.print("|");
//          //USB.print(F("Версия прошивки: "));
//          USB.print(auxFVMajor);
//          USB.print(F("."));
//          //USB.println(auxFVMinor);
//          USB.print(auxFVMinor);
//          USB.print("|");
//          USB.print(check_md5);
//          USB.print("|");
//          USB.print(filename);
//          USB.println(F("|>"));
//          delay(5000);
//          break;
        default:
          //USB.println(F("Wrong command!"));  
          break;      
      }
    }
  }
}

//------------------------------------------
long FloatToLong(float val) {
  long a = val * 1000;
  return a;
}

float LongToFloat(long val1) {
  float b = (float) val1/1000.0;
  return b; 
}

//запись в EEPROM данных типа long
void EEPROMWriteLong(int address, long value) {
  byte four = (value & 0xFF);
  byte three = ((value >> 8) & 0xFF);
  byte two = ((value >> 16) & 0xFF);
  byte one = ((value >> 24) & 0xFF);

  Utils.writeEEPROM(address, four);
  Utils.writeEEPROM(address + 1, three);
  Utils.writeEEPROM(address + 2, two);
  Utils.writeEEPROM(address + 3, one);
}

//чтение из EEPROM данных типа long
long EEPROMReadLong(int address1) {
  long four = Utils.readEEPROM(address1);
  long three = Utils.readEEPROM(address1 + 1);
  long two = Utils.readEEPROM(address1 + 2);
  long one = Utils.readEEPROM(address1 + 3);

  return ((four << 0) & 0xFF) + ((three << 8) & 0xFFFF) + ((two << 16) & 0xFFFFFF) + ((one << 24) & 0xFFFFFFFF);
}

//--------------------------------------
// калибровка датчика в сокете А, точка калибровки задается аргументом conc_temp 
void Socket_A_Calib(int conc_temp)
{
  float volts = 0.0;
  float concent[] = {concent_A_1, concent_A_2, concent_A_3};
  //float temp_volts_A_Cal[] = {0.0, 0.0, 0.0};
  aux_A_p1 = EEPROMReadLong(addr_A_p1);
  aux_A_p2 = EEPROMReadLong(addr_A_p2);
  aux_A_p3 = EEPROMReadLong(addr_A_p3);
  const float temp_volts_A[] = {LongToFloat(aux_A_p1), LongToFloat(aux_A_p2), LongToFloat(aux_A_p3)};
  SensorSocketA.setCalibrationPoints(temp_volts_A, concent, 3);
  for (int i = 0; i < counter; i++) {
    SWIonsBoard.ON();
    volts = SensorSocketA.read();
    if (debug == 1) {
      USB.print(volts);
      USB.print(F(" - "));
      USB.println(i);
    }
    SWIonsBoard.OFF();
    delay(zadergka);
  }
  if (conc_temp == concent_A_1)
  {
    EEPROMWriteLong(addr_A_p1, FloatToLong(volts));
    const float temp_volts_A_Cal_10[] = {volts, LongToFloat(aux_A_p2), LongToFloat(aux_A_p3)};
    SensorSocketA.setCalibrationPoints(temp_volts_A_Cal_10, concent, 3);
  }
  else if (conc_temp == concent_A_2)
  {
    EEPROMWriteLong(addr_A_p2, FloatToLong(volts));
    const float temp_volts_A_Cal_100[] = {LongToFloat(aux_A_p1), volts, LongToFloat(aux_A_p3)};
    SensorSocketA.setCalibrationPoints(temp_volts_A_Cal_100, concent, 3);
  }
  else if (conc_temp == concent_A_3)
  {
    EEPROMWriteLong(addr_A_p3, FloatToLong(volts));
    const float temp_volts_A_Cal_1000[] = {LongToFloat(aux_A_p1), LongToFloat(aux_A_p2), volts};
    SensorSocketA.setCalibrationPoints(temp_volts_A_Cal_1000, concent, 3);
  }
  else
  {
    //const float temp_volts_A_Cal = {volts, volts, volts};//do nothing
  }
  
  //SensorSocketA.setCalibrationPoints(temp_volts_A_Cal, concent, 3);
  if (debug == 1) {
    USB.print(F("Калибровка датчика в сокете А для раствора с концентрацией ")); 
    USB.print(conc_temp);
    USB.println(" завершена.");  
  }
}

//--------------------------------------
// калибровка датчика в сокете B, точка калибровки задается аргументом conc_temp 
void Socket_B_Calib(int conc_temp)
{
  float volts = 0.0;
  float concent[] = {concent_B_1, concent_B_2, concent_B_3};  
  aux_B_p1 = EEPROMReadLong(addr_B_p1);
  aux_B_p2 = EEPROMReadLong(addr_B_p2);
  aux_B_p3 = EEPROMReadLong(addr_B_p3);
  const float temp_volts_B[] = {LongToFloat(aux_B_p1), LongToFloat(aux_B_p2), LongToFloat(aux_B_p3)};
  SensorSocketB.setCalibrationPoints(temp_volts_B, concent, 3);
  for (int i = 0; i < counter; i++) {
    SWIonsBoard.ON();
    volts = SensorSocketB.read();
    if (debug == 1) {
      USB.print(volts);
      USB.print(F(" - "));
      USB.println(i);
    }
    SWIonsBoard.OFF();
    delay(zadergka);
  }
  if (conc_temp == concent_B_1)
  {
    EEPROMWriteLong(addr_B_p1, FloatToLong(volts));
    const float temp_volts_B_Cal_10[] = {volts, LongToFloat(aux_B_p2), LongToFloat(aux_B_p3)};
    SensorSocketB.setCalibrationPoints(temp_volts_B_Cal_10, concent, 3);
  }
  else if (conc_temp == concent_B_2)
  {
    EEPROMWriteLong(addr_B_p2, FloatToLong(volts));
    const float temp_volts_B_Cal_100[] = {LongToFloat(aux_B_p1), volts, LongToFloat(aux_B_p3)};
    SensorSocketB.setCalibrationPoints(temp_volts_B_Cal_100, concent, 3);
  }
  else if (conc_temp == concent_B_3)
  {
    EEPROMWriteLong(addr_B_p3, FloatToLong(volts));
    const float temp_volts_B_Cal_1000[] = {LongToFloat(aux_B_p1), LongToFloat(aux_B_p2), volts};
    SensorSocketB.setCalibrationPoints(temp_volts_B_Cal_1000, concent, 3);
  }
  else
  {
    //do nothing
  }
  
  //SensorSocketB.setCalibrationPoints(temp_volts_B_Cal, concent, 3);
  if (debug == 1) {
    USB.print(F("Калибровка датчика в сокете B для раствора с концентрацией ")); 
    USB.print(conc_temp);
    USB.println(" завершена.");  
  }
}

//--------------------------------------
// калибровка датчика в сокете C, точка калибровки задается аргументом conc_temp 
void Socket_C_Calib(int conc_temp)
{
  float volts = 0.0;
  float concent[] = {concent_C_1, concent_C_2, concent_C_3};
  float temp_volts_C_Cal[] = {0.0, 0.0, 0.0};
  aux_C_p1 = EEPROMReadLong(addr_C_p1);
  aux_C_p2 = EEPROMReadLong(addr_C_p2);
  aux_C_p3 = EEPROMReadLong(addr_C_p3);
  float temp_volts_C[] = {LongToFloat(aux_C_p1), LongToFloat(aux_C_p2), LongToFloat(aux_C_p3)};
  SensorSocketC.setCalibrationPoints(temp_volts_C, concent, 3);
  SWIonsBoard.ON();
  for (int i = 0; i < counter; i++) {
    SWIonsBoard.ON();
    volts = SensorSocketC.read();
    if (debug == 1) {
      USB.print(volts);
      USB.print(F(" - "));
      USB.println(i);
    }
    SWIonsBoard.OFF();
    delay(zadergka);
  }
  if (conc_temp == concent_C_1)
  {
    EEPROMWriteLong(addr_C_p1, FloatToLong(volts));
    const float temp_volts_C_Cal_10[] = {volts, LongToFloat(aux_C_p2), LongToFloat(aux_C_p3)};
    SensorSocketC.setCalibrationPoints(temp_volts_C_Cal_10, concent, 3);
  }
  else if (conc_temp == concent_C_2)
  {
    EEPROMWriteLong(addr_C_p2, FloatToLong(volts));
    const float temp_volts_C_Cal_100[] = {LongToFloat(aux_C_p1), volts, LongToFloat(aux_C_p3)};
    SensorSocketC.setCalibrationPoints(temp_volts_C_Cal_100, concent, 3);
  }
  else if (conc_temp == concent_C_3)
  {
    EEPROMWriteLong(addr_C_p3, FloatToLong(volts));
    const float temp_volts_C_Cal_1000[] = {LongToFloat(aux_C_p1), LongToFloat(aux_C_p2), volts};
    SensorSocketC.setCalibrationPoints(temp_volts_C_Cal_1000, concent, 3);
  }
  else
  {
    //do nothing
  }
  
  //SensorSocketС.setCalibrationPoints(temp_volts_С_Cal, concent, 3);
  if (debug == 1) {
    USB.print(F("Калибровка датчика в сокете C для раствора с концентрацией ")); 
    USB.print(conc_temp);
    USB.println(" завершена.");  
  }
}

//--------------------------------------
// калибровка датчика в сокете D, точка калибровки задается аргументом conc_temp 
void Socket_D_Calib(int conc_temp)
{
  float volts = 0.0;
  float concent[] = {concent_D_1, concent_D_2, concent_D_3};
  float temp_volts_D_Cal[] = {0.0, 0.0, 0.0};
  aux_D_p1 = EEPROMReadLong(addr_D_p1);
  aux_D_p2 = EEPROMReadLong(addr_D_p2);
  aux_D_p3 = EEPROMReadLong(addr_D_p3);
  float temp_volts_D[] = {LongToFloat(aux_D_p1), LongToFloat(aux_D_p2), LongToFloat(aux_D_p3)};
  SensorSocketD.setCalibrationPoints(temp_volts_D, concent, 3);
  for (int i = 0; i < counter; i++) {
    SWIonsBoard.ON();
    volts = SensorSocketD.read();
    if (debug == 1) {
      USB.print(volts);
      USB.print(F(" - "));
      USB.println(i);
    }
    SWIonsBoard.OFF();
    delay(zadergka);
  }
  if (conc_temp == concent_D_1)
  {
    EEPROMWriteLong(addr_D_p1, FloatToLong(volts));
    const float temp_volts_D_Cal_10[] = {volts, LongToFloat(aux_D_p2), LongToFloat(aux_D_p3)};
    SensorSocketD.setCalibrationPoints(temp_volts_D_Cal_10, concent, 3);
  }
  else if (conc_temp == concent_D_2)
  {
    EEPROMWriteLong(addr_D_p2, FloatToLong(volts));
    const float temp_volts_D_Cal_100[] = {LongToFloat(aux_D_p1), volts, LongToFloat(aux_D_p3)};
    SensorSocketD.setCalibrationPoints(temp_volts_D_Cal_100, concent, 3);
  }
  else if (conc_temp == concent_D_3)
  {
    EEPROMWriteLong(addr_D_p3, FloatToLong(volts));
    const float temp_volts_D_Cal_1000[] = {LongToFloat(aux_D_p1), LongToFloat(aux_D_p2), volts};
    SensorSocketD.setCalibrationPoints(temp_volts_D_Cal_1000, concent, 3);
  }
  else
  {
    //do nothing
  }
  
  //SensorSocketD.setCalibrationPoints(temp_volts_D_Cal, concent, 3);
  if (debug == 1) {
    USB.print(F("Калибровка датчика в сокете D для раствора с концентрацией ")); 
    USB.print(conc_temp);
    USB.println(" завершена.");  
  }
}

void ShowSerialNumber() {
  //USB.print(F("Serial ID:"));
  USB.printHex(_serial_id[0]);
  USB.printHex(_serial_id[1]);
  USB.printHex(_serial_id[2]);
  USB.printHex(_serial_id[3]);
  USB.printHex(_serial_id[4]);
  USB.printHex(_serial_id[5]);
  USB.printHex(_serial_id[6]);
  USB.printHex(_serial_id[7]);
  //USB.println();
  //delay(1000);
}

void ShowCoeff() {
  cal_temp = LongToFloat(EEPROMReadLong(addr_cal_temp));
//  cal_point_10 = LongToFloat(EEPROMReadLong(addr_p10));
//  cal_point_7 = LongToFloat(EEPROMReadLong(addr_p7));
//  cal_point_4 = LongToFloat(EEPROMReadLong(addr_p4));
  A_point1_V = LongToFloat(EEPROMReadLong(addr_A_p1));
  A_point2_V = LongToFloat(EEPROMReadLong(addr_A_p2));
  A_point3_V = LongToFloat(EEPROMReadLong(addr_A_p3));
  B_point1_V = LongToFloat(EEPROMReadLong(addr_B_p1));
  B_point2_V = LongToFloat(EEPROMReadLong(addr_B_p2));
  B_point3_V = LongToFloat(EEPROMReadLong(addr_B_p3));
  C_point1_V = LongToFloat(EEPROMReadLong(addr_C_p1));
  C_point2_V = LongToFloat(EEPROMReadLong(addr_C_p2));
  C_point3_V = LongToFloat(EEPROMReadLong(addr_C_p3));
  D_point1_V = LongToFloat(EEPROMReadLong(addr_D_p1));
  D_point2_V = LongToFloat(EEPROMReadLong(addr_D_p2));
  D_point3_V = LongToFloat(EEPROMReadLong(addr_D_p3));

//  USB.println(cal_point_10);
//  USB.println(cal_point_7);
//  USB.println(cal_point_4);
  USB.println(F("NH4: concent - volts"));
  USB.print(concent_A_1);
  USB.print(F(" - "));
  USB.println(A_point1_V);
  USB.print(concent_A_2);
  USB.print(F(" - "));
  USB.println(A_point2_V);
  USB.print(concent_A_3);
  USB.print(F(" - "));
  USB.println(A_point3_V);
  USB.println(F("NO3: concent - volts"));
  USB.print(concent_B_1);
  USB.print(F(" - "));
  USB.println(B_point1_V);
  USB.print(concent_B_2);
  USB.print(F(" - "));
  USB.println(B_point2_V);
  USB.print(concent_B_3);
  USB.print(F(" - "));
  USB.println(B_point3_V);
  USB.println(F("NO2: concent - volts"));
  USB.print(concent_C_1);
  USB.print(F(" - "));
  USB.println(C_point1_V);
  USB.print(concent_C_2);
  USB.print(F(" - "));
  USB.println(C_point2_V);
  USB.print(concent_C_3);
  USB.print(F(" - "));
  USB.println(C_point3_V);
  USB.println(F("Cl: concent - volts"));
  USB.print(concent_D_1);
  USB.print(F(" - "));
  USB.println(D_point1_V);
  USB.print(concent_D_2);
  USB.print(F(" - "));
  USB.println(D_point2_V);
  USB.print(concent_D_3);
  USB.print(F(" - "));
  USB.println(D_point3_V);
  USB.println(F("temp: concent - volts"));
  USB.println(cal_temp);  
}

void SensorData() {
  SWIonsBoard.ON();
    //delay(2000);
    ///////////////////////////////////////////
    // 2. Read sensors
    /////////////////////////////////////////// 
    value_temp = tempSensor.read();    
    // value_pH = pHSensor.read();
    // value_pH_calculated = pHSensor.pHConversion(value_pH, value_temp);
    // delay(500);
    SOCK_A_Raw = SensorSocketA.read();
    SOCK_A_Calc = SensorSocketA.calculateConcentration(SOCK_A_Raw);
    if (debug == 1) {
      USB.print(F("NH4 volts: "));
      USB.print(SOCK_A_Raw);
      USB.print(F(", value: "));
      USB.println(SOCK_A_Calc);
    }
    delay(1500);
    SOCK_B_Raw = SensorSocketB.read();
    SOCK_B_Calc = SensorSocketB.calculateConcentration(SOCK_B_Raw);
    if (debug == 1) {
      USB.print(F("NO3 volts: "));
      USB.print(SOCK_B_Raw);
      USB.print(F(", value: "));
      USB.println(SOCK_B_Calc);
    }
    delay(1500);
    SOCK_C_Raw = SensorSocketC.read();
    SOCK_C_Calc = SensorSocketC.calculateConcentration(SOCK_C_Raw);
    if (debug == 1) {
      USB.print(F("NO2 volts: "));
      USB.print(SOCK_C_Raw);
      USB.print(F(", value: "));
      USB.println(SOCK_C_Calc);
    }
    delay(1500);
    SOCK_D_Raw = SensorSocketD.read();
    SOCK_D_Calc = SensorSocketD.calculateConcentration(SOCK_D_Raw);
    if (debug == 1) {
      USB.print(F("Cl volts: "));
      USB.print(SOCK_D_Raw);
      USB.print(F(", value: "));
      USB.println(SOCK_D_Calc);
    }
    delay(500);  
    ///////////////////////////////////////////
    // 3. Turn off the sensors
    /////////////////////////////////////////// 
    SWIonsBoard.OFF();

    USB.print("$i|");
    USB.print(value_temp);
    USB.print("|");
    USB.print(SOCK_A_Calc);
    USB.print("|");
    USB.print(SOCK_B_Calc);
    USB.print("|");
    USB.print(SOCK_C_Calc);
    USB.print("|");
    USB.print(SOCK_D_Calc);
    USB.print("|");
    USB.print(PWR.getBatteryLevel(), DEC);
    USB.print("|$");
    USB.println();
}