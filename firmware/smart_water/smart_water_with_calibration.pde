#include <WaspSensorSW.h>
#include <WaspFrame.h>
#include <TurbiditySensor.h>

//----------------------------------------------------------------------------------------
//!!!!!!!!!!!!!!!!!!!!!!!!!!!!!!!!!!!!!!!!!!!!!!!!!!!!!!!!!!!!!!!!!!!!!!!!!!!!!!!!!!!!!!!!
//ОСТАЛЬНЫЕ НАСТРОЙКИ МЕНЯТЬ ЗАПРЕЩАЕТСЯ!
//!!!!!!!!!!!!!!!!!!!!!!!!!!!!!!!!!!!!!!!!!!!!!!!!!!!!!!!!!!!!!!!!!!!!!!!!!!!!!!!!!!!!!!!!
char check_md5[] = "64d73b68f07a8480ecdceeb437ef63b9";
char filename[] = "SmartWater_FRMW_V1_1.hex";

// Выбор датчика температуры
// 0 - pt1000;
// 1 - датчик температуры, встроенный в датчик мутности
//!ПРИ ИСПОЛЬЗОВАНИИ ДАТЧИКА МУТНОСТИ ДАТЧИК ТЕМПЕРАТУРЫ pt1000 ДОЛЖЕН БЫТЬ ОТКЛЮЧЕН!
int TEMP_SENSOR_TYPE = 0; 
int debug = 0;
// 0 - show nothing (calibration mode)
// 1 - show frame
// 2 - show data
int ShowData = 2;


// Время калибровки
int counter = 10;
int zadergka = 500;


//версия прошивки (хранится в EEPROM)
int addressFVMajor = 1025;
int addressFVMinor = 1026;
int FVMajor = 1;
int FVMinor = 1;
int auxFVMajor = 0;
int auxFVMinor = 0;


unsigned long timer = 0;
int command = 0;


float value_pH; // pH values in volts
float value_temp; // temp in celsius
float value_pH_calculated;
float value_orp; // orp in volts
float value_orp_calculated;
//float value_di; 
float value_do; // oxygen in volts
float value_do_calculated;
float value_cond; // conductivity in Ohms
float value_cond_calculated;
float value_turbidity;

//-----------------------------------
// калибровочные точки для pH датчика
float cal_point_10 = 1.985;
float cal_point_7 = 2.070;
float cal_point_4 = 2.227;
int addr_p10 = 1030;
int addr_p7 = 1035;
int addr_p4 = 1039;
long aux_p10 = 0;
long aux_p7 = 0;
long aux_p4 = 0;
//-----------------------------------
// Температура калибровки
float cal_temp = 23.7;
int addr_cal_temp = 1043;
long aux_cal_temp = 0;
//-----------------------------------
// оффсет для ORP датчика (показание - 0.225)
float calibration_offset = 0.015;
int addr_orp_offset = 1047;
long aux_orp_offset = 0;
//-----------------------------------
// показание датчика кислорода для нормального воздуха
float air_calibration = 2.65;
// нулевая точка для датчика кислорода (0%)
float zero_calibration = 0.0;
int addr_air_calib = 1051;
int addr_zero_air = 1055;
long aux_air_calib = 0;
long aux_zero_air = 0;
//-----------------------------------
// значение первого раствора для датчика проводимости
long point1_cond = 10500;
// значение второго раствора для датчика проводимости
long point2_cond = 40000;
// точка калибровки для первого раствора для датчика проводимости
float point1_cal = 197.00;
// точка калибровки для второго раствора для датчика проводимости
float point2_cal = 150.00;
int addr_p1_cond = 1059;
int addr_p2_cond = 1063;
int addr_p1 = 1067;
int addr_p2 = 1071;
long aux_p1_cond = 0;
long aux_p2_cond = 0;
long aux_p1 = 0;
long aux_p2 = 0;


pHClass pHSensor;
ORPClass ORPSensor;
//DIClass DISensor;
DOClass DOSensor;
conductivityClass ConductivitySensor;
pt1000Class TemperatureSensor;
turbidityClass Turbidity;

void setup() 
{
  USB.ON();

  cal_point_10 = LongToFloat(EEPROMReadLong(addr_p10));
  cal_point_7 = LongToFloat(EEPROMReadLong(addr_p7));
  cal_point_4 = LongToFloat(EEPROMReadLong(addr_p4));
  air_calibration = LongToFloat(EEPROMReadLong(addr_air_calib));
  zero_calibration = LongToFloat(EEPROMReadLong(addr_zero_air));
  point1_cond = EEPROMReadLong(addr_p1_cond);
  point2_cond = EEPROMReadLong(addr_p2_cond);
  point1_cal = LongToFloat(EEPROMReadLong(addr_p1));
  point2_cal = LongToFloat(EEPROMReadLong(addr_p2));
  cal_temp = LongToFloat(EEPROMReadLong(addr_cal_temp));
  calibration_offset = LongToFloat(EEPROMReadLong(addr_orp_offset));

  Utils.writeEEPROM(addressFVMajor, FVMajor);
  Utils.writeEEPROM(addressFVMinor, FVMinor);
  auxFVMajor = Utils.readEEPROM(addressFVMajor);
  auxFVMinor = Utils.readEEPROM(addressFVMajor);

  delay(5000);

  if (debug == 1) {
    USB.println(cal_point_10);
    USB.println(cal_point_7);
    USB.println(cal_point_4);
    USB.println(air_calibration);
    USB.println(zero_calibration);
    USB.println(point1_cond);
    USB.println(point2_cond);
    USB.println(point1_cal);
    USB.println(point2_cal);
    USB.println(cal_temp);
    USB.println(calibration_offset);
  }
  
  // Configure the calibration values
  pHSensor.setCalibrationPoints(cal_point_10, cal_point_7, cal_point_4, cal_temp);
  DOSensor.setCalibrationPoints(air_calibration, zero_calibration);
  ConductivitySensor.setCalibrationPoints(point1_cond, point1_cal, point2_cond, point2_cal);
//------------------------------------------------------------------------------------------
  //-----------------------------------------------------------------
  USB.flush();
}

void loop()
{
  ///////////////////////////////////////////
  // 1. Turn on the board
  /////////////////////////////////////////// 

  if (ShowData == 2) {
    SesorData();
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
        case 66: //B
          ShowData = 1;
          USB.println(F("#+"));
          break;
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
          TEMP_SENSOR_TYPE = 1;
          if (debug == 1) { USB.println(F("Temp from turbidity sensor")); }
          break;
        case 101: // e
          TEMP_SENSOR_TYPE = 0;
          if (debug == 1) { USB.println(F("temp from pt1000")); }
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
          //USB.flush();
          delay(1000);
          USB.println(F("#?"));
          //CondCalib_R1(220, 3000);
          CondCalib_R1(84, 1413);
          USB.println(F("#a"));                   
          break;
        case 98: // b
          delay(1000);
          USB.println(F("#?"));
          //CondCalib_R1(10500, 40000);
          CondCalib_R1(12880, 150000);
          USB.println(F("#b"));
          break;
        case 99: // c
          delay(1000);
          USB.println(F("#?"));
          //CondCalib_R1(62000, 90000);
          CondCalib_R1(12880, 80000);
          USB.println(F("#c"));
          break;
        case 107: // k
          delay(1000);
          USB.println(F("#?"));
          //CondCalib_R2(220, 3000);
          CondCalib_R2(84, 1413);
          USB.println(F("#k"));
          break;
        case 108: // l
          delay(1000);
          USB.println(F("#?"));
          //CondCalib_R2(10500, 40000);
          CondCalib_R2(12880, 150000);
          USB.println(F("#l"));
          break;
        case 109: // m
          delay(1000);
          USB.println(F("#?"));
          //CondCalib_R2(62000, 90000);
          CondCalib_R2(12880, 80000);
          USB.println(F("#m"));
          break;
        case 110: // n
          delay(1000);
          USB.println(F("#?"));
          OxygenCalib_100p();
          USB.println(F("#n"));
          break;
        case 111: // o
          delay(1000);
          USB.println(F("#?"));
          OxygenCalib_0p();
          USB.println(F("#0"));
          break;
        case 112: // p
          delay(1000);
          USB.println(F("#?"));
          pHSensorCalibP10();
          USB.println(F("#p"));
          break;
        case 113: // q
          delay(1000);
          USB.println(F("#?"));
          pHSensorCalibP7();
          USB.println(F("#q"));
          break;
        case 114: // r
          delay(1000);
          USB.println(F("#?"));
          pHSensorCalibP4();
          USB.println(F("#r"));
          break;
        case 115: // s
          delay(1000);
          USB.println(F("#?"));
          ORPSensorCalib();
          USB.println(F("#s"));
          break;
        // case 102: // f
        //   auxFVMajor = Utils.readEEPROM(addressFVMajor);
        //   auxFVMinor = Utils.readEEPROM(addressFVMajor);
        //   //USB.print(F("Имя устройства: "));
        //   //USB.println(node_ID);
        //   USB.print(F("<|"));
        //   //delay(1000);
        //   //USB.println();
        //   USB.print(node_ID);
        //   USB.print("|");
        //   ShowSerialNumber();
        //   USB.print("|");
        //   //USB.print(F("Версия прошивки: "));
        //   USB.print(auxFVMajor);
        //   USB.print(F("."));
        //   //USB.println(auxFVMinor);
        //   USB.print(auxFVMinor);
        //   USB.print("|");
        //   USB.print(check_md5);
        //   USB.print("|");
        //   USB.print(filename);
        //   USB.println(F("|>"));
        //   delay(5000);
        //   break;
        default:
          //USB.println(F("Wrong command!"));  
          break;      
      }
      USB.flush();

    }
    if (millis() < timer)
    {
      timer = millis();
    }
  }
  //frame.showFrame();
}

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
// Выбор с какого датчика читаем температуру с одного из двух датчиков
float readTemp() {
  float val_temp = 0.0;
  if (TEMP_SENSOR_TYPE == 1)
  {
    val_temp = Turbidity.getTemperature();
  }
  else 
  {
    val_temp = TemperatureSensor.readTemperature();
  }

  return val_temp;
}

//--------------------------------------
// автоматический выбор датчика температуры (тестовая функция!)
float readTempAuto() {
  float val_temp1 = 0.0;
  val_temp1 = Turbidity.getTemperature();
  if (val_temp1 == -1000.0)
  {
    val_temp1 = TemperatureSensor.readTemperature();
  }
  return val_temp1;

}

//--------------------------------------
// Калибровка рН датчика в растворе рН10
void pHSensorCalibP10() {
  float pH_val_ohm = 0.0;
  float temp1 = 0.0;
  aux_p10 = EEPROMReadLong(addr_p10);
  aux_p7 = EEPROMReadLong(addr_p7);
  aux_p4 = EEPROMReadLong(addr_p4);
  aux_cal_temp = EEPROMReadLong(addr_cal_temp);
  pHSensor.setCalibrationPoints(LongToFloat(aux_p10), LongToFloat(aux_p7), LongToFloat(aux_p4), LongToFloat(aux_cal_temp));
  Water.ON();

  for (int i = 0; i < counter; i++) {
    pH_val_ohm = pHSensor.readpH();
    //temp1 = readTemp();
    temp1 = readTempAuto();
    if (debug == 1) {
      USB.print(pH_val_ohm);
      USB.print(F("||"));
      USB.println(temp1);
    }
    delay(zadergka);
  }
  Water.OFF();
  EEPROMWriteLong(addr_p10, FloatToLong(pH_val_ohm));
  EEPROMWriteLong(addr_cal_temp, FloatToLong(temp1));
  //EEPROMWriteLong(addr_p7, FloatToLong(aux_p7));
  //EEPROMWriteLong(addr_p4, FloatToLong(aux_p4));
  pHSensor.setCalibrationPoints(pH_val_ohm, LongToFloat(aux_p7), LongToFloat(aux_p4), temp1);
  if (debug == 1) {
    USB.println(F("Калибровка для раствора pH10 завершена."));
    }
}

//--------------------------------------
// Калибровка рН датчика в растворе рН7
void pHSensorCalibP7() {
  float pH_val_ohm = 0.0;
  float temp1 = 0.0;
  aux_p10 = EEPROMReadLong(addr_p10);
  aux_p7 = EEPROMReadLong(addr_p7);
  aux_p4 = EEPROMReadLong(addr_p4);
  aux_cal_temp = EEPROMReadLong(addr_cal_temp);
  pHSensor.setCalibrationPoints(LongToFloat(aux_p10), LongToFloat(aux_p7), LongToFloat(aux_p4), LongToFloat(aux_cal_temp));
  Water.ON();

  for (int i = 0; i < counter; i++) {
    pH_val_ohm = pHSensor.readpH();
    //temp1 = readTemp();
    temp1 = readTempAuto();
    if (debug == 1) {
      USB.print(pH_val_ohm);
      USB.print(F("||"));
      USB.println(temp1);
    }
    delay(zadergka);
  }
  Water.OFF();
  EEPROMWriteLong(addr_p7, FloatToLong(pH_val_ohm));
  //EEPROMWriteLong(addr_p7, FloatToLong(aux_p7));
  //EEPROMWriteLong(addr_p4, FloatToLong(aux_p4));
  pHSensor.setCalibrationPoints(LongToFloat(aux_p10), pH_val_ohm, LongToFloat(aux_p4), temp1);
  if (debug == 1) {
    USB.println(F("Калибровка для раствора pH7 завершена."));
  }
}

//--------------------------------------
// Калибровка рН датчика в растворе рН4
void pHSensorCalibP4() {
  float pH_val_ohm = 0.0;
  float temp1 = 0.0;
  aux_p10 = EEPROMReadLong(addr_p10);
  aux_p7 = EEPROMReadLong(addr_p7);
  aux_p4 = EEPROMReadLong(addr_p4);
  aux_cal_temp = EEPROMReadLong(addr_cal_temp);
  pHSensor.setCalibrationPoints(LongToFloat(aux_p10), LongToFloat(aux_p7), LongToFloat(aux_p4), LongToFloat(aux_cal_temp));
  Water.ON();

  for (int i = 0; i < counter; i++) {
    pH_val_ohm = pHSensor.readpH();
    //temp1 = readTemp();
    temp1 = readTempAuto();
    if (debug == 1) {
      USB.print(pH_val_ohm);
      USB.print(F("||"));
      USB.println(temp1);
    }
    delay(zadergka);
  }
  Water.OFF();
  EEPROMWriteLong(addr_p4, FloatToLong(pH_val_ohm));
  //EEPROMWriteLong(addr_p7, FloatToLong(aux_p7));
  //EEPROMWriteLong(addr_p4, FloatToLong(aux_p4));
  pHSensor.setCalibrationPoints(LongToFloat(aux_p10), LongToFloat(aux_p7), pH_val_ohm, temp1);
  if (debug == 1) {
    USB.println(F("Калибровка для раствора pH4 завершена."));
  }
}

//--------------------------------------
// Калибровка датчика окислительно-восстановительного процесса
void ORPSensorCalib() {
  float orp_calib = 0.0;
  float orp_calculated = 0.0;
  aux_orp_offset = LongToFloat(EEPROMReadLong(addr_orp_offset));
  Water.ON();
  for (int k = 0; k < counter; k++) {
    orp_calib = 1000*ORPSensor.readORP();
    // orp_calculated = orp_calib - LongToFloat(aux_orp_offset);
    if (debug == 1) {
      USB.println(orp_calib);
    }
    delay(zadergka);
  }
  Water.OFF();
  aux_orp_offset = orp_calib - 225;
  EEPROMWriteLong(addr_orp_offset, FloatToLong(aux_orp_offset));
  calibration_offset = LongToFloat(EEPROMReadLong(addr_orp_offset));
  if (debug == 1) {
    USB.print(F("Калибровка завершена. Значение калибровочного смещения(вольт): "));
    USB.println(aux_orp_offset);
  }
}

//--------------------------------------
// Калибровка датчика кислорода в 100% растворе кислорода
void OxygenCalib_100p() {
  aux_air_calib = EEPROMReadLong(addr_air_calib);
  aux_zero_air = EEPROMReadLong(addr_zero_air);
  float result = 0.0;
  DOSensor.setCalibrationPoints(LongToFloat(aux_air_calib), LongToFloat(aux_zero_air));
  Water.ON();
  for (int j = 0; j < counter; j++) {
    result = DOSensor.readDO();
    if (debug == 1) {
      USB.println(result);
    }
    delay(zadergka);
  }
  Water.OFF();
  EEPROMWriteLong(addr_air_calib, FloatToLong(result));
  DOSensor.setCalibrationPoints(result, LongToFloat(aux_zero_air));
  if (debug == 1) {
    USB.print(F("Калибровка первой точки (100) завершена."));
  }
}

//--------------------------------------
// Калибровка датчика кислорода в 0% растворе кислорода
void OxygenCalib_0p() {
  aux_air_calib = EEPROMReadLong(addr_air_calib);
  aux_zero_air = EEPROMReadLong(addr_zero_air);
  float result = 0.0;
  DOSensor.setCalibrationPoints(LongToFloat(aux_air_calib), LongToFloat(aux_zero_air));
  Water.ON();
  for (int j = 0; j < counter; j++) {
    result = DOSensor.readDO();
    if (debug == 1) {
      USB.println(result);
    }
    delay(zadergka);
  }
  Water.OFF();
  EEPROMWriteLong(addr_zero_air, FloatToLong(result));
  DOSensor.setCalibrationPoints(LongToFloat(aux_air_calib), result);
  if (debug == 1) {
    USB.print(F("Калибровка второй точки (0) завершена."));
  }
}

//--------------------------------------
// Калибровка датчика проводимости для 1-го раствора из комплекта
// Калибровочные растворы
// K = 0.1  220 uS  3000 uS
// K = 1 10500 uS 40000 uS
// K = 10 62000 uS 90000 uS
void CondCalib_R1(long rastvor1, long rastvor2) {
  //aux_p1_cond = EEPROMReadLong(addr_p1_cond);
  //aux_p2_cond = EEPROMReadLong(addr_p2_cond);
  aux_p1_cond = rastvor1;
  aux_p2_cond = rastvor2;
  EEPROMWriteLong(addr_p1_cond, aux_p1_cond);
  EEPROMWriteLong(addr_p2_cond, aux_p2_cond);
  aux_p1 = EEPROMReadLong(addr_p1);
  aux_p2 = EEPROMReadLong(addr_p2);
  float resist = 0.0;
  ConductivitySensor.setCalibrationPoints(aux_p1_cond, LongToFloat(aux_p1), aux_p2_cond, LongToFloat(aux_p2));
  Water.ON();
  for (int n = 0; n < counter; n++) {
    resist = ConductivitySensor.readConductivity();
    if (debug == 1) {
      USB.println(resist);
    }
    delay(zadergka);
  }
  Water.OFF();
  EEPROMWriteLong(addr_p1, FloatToLong(resist));
  ConductivitySensor.setCalibrationPoints(rastvor1, resist, aux_p2_cond, LongToFloat(aux_p2));
  if (debug == 1) {
    USB.print(F("Раствор: "));
    USB.println(aux_p1_cond);
    USB.print(F("Коэффициент: "));
    USB.println(resist);
    USB.print(F("Калибровка для первого раствора завершена."));
  }
}

//--------------------------------------
// Калибровка датчика проводимости для 1-го раствора из комплекта
// Калибровочные растворы
// K = 0.1  220 uS  3000 uS
// K = 1 10500 uS 40000 uS
// K = 10 62000 uS 90000 uS
void CondCalib_R2(long rastvor1, long rastvor2) {
  //aux_p1_cond = EEPROMReadLong(addr_p1_cond);
  //aux_p2_cond = EEPROMReadLong(addr_p2_cond);
  aux_p1_cond = rastvor1;
  aux_p2_cond = rastvor2;
  EEPROMWriteLong(addr_p1_cond, aux_p1_cond);
  EEPROMWriteLong(addr_p2_cond, aux_p2_cond);
  aux_p1 = EEPROMReadLong(addr_p1);
  aux_p2 = EEPROMReadLong(addr_p2);
  float resist = 0.0;
  ConductivitySensor.setCalibrationPoints(aux_p1_cond, LongToFloat(aux_p1), aux_p2_cond, LongToFloat(aux_p2));
  Water.ON();
  for (int n = 0; n < counter; n++) {
    resist = ConductivitySensor.readConductivity();
    if (debug == 1) {
      USB.println(resist);
    }
    delay(zadergka);
  }
  Water.OFF();
  EEPROMWriteLong(addr_p2, FloatToLong(resist));
  ConductivitySensor.setCalibrationPoints(aux_p1_cond, LongToFloat(aux_p1), rastvor2, resist);
  if (debug == 1) {
    USB.print(F("Раствор: "));
    USB.println(aux_p2_cond);
    USB.print(F("Коэффициент: "));
    USB.println(resist);
    USB.print(F("Калибровка для второго раствора завершена."));
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
  cal_point_10 = LongToFloat(EEPROMReadLong(addr_p10));
  cal_point_7 = LongToFloat(EEPROMReadLong(addr_p7));
  cal_point_4 = LongToFloat(EEPROMReadLong(addr_p4));
  air_calibration = LongToFloat(EEPROMReadLong(addr_air_calib));
  zero_calibration = LongToFloat(EEPROMReadLong(addr_zero_air));
  point1_cond = EEPROMReadLong(addr_p1_cond);
  point2_cond = EEPROMReadLong(addr_p2_cond);
  point1_cal = LongToFloat(EEPROMReadLong(addr_p1));
  point2_cal = LongToFloat(EEPROMReadLong(addr_p2));
  cal_temp = LongToFloat(EEPROMReadLong(addr_cal_temp));
  calibration_offset = LongToFloat(EEPROMReadLong(addr_orp_offset));

  USB.println(F("Ph (10, 7, 4):"));
  USB.println(cal_point_10);
  USB.println(cal_point_7);
  USB.println(cal_point_4);
  USB.println(F("DO (air, zero):"));
  USB.println(air_calibration);
  USB.println(zero_calibration);
  USB.println(F("Cond solutions:"));
  USB.println(point1_cond);
  USB.println(point2_cond);
  USB.println(F("Cond cal points:"));
  USB.println(point1_cal);
  USB.println(point2_cal);
  USB.println(F("Cal temp:"));
  USB.println(cal_temp);
  USB.println(F("ORP offset:"));
  USB.println(calibration_offset);  
}

void SesorData() {
  Water.ON();
  Turbidity.ON();
  delay(2000);

    ///////////////////////////////////////////
    // 2. Read sensors
    ///////////////////////////////////////////  

    // Read the ph sensor
  value_pH = pHSensor.readpH();
  if (debug == 1) {
    USB.print(F("Ph_volt: "));
    USB.println(value_pH);
  }
    // Read the temperature sensor
    //value_temp = TemperatureSensor.readTemperature();
    //value_temp = Turbidity.getTemperature();
  //value_temp = readTemp();
  value_temp = readTempAuto();
  if (debug == 1) {
    USB.print(F("Temp: "));
    USB.println(value_temp);
  }
  value_turbidity = Turbidity.getTurbidity();
  // Convert the value read with the information obtained in calibration
  value_pH_calculated = pHSensor.pHConversion(value_pH,value_temp); 
  if (debug == 1) {
    USB.print(F("Ph final: "));
    USB.println(value_pH_calculated);
  } 
  // Reading of the ORP sensor
  value_orp = 1000*ORPSensor.readORP();
  if (debug == 1) {
    USB.print(F("ORP volts: "));
    USB.println(value_orp);
  } 
  // Apply the calibration offset
  value_orp_calculated = value_orp - calibration_offset;
  if (debug == 1) {
    USB.print(F("ORP final: "));
    USB.println(value_orp_calculated);
  } 
    // Reading of the DI sensor
    //value_di = DISensor.readDI();
    // Reading of the ORP sensor
  value_do = DOSensor.readDO();
  if (debug == 1) {
    USB.print(F("DO volts: "));
    USB.println(value_do);
  } 
    // Conversion from volts into dissolved oxygen percentage
  value_do_calculated = DOSensor.DOConversion(value_do);
  if (debug == 1) {
    USB.print(F("DO final: "));
    USB.println(value_do_calculated);
  } 
    // Reading of the Conductivity sensor
  value_cond = ConductivitySensor.readConductivity();
  if (debug == 1) {
    USB.print(F("Cond before calc: "));
    USB.println(value_cond);
  } 
    // Conversion from resistance into ms/cm
  value_cond_calculated = ConductivitySensor.conductivityConversion(value_cond);  
  if (debug == 1) {
    USB.print(F("Cond final: "));
    USB.println(value_cond_calculated);
  } 
  
    ///////////////////////////////////////////
    // 3. Turn off the sensors
    /////////////////////////////////////////// 

  Water.OFF();
  USB.print("$w|");
  //USB.print(FloatToLong(value_temp));
  USB.print(value_temp);
  USB.print("|");
  //USB.print(FloatToLong(value_pH_calculated));
  USB.print(value_pH_calculated);
  USB.print("|");
  //USB.print(FloatToLong(value_cond_calculated));
  USB.print(value_cond_calculated);
  USB.print("|");
  //USB.print(FloatToLong(value_do_calculated));
  USB.print(value_do_calculated);
  USB.print("|");
  //USB.print(FloatToLong(value_orp_calculated));
  USB.print(value_orp_calculated);
  USB.print("|");
  //USB.print(FloatToLong(value_turbidity));
  USB.print(value_turbidity);
  USB.print("|");
  USB.print(PWR.getBatteryLevel(), DEC);
  USB.print("|$");
  USB.println();
}