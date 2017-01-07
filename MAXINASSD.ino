#include "MAX17043.h"
#include "Wire.h"
#include <Adafruit_INA219.h>
#include <Adafruit_GFX.h>
#include <Adafruit_SSD1306.h>
#include "I2Cdev.h"
#include "MPU6050.h"
#include "BMP085.h"
#include "HMC5883L.h"
#include <RtcDS3231.h>
#include "Adafruit_VEML6070.h"
#include "Adafruit_TCS34725.h"
//#include "RF24.h"
//#include "RF24Network.h"
//#include "RF24Mesh.h"
#include <SPI.h>


MAX17043 batteryMonitor;
Adafruit_INA219 ina219;
MPU6050 accelgyro;
int16_t ax, ay, az;
int16_t gx, gy, gz;
#define OUTPUT_READABLE_ACCELGYRO
BMP085 barometer;
float temperature;
float pressure;
float altitude;
int16_t accelgyro_temp;
HMC5883L mag;
int16_t mx, my, mz;
RtcDS3231<TwoWire> Rtc(Wire);
Adafruit_VEML6070 uv = Adafruit_VEML6070();
#define TCAADDR 0x70
Adafruit_TCS34725 tcs = Adafruit_TCS34725(TCS34725_INTEGRATIONTIME_700MS, TCS34725_GAIN_1X);
//RF24 radio(4, 5);
//RF24Network network(radio);
//RF24Mesh mesh(radio, network);
//#define nodeID 3
//#define otherNodeID 2

//Uncomment this block to use hardware SPI
#define OLED_DC     6
#define OLED_CS     7
#define OLED_RESET  8
Adafruit_SSD1306 display(OLED_DC, OLED_RESET, OLED_CS);

float shuntvoltage = 0;
float busvoltage = 0;
float current_mA = 0;
float loadvoltage = 0;

uint8_t oled_page = 0, page_stay_cnt = 0;
unsigned long timming_modules[10];
unsigned long loop_counter;

void setup() {
//  uint32_t currentFrequency;
  delay(5000);
  Wire.begin(); 
  Serial.begin(115200);
//  Serial.println(F("MAX17043 / INA219 / SSD1306"));
//  Serial.println();
  scan_i2c();
  i2ctca_scanner();
  tcaselect(0);
  batteryMonitor.reset();
  batteryMonitor.quickStart();
  tcaselect(1);
  ina219.begin();
  display.begin(SSD1306_SWITCHCAPVCC);
  display.clearDisplay();
  display.setTextSize(1);
  display.setTextColor(WHITE);
  tcaselect(3);
  accelgyro.initialize();
  Serial.println(accelgyro.testConnection() ? "MPU6050 OK" : "MPU6050 FAIL");
  barometer.initialize();
  Serial.println(barometer.testConnection() ? "BMP180 OK" : "BMP180 FAIL");
  mag.initialize();
  Serial.println(mag.testConnection() ? "HMC5883L OK" : "HMC5883L FAIL");
  tcaselect(4);
  uv.begin(VEML6070_1_T);   //doesnt have return value
  tcaselect(2);
  Rtc.Begin();
  Rtc.Enable32kHzPin(false);
  Rtc.SetSquareWavePin(DS3231SquareWavePin_ModeNone);
  tcaselect(5);
  if (!tcs.begin()) Serial.print(F("<COLOR_BOOT=FAIL>"));
  timming_modules[0] = millis();
//  mesh.setNodeID(nodeID);
//  mesh.begin();
}

void loop() {
//if (oled_page == 0 ) {  
  tcaselect(2);
  RtcDateTime now = Rtc.GetDateTime();
  RtcTemperature temp = Rtc.GetTemperature();
//}

//RtcDateTime anow;
//RtcTemperature temp;
uint16_t r, g, b, c, colorTemp, lux;
uint16_t valorUV;
float cellVoltage, stateOfCharge;
float heading;
        
  display.clearDisplay();
  display.setCursor(0,0);

  switch (oled_page) {
    case 0: // rtc
      //timming_modules[6] =  millis(); 
      display.println(F(" - DS3231 -"));
      display.println();
      display.print(now.Year()); display.print("/");
      display.print(now.Month()); display.print("/");
      display.println(now.Day());
      if (now.Hour() < 10) display.print("0");
      display.print(now.Hour());display.print(":");
      if (now.Minute() < 10) display.print("0");
      display.print(now.Minute());display.print(":");
      if (now.Second() < 10) display.print("0");
      display.println(now.Second());
      display.println("");
      display.print(F("RTC Temp: ")); display.print(temp.AsFloat()); display.println(" 'C");
      
      break;
    case 1: // mpu6050
      tcaselect(3);
      accelgyro.getMotion6(&ax, &ay, &az, &gx, &gy, &gz);
      //timming_modules[4] =  millis();
      accelgyro_temp = accelgyro.getTemperature()/340 + 36.53;
      display.println(F(" - MPU6050 -"));
      display.print(F("Acc  X: ")); display.print(ax); display.println(F(" ."));
      display.print(F("Acc  Y: ")); display.print(ay); display.println(F(" ."));
      display.print(F("ACC  Z: ")); display.print(az); display.println(F(" ."));
      display.print(F("Gyro X: ")); display.print(gx); display.println(F(" ."));
      display.print(F("Gyro Y: ")); display.print(gy); display.println(F(" ."));
      display.print(F("Gyro Z: ")); display.print(gz); display.println(F(" ."));
      display.print(F("Temp  : ")); display.print(accelgyro_temp); display.println(F(" 'C"));
      break;
    case 2: // light
      tcaselect(5);
      tcs.getRawData(&r, &g, &b, &c);
      colorTemp = tcs.calculateColorTemperature(r, g, b);
      lux = tcs.calculateLux(r, g, b);
      timming_modules[8] =  millis();
      tcaselect(4);
      valorUV = uv.readUV();
      timming_modules[7] =  millis();
      display.print(F("UV:     ")); display.println(valorUV); 
      display.println(F(" - TCS34725 -"));
      display.print(F("Red:     ")); display.println(r);
      display.print(F("Green:   ")); display.println(g);
      display.print(F("Blue:    ")); display.println(b);
      display.print(F("Clear:   ")); display.println(c);
      display.print(F("Strength: ")); display.print(lux); display.println(F(" lux"));
      display.print(F("Color Temp: ")); display.print(colorTemp); display.println(F(" K"));
      break;
    case 3: // baro
      tcaselect(3);
      barometer.setControl(BMP085_MODE_TEMPERATURE);
      temperature = barometer.getTemperatureC();
      barometer.setControl(BMP085_MODE_PRESSURE_3);
      pressure = barometer.getPressure();
      altitude = barometer.getAltitude(pressure);
      timming_modules[5] =  millis();
      display.println(F(" - BMP180 -"));
      display.println();
      display.print(F("Temp:     ")); display.print(temperature); display.println(F(" 'C"));
      display.print(F("Press:    ")); display.print(pressure); display.println(F(" ."));
      display.print(F("Altitude: ")); display.print(altitude); display.println(F(" m"));
      break;
    case 4: // pwr
      tcaselect(0);
       cellVoltage = batteryMonitor.getVCell();
       stateOfCharge = batteryMonitor.getSoC();
      timming_modules[2] = millis();
      tcaselect(1);
      shuntvoltage = ina219.getShuntVoltage_mV();
      busvoltage = ina219.getBusVoltage_V();
      current_mA = ina219.getCurrent_mA();
      loadvoltage = busvoltage + (shuntvoltage / 1000);
      timming_modules[3] =  millis();
      display.println(F(" - MAX17043 -"));
      display.print(F("Charge:     "));  display.print(stateOfCharge);  display.println(F("%")); 
      display.print(F("Voltage:    "));  display.print(cellVoltage, 4);  display.println(F(" V"));
      display.println(F(" - INA219 -"));
      display.print(F("Bus Volt:   ")); display.print(busvoltage); display.println(F(" V"));
      display.print(F("Shunt Volt: ")); display.print(shuntvoltage); display.println(F(" mV"));
      display.print(F("Load Volt:  ")); display.print(loadvoltage); display.println(F(" V"));
      display.print(F("Current:    ")); display.print(current_mA); display.println(F(" mA"));
      break;
    case 5: // mag
      tcaselect(3);
      mag.getHeading(&mx, &my, &mz);
      heading = atan2(my, mx);
      if(heading < 0) heading += 2 * M_PI;
      display.println(F(" - HMC5883L -"));
      display.println();
      display.print(F("Mag  X : ")); display.print(mx); display.println(F(" ."));
      display.print(F("Mag  Y : ")); display.print(my); display.println(F(" ."));
      display.print(F("Mag  Z : ")); display.print(mz); display.println(F(" ."));
      display.print(F("Heading: ")); display.print(heading * 180/M_PI); display.println(F("'"));

      break;
    default:
      display.print(F("Page Error"));
      break;
  }
  display.display();
  timming_modules[9] =  millis();
  page_stay_cnt++;
  if (page_stay_cnt == 20) {
    oled_page++;
    if ( oled_page > 5 ) {
      oled_page = 0;
      //timming_modules[0] = millis();
    }
    page_stay_cnt = 0;    
  }

  /*Serial.print(F("Loop Counter: ")); Serial.println(loop_counter);
  unsigned long last_loop_print;
  for (uint8_t loop_print = 0; loop_print<=9; loop_print++) {
    Serial.print(loop_print);
    Serial.print(F(": "));

    if (loop_print == 1) last_loop_print = timming_modules[1];
    
    if (loop_print > 1){
      Serial.print(timming_modules[loop_print] - last_loop_print);
      last_loop_print = timming_modules[loop_print];
    }
    
    if (loop_print == 0) Serial.print(timming_modules[0]);  //fixo
    
    Serial.println(F(" ms"));
  }
  */
  
  delay(500);
  loop_counter++;
}

void scan_i2c() {
  byte error, address;
  int nDevices;

  Serial.println(F("\nSimple I2C Scanner"));
  configMPU6050(0x68);
  configMPU6050(0x69);

  nDevices = 0;
  for(address = 1; address < 127; address++ ) {
    // The i2c_scanner uses the return value of
    // the Write.endTransmisstion to see if
    // a device did acknowledge to the address.
    Wire.beginTransmission(address);
    error = Wire.endTransmission();

    if (error == 0){
      Serial.print(F("I2C device: 0x"));
      if (address<16) Serial.print("0");
      Serial.print(address,HEX);
      Serial.println(" ");
      i2caddr2name(address);
      Serial.println("  !");
      nDevices++;
    }
    else if (error==4) {
      Serial.print(F("I2C error: 0x"));
      if (address<16) Serial.print("0");
      Serial.println(address,HEX);
      
    }    
    delay(5); //Scan not too fast
  }
  if (nDevices == 0)
    Serial.println(F("No I2C devices found\n"));
  else
    Serial.println(F("done\n"));
}

void i2caddr2name(uint8_t addr) {
  
  switch (addr){
    case 30:
      Serial.print(F("HMC5883L"));
      break;
    case 54:
      Serial.print(F("MAX17043"));
      break;
    case 56:
      Serial.print(F("VEML6070_H"));
      break;
    case 57:
      Serial.print(F("VEML6070_L"));
      break;
    case 58:
      Serial.print(F("INA219"));
      break;
    case 87:
      Serial.print(F("DS3231"));
      break;
    case 104:
      Serial.print(F("MPU6050"));
      break;
    case 112:
      Serial.print(F("TCA9548A"));
      break;
    case 119:
      Serial.print(F("BMP180")); //BME280 - BMP280 - BMP085
      break;
    default:
      Serial.print(F("Unknown"));
      break;
  }
}

void i2ctca_scanner() {
  Serial.println(F("I2C Mux Scanner"));
  
  for (uint8_t t=0; t<8; t++) {
    tcaselect(t);
    Serial.print(F("TCA Port #")); 
    Serial.println(t);
    configMPU6050(0x68);
    configMPU6050(0x69);

    for (uint8_t addr = 0; addr<=127; addr++) {
      if (addr == TCAADDR) continue;
    
      //uint8_t data;
      Wire.beginTransmission(addr);
      byte error = Wire.endTransmission();
      //if (! twi_writeTo(addr, &data, 0, 1, 1)) {
      if (error == 0) {
         Serial.print(F("Found I2C 0x"));  
         Serial.print(addr,HEX);
         Serial.println(" ");
         i2caddr2name(addr);
         Serial.println("");
      }
    }
  }
  Serial.println(F("done"));
}

void configMPU6050(byte address){
  
  //Enable Bypass Mode
  Wire.beginTransmission(address);
  Wire.write(0x37);
  Wire.write(0x02);
  Wire.endTransmission();
  
  //Disable I2C Master Mode for MPU6050 -->Arduino is only Master
  Wire.beginTransmission(address);
  Wire.write(0x6A);
  Wire.write(0x00);
  Wire.endTransmission();
  
  //Disable Sleep Mode
  Wire.beginTransmission(address);
  Wire.write(0x6B);
  Wire.write(0x00);
  Wire.endTransmission();  
}

void tcaselect(uint8_t i) {
  if (i > 7) return;
 
  Wire.beginTransmission(TCAADDR);
  Wire.write(1 << i);
  Wire.endTransmission();  
}




