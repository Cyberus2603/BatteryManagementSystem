//Arduino system and I2C libraries
#include <Arduino.h>
#include <Wire.h>
#include <sys/random.h>

//Network related libraries
//#include <WiFi.h>
#include <WiFiManager.h> //https://github.com/tzapu/WiFiManager
#include <AsyncTCP.h> //https://github.com/me-no-dev/AsyncTCP
#include <ESPAsyncWebServer.h> //https://github.com/me-no-dev/ESPAsyncWebServer
#include <AsyncElegantOTA.h> //https://github.com/ayushsharma82/AsyncElegantOTA
#include <ArduinoJson.h>  //https://github.com/bblanchon/ArduinoJson
#include <ArduinoMqttClient.h> //https://github.com/arduino-libraries/ArduinoMqttClient

//Screen handling libraries
#include "TFTColors.h"  //File containing color definitions for text
#include <TFT_eSPI.h> // Graphics and font library for ILI9341 driver chip

//Battery control libraries
#include "registers.h" //Register definitions for BQ56940

// ---------- BQ76940 DEFINITIONS ---------- 

//The bq76940 without CRC has the 7-bit address 0x08. The bq76940 with CRC has the address 0x18. (CRC mode is not supported by this library) 
//Please see the datasheet for more info
//https://www.ti.com/lit/ds/symlink/bq76940.pdf
#define I2C_ADDRESS 0x08 //7-bit I2C address
#define SCL_PIN 43	//I2C SCL pin to bq76940
#define SDA_PIN 44	//I2C SDA pin to bq76940
#define ALERT_PIN 16 //Alert pin from bq76940 for interrupt
#define SHUNT_RESISTOR_VALUE_MILLI_OHM 10
#define RESISTANCE_CORRECTION_VALUE 1.11
#define THERMISTORS_COUNT 3
#define THERMISTOR_BETA_VALUE 3435 //Value for Adelid Probe-Tes_2m

//The bq76940 supports 9 to 15 cells.
//Array above represents cells that are connected to BMS, for how to connect check the datasheet
//Size of array with voltages is calculated automatically, cell number here references the one on circuit
const bool CONNECTED_CELLS[15] = {
    true,	  //Cell 1
    true,	  //Cell 2
    true,	  //Cell 3
    false,	//Cell 4
    true,	  //Cell 5
    true,	  //Cell 6
    true,	  //Cell 7
    true,	  //Cell 8
    false,	//Cell 9
    true,	  //Cell 10
    true,	  //Cell 11
    true,	  //Cell 12
    true,	  //Cell 13
    false,	//Cell 14
    true	  //Cell 15
};

int *cell_voltages; //Pointer to array containing cell voltages (in milivolts)
int battery_voltage; //Variable that contains entire battery voltage
int adc_offset; //Variable for adc_offset, read from BQ76940
int adc_gain; //Variable for adc_gain, read from BQ76940
int conn_cells = 0; //Variable for number of connected cells
int max_voltage_cell_id; //Variable to point which cell has the highest voltage
int min_voltage_cell_id; //Variable to point which cell has the lowest voltage
int charge_current;  //Variable that contains battery charging current
float temperatures[THERMISTORS_COUNT];  //Array containing temperature data read from BQ76940
bool bq76940_alert = false; //Flag to handle ALERT from BQ76940

// ---------- SCREEN HANDLING DEFINITIONS ---------- 

//Pin definitions for screen and button
#define BUTTON_PIN 14

TFT_eSPI tft = TFT_eSPI();  // Invoke library, pins defined in User_Setup.h

enum ScreenType { General = 0, Detailed = 1, Network = 2, Alarms = 3 } screen_type; // Variable to decide which screen to render - detailed (cells voltages, temperatures) or general (IP, SOC etc.)

// ---------- NETWORK HANDLING DEFINITIONS ----------
#define FALLBACK_AP_NAME "BMS"
#define FALLBACK_AP_PASSWORD "1234qwer"
#define WEB_SERVER_PORT 80

bool ap_mode = false;

AsyncWebServer server(WEB_SERVER_PORT);

WiFiManager wifi_manager;
//TODO: Get real data and do save on wifi manager
WiFiManagerParameter mqtt_server_address("mqtt_server_ip", "MQTT Server IP", "192.168.5.104",20);
WiFiManagerParameter mqtt_server_port("mqtt_server_port", "MQTT Server Port", "8080", 6);

String current_ssid;

WiFiClient self_diag_client;

// ---------- DATABASE ---------- 

const float VOLTAGE_DIFFERENCE_THRESHOLD = 0.03; // Value in volts
const float TEMPERATURE_THRESHOLD = 40.0;
bool alert_icon_enabled = false;

struct DB_t {
  float *cells_voltages = nullptr;
  float *temperatures = nullptr;
  float sumV = 0.0;
  float minV = 0.0;
  float maxV = 0.0;
  float min_limit = 2.75;
  float max_limit = 4.20;
  float delta = 0.0;
  float state_of_charge = 0.0;
  float charge_current = 0.0;
  float maximum_temp = 0.0;
  bool battery_is_charging = true;
} database;

// ---------- BQ76940 FUNCTIONS ----------
uint8_t crc8Calc (uint8_t in_crc, uint8_t in_data);
void initBQ76940();
void updateVoltages();
int readRegister(byte address);
void writeRegister(byte address, byte data);

//CRC for BQ76940, needed if you want to write anything to a chip
uint8_t crc8Calc (uint8_t in_crc, uint8_t in_data) {
  uint8_t i;
  uint8_t data;
  data = in_crc ^ in_data;
  for ( i = 0; i < 8; i++ ) {
    if (( data & 0x80 ) != 0 ) {
      data <<= 1;
      data ^= 0x07;
    }
    else data <<= 1;
  }

  return data;
}

//Init communication to BQ76940 and get necessary data from it
void initBQ76940() {
  //Count connected cells and create array for them
  for (int i = 0; i < 15; ++i) {
    if (CONNECTED_CELLS[i]) {
      conn_cells++;
    }
  }
  cell_voltages = new int[conn_cells];
  for (int i = 0; i < conn_cells; i++) cell_voltages[i] = 0;

  Wire.begin(SDA_PIN, SCL_PIN); //Start I2C communication

  //Initial settings for bq769x0
  writeRegister(CC_CFG, 0x19);  //Required in the datasheet
  writeRegister(SYS_STAT, B00011111);  //Clear error indicators
  writeRegister(SYS_CTRL1, B00011000);  //Switch ADC on (ADC_EN) and thermistors reading storage on (TEMP_SEL)
  writeRegister(SYS_CTRL2, B01000000);  //Switch CC_EN on

  //Get ADC offset and gain
  adc_offset = (signed int) readRegister(ADCOFFSET);  //Convert from 2's complement
  adc_gain = 365 + (((readRegister(ADCGAIN1) & B00001100) << 1) | ((readRegister(ADCGAIN2) & B11100000) >> 5)); // uV/LSB
}

//Function to read voltages data from BQ76940 - according to datasheet you shouldn't call this function more often than 250ms
void updateVoltages() {
  long adc_val = 0;
  max_voltage_cell_id = 0; //Resets to zero before writing values
  min_voltage_cell_id = 0;

  int array_counter = 0;
  //Will run once for each cell but record only connected ones
  for (int i = 0; i < 15; i++) {
    if (CONNECTED_CELLS[i]) {
      //Combine VCx_HI and VCx_LO bits and calculate cell voltage
      adc_val = (readRegister(VC1_HI_BYTE + (i * 2))  & 0b00111111) << 8 | readRegister(VC1_LO_BYTE + (i * 2)); //Read VCx_HI bits and drop the first two bits (ADC in BQ76940 is 14-bit), shift left then append VCx_LO bits
      cell_voltages[array_counter] = adc_val * adc_gain / 1000 + adc_offset;  //Calculate real voltage in mV

      if (cell_voltages[array_counter] > cell_voltages[max_voltage_cell_id]) {
        max_voltage_cell_id = array_counter;
      }

      if (cell_voltages[array_counter] < cell_voltages[min_voltage_cell_id]) {
        min_voltage_cell_id = array_counter;
      }
      array_counter++;
    }
  }

  //Read voltage of the entire battery
  long adc_pack_voltage = ((readRegister(BAT_HI_BYTE) << 8) | readRegister(BAT_LO_BYTE)) & 0b1111111111111111;
  battery_voltage = 4 * adc_gain * adc_pack_voltage / 1000 + (conn_cells * adc_offset);
}

//Function to read current data from BQ76940
void updateCurrent() {
  if (readRegister(SYS_STAT) & B10000000) {
    short adc_val = (readRegister(CC_HI_BYTE) << 8) | readRegister(CC_LO_BYTE);
    charge_current = -1 * (int)((adc_val * 8.44 / SHUNT_RESISTOR_VALUE_MILLI_OHM) / RESISTANCE_CORRECTION_VALUE);  // mA

    writeRegister(SYS_STAT, B10000000);  // Clear CC ready flag
  }
}

//Function to read temperatures from BQ76940
void updateTemperatures() {
  short adc_val = 0;
  int thermistor_voltage;
  int thermistor_resistance;
  float tmp;

  for (int i = 0; i < THERMISTORS_COUNT; ++i) {
    //Combine TSx_HI and TSx_LO bits and calculate temperature
    adc_val = (readRegister(TS1_HI_BYTE + (2 * i)) & 0b00111111) << 8 | readRegister(TS1_LO_BYTE + (2 * i)); //Read TSx_HI bits and drop the first two bits (ADC in BQ76940 is 14-bit), shift left then append TSx_LO bits

    //Temperature calculation using Beta equation
    thermistor_voltage = adc_val * 0.382; //mV
    thermistor_resistance = (10000.0 * thermistor_voltage) / (3300.0 - thermistor_voltage); //Ohm

    tmp = 1.0/(1.0/(273.15 + 20) + 1.0/THERMISTOR_BETA_VALUE * log(thermistor_resistance/10000.0)); //Kelvin
    temperatures[i] = tmp - 273.15;
  }
}

//Function to read data from bq76940, for register definitions check datasheet
int readRegister(byte address) {
  Wire.beginTransmission(I2C_ADDRESS);
  Wire.write(address);
  Wire.endTransmission();
  Wire.requestFrom(I2C_ADDRESS, 1);
  return Wire.read();
}

//Function to write data to bq76940, for register definitions check datasheet
void writeRegister(byte address, byte data) {
  uint8_t crc = 0;
  char buf[3];
  buf[0] = (char) address;
  buf[1] = (char) data;

  // note that writes to the bq769x0 IC are: 1) start - 2) address - 3) address - 4) data - 5) CRC8 - 6) stop bit
  Wire.beginTransmission(I2C_ADDRESS); // writes start bit - the first step
  Wire.write(buf[0]);                 // writes register address
  Wire.write(buf[1]);                 // writes data - the fourth step

  // CRC is calculated over the slave address (including R/W bit), register address, and data.
  crc = crc8Calc(crc, (I2C_ADDRESS << 1) | 0);
  crc = crc8Calc(crc, buf[0]);
  crc = crc8Calc(crc, buf[1]);
  buf[2] = (char) crc;

  Wire.write(buf[2]); // writes CRC

  Wire.endTransmission();
}

//Function to read cell balancing settings
byte getCellBalancingRegisters(int pack_id) {
  if ((pack_id < 1) || (pack_id > 3)) return -1;

  byte stat = readRegister(CELLBAL1 + (pack_id - 1));
  return (stat & B00011111);
}

//Function to set cell balancing settings
void setCellBalancingRegisters(int pack_id, byte value) {
  if ((pack_id < 1) || (pack_id > 3)) return;

  writeRegister(CELLBAL1 + (pack_id - 1), value & B00011111);
}

//Function to get data from Protect1 register
byte getProtect1Register() {
  return (byte)(readRegister(PROTECT1));
}

//Function to get data from Protect2 register
byte getProtect2Register() {
  return (byte)(readRegister(PROTECT2));
}

//Function to get data from Protect3 register
byte getProtect3Register() {
  return (byte)(readRegister(PROTECT3));
}

//Function to get data from OV_TRIP register
byte getOVTripRegister() {
  return (byte)(readRegister(OV_TRIP));
}

//Function to get data from UV_TRIP register
byte getUVTripRegister() {
  return (byte)(readRegister(UV_TRIP));
}

//Function called by interrupt on Alert Pin from BQ76940
void IRAM_ATTR alertFlagSetter() {
  bq76940_alert = true;
}

//Function that handles BQ76940 alerts
void handleAlert() {
  if (bq76940_alert) {
    //TODO: Events handling
    bq76940_alert = false;
  }
}

// ---------- SCREEN HANDLING FUNCTIONS ---------- 

void calculateParameters() {
  for ( int i = 0; i < conn_cells ; i++ ) {
    database.cells_voltages[i] = (float)(cell_voltages[i] / 1000.0);
  }
  database.minV = database.cells_voltages[min_voltage_cell_id];
  database.maxV = database.cells_voltages[max_voltage_cell_id];
  database.sumV = (float)(battery_voltage / 1000.0);
  database.state_of_charge = (database.sumV - ( float(conn_cells) * database.min_limit))/((database.max_limit - database.min_limit) * float(conn_cells)) * 100;
  database.delta = abs(database.maxV - database.minV);

  float highest_temperature = -273.15;
  for (int i = 0; i < THERMISTORS_COUNT; ++i) {
    database.temperatures[i] = temperatures[i];
    if (temperatures[i] >= highest_temperature) highest_temperature = temperatures[i];
  }
  database.maximum_temp = highest_temperature;
  database.charge_current = (float)(charge_current/1000.0);
  database.battery_is_charging = (charge_current >= 0);
}

void RenderScreen(){
  switch (screen_type) {
    case General: {
      // All cells voltage sum
      tft.setTextSize(3);
      tft.setTextColor(TFT_GREEN, TFT_BLACK);
      tft.drawFloat(database.sumV, 3, 10, 1, 2);
      tft.setCursor(155,10);
      tft.setTextSize(4);
      tft.print("V");

      // Summarised voltage delta
      tft.setTextSize(3);
      if (database.delta < VOLTAGE_DIFFERENCE_THRESHOLD) {
        tft.setTextColor(TFT_GREEN, TFT_BLACK);
        tft.fillRect(190, 5, 75, 40, TFT_BLACK); // Color Black
      }
      else {
        tft.setTextColor(TFT_RED, TFT_BLACK);
        tft.fillRect(190, 5, 75, 40, TFT_BLACK); // Color Black
      }
      if (database.delta > 0.1) {
        tft.drawNumber((long)(1000 * database.delta), 190, 1, 2);
      } else {
        tft.drawNumber((long)(1000 * database.delta), 210, 1, 2);
      }
      tft.setCursor(270,10);
      tft.setTextSize(4);
      tft.print("mV");

      // State of charge
      tft.setTextSize(8);
      tft.setTextColor(TFT_ORANGE, TFT_BLACK);
      tft.drawFloat(database.state_of_charge, 1, 55, 65, 1);
      tft.setCursor(235,65);
      tft.print("%");

      // Charging current
      tft.fillRect(1,131, 150, 38, TFT_BLACK);
      tft.setTextSize(3);
      tft.setTextColor(TFT_WHITE, TFT_BLACK);
      if (database.charge_current < 0.0) {
        tft.drawFloat(database.charge_current, 3, 10, 125, 2);
      } else {
        tft.drawFloat(database.charge_current, 3, 30, 125, 2);
      }
      tft.setCursor(145,133);
      tft.setTextSize(4);
      tft.print("A");

      //Maximum sensor temperature
      tft.setTextSize(3);
      if (database.maximum_temp < TEMPERATURE_THRESHOLD) {
        tft.setTextColor(TFT_GREEN, TFT_BLACK);
      }
      else {
        tft.setTextColor(TFT_RED, TFT_BLACK);
      }
      tft.drawFloat(database.maximum_temp, 1, 180, 125, 2);
      tft.setCursor(270,133);
      tft.setTextSize(1);
      tft.print("deg");
      tft.setCursor(290,133);
      tft.setTextSize(4);
      tft.print("C");

      // Charging arrow
      if (database.battery_is_charging) {
        tft.fillRect(20, 65, 5, 50, TFT_GREEN);
        tft.fillRect(15, 70, 5, 5, TFT_GREEN);
        tft.fillRect(25, 70, 5, 5, TFT_GREEN);
        tft.fillRect(10, 75, 10, 5, TFT_GREEN);
        tft.fillRect(25, 75, 10, 5, TFT_GREEN);
        tft.fillRect(10, 100, 10, 10, TFT_BLACK);
        tft.fillRect(25, 100, 10, 10, TFT_BLACK);
      } else {
        tft.fillRect(20, 65, 5, 50, TFT_RED);
        tft.fillRect(15, 105, 5, 5, TFT_RED);
        tft.fillRect(25, 105, 5, 5, TFT_RED);
        tft.fillRect(10, 100, 10, 5, TFT_RED);
        tft.fillRect(25, 100, 10, 5, TFT_RED);
        tft.fillRect(10, 70, 10, 10, TFT_BLACK);
        tft.fillRect(25, 70, 10, 10, TFT_BLACK);
      }

      // Alarm indicator
      if (alert_icon_enabled) {
        tft.drawCircle(295,88, 20, TFT_RED);
        tft.fillRect(293, 73, 5, 20, TFT_RED);
        tft.fillRect(293, 98, 5, 5, TFT_RED);
      }

      // --- Rows and column lines ---
      uint FRAME_COLOR = TFT_DARKGREY;
      // X lines
      tft.drawLine(0, 0, 319, 0, FRAME_COLOR);
      tft.drawLine(0, 50, 319, 50, FRAME_COLOR);
      tft.drawLine(0, 130, 319, 130, FRAME_COLOR);
      tft.drawLine(0, 169, 319, 169, FRAME_COLOR);
      // Y lines
      tft.drawLine(0, 0, 0, 169, FRAME_COLOR);
      tft.drawLine(319, 0, 319, 169, FRAME_COLOR);

      break;
    }
    case Detailed: {
      // Cell info
      uint8_t baseX = 0;
      uint8_t baseY = 0;
      for ( int i = 0; i < conn_cells ; i++ ) {
        baseX = ( i % 3 ) * 105 + 8;
        baseY = ( (i/3) % 4 ) * 34;
        tft.setTextColor(TFT_WHITE, TFT_BLACK);
        if ( database.cells_voltages[i] == database.minV )    tft.setTextColor(TFT_SKYBLUE, TFT_BLACK);
        if ( database.cells_voltages[i] == database.maxV )    tft.setTextColor(TFT_SKYRED, TFT_BLACK);
        if ( database.cells_voltages[i] <  database.min_limit )  tft.setTextColor(TFT_BLUE, TFT_BLACK);
        if ( database.cells_voltages[i] >  database.max_limit )  tft.setTextColor(TFT_RED, TFT_BLACK);

        tft.setTextSize(2);
        tft.drawFloat(database.cells_voltages[i], 3, baseX, baseY, 2);
        tft.setCursor((int16_t)(baseX + 76),(int16_t)(baseY + 6));
        tft.setTextSize(3);
        tft.print("V");
      }

      // Temperatures info
      tft.setTextColor(TFT_WHITE, TFT_BLACK);
      for (int i = 0; i < THERMISTORS_COUNT; ++i) {
        tft.setTextSize(2);
        tft.drawFloat(database.temperatures[i], 1, (i * 105) + 8, 136, 2);
        tft.setCursor((int16_t)((i * 105) + 68),142);
        tft.setTextSize(1);
        tft.print("deg");
        tft.setCursor((int16_t)((i * 105) + 88),142);
        tft.setTextSize(3);
        tft.print("C");
      }

      // --- Rows and column lines ---
      uint FRAME_COLOR = TFT_DARKGREY;
      // X lines
      for (int i = 0; i < 5; i++) { tft.drawLine(0, 34 * i, 319, 34 * i, FRAME_COLOR); }
      tft.drawLine(0, 169, 319, 169, FRAME_COLOR);
      // Y lines
      for (int i = 0; i < 3; i++) { tft.drawLine(106 * i, 0, 106 * i, 169, FRAME_COLOR); }
      tft.drawLine(319, 0, 319, 169, FRAME_COLOR);

      break;
    }
    case Network: {
      if (ap_mode) {
        // Connection status
        tft.setTextColor(TFT_YELLOW, TFT_BLACK);
        tft.setCursor(80,10);
        tft.setTextSize(4);
        tft.print("AP MODE");

        //SSID
        tft.setTextColor(TFT_WHITE, TFT_BLACK);
        tft.setCursor(80,60);
        tft.setTextSize(2);
        tft.print("SSID:");
        tft.setTextColor(TFT_BLUE, TFT_BLACK);
        tft.setCursor(140,60);
        tft.print(current_ssid);

        //PASS
        tft.setTextColor(TFT_WHITE, TFT_BLACK);
        tft.setCursor(80,85);
        tft.setTextSize(2);
        tft.print("PASS:");
        tft.setTextColor(TFT_BLUE, TFT_BLACK);
        tft.setCursor(140,85);
        tft.print(FALLBACK_AP_PASSWORD);

        //IP
        tft.setTextColor(TFT_WHITE, TFT_BLACK);
        tft.setCursor(75,110);
        tft.setTextSize(2);
        tft.print("IP:");
        tft.setTextColor(TFT_BLUE, TFT_BLACK);
        tft.setCursor(115,110);
        tft.print(IPAddress(192,168,4,1));

      } else {
        // Connection status
        if (WiFi.status() == WL_CONNECTED) {
          tft.setTextColor(TFT_GREEN, TFT_BLACK);
          tft.setCursor(60,10);
          tft.setTextSize(4);
          tft.print("CONNECTED");
        } else {
          tft.setTextColor(TFT_RED, TFT_BLACK);
          tft.setCursor(20,10);
          tft.setTextSize(4);
          tft.print("DISCONNECTED");
        }

        //SSID
        tft.setTextColor(TFT_WHITE, TFT_BLACK);
        tft.setCursor(80,60);
        tft.setTextSize(2);
        tft.print("SSID:");
        tft.setTextColor(TFT_BLUE, TFT_BLACK);
        tft.setCursor(140,60);
        tft.print(current_ssid);

        //IP
        tft.setTextColor(TFT_WHITE, TFT_BLACK);
        tft.setCursor(75,85);
        tft.setTextSize(2);
        tft.print("IP:");
        tft.setTextColor(TFT_BLUE, TFT_BLACK);
        tft.setCursor(115,85);
        tft.print(WiFi.localIP());

        //MAC
        tft.setTextColor(TFT_WHITE, TFT_BLACK);
        tft.setCursor(40,110);
        tft.setTextSize(2);
        tft.print("MAC:");
        tft.setTextColor(TFT_BLUE, TFT_BLACK);
        tft.setCursor(90,110);
        tft.print(WiFi.macAddress());

        //WEB Self diagnostic
        tft.setTextColor(TFT_WHITE, TFT_BLACK);
        tft.setCursor(10,140);
        tft.setTextSize(3);
        tft.print("WEB:");
        if (self_diag_client.connect(WiFi.localIP(), WEB_SERVER_PORT)) {
          tft.setTextColor(TFT_GREEN, TFT_BLACK);
          tft.setCursor(80,140);
          tft.print("OK");
          self_diag_client.stop();
        } else {
          tft.setTextColor(TFT_RED, TFT_BLACK);
          tft.setCursor(80,140);
          tft.print("OFF");
        }

        //MQTT Self diagnostic
        tft.setTextColor(TFT_WHITE, TFT_BLACK);
        tft.setCursor(170,140);
        tft.setTextSize(3);
        tft.print("MQTT:");
        if (false) { //TODO: MQTT diagnostic
          tft.setTextColor(TFT_GREEN, TFT_BLACK);
          tft.setCursor(260,140);
          tft.print("OK");
          self_diag_client.stop();
        } else {
          tft.setTextColor(TFT_RED, TFT_BLACK);
          tft.setCursor(260,140);
          tft.print("OFF");
        }
      }

      // --- Rows and column lines ---
      uint FRAME_COLOR = TFT_DARKGREY;
      // X lines
      tft.drawLine(0, 0, 319, 0, FRAME_COLOR);
      tft.drawLine(0, 50, 319, 50, FRAME_COLOR);
      tft.drawLine(0, 130, 319, 130, FRAME_COLOR);
      tft.drawLine(0, 169, 319, 169, FRAME_COLOR);
      // Y lines
      tft.drawLine(0, 0, 0, 169, FRAME_COLOR);
      tft.drawLine(319, 0, 319, 169, FRAME_COLOR);
      break;
    }
    case Alarms: {

      break;
    }
  }
}

// ---------- NETWORK HANDLING FUNCTIONS ----------

String makeJsonDataPack() {
  ArduinoJson6194_F1::DynamicJsonDocument data_pack_obj(1024);
  for (int i = 0; i < 15; ++i) {
    data_pack_obj["connected_cells"][i] = CONNECTED_CELLS[i];
  }
  for (int i = 0; i < conn_cells; ++i) {
    data_pack_obj["voltage_data"][i] = database.cells_voltages[i];
  }
  for (int i = 0; i < THERMISTORS_COUNT; ++i) {
    data_pack_obj["temperatures"][i] = database.temperatures[i];
  }
  data_pack_obj["voltage_sum"] = database.sumV;
  data_pack_obj["voltage_min"] = database.minV;
  data_pack_obj["voltage_max"] = database.maxV;
  data_pack_obj["max_cell_difference"] = database.delta;
  data_pack_obj["state_of_charge"] = database.state_of_charge;
  data_pack_obj["charge_current"] = database.charge_current;

  String data_str;
  ArduinoJson6194_F1::serializeJson(data_pack_obj, data_str);
  return data_str;
}

String makeJsonSystemOverviewPack() {
  ArduinoJson6194_F1::DynamicJsonDocument data_pack_obj(1024);

  data_pack_obj["cell_balancing_1"] = getCellBalancingRegisters(1);
  data_pack_obj["cell_balancing_2"] = getCellBalancingRegisters(2);
  data_pack_obj["cell_balancing_3"] = getCellBalancingRegisters(3);

  //TODO: Rest of the registers

  String data_str;
  ArduinoJson6194_F1::serializeJson(data_pack_obj, data_str);
  return data_str;
}

void webServerSetup() {
  //Main Page
  server.on("/", HTTP_GET, [](AsyncWebServerRequest *request) {
    request->send(200, "text/plain", "BMS Prototype");
  });

  //Data Page
  server.on("/data", HTTP_GET, [](AsyncWebServerRequest *request) {
    request->send(200, "application/json", makeJsonDataPack());
  });

  //TODO: HTTP_POST on commands
  //Commands page
  server.on("/command", HTTP_GET, [](AsyncWebServerRequest *request) {
    request->send(200, "application/json", makeJsonSystemOverviewPack());
  });

  //TODO: DEBUG WHY OTA DOESNT WORK
  //Start ElegantOTA - starts on "/update"
  AsyncElegantOTA.begin(&server);
  server.begin();
  Serial.println("HTTP server started");

}

//This function is called when device fails to connect to wifi and enters config mode
void configModeFailCallback(WiFiManager *wi_fi_manager) {
  Serial.print("Starting AP with SSID: ");
  Serial.print(wi_fi_manager->getConfigPortalSSID());
  Serial.print(" password: ");
  Serial.println(FALLBACK_AP_PASSWORD);

  current_ssid = wi_fi_manager->getConfigPortalSSID();
  screen_type = Network;
  ap_mode = true;
  tft.fillScreen(TFT_BLACK);
}

//This function is called when config portal closes
void configModeSaveCallback() {
  ESP.restart();
}

void wifiSetup() {
  WiFi.mode(WIFI_STA);
  WiFi.setAutoReconnect(true);

  wifi_manager.setAPCallback(configModeFailCallback);
  wifi_manager.setSaveConfigCallback(configModeSaveCallback);
  wifi_manager.setClass("invert"); //Set dark theme to save eyes
  wifi_manager.setConfigPortalBlocking(false);
  bool wifi_result = wifi_manager.autoConnect(FALLBACK_AP_NAME, FALLBACK_AP_PASSWORD);

  if(!wifi_result) {
    Serial.println("Failed to connect");
    Serial.println("Starting AP mode");
  }
  else {
    Serial.print("Connected to ");
    Serial.println(wifi_manager.getWiFiSSID());
    current_ssid = wifi_manager.getWiFiSSID();

    Serial.print("IP address: ");
    Serial.println(WiFi.localIP());
  }
}

// ---------- FUNCTION EXECUTED ON SECOND CORE ----------

TaskHandle_t second_core_task;

void secondCoreTaskFunction(void * params) {
  //Erase Wi-Fi config if button is press on launch
  if (digitalRead(BUTTON_PIN) == LOW) {
    wifi_manager.erase();
  }

  //Connect to Wi-Fi or start AP for config
  wifiSetup();

  if (!ap_mode) {
    //Configure and start web server
    webServerSetup();
  }

  while (true) {
    wifi_manager.process(); //Process config portal if in AP mode
  }
}

// ---------- INIT AND MAIN LOOP ---------- 

void setup() {
  // PIN setup 
  pinMode(BUTTON_PIN, INPUT);

  // Start Serial
  Serial.begin(115200);
  Serial.println("Starting...");

  // Init Display
  tft.init();
  tft.setRotation(1);
  tft.fillScreen(TFT_BLACK);

  //Connect and setup BQ76940
  initBQ76940();

  //Initialize cells array and temperatures array in database
  database.cells_voltages = new float[conn_cells];
  database.temperatures = new float[THERMISTORS_COUNT];

  //Attach ALERT pin interrupt flag function
  attachInterrupt(ALERT_PIN, alertFlagSetter, RISING);

  //Setup network task on second core
  xTaskCreatePinnedToCore(secondCoreTaskFunction, "NetworkTask", 10000, nullptr, 0, &second_core_task, 0);

  // Initial screen type
  screen_type = General;
}

void loop() {
  updateVoltages(); //Read voltage data from BQ76940
  updateCurrent(); //Read current data from BQ76940
  updateTemperatures(); //Read temperatures from BQ76940
  handleAlert(); //Check chip and handle alert event from BQ76940

  //Screen switch
  if (digitalRead(BUTTON_PIN) == LOW) {
    screen_type = (ScreenType)((((int)screen_type) + 1) % 4); // Cast to int, do math operations, cast to enum again
    tft.fillScreen(TFT_BLACK);
  }

  calculateParameters(); //Calculate delta, convert from milivolts to volts etc.
  RenderScreen(); //Refresh display with appropriate screen

  delay(250);
}

