#include <ESP8266httpUpdate.h>
#include <ESP8266WiFi.h>
#include <ESP_SSD1306.h>
#include <SPI.h>            // only for compilation
#include <Wire.h>
#include <Adafruit_GFX.h>
#include <WiFiManager.h>
#include <EEPROM.h>
#include <Arduino.h>
#include <OneWire.h>
#include <DallasTemperature.h>
#include <ArduinoJson.h>

//EEPROM
#define EEPROM_SIZE 20

//Serial
#define BAUDRATE 115200

#define ONE_WIRE_BUS D5  // DS18B20 pin
#define I2C_ADDRESS 0x3c

#define OLED_RESET  16
#define PIN_SDA D7
#define PIN_SCL D6

#define PLUS_PIN D1
#define MINUS_PIN D8
#define PIN_RELAY D2

#define BUTTON_TIMEOUT 3000
#define BUTTON_DEBOUNCE 50
#define RESET_DEBOUNCE 1000

//#define SENSOR_TIMEOUT 60000
#define LAST_SEND_ATTEMPT_MSG_DISPLAY_TIMEOUT 1000

// OLED WIFI LOGO DEFINES
#define WIFI_LOGO_X 117
#define WIFI_LOGO_Y 4
#define WIFI_LOGO_BAR_W 2
#define WIFI_LOGO_BAR_SPACE 1
#define WIFI_LOGO_W 11
#define WIFI_LOGO_H 11
const unsigned char PROGMEM wifiLogo [] = {
0xff,0xe0
,0x84,0x20
,0x44,0x40
,0x24,0x80
,0x15,0x00
,0x0e,0x00
,0x04,0x00
,0x04,0x00
,0x04,0x00
,0x04,0x00
,0x04,0x00
};    

// BRUNING LOGO
const unsigned char PROGMEM burnLogo [] = {
 0x08,0x00
,0x0c,0x00
,0x1e,0x00
,0x1f,0x00
,0x3f,0x80
,0x3f,0xc0
,0x7d,0xc0
,0x7c,0xe0
,0xfc,0x60
,0xec,0x60
,0xe0,0x60
,0xc0,0x60
,0xc0,0x60
,0x40,0x40
,0x20,0x80
};

// STARTUP LOGO
const unsigned char PROGMEM startupLogo [] = {
0x00,0x00,0x00,0x00,0x00,0x03,0xfe,0x00
,0x00,0x00,0x00,0x00,0x00,0x03,0xfe,0x00
,0x00,0x00,0x00,0x01,0x00,0x03,0xfe,0x00
,0x00,0x00,0x00,0x07,0xc0,0x03,0xfe,0x00
,0x00,0x00,0x00,0x1f,0xf0,0x03,0xfe,0x00
,0x00,0x00,0x00,0x3f,0xf0,0x03,0xfe,0x00
,0x00,0x00,0x00,0x7f,0xfc,0x03,0xfe,0x00
,0x00,0x00,0x01,0xff,0xff,0x03,0xfe,0x00
,0x00,0x00,0x03,0xff,0xff,0x83,0xfe,0x00
,0x00,0x00,0x0f,0xff,0xff,0xc0,0xfe,0x00
,0x00,0x00,0x1f,0xff,0xff,0xf0,0x7e,0x00
,0x00,0x00,0x7f,0xff,0xff,0xf8,0x1e,0x00
,0x00,0x00,0xff,0xff,0xff,0xfc,0x0e,0x00
,0x00,0x03,0xff,0xff,0xff,0xff,0x00,0x00
,0x00,0x07,0xff,0xfe,0xff,0xff,0x80,0x00
,0x00,0x0f,0xff,0xf8,0x7f,0xff,0xe0,0x00
,0x00,0x3f,0xff,0xf0,0x1f,0xff,0xf0,0x00
,0x00,0x7f,0xff,0xc0,0x0f,0xff,0xfc,0x00
,0x01,0xff,0xff,0x80,0x03,0xff,0xfe,0x00
,0x03,0xff,0xfe,0x03,0x81,0xff,0xff,0x80
,0x07,0xff,0xfc,0x07,0xc0,0xff,0xff,0xc0
,0x1f,0xff,0xf8,0x1f,0xf0,0x3f,0xff,0xf0
,0x3f,0xff,0xe0,0x7f,0xf8,0x1f,0xff,0xf8
,0x7f,0xff,0xc0,0x7f,0xfc,0x07,0xff,0xfc
,0xff,0xff,0x01,0xff,0xff,0x03,0xff,0xfe
,0xff,0xfe,0x07,0xff,0xff,0xc0,0xff,0xfe
,0xff,0xfc,0x0f,0xff,0xff,0xe0,0x7f,0xfc
,0x7f,0xf0,0x1f,0xff,0xff,0xf0,0x1f,0xf8
,0x3f,0xe0,0x7f,0xff,0xff,0xfc,0x0f,0xf8
,0x3f,0xc0,0xff,0xff,0xff,0xfe,0x07,0xf0
,0x1f,0x03,0xff,0xff,0xff,0xff,0x01,0xf0
,0x0c,0x07,0xff,0xff,0xff,0xff,0xc0,0x60
,0x00,0x1f,0xff,0xff,0xff,0xff,0xe0,0x00
,0x00,0x3f,0xff,0xff,0xff,0xff,0xf8,0x00
,0x00,0x7f,0xff,0xff,0xff,0xff,0xfc,0x00
,0x01,0xff,0xff,0x00,0x01,0xff,0xfe,0x00
,0x01,0xff,0xf8,0x00,0x00,0x3f,0xfe,0x00
,0x01,0xff,0xe0,0x00,0x00,0x1f,0xfe,0x00
,0x01,0xff,0x80,0x00,0x00,0x03,0xfe,0x00
,0x01,0xff,0x00,0x7f,0xfc,0x01,0xfe,0x00
,0x01,0xfe,0x01,0xff,0xff,0x00,0xfe,0x00
,0x01,0xf8,0x0f,0xff,0xff,0xe0,0x7e,0x00
,0x01,0xf0,0x1f,0xfc,0x7f,0xf8,0x1e,0x00
,0x01,0xf0,0x7f,0xc0,0x07,0xf8,0x1e,0x00
,0x01,0xf0,0xfe,0x00,0x00,0xfe,0x1e,0x00
,0x01,0xf9,0xf8,0x00,0x00,0x3f,0x3e,0x00
,0x01,0xff,0xf0,0x00,0x00,0x1f,0xfe,0x00
,0x01,0xff,0xe0,0x3f,0xf0,0x0f,0xfe,0x00
,0x01,0xff,0xc0,0xff,0xfe,0x07,0xfe,0x00
,0x01,0xff,0xc1,0xff,0xff,0x07,0xfe,0x00
,0x01,0xff,0xc3,0xfc,0x7f,0xc7,0xfe,0x00
,0x01,0xff,0xff,0xc0,0x07,0xff,0xfe,0x00
,0x01,0xff,0xff,0x80,0x03,0xff,0xfe,0x00
,0x01,0xff,0xff,0x00,0x01,0xff,0xfe,0x00
,0x01,0xff,0xfe,0x01,0x80,0xff,0xfe,0x00
,0x01,0xff,0xfe,0x0f,0xe0,0xff,0xfe,0x00
,0x01,0xff,0xff,0x1f,0xf1,0xff,0xfe,0x00
,0x01,0xff,0xff,0xff,0xff,0xff,0xfe,0x00
,0x01,0xff,0xff,0xff,0xff,0xff,0xfe,0x00
,0x01,0xff,0xff,0xff,0xff,0xff,0xfe,0x00
,0x01,0xff,0xff,0xff,0xff,0xff,0xfe,0x00
,0x01,0xff,0xff,0xff,0xff,0xff,0xfe,0x00
};

/// Globals
// Variable saved to EPORMM
byte DOMOTICZ_IP_ADDRESS[4];
float SETPOINT = 0.0;
unsigned int DOMOTICZ_PORT = 0;
unsigned int SENSOR_TIMEOUT = 10;
unsigned int THERMOSTAT_IDX = 0;
unsigned int HEATER_SWITCH_IDX = 0;
unsigned int TEMPSENSOR_IDX = 0;


String DOMOTICZ_IP_ADDRESS_STR = "000.000.000.000";
int SENSOR_TIMEOUT_MS = 0;
int eeAddress = 0;
float TEMPERATURE = 0;
unsigned int internalServerPort = 8081;

float therm_hysteresis = 0.5;
bool heating = false;

WiFiServer server(internalServerPort);
ESP_SSD1306 oled(OLED_RESET);
OneWire oneWire(ONE_WIRE_BUS);
DallasTemperature DS18B20(&oneWire);

int getSsidQuality(void);
void oledPushMessage(String);

void eepromShow()
{
  for (int address = 0; address < EEPROM_SIZE; address++) {
      // read a byte from the current address of the EEPROM
      int value = EEPROM.read(address);
    
      Serial.print(address);
      Serial.print("\t");
      Serial.print(value, DEC);
      Serial.println();
  }
}

void eepromClear()
{  
  // write a 0 to all 512 bytes of the EEPROM
  for (int i = 0; i < EEPROM_SIZE; i++) {
      EEPROM.write(i, 0);
      EEPROM.commit();
  }  
}

float eepromReadFloat(int address){   
   union u_tag {
     byte b[4];
     float fval;
   } u;   
   u.b[0] = EEPROM.read(address);
   u.b[1] = EEPROM.read(address+1);
   u.b[2] = EEPROM.read(address+2);
   u.b[3] = EEPROM.read(address+3);
   return u.fval;
}
 
void eepromWriteFloat(int address, float value){
   Serial.println("Saving SETPOINT to EEPROM");
   union u_tag {
     byte b[4];
     float fval;
   } u;
   u.fval=value;
 
   EEPROM.write(address  , u.b[0]);
   EEPROM.write(address+1, u.b[1]);
   EEPROM.write(address+2, u.b[2]);
   EEPROM.write(address+3, u.b[3]);
   EEPROM.commit();
}


bool getSensorValues()
{
    int i = 10;
    DS18B20.requestTemperatures();
    do {
      i -= 1;
      TEMPERATURE = DS18B20.getTempCByIndex(0);
    } while ((TEMPERATURE == 85.0 || TEMPERATURE == (-127.0)) && i > 0 );
    // Check if any reads failed and exit early (to try again).
    if (isnan(TEMPERATURE) || TEMPERATURE == 85.0 || TEMPERATURE == (-127.0)) {
        Serial.println("Failed to read from DS18B20 sensor!");
        TEMPERATURE = 99;
        return false;    
    } else {
        Serial.println("Sensor Temp : " + String(TEMPERATURE));
        return true;
    }
}

unsigned long lastSensorSendTime = 0;


float getSetPoint() {
  Serial.println("Fetching SetPoint in Domoticz database...");
  WiFiClient client;
  client.setNoDelay(true);
  client.stop();
  //Serial.print("Connectiong Client...");
  if (!client.connect(DOMOTICZ_IP_ADDRESS, DOMOTICZ_PORT)) {
    Serial.println("Fail to contact server");
    client.stop();
    oledPushMessage("Server down");
    return false;
  }
  //Serial.print("POSTING data to URL...");
  client.print("GET /json.htm?type=devices&rid=" + String(THERMOSTAT_IDX));
  client.println( " HTTP/1.1");
  client.print( "Host: " );
  client.println(DOMOTICZ_IP_ADDRESS_STR);
  client.println( "Connection: close" );  
  client.println();
  client.println();  
  delay(500); // wait for server to respond
  // read response
  String result = "";
  String section = "headers";
  while(client.available()){
    String line = client.readStringUntil('\r');
    //Serial.println(line);
    //Serial.println("*********************");
    
    if (section == "headers") {
      //Serial.println("//");
      if (line=="\n") { // skips the empty space at the beginning 
          section="json";
      }
    }
    else if (section == "json") {
      result = line;
      section = "'ignore";
    }
  }
  //Serial.print("closing connection. ");
  client.stop();
  //Serial.println(result);
  int size = result.length() + 1;
  char json[size];
  result.toCharArray(json, size);
  //Serial.println(json);
  StaticJsonBuffer<800> jsonBuffer;
  JsonObject& root = jsonBuffer.parseObject(const_cast<char*>(json));
  if (!root.success()) {
    Serial.println("parseObject() failed");
    return SETPOINT;
  }
  const char* setPoint = root["result"][0]["SetPoint"];
  Serial.println("    got " + String(setPoint));
  oledPushMessage("Online");
  return String(setPoint).toFloat();
}


bool pushSetPoint() {
  WiFiClient client;
  client.setNoDelay(true);
  Serial.println("Pushing setPoint...");
  if (!client.connect(DOMOTICZ_IP_ADDRESS, DOMOTICZ_PORT)) {
    Serial.println("Fail to contact server");
    client.stop();
    oledPushMessage("Server down");
    return false;
  }
  //Serial.print("POSTING data to URL...");
  client.print("GET /json.htm?type=command&param=udevice&idx="+String(THERMOSTAT_IDX)+"&nvalue=0&svalue=");
  client.print( SETPOINT );
  client.println( " HTTP/1.1");
  client.print( "Host: " );
  client.println(DOMOTICZ_IP_ADDRESS_STR);
  client.println( "Connection: keep-alive" );  
  client.println();
  client.println();
  Serial.println("Done");
  client.flush();
  client.stop();
  oledPushMessage("Online");
  delay(1);
}

bool pushTemperature() {
  WiFiClient client;
  client.setNoDelay(true);
  Serial.println("Pushing Temperature...");
  if (!client.connect(DOMOTICZ_IP_ADDRESS, DOMOTICZ_PORT)) {
    Serial.println("Fail to contact server");
    client.stop();
    oledPushMessage("Server down");
    return false;
  }
  //Serial.print("POSTING data to URL...");
  client.print("GET /json.htm?type=command&param=udevice&idx="+String(TEMPSENSOR_IDX)+"&nvalue=0&svalue=");
  client.print( TEMPERATURE );
  client.println( " HTTP/1.1");
  client.print( "Host: " );
  client.println(DOMOTICZ_IP_ADDRESS_STR);
  client.println( "Connection: keep-alive" );  
  client.println();
  client.println();
  Serial.println("Done");
  client.flush();
  client.stop();
  oledPushMessage("Online");
  delay(1);
}

bool pushHeaterStatus() {
  WiFiClient client;
  client.setNoDelay(true);
  Serial.println("Pushing Heater status...");
  if (!client.connect(DOMOTICZ_IP_ADDRESS, DOMOTICZ_PORT)) {
    Serial.println("Fail to contact server");
    client.stop();
    oledPushMessage("Server down");
    return false;
  }
  String url = "GET /json.htm?type=command&param=switchlight&idx="+String(HEATER_SWITCH_IDX)+"&switchcmd=";
  if(heating) {
    url += "On";
  }
  else{
   url += "Off";
  }
  url += "&passcode=lalalou84";
  client.print(url);
  client.println( " HTTP/1.1");
  client.print( "Host: " );
  client.println(DOMOTICZ_IP_ADDRESS_STR);
  client.println( "Connection: keep-alive" );  
  client.println();
  client.println();
  Serial.println("Done");
  client.flush();
  client.stop();
  oledPushMessage("Online");
  delay(1);
}

void displayOff() {
  return oled.ssd1306_command(0xAE);
}

void displayOn() {
  oled.ssd1306_command(0xAF);
}

void clearWifiLogo(void) {
    oled.fillRect(WIFI_LOGO_X - 5*(WIFI_LOGO_BAR_W + WIFI_LOGO_BAR_SPACE), WIFI_LOGO_Y, 5*(WIFI_LOGO_BAR_W + WIFI_LOGO_BAR_SPACE + WIFI_LOGO_W), WIFI_LOGO_H, 0);
}

void drawWifilogo(int quality) {
     int h = 0;
     clearWifiLogo();
     if(quality > 80) {
          h = 11;
          oled.fillRect(WIFI_LOGO_X - 5*(WIFI_LOGO_BAR_W + WIFI_LOGO_BAR_SPACE), WIFI_LOGO_Y + (WIFI_LOGO_H - h) , WIFI_LOGO_BAR_W, h, 1);
     }
     if(quality > 60) {
          h = 9;
          oled.fillRect(WIFI_LOGO_X - 4*(WIFI_LOGO_BAR_W + WIFI_LOGO_BAR_SPACE), WIFI_LOGO_Y + (WIFI_LOGO_H - h) , WIFI_LOGO_BAR_W, h, 1);
     }
     if(quality > 40) {
          h = 7;
          oled.fillRect(WIFI_LOGO_X - 3*(WIFI_LOGO_BAR_W + WIFI_LOGO_BAR_SPACE), WIFI_LOGO_Y + (WIFI_LOGO_H - h) , WIFI_LOGO_BAR_W, h, 1);
     }
     if(quality > 20) {
          h = 5;
          oled.fillRect(WIFI_LOGO_X - 2*(WIFI_LOGO_BAR_W + WIFI_LOGO_BAR_SPACE), WIFI_LOGO_Y + (WIFI_LOGO_H - h) , WIFI_LOGO_BAR_W, h, 1);
     }
     h = 3;
     oled.fillRect(WIFI_LOGO_X - (WIFI_LOGO_BAR_W + WIFI_LOGO_BAR_SPACE), WIFI_LOGO_Y + (WIFI_LOGO_H - h) , WIFI_LOGO_BAR_W, h, 1);
     // draw antenna
     oled.drawBitmap(WIFI_LOGO_X, WIFI_LOGO_Y,  wifiLogo, WIFI_LOGO_W, WIFI_LOGO_H, 1);
     oled.display();
}


void oledPushMessage(String message) {
    oled.fillRect(0, 0, SSD1306_LCDWIDTH - (5*(WIFI_LOGO_BAR_W + WIFI_LOGO_BAR_SPACE) + WIFI_LOGO_W), WIFI_LOGO_Y + WIFI_LOGO_H, 0);
    oled.setTextColor(WHITE);
    oled.setCursor(0, WIFI_LOGO_Y);
    oled.setTextSize(1);
    oled.print(message);
    oled.display();
}


void oledPushTemps(float temperature, float setpoint) {
    oled.fillRect(0, 20, SSD1306_LCDWIDTH - 23, 28, 0);
    oled.setTextColor(WHITE);
    oled.setTextSize(4);
    oled.setCursor(10, 20);
    oled.print(String(int(temperature)));
    oled.setTextSize(1);
    oled.print(String(temperature,1).substring(3));
    oled.setTextSize(2);
    oled.setCursor(55, 34);
    oled.print(" " + String(int(setpoint)));
    oled.setTextSize(1);
    oled.print(String(setpoint,1).substring(3));
    oled.display();
}

void oledPushHeaterStatus(bool heaterStatus) {
     oled.fillRect(105, 34, SSD1306_LCDWIDTH - 105, 15, 0);
     if(heaterStatus) {
         oled.drawBitmap(105, 34,  burnLogo, 11, 15, 1);
     oled.display();
     }
     oled.display();
}

void oledDrawTimeToNextSend() {
    oled.fillRect(0, SSD1306_LCDHEIGHT - 3, SSD1306_LCDWIDTH, 3, 1);
    unsigned long perCent =  (millis() - lastSensorSendTime)/(SENSOR_TIMEOUT * 10);
    int barSizePx = (SSD1306_LCDWIDTH * perCent)/100;
    //Serial.println("Progressbar px : " + String(barSizePx));
    oled.fillRect(SSD1306_LCDWIDTH - barSizePx, SSD1306_LCDHEIGHT - 3, barSizePx, 3, 0);
    oled.display();
}

void handleButton(bool plus) {
   delay(BUTTON_DEBOUNCE);
   SETPOINT += (plus ? 0.5 : -0.5);
}

void heat(bool ON) {
    if (ON == true) {
        pinMode(PIN_RELAY, OUTPUT);
        digitalWrite(PIN_RELAY, LOW);
        heating = true;
    }
    else {
        pinMode(PIN_RELAY, INPUT);
        heating = false;
    }
    oledPushHeaterStatus(ON);
}


void handleThermostat() {
    // If Sensor failure
    if(TEMPERATURE == 99) {
        heat(false);
    }
    if (TEMPERATURE < SETPOINT - therm_hysteresis) {
        heat(true);
    }
    if (TEMPERATURE > SETPOINT + therm_hysteresis) {
        heat(false);
    }
}


void parseBytes(const char* str, char sep, byte* bytes, int maxBytes, int base) {
    for (int i = 0; i < maxBytes; i++) {
        bytes[i] = strtoul(str, NULL, base);  // Convert byte
        str = strchr(str, sep);               // Find next separator
        if (str == NULL || *str == '\0') {
            break;                            // No more separators, exit
        }
        str++;                                // Point to next character after separator
    }
}

// EEPROM SAVE AND READ FUNCTIONS

void saveThermostatSetpointIDX(unsigned int IDX, int address) {
	EEPROM.write(address  , (uint8_t)THERMOSTAT_IDX);
	EEPROM.commit();
}

void saveHeaterSwitchIDX(unsigned int IDX, int address) {
	EEPROM.write(address+1  , (uint8_t)HEATER_SWITCH_IDX);
	EEPROM.commit();
}

void saveTempSensorIDX(unsigned int IDX, int address) {
	EEPROM.write(address+2  , (uint8_t)TEMPSENSOR_IDX);
	EEPROM.commit();
}

void saveServerIp(String IP, int address) {
    char temp[20];
    IP.toCharArray(temp,15);
    Serial.println("SAVING SERVER IP TO EEPROM :");
    Serial.println(temp);
    parseBytes(temp, '.', DOMOTICZ_IP_ADDRESS, 4, 10);
    EEPROM.write(address+4  , DOMOTICZ_IP_ADDRESS[0]);
    EEPROM.write(address+5, DOMOTICZ_IP_ADDRESS[1]);
    EEPROM.write(address+6, DOMOTICZ_IP_ADDRESS[2]);
    EEPROM.write(address+7, DOMOTICZ_IP_ADDRESS[3]);
    EEPROM.commit();
    Serial.println("Done");
}

void saveServerPort(unsigned int PORT, int address) {
    EEPROM.write(address+8, PORT);
    EEPROM.write(address+9, PORT >> 8);
    EEPROM.commit();
    Serial.println("Done");
}

void saveSensorTimeout(int timeout, int address) {
    EEPROM.write(address+10, SENSOR_TIMEOUT);
    EEPROM.write(address+11, SENSOR_TIMEOUT >> 8);
    EEPROM.commit();
    Serial.println("SAVING SENSOR TIMOUT TO EEPROM : " + String(SENSOR_TIMEOUT) + " S");
}

void readSettingsFromEEPROM(int address) {
	THERMOSTAT_IDX = (int)EEPROM.read(address);
	HEATER_SWITCH_IDX = (int)EEPROM.read(address+1);
	TEMPSENSOR_IDX = (int)EEPROM.read(address+2);
    byte port_lsb = 0;
    byte port_msb = 0;
    byte sensorTimeout_lsb = 0;
    byte sensorTimeout_msb = 0;
    Serial.print("READING SERVER IP FROM EEPROM : ");
    DOMOTICZ_IP_ADDRESS[0] = EEPROM.read(address+4);
    DOMOTICZ_IP_ADDRESS[1] = EEPROM.read(address+5);
    DOMOTICZ_IP_ADDRESS[2] = EEPROM.read(address+6);
    DOMOTICZ_IP_ADDRESS[3] = EEPROM.read(address+7);
    port_lsb           = EEPROM.read(address+8);
    port_msb           = EEPROM.read(address+9);
    DOMOTICZ_PORT          = (port_msb << 8);
    DOMOTICZ_PORT          = DOMOTICZ_PORT + port_lsb;
    sensorTimeout_lsb  = EEPROM.read(address+10);
    sensorTimeout_msb  = EEPROM.read(address+11);
    SENSOR_TIMEOUT     = (sensorTimeout_msb << 8);
    SENSOR_TIMEOUT     = SENSOR_TIMEOUT + sensorTimeout_lsb;
    SENSOR_TIMEOUT_MS  = SENSOR_TIMEOUT * 1000;
    String IP = String(DOMOTICZ_IP_ADDRESS[0],10);
    IP += ".";
    IP += String(DOMOTICZ_IP_ADDRESS[1],10);
    IP += ".";
    IP += String(DOMOTICZ_IP_ADDRESS[2],10);
    IP += ".";
    IP += String(DOMOTICZ_IP_ADDRESS[3],10);
    DOMOTICZ_IP_ADDRESS_STR = IP;
    Serial.println(DOMOTICZ_IP_ADDRESS_STR);
    Serial.println("PORT : " + String(DOMOTICZ_PORT));
    Serial.println("SENSOR TIMEOUT : " + String(SENSOR_TIMEOUT));
    
    Serial.println("Done");
}

void configModeCallback (WiFiManager *myWiFiManager) {
  Serial.println("Entered config mode");
  Serial.println(WiFi.softAPIP());
  oled.clearDisplay();
  oled.setCursor(0,0);
  oled.println("...failed");
  oled.print("connect to " + String(myWiFiManager->getConfigPortalSSID()) + " to configure wifi");
  oled.print(WiFi.softAPIP().toString() + ":" + String(internalServerPort));
  oled.display();
  Serial.println("Connect to " + myWiFiManager->getConfigPortalSSID() + ":" + String(internalServerPort) + " to configure wifi");
}


bool isIntervalElapsed(unsigned long interval, unsigned long referenceTime) {
  return (unsigned long)(millis() - referenceTime) >= interval;
}


String getHttpRequestParamValue(String input_str, String param) {
      int param_index = input_str.indexOf(param);
      if(param_index == -1) {return "\0";}
      int start_chr = input_str.indexOf("=", param_index + 1) + 1;
      if(input_str.charAt(param_index + param.length()) == 0x26) {return "NULL";}
      int end_chr = input_str.indexOf("&", start_chr);
      return input_str.substring(start_chr, end_chr);
}

bool checkHttpRequestParam(String input_str, String param) {
    int param_index = input_str.indexOf(param);
    if(param_index == -1) {return false;}
    else {return true;}
}


int getSsidQuality(void) {
    int quality = 2 * (int(WiFi.RSSI()) + 100);
    if(quality > 100) {return 100;}
    else if(quality < 0) {return 0;}
    else {return quality;}
}

String toHtml(String input_string) {
    return "HTTP/1.1 200 OK\r\nContent-Type: text/html\r\n\r\n<!DOCTYPE HTML>\r\n<html>\r\n" + input_string + "\n</html>\n";
}

//###################################################################################### SETUP

void setup() {
  Serial.begin(BAUDRATE);
  Serial.println("########## I AM A DOMOTICZ THERMOSTAT ################");
  WiFiManager wifiManager;
  pinMode(PLUS_PIN, INPUT);           // set pin to input
  pinMode(MINUS_PIN, INPUT);           // set pin to input
  pinMode(PIN_RELAY, INPUT);           // set pin to input  
  Wire.begin(PIN_SDA, PIN_SCL);
  EEPROM.begin(EEPROM_SIZE);
  DS18B20.begin();
  DS18B20.setResolution(10);
  oled.begin(SSD1306_SWITCHCAPVCC, I2C_ADDRESS, true);
  displayOn();
  oled.clearDisplay();
  oled.setTextWrap(true);
  oled.drawBitmap(0, 5,  startupLogo, 63, 62, 1);
  oled.setTextSize(1);
  oled.setTextColor(WHITE);
  oled.setCursor(65, 30);
  oled.print("Connecting");
  oled.display();
  wifiManager.setAPCallback(configModeCallback);
  wifiManager.setConfigPortalTimeout(300);
  wifiManager.autoConnect("AutoConnectAP");
  oled.setCursor(65, 45);
  oled.print("OK");
  oled.display();
  delay(1000);
  server.begin();
  Serial.println("Server started");
  //eepromShow();
  //SETPOINT = eepromReadFloat(eeAddress);
  readSettingsFromEEPROM(eeAddress);

  // Print the IP address
  Serial.println(WiFi.localIP());
  Serial.println("SSID : " + String(WiFi.SSID()));
  Serial.println("RSSI : " + String(WiFi.RSSI()));
  Serial.println("Quality : " + String(getSsidQuality()));
  oled.clearDisplay();
  oledPushMessage(WiFi.localIP().toString());
  drawWifilogo(getSsidQuality());
  Serial.println("WiFi Status : " + String(WiFi.status()));
  SETPOINT = getSetPoint();  
  getSensorValues();
  handleThermostat();
  oledPushTemps(TEMPERATURE, SETPOINT);
  lastSensorSendTime = millis();
}

//###################################################################################### LOOP

bool inputMode = false;
unsigned long lastButtonActivationTime = 0;
//unsigned long lastSensorSendTime = 0;
unsigned long lastMsgDisplayTime = 0;

void loop() {
    int plusState = digitalRead(PLUS_PIN);
    int minusState = digitalRead(MINUS_PIN);
    
    if (inputMode || plusState == HIGH || minusState == HIGH) { // the user is pushing the buttons      
        if (plusState == HIGH || minusState == HIGH) { // the buttons were pressed since the last loop
            inputMode = true;
            lastButtonActivationTime = millis();
            if (plusState == LOW || minusState == LOW) {
                handleButton(plusState == HIGH);
                oledPushTemps(TEMPERATURE, SETPOINT);
            } else {
                SETPOINT = 10; // set to the absent variable
                //updateScreen(false);
                oledPushTemps(TEMPERATURE, SETPOINT);
                delay(RESET_DEBOUNCE);
            }
        } else if (isIntervalElapsed(BUTTON_TIMEOUT, lastButtonActivationTime)) {
            //eepromWriteFloat(eeAddress, SETPOINT);
            handleThermostat();
            pushHeaterStatus();
            pushSetPoint();
            inputMode = false;
        }
    } else { // idle mode
        WiFiClient client = server.available();
        if (client) {
            String rawRequest = client.readStringUntil('\r');
            String request = rawRequest;
            request.remove(rawRequest.indexOf("HTTP/1.1"));
            Serial.println("New Client Request : " + request);
            IPAddress remote = client.remoteIP();
            // send a standard http response header
            client.println("HTTP/1.1 200 OK ");
            client.println("Content-Type: text/html");
            client.println("Connection: close");  // the connection will be closed after completion of the response
            client.println();
            String html_response = ("<!DOCTYPE HTML>");
            html_response += ("<html>");
            // Check method
            // If post, set variables
            if(checkHttpRequestParam(request, "POST")) {
                Serial.println("POST request type");
                bool faltyRequest = true;
                // Check command
                // if setpoint
                if(checkHttpRequestParam(request, "setpoint")) {
                    faltyRequest = false;
                    String setpointStr = getHttpRequestParamValue(request, "setpoint");
                    SETPOINT = setpointStr.toFloat();
                    //eepromWriteFloat(eeAddress, SETPOINT);
                    Serial.println("New Setpoint : " + setpointStr);
                    html_response += "New setpoint " + String(DOMOTICZ_PORT)+ " °C set<br>";
                }
				// if setServerIp
                if(checkHttpRequestParam(request, "setServerIp")) {
                    // Set ServerIp
                    DOMOTICZ_IP_ADDRESS_STR  = getHttpRequestParamValue(request, "setServerIp");
                    faltyRequest = false;                 
                    Serial.println("IP : " + String(DOMOTICZ_IP_ADDRESS_STR) + " set");
                    html_response += "ip " + String(DOMOTICZ_IP_ADDRESS_STR)+ " set<br>";
                    saveServerIp(DOMOTICZ_IP_ADDRESS_STR, eeAddress);
                }
                // if setServerPort
                if(checkHttpRequestParam(request, "setServerPort")) {
                    // Set ServerPort
                    String port = getHttpRequestParamValue(request, "setServerPort");
                    DOMOTICZ_PORT = port.toInt();
                    faltyRequest = false;                 
                    Serial.println("PORT : " + String(DOMOTICZ_PORT));
                    html_response += "port " + String(DOMOTICZ_PORT)+ " set<br>";
                    saveServerPort(DOMOTICZ_PORT, eeAddress);
                }
                // if setSensorTimeout
                if(checkHttpRequestParam(request, "setSensorTimeout")) {
                    // Set ServerIP
                    String sensorTimout = getHttpRequestParamValue(request, "setSensorTimeout");
                    SENSOR_TIMEOUT = sensorTimout.toInt();
                    SENSOR_TIMEOUT_MS = SENSOR_TIMEOUT * 1000;
                    faltyRequest = false;                 
                    Serial.println("SENSOR_TIMEOUT : " + String(SENSOR_TIMEOUT));
                    html_response += "Sensor timeout " + String(SENSOR_TIMEOUT) + "s set<br><br>";
                    saveSensorTimeout(SENSOR_TIMEOUT, eeAddress);
                }
				// If one of idx
				if(checkHttpRequestParam(request, "setThermostatSetpointIdx")) {
					faltyRequest = false;
					THERMOSTAT_IDX = getHttpRequestParamValue(request, "setThermostatSetpointIdx").toInt();
                    Serial.println("THERMOSTAT_IDX : " + String(THERMOSTAT_IDX));
                    html_response += "thermostat_idx " + String(THERMOSTAT_IDX)+ " set<br>";
					saveThermostatSetpointIDX(THERMOSTAT_IDX, eeAddress);
				}
				if(checkHttpRequestParam(request, "setHeaterSwitchIdx")) {
					faltyRequest = false;
					HEATER_SWITCH_IDX = getHttpRequestParamValue(request, "setHeaterSwitchIdx").toInt();
                    Serial.println("HEATER_SWITCH_IDX : " + String(HEATER_SWITCH_IDX));
                    html_response += "heaterSwitch_idx " + String(HEATER_SWITCH_IDX)+ " set<br>";
					saveHeaterSwitchIDX(HEATER_SWITCH_IDX, eeAddress);
				}
				if(checkHttpRequestParam(request, "setTempSensorIdx")) {
					faltyRequest = false;
					TEMPSENSOR_IDX = getHttpRequestParamValue(request, "setTempSensorIdx").toInt();
                    Serial.println("THERMOSTAT_IDX : " + String(TEMPSENSOR_IDX));
                    html_response += "tempSensor_idx " + String(TEMPSENSOR_IDX)+ " set<br>";
					saveTempSensorIDX(TEMPSENSOR_IDX, eeAddress);
				}
                // if httpUpdate
                if(checkHttpRequestParam(request, "httpUpdate")) {
                    faltyRequest = false;
                    // Respond to client
                    String binPath = getHttpRequestParamValue(request, "httpUpdate");
					unsigned int firstDotsIndex = binPath.indexOf(":");
					unsigned int firstSlashIndex = binPath.indexOf("/");
					String httpUpdateIp   = binPath.substring(0, firstDotsIndex);
					String httpUpdatePort = binPath.substring(firstDotsIndex + 1, firstSlashIndex);
					String httpUpdatePath = binPath.substring(firstSlashIndex);
					html_response += "httpUpdate toggled, path : " + httpUpdateIp + ":" + httpUpdatePort + httpUpdatePath + "<br>";
                    Serial.println("httpUpdate toggled, path : " + httpUpdateIp + ":" + httpUpdatePort + httpUpdatePath);
					
					Serial.println(httpUpdateIp);
					Serial.println(httpUpdatePort);
					Serial.println(httpUpdatePath);
					
					// Fetch the new bin to flash
                    t_httpUpdate_return ret = ESPhttpUpdate.update(httpUpdateIp, httpUpdatePort.toInt(), httpUpdatePath);
                    delay(1000);
                    switch(ret) {
                        case HTTP_UPDATE_FAILED:
                            html_response += "    [update] Update failed<br>";
                            Serial.println("[update] Update failed.");
                            Serial.println("Error" + String(ESPhttpUpdate.getLastError())  + ESPhttpUpdate.getLastErrorString().c_str());
                        break;
                        case HTTP_UPDATE_NO_UPDATES:
                            html_response += "    [update] Update no Update<br>";
                            Serial.println("[update] Update no Update.");
                        break;
                    }
                }
                // if unknown request
                if (faltyRequest == true) {
                    html_response += "Request Error : variable to set unknown<br>";
                    Serial.println("Request Error : variable to set unauthorized");
                }
            }
            else if(checkHttpRequestParam(request, "GET")) {
                Serial.println("GET request type");
                if(checkHttpRequestParam(request, "ping")) {
                    html_response += "Pong :-)<br>Toggling setpoint fetch, handle thermostat and pushHeaterStatus<br>";
                }
                if(checkHttpRequestParam(request, "whoAreYou")) {
                    html_response += "<br>I am a domoticz thermostat<br>";
                    html_response += "Current data :<br>Server IP : " + String(DOMOTICZ_IP_ADDRESS_STR) + "<br>";
                    html_response += "<br>Server port : " + String(DOMOTICZ_PORT) + "<br>";
                    html_response += "<br>Sensor timeout : " + String(SENSOR_TIMEOUT) + "<br>";
                    html_response += "<br>Temperature:" + String(TEMPERATURE) + "C<br>";
                    html_response += "<br>Setpoint:" + String(SETPOINT) + "C<br>";
                    html_response += "<br>Heater Status:" + String(heating) + "<br>";
                    html_response += "<br>RSSI : " + String(getSsidQuality()) + "%<br>";
					html_response += "<br>";
					html_response += "<br>Thermostat SetPoint IDX : " + String(THERMOSTAT_IDX);
					html_response += "<br>Heater IDX : " + String(HEATER_SWITCH_IDX);
					html_response += "<br>TempSensor IDX : " + String(TEMPSENSOR_IDX);
                }
            }
            html_response += "\n</html>\n";
            client.print(html_response);
            client.println();
            client.flush();
            client.stop();
            Serial.println("Client disonnected");
            delay(500);
            SETPOINT = getSetPoint();
            handleThermostat();
            pushHeaterStatus();
            oledPushTemps(TEMPERATURE, SETPOINT);
            oledDrawTimeToNextSend();
        }

        
        else if (isIntervalElapsed(SENSOR_TIMEOUT_MS, lastSensorSendTime)) {
            oledDrawTimeToNextSend();
            if(getSensorValues()) {
                handleThermostat();
                Serial.println("Updating RSSI on OLED");
            }
            else {
                Serial.println("ERROR : Temps sensor down");
                heat(false);
                oledPushMessage("Sensor down");
            }
            if(WiFi.status() == WL_CONNECTED) {
                drawWifilogo(getSsidQuality());
                Serial.println("Wifi Ok, trying to send to server...");
                SETPOINT = getSetPoint();
                pushHeaterStatus();
                pushTemperature();
                lastSensorSendTime = millis();
            }
            else {
                Serial.println("Send aborted, no Wifi connection");
                oledPushMessage("No Wifi");
                clearWifiLogo();
                Serial.println("No Wifi, aborting send to server");
            }
        oledPushTemps(TEMPERATURE, SETPOINT);
        lastSensorSendTime = millis();
        }
        else if (isIntervalElapsed(LAST_SEND_ATTEMPT_MSG_DISPLAY_TIMEOUT, lastMsgDisplayTime)) {
            oledDrawTimeToNextSend();
            lastMsgDisplayTime = millis();
        }
    }
}

