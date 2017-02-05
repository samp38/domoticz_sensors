#include <ESP8266httpUpdate.h>
#include <ESP8266WiFi.h>
#include <WiFiManager.h>
#include <EEPROM.h>
#include <OneWire.h>
#include <DallasTemperature.h>

//EEPROM
#define EEPROM_SIZE 512

//Serial
#define BAUDRATE 115200

#define ONE_WIRE_BUS D5  // DS18B20 pin

//#define SENSOR_TIMEOUT 60000
#define LAST_SEND_ATTEMPT_MSG_DISPLAY_TIMEOUT 1000

#define DS18B20_RESOLUTION 11

// Variable saved to EPORMM
byte DOMOTICZ_IP_ADDRESS[4];
unsigned int DOMOTICZ_PORT = 0;
String DOMOTICZ_IP_ADDRESS_STR = "000.000.000.000";
unsigned int SENSOR_TIMEOUT = 10;
unsigned int TEMPSENSOR_IDX = 4;

int SENSOR_TIMEOUT_MS = 0;
int eeAddress = 500;
String LOCALIP = "";
float TEMPERATURE = 0;

WiFiServer server(8081);
OneWire oneWire(ONE_WIRE_BUS);
DallasTemperature DS18B20(&oneWire);
byte mac[6];
int getSsidQuality(void);

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

bool pushTemperature() {
  WiFiClient client;
  Serial.println("Pushing setPoint...");
  if (!client.connect(DOMOTICZ_IP_ADDRESS, DOMOTICZ_PORT)) {
    Serial.println("Fail to contact server");
    client.stop();
    return false;
  }
  //Serial.print("POSTING data to URL...");
  client.print("GET /json.htm?type=command&param=udevice&idx="+String(TEMPSENSOR_IDX)+"&nvalue=0&svalue=");
  client.print( TEMPERATURE );
  client.println( " HTTP/1.1");
  client.print( "Host: " );
  client.println(DOMOTICZ_IP_ADDRESS_STR);
  client.println( "Connection: close" );  
  client.println();
  client.println();
  Serial.println("Done");
  client.flush();
  client.stop();
  delay(1);
  // reset timer
  lastSensorSendTime = millis();
  return true;
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

void saveServerIpAddress(String IP, unsigned int PORT, int address) {
    char temp[20];
    IP.toCharArray(temp,15);
    Serial.println("SAVING SERVER IP TO EEPROM :");
    Serial.println(temp);
    parseBytes(temp, '.', DOMOTICZ_IP_ADDRESS, 4, 10);
    EEPROM.write(address+4  , DOMOTICZ_IP_ADDRESS[0]);
    EEPROM.write(address+5, DOMOTICZ_IP_ADDRESS[1]);
    EEPROM.write(address+6, DOMOTICZ_IP_ADDRESS[2]);
    EEPROM.write(address+7, DOMOTICZ_IP_ADDRESS[3]);
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
    byte port_lsb = 0;
    byte port_msb = 0;
    byte sensorTimeout_lsb = 0;
    byte sensorTimeout_msb = 0;
    Serial.println("READING SETTINGS IP FROM EEPROM :");
    // Read Server IP
    DOMOTICZ_IP_ADDRESS[0] = EEPROM.read(address+4);
    DOMOTICZ_IP_ADDRESS[1] = EEPROM.read(address+5);
    DOMOTICZ_IP_ADDRESS[2] = EEPROM.read(address+6);
    DOMOTICZ_IP_ADDRESS[3] = EEPROM.read(address+7);
    String IP = String(DOMOTICZ_IP_ADDRESS[0],10);
    IP += ".";
    IP += String(DOMOTICZ_IP_ADDRESS[1],10);
    IP += ".";
    IP += String(DOMOTICZ_IP_ADDRESS[2],10);
    IP += ".";
    IP += String(DOMOTICZ_IP_ADDRESS[3],10);
    DOMOTICZ_IP_ADDRESS_STR = IP;
    // Read port
    port_lsb           = EEPROM.read(address+8);
    port_msb           = EEPROM.read(address+9);
    DOMOTICZ_PORT          = (port_msb << 8);
    DOMOTICZ_PORT          = DOMOTICZ_PORT + port_lsb;
    // Read timeout
    sensorTimeout_lsb  = EEPROM.read(address+10);
    sensorTimeout_msb  = EEPROM.read(address+11);
    SENSOR_TIMEOUT     = (sensorTimeout_msb << 8);
    SENSOR_TIMEOUT     = SENSOR_TIMEOUT + sensorTimeout_lsb;
    SENSOR_TIMEOUT_MS  = SENSOR_TIMEOUT * 1000;
    Serial.println("ServerIP       :" + String(DOMOTICZ_IP_ADDRESS_STR));
    Serial.println("PORT           :" + String(DOMOTICZ_PORT));
    Serial.println("SENSOR TIMEOUT : " + String(SENSOR_TIMEOUT));
    Serial.println("Done");
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
  WiFiManager wifiManager;
  EEPROM.begin(EEPROM_SIZE);
  DS18B20.begin();
  DS18B20.setResolution(DS18B20_RESOLUTION);
  wifiManager.setConfigPortalTimeout(600);
  wifiManager.autoConnect("AutoConnectAP");
  delay(1000);
  WiFi.macAddress(mac);
  Serial.print("MAC: ");
  Serial.print(mac[5],HEX);
  Serial.print(":");
  Serial.print(mac[4],HEX);
  Serial.print(":");
  Serial.print(mac[3],HEX);
  Serial.print(":");
  Serial.print(mac[2],HEX);
  Serial.print(":");
  Serial.print(mac[1],HEX);
  Serial.print(":");
  Serial.println(mac[0],HEX);
  
  server.begin();
  Serial.println("Server started");
  //eepromShow();
  readSettingsFromEEPROM(eeAddress);

  // Print the IP address
  Serial.println(WiFi.localIP());
  Serial.println("SSID : " + String(WiFi.SSID()));
  Serial.println("RSSI : " + String(WiFi.RSSI()));
  Serial.println("Quality : " + String(getSsidQuality()) + "%");
  getSensorValues();
  lastSensorSendTime = millis();
  Serial.println("WiFi Status : " + String(WiFi.status()));
}

//###################################################################################### LOOP

void loop() {
    WiFiClient client = server.available();
    if (client) {
        String rawRequest = client.readStringUntil('\r');
        String request = rawRequest;
        request.remove(rawRequest.indexOf("HTTP/1.1"));
        Serial.println("New Client Request : " + request);
        IPAddress remote = client.remoteIP();
        client.flush();
        client.print("HTTP/1.1 200 OK\r\nContent-Type: text/html\r\n\r\n<!DOCTYPE HTML>\r\n<html>\r\n");
        // Check method
        // If post, set variables
        if(checkHttpRequestParam(request, "POST")) {
            Serial.println("POST request type");
            bool faltyRequest = true;
            // Check command
            // if setServerIp
            if(checkHttpRequestParam(request, "setServerPort")) {
                // Set ServerIP
                String port = getHttpRequestParamValue(request, "setServerPort");
                DOMOTICZ_PORT = port.toInt();
                faltyRequest = false;                 
                DOMOTICZ_IP_ADDRESS_STR = String(remote[0]) + "." + String(remote[1]) + "." + String(remote[2]) + "." + String(remote[3]);
                Serial.println("PORT : " + String(DOMOTICZ_PORT));
                client.print("port " + String(DOMOTICZ_PORT)+ " set<br>" );
                saveServerIpAddress(DOMOTICZ_IP_ADDRESS_STR, DOMOTICZ_PORT, eeAddress);
            }
            // if setSensorTimout
            if(checkHttpRequestParam(request, "setSensorTimeout")) {
                // Set ServerIP
                String sensorTimout = getHttpRequestParamValue(request, "setSensorTimeout");
                SENSOR_TIMEOUT = sensorTimout.toInt();
                SENSOR_TIMEOUT_MS = SENSOR_TIMEOUT * 1000;
                faltyRequest = false;                 
                Serial.println("SENSOR_TIMEOUT : " + String(SENSOR_TIMEOUT));
                client.print("Sensor timeout " + String(SENSOR_TIMEOUT) + "s set<br><br>");
                saveSensorTimeout(SENSOR_TIMEOUT, eeAddress);
            }
            // if httpUpdate
            if(checkHttpRequestParam(request, "httpUpdate")) {
                faltyRequest = false;
                // Respond to client
                String binPath = getHttpRequestParamValue(request, "httpUpdate");
                Serial.println("httpUpdate toggled, path: " + DOMOTICZ_IP_ADDRESS_STR + binPath);
                client.print("httpUpdate toggled, path : "+ DOMOTICZ_IP_ADDRESS_STR + binPath + "<br><br>");
                t_httpUpdate_return ret = ESPhttpUpdate.update(DOMOTICZ_IP_ADDRESS_STR, DOMOTICZ_PORT, binPath);
                delay(1000);
                switch(ret) {
                    case HTTP_UPDATE_FAILED:
                        client.print("    [update] Update failed<br>");
                        Serial.println("[update] Update failed.");
                        Serial.println("Error" + String(ESPhttpUpdate.getLastError())  + ESPhttpUpdate.getLastErrorString().c_str());
                    break;
                    case HTTP_UPDATE_NO_UPDATES:
                        client.print("    [update] Update no Update<br>");
                        Serial.println("[update] Update no Update.");
                    break;
                }
            }
            // if unknown request
            if (faltyRequest == true) {
                client.print("Request Error : variable to set unknown<br>");
                Serial.println("Request Error : variable to set unknown");
            }
        }                        
        // Send variables and respond to client
        getSensorValues();
        client.print("Current data :<br>Server IP : " + String(DOMOTICZ_IP_ADDRESS_STR) + "<br>");
        client.print("<br>Server port : " + String(DOMOTICZ_PORT) + "<br>");
        client.print("<br>Sensor timeout : " + String(SENSOR_TIMEOUT) + "<br>");
        client.print("<br>Temperature:" + String(TEMPERATURE) + "°C<br>");
        client.print("<br>RSSI : " + String(getSsidQuality()) + "%<br>");
        client.print("\n</html>\n");
        client.println();
        client.println();
        client.stop();
        Serial.println("Client disonnected");
        lastSensorSendTime = millis();
    } 

    
    else if (isIntervalElapsed(SENSOR_TIMEOUT_MS, lastSensorSendTime)) {
        if(!getSensorValues()) {
            Serial.println("ERROR : Temps sensor down");
        }
        if(WiFi.status() == WL_CONNECTED) {
            Serial.println("Wifi Ok, trying to send to server...");
            pushTemperature();
        }
        else {
            Serial.println("No Wifi, aborting send to server");
        }
    lastSensorSendTime = millis();
    }
}

