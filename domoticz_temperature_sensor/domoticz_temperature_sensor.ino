#include <ESP8266httpUpdate.h>
#include <ESP8266WiFi.h>
#include <WiFiManager.h>
#include <EEPROM.h>
#include <OneWire.h>
#include <DallasTemperature.h>
#include <ArduinoJson.h>

//EEPROM
#define EEPROM_SIZE 512

//Serial
#define BAUDRATE 115200

#define ONE_WIRE_BUS D5  // DS18B20 pin

//#define SENSOR_TIMEOUT 60000
#define LAST_SEND_ATTEMPT_MSG_DISPLAY_TIMEOUT 1000

#define DS18B20_RESOLUTION 11


/// Globals
// Variable saved to EPORMM
byte DOMOTICZ_IP_ADDRESS[4];
unsigned int DOMOTICZ_PORT = 0;
String DOMOTICZ_IP_ADDRESS_STR = "000.000.000.000";
unsigned int SENSOR_TIMEOUT = 120;
unsigned int TEMPSENSOR_IDX = 0;
unsigned long lastSensorSendTime = 0;
int getSsidQuality(void);


int SENSOR_TIMEOUT_MS = 0;
int eeAddress = 500;
String LOCALIP = "";
float TEMPERATURE = 0;

WiFiServer server(8081);
OneWire oneWire(ONE_WIRE_BUS);
DallasTemperature DS18B20(&oneWire);

bool isIntervalElapsed(unsigned long interval, unsigned long referenceTime) {
  return (unsigned long)(millis() - referenceTime) >= interval;
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

int getSsidQuality(void) {
    int quality = 2 * (int(WiFi.RSSI()) + 100);
    if(quality > 100) {return 100;}
    else if(quality < 0) {return 0;}
    else {return quality;}
}

// ######################################################################## EEPROM FUNCTIONS

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

void saveTempSensorIDX(unsigned int IDX, int address) {
	EEPROM.write(address  , (uint8_t)TEMPSENSOR_IDX);
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
    byte port_lsb = 0;
    byte port_msb = 0;
    byte sensorTimeout_lsb = 0;
    byte sensorTimeout_msb = 0;
    Serial.println("READING SETTINGS FROM EEPROM :");
	// Read TEMPSENSOR_IDX
	TEMPSENSOR_IDX = (int)EEPROM.read(address);
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
    Serial.println("ServerIP       : " + String(DOMOTICZ_IP_ADDRESS_STR));
    Serial.println("PORT           : " + String(DOMOTICZ_PORT));
    Serial.println("SENSOR TIMEOUT : " + String(SENSOR_TIMEOUT));
	Serial.println("TEMPSENSOR_IDX : " + String(TEMPSENSOR_IDX));
    Serial.println("Done");
}

// ######################################################################## SENSOR FUNCTIONS

bool getSensorValues() {
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

// ######################################################################## HTTP FUNCTIONS

bool pushTemperature() {
  WiFiClient client;
  client.setNoDelay(true);
  Serial.println("Pushing temperature...");
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
  client.stop();
  delay(1);
  // reset timer
  lastSensorSendTime = millis();
  return true;
}

String getDomoticzDeviceName(int idx) {
  Serial.println("Fetching device name in Domoticz database...");
  WiFiClient client;
  client.setNoDelay(true);
  client.stop();
  //Serial.print("Connectiong Client...");
  if (!client.connect(DOMOTICZ_IP_ADDRESS, DOMOTICZ_PORT)) {
    Serial.println("Fail to contact server");
    client.stop();
    return "NC";
  }
  //Serial.print("POSTING data to URL...");
  client.print("GET /json.htm?type=devices&rid=" + String(idx));
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
    return "NC";
  }
  const char* name = root["result"][0]["Name"];
  Serial.println("    got " + String(name));
  return String(name);
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
  server.begin();
  Serial.println("Server started");
  //eepromShow();
  readSettingsFromEEPROM(eeAddress);

  // Print the IP address
  Serial.println(WiFi.localIP());
  Serial.println("MAC ADDRESS : " + WiFi.macAddress());
  Serial.println("SSID : " + String(WiFi.SSID()));
  Serial.println("RSSI : " + String(WiFi.RSSI()));
  Serial.println("Quality : " + String(getSsidQuality()) + "%");
  getSensorValues();
  lastSensorSendTime = millis();
  Serial.println("WiFi Status : " + String(WiFi.status()));
  
  // Added for sleepy
  // pushTemperature();
  // Serial.println("Going to sleep for " + String(5 * SENSOR_TIMEOUT) + "S");
  // ESP.deepSleep(5 * SENSOR_TIMEOUT * 1000000);
  // delay(100);
  // Added for sleepy
}

//###################################################################################### LOOP

void loop() {
    WiFiClient client = server.available();
    if (client) {
        Serial.println("--> Request begin");
		String response = "";
		String requestToParse = "";
		String method = "UNKNOWN";
        bool body = false;
		bool faultyRequest = true;
		unsigned long now = millis();
        while(client.connected()) {
			if(millis() - now > 5000) {
				faultyRequest = true;
				response += "client timeout";
				Serial.println("client timeout");
				break;
			}
            String requestPart = client.readStringUntil('\n');
			if(requestPart.indexOf("GET") != -1) {
				method = "GET";
			}
			if(requestPart.indexOf("POST") != -1) {
				method = "POST";
			}
			if(method == "POST") {
    	        if(body) {
    	            requestToParse = requestPart;
    	            break;
    	        }
    	        if(requestPart == "\r") {
    	            body = true;
    	        }
			}
			else if(method == "GET") {
				requestToParse = requestPart;
				requestToParse.remove(requestPart.indexOf("HTTP/1.1"));
				break;
			}
			else {
				response += "Unsupported method";
				break;
			}
        }
        // Check method
		if(method != "UNKNOWN") {
    	    // If post, set variables
    	    if(method == "POST") {
    	        Serial.println("POST request type");
    	        // Check command
				if(checkHttpRequestParam(requestToParse, "tempSensorIdx")) {
					faultyRequest = false;
					TEMPSENSOR_IDX = getHttpRequestParamValue(requestToParse, "tempSensorIdx").toInt();
    	            Serial.println("THERMOSTAT_IDX : " + String(TEMPSENSOR_IDX));
    	            response += "tempSensor_idx " + String(TEMPSENSOR_IDX)+ " set<br>";
					saveTempSensorIDX(TEMPSENSOR_IDX, eeAddress);
				}
				// if setServerIp
    	        if(checkHttpRequestParam(requestToParse, "serverIp")) {
    	            // Set ServerIp
    	            DOMOTICZ_IP_ADDRESS_STR  = getHttpRequestParamValue(requestToParse, "serverIp");
    	            faultyRequest = false;                 
    	            Serial.println("IP : " + String(DOMOTICZ_IP_ADDRESS_STR) + " set");
    	            response += "ip " + String(DOMOTICZ_IP_ADDRESS_STR)+ " set<br>";
    	            saveServerIp(DOMOTICZ_IP_ADDRESS_STR, eeAddress);
    	        }
    	        // if setServerPort
    	        if(checkHttpRequestParam(requestToParse, "serverPort")) {
    	            // Set ServerPort
    	            String port = getHttpRequestParamValue(requestToParse, "serverPort");
    	            DOMOTICZ_PORT = port.toInt();
    	            faultyRequest = false;                 
    	            Serial.println("PORT : " + String(DOMOTICZ_PORT));
    	            response += "port " + String(DOMOTICZ_PORT)+ " set<br>";
    	            saveServerPort(DOMOTICZ_PORT, eeAddress);
    	        }
    	        // if setSensorTimout
    	        if(checkHttpRequestParam(requestToParse, "sensorTimeout")) {
    	            // Set ServerIP
    	            String sensorTimout = getHttpRequestParamValue(requestToParse, "sensorTimeout");
    	            SENSOR_TIMEOUT = sensorTimout.toInt();
    	            SENSOR_TIMEOUT_MS = SENSOR_TIMEOUT * 1000;
    	            faultyRequest = false;                 
    	            Serial.println("SENSOR_TIMEOUT : " + String(SENSOR_TIMEOUT));
    	            response += "Sensor timeout " + String(SENSOR_TIMEOUT) + "s set<br><br>";
    	            saveSensorTimeout(SENSOR_TIMEOUT, eeAddress);
    	        }
    	        // if httpUpdate
    	        if(checkHttpRequestParam(requestToParse, "httpUpdate")) {
    	            faultyRequest = false;
    	            // Respond to client
    	            String binPath = getHttpRequestParamValue(requestToParse, "httpUpdate");
					unsigned int firstDotsIndex = binPath.indexOf(":");
					unsigned int firstSlashIndex = binPath.indexOf("/");
					String httpUpdateIp   = binPath.substring(0, firstDotsIndex);
					String httpUpdatePort = binPath.substring(firstDotsIndex + 1, firstSlashIndex);
					String httpUpdatePath = binPath.substring(firstSlashIndex);
					response += "httpUpdate toggled, path : " + httpUpdateIp + ":" + httpUpdatePort + httpUpdatePath + "<br>";
    	            Serial.println("httpUpdate toggled, path : " + httpUpdateIp + ":" + httpUpdatePort + httpUpdatePath);
					// Fetch the new bin to flash
    	            t_httpUpdate_return ret = ESPhttpUpdate.update(httpUpdateIp, httpUpdatePort.toInt(), httpUpdatePath);
    	            delay(1000);
    	            switch(ret) {
    	                case HTTP_UPDATE_FAILED:
    	                    response += "[update] Update failed<br>";
							response += "Error" + String(ESPhttpUpdate.getLastError())  + ESPhttpUpdate.getLastErrorString().c_str() + "<br>";
    	                    Serial.println("[update] Update failed.");
    	                    Serial.println("Error" + String(ESPhttpUpdate.getLastError())  + ESPhttpUpdate.getLastErrorString().c_str());
    	                break;
    	                case HTTP_UPDATE_NO_UPDATES:
    	                    response += "    [update] Update no Update<br>";
    	                    Serial.println("[update] Update no Update.");
    	                break;
    	            }
    	        }
    	        // if unknown request
    	        if (faultyRequest == true) {
    	            response += "Request Error : variable to set unknown";
    	            Serial.println("Request Error : variable to set unknown");
    	        }
    	    }
    	    else if(method == "GET") {
				Serial.println("GET request type");
    	        if(checkHttpRequestParam(requestToParse, "ping")) {
    	        	response += "Toggling temperature data transfer to server<br>";
				}
    	        if(checkHttpRequestParam(requestToParse, "whoAreYou")) {
					String domoticzDeviceName = getDomoticzDeviceName(TEMPSENSOR_IDX);
					response += "<br>I am a Domoticz temperature sensor<br>";
					response += "Server IP : " + String(DOMOTICZ_IP_ADDRESS_STR) + "<br>";
					response += "Server port : " + String(DOMOTICZ_PORT) + "<br>";
					response += "Sensor Idx (Domoticz) : " + String(TEMPSENSOR_IDX) + "<br>";
					response += "Sensor Name (Domoticz) : " + domoticzDeviceName + "<br>";
					response += "Sensor timeout : " + String(SENSOR_TIMEOUT) + "s<br>";
					response += "Temperature:" + String(TEMPERATURE) + "C<br>";
					response += "Wifi network ssid : " + WiFi.SSID() + "<br>";
					response += "RSSI : " + String(getSsidQuality()) + "%<br>";
					response += "MAC ADDRESS : " + WiFi.macAddress();
					response += "<br>";
					response += "<br>POST settable variables :";
					response += "<br>    tempSensorIdx	: set matching domoticz device's idx";
					response += "<br>    serverIp		: set domoticz server ip";
					response += "<br>    serverPort	: set domoticz server port";
					response += "<br>    sensorTimeout	: time between two sensor sendings";
					response += "<br>    httpUpdate	: url of binary to flash [IP_ADDRESS:PORT/PATH/TO/FILE.BIN]";
					response += "<br>					";
					response += "<br>GET requests :";
					response += "<br>    /ping			: raises the sensor, send value to domoticz";
					response += "<br>    /whoAreYou	: display this menu";
    	        }
			}
				                 
    	    // Send variables and respond to client in both cases POST and GET
    	    getSensorValues();
			client.print("HTTP/1.1 200 OK\r\nContent-Type: text/html\r\n\r\n<!DOCTYPE HTML>\r\n<html>\r\n");
			client.print(response);
    	    client.print("\n</html>\n");
    	    client.println();
    	    client.println();
    	    client.stop();
    	    Serial.println("Client disonnected");
    	    delay(10);
    	    pushTemperature();
    	    lastSensorSendTime = millis();
    	}
		// if method is unknown
		else {
			client.print("HTTP/1.1 400 " + response + "\r\n");
	        client.stop();
		}
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

