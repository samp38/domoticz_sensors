# domoticz_sensors

## Description
This repo contains firmwares of [Domoticz](https://domoticz.com) devices based on ESP8266 programmed with arduino. The devices are able to communicate with domoticz and to receive request in order to be managed.

### Thermostat
The thermostat is an "all in one" device: the temperature probe and the relay that drives the heater are connected to the same mcu wifi board (ESP8266, NodeMCU or Wemos D1 mini in my case). In that way, the thermostat can run even with the server down. Moreover, you can set the temperature setpoint via Domoticz and manually via the hardware interface. This is mandatory in case of server carsh.
### Simple temperature sensor
Simple temperature probe (DS18B20) sending its data to domoticz. Accepts requests to be managed.

### Simple sleepy temperature sensor
Simple temperature probe (DS18B20) sends its data to domoticz, but sleeps inbetween. It can accept requests for management, but the requests are to be sent at the right time, depending on the sensor_timeout.


## Setup & Management
Requirements:
* a working Wifi network
* a server with domoticz running (you will need the server IP address and the Domoticz listening port, which is 8080 by default)
* a Wifi device, such as laptop or smartphone, to configure your domoticz's device Wifi credentials

### Step 1 : create your Domoticz virtual sensor(s)
The very first thing to do is to create a dummy hardware in the Setup --> Hardware menu. Name it as you want, for example 'thermostatHardware' for the thermostat or 'temperatureSensorHardware' for the simple temperature sensor. These names don't matter. Here is an example (you don't have to create both hardwares if you don't use both, of course):
![example0]
(pics/example0.tiff)
From this panel you can now create virtual sensors corresponding to the hardware by clicking on the blue button.
#### Temperature sensor
It needs only one virtual device to work. Click on __Create Virtual Sensor__ and choose a temperature device. Now you can see it in the panel Setup --> Devices. Note its associated idx, which is the identity of the Domoticz virtual sensor. It will be needed afterward.
![example1]
(pics/example1.tiff)
On this picture you can see the virtual temperature sensor associated with the tempSensor hardware created before. Its idx is 4, remember it.

#### Thermostat
The thermostat is designed to work with three virtual sensors :
* a thermostat setpoint to set the desired temperature
* a temperature temperature sensor to display the probe value of the thermostat
* a switch to display the state of the heater driven by the thermostat (note that this switch is not to be manually controlled. It is used to show the heater state only)
Here is an example of the created devices. Again, the name of your virtual sensors are up to you.
![example2]
(pics/example2.tiff)
Note the idx of each sensor.

You are now ready to follow up to the hardware setup!

### Step 2 : connect the device to WiFi
This step is the same for all the devices. Just plug it to a current source and wait a few seconds. If the device finds a known wifi SSID it will try to connect to with the credentials saved in EEPROM. If not, or if there are no credentials saved, it will create a WiFi network named _autoconnectAP_. You can use a smartphone or a laptop to connect to. Then, go to the url _192.168.4.1_ and configure your WiFi connection. Easy!

### Step 3 : configure the  device by sending http requests
By design, the way to configure the devices is http requests. If you use a linux server (Raspberry Pi) or a Mac, _curl_ is an easy way, although any web browser will do the job. __The devices run a server that listen on port 8081__. To begin configuration, you need the IP address the device. Figure it out through your router admin page. To better readability, you can pipe the output to html2text.

__GET requests__ :

* read the settings of the device
```sh
curl "192.168.XXX.X:8081/whAreYou" | html2text
```
or with web browser:
_192.168.XXX.X:8081/whoAreYou_

Here is a response example for the thermostat:
```
I am a domoticz thermostat
Server IP : 192.168.0.99
Server port : 8081
Thermostat SetPoint idx (Domoticz) : 1 (Consigne Thermostat) in Domoticz
Heater idx (Domoticz) : 2 (Chaudiere) in Domoticz
TempSensor idx (Domoticz) : 3 (Sonde Thermostat) in Domoticz
Sensor timeout : 60
Temperature:11.25
Setpoint : 8.00
Heater Status : 0
Wifi network ssid : chambre_ritou_zviloff
RSSI : 36%
MAC ADDRESS : 5C:CF:7F:23:F0:86

POST settable variables :
tempSensorIdx : set matching domoticz device's idx
heaterSwitchIdx : set matching domoticz device's idx
thermostatSetpointIdx : set matching domoticz device's idx
serverIp : set domoticz server ip
serverPort : set domoticz server port
sensorTimeout : time between two sensor sendings
httpUpdate : url of binary to flash [IP_ADDRESS:PORT/PATH/TO/FILE.BIN]

GET requests :
/ping : raises the sensor, send value to domoticz
/whoAreYou : display this menu
```

* toggle the sensor and make it send its data to domoticz
```sh
curl "192.168.XXX.X:8081/ping" | html2text
```

__POST REQUESTS__ :

POST requests allow you to set the variables described in the _whoAreYou_ menu. This can be done using curl again :
```sh
curl -d "variableName=value" 192.168.XXX.X:8081
```

All you've left to do is to set matching idx, server IP and PORT, sensorTimeout, etc.. via POST requests.

## OTA (Over The Air) firmware update
You can toggle a firmware update with a POST request, passing the binary path to the _httpUpdate_ variable.
```sh
curl -d "httpUpdate=AnyServerIp:port/path/to/bin.bin" 192.168.XXX.X:8081
```
The only requirement is to properly serve the bin file. Once toggled, the chip will auto flash its firmware and reboot.