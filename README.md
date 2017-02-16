# domoticz_sensors

## Description
This repo contains firmwares of Domoticz devices based on ESP8266 programmed with arduino. The devices are able to communicate with domoticz and to receive request in order to be managed.

### Thermostat
The thermostat is an "all in one" device: the temperature probe and the relay that drives the heater are connected to the same mcu wifi board (ESP8266, NodeMCU or Wemos D1 mini in my case). In that way, the thermostat can run even with the server down. Moreover, you can set the temperature setpoint via Domoticz and manually via the hardware interface. This is mandatory in case of server carsh.
### Simple temperature sensor
Simple temperature probe (DS18B20) sending its data to domoticz. Accepts requests to be managed.

### Simple sleepy temperature sensor
Simple temperature probe (DS18B20) sending its data to domoticz, but sleeps inbetween two sents. Accepts requests to be managed, but the requests are to be sent at the right time, depending of the sensor_timeout.


## Setup & Management
Requirements:
* a working Wifi network
* a server with domoticz running (you will need the server IP address and the Domoticz listening port, which is 8080 by default)
* a Wifi device, such as laptop or smartphone, to configure your domoticz's device Wifi credentials

### Step 1 : Create your Domoticz virtual sensor(s)
The very first thing to do is to create a dummy hardware in the Setup --> Hardware menu. Name it as you want, for example 'thermostatHardware' for the thermostat or 'temperatureSensorHardware' for the simple temperature sensor. These names don't matter. Here is an example (you don't have to create both hardwares if you don't use both, of course):
![example0]
(pics/example0.tiff)
From this panel you can now create virtual sensors corresponding to the hardware by clicking on the blue button.
#### Temperature sensor
It needs only one virtual device to work. Click on __Create Virtual Sensor__ and choose a temperature device. Now you can see it in the panel Setup --> Devices. Note its associated idx, which is the identity of the Domoticz virtual sensor. It will be needed afterward.
![example1]
(pics/example1.tiff)
On this picture you can see the virtual temperature sensor associated with the tempSensor hardware created before. Its idx is 4, remember it.