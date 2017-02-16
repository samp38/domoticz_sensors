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