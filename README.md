# domoticz_sensors

## Description
There are the firmwares of Domoticz sensors based on ESP8266 programmed with arduino. The sensors are able to communicate with domoticz and to receive request in order to be managed.

### Thermostat
The thermostat is an "all in one" device: the temperature probe and the relay that drives the heater are connected to the same mcu wifi board (ESP8266, NodeMCU or Wemos D1 mini in my case). In that way, the thermostat can run even with the server down. 
### Simple temperature sensor
Simple temperature probe (DS18B20) sending its data to domoticz. Accepts requests to be managed.

### Simple sleepy temperature sensor
Simple temperature probe (DS18B20) sending its data to domoticz, but sleeps inbetween two sents. Accepts requests to be managed, but the requests are to be sent at the right time, depending of the sensor_timeout.
