import paho.mqtt.client as mqtt
import time
import os
import logging
import subprocess
import Adafruit_GPIO.I2C as I2C

mqttc = mqtt.Client();
#connects to the hub
mqttc.connect("172.17.4.95");
message = "";
#connects to the divice 0x08 on the I2C-2 bus
i2c = I2C.get_i2c_device(0x08, 2);

while True;
	#reads a byte on the I2C bus
	byte = I2C.Device.readRaw8(i2c);
	message = str(byte);
	#sends the byte on the hub
	mqttc.publish("topic/1f04", message);
	time.sleep(1);