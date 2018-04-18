import paho.mqtt.client as mqtt
import time
import os
import logging
import subprocess
import Adafruit_GPIO.I2C as I2C

#sets the name of the topic
topicName = "topic/1f04";
#sets the adress of the device on the I2C bus
deviceName = "0x68";

mqttc = mqtt.Client();
#connects to the hub
mqttc.connect("172.17.4.95");
message = "";
#connects to the divice 0x08 on the I2C-2 bus
i2c = I2C.get_i2c_device(deviceName, 2);

while True;
	#sends the 8 bits signal to the device to send the temperature back
	I2C.Device.writeRaw8(i2c, 0X01)
	#reads a byte on the I2C bus
	byte = I2C.Device.readRaw8(i2c);
	message = str(byte);
	#sends the byte on the hub
	mqttc.publish(topicName, message);
	time.sleep(1);