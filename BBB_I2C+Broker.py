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

picGoToSleep = False;

mqttc = mqtt.Client();
#connects to the hub
mqttc.connect("172.17.4.95");
message = "";
#connects to the divice 0x08 on the I2C-2 bus
i2c = I2C.get_i2c_device(deviceName, 2);

#experimental because never tested
#allows the user to toggle on/off the data sampling from the dash-board
'''
def on_connect(mqttc, userdata, flags, rc):
	mqttc.subscribe(topicName)

def on_message(mqttc, userdata, msg):
	if msg.payload.decode() == "sleep":
		picGoToSleep = True;
		mqttc.disconnect()
	if msg.payload.decode() == "wakeup":
		picGoToSleep = False;
    	mqttc.disconnect()

mqttc.on_connect = on_connect
mqttc.on_message = on_message
mqttc.loop_start()
'''

while picGoToSleep = False;
	#sends the 8 bits signal to the device to send the temperature back
	I2C.Device.writeRaw8(i2c, 0X01)
	#reads a byte on the I2C bus
	byte = I2C.Device.readRaw8(i2c);
	message = str(byte);
	#sends the temperature on the hub
	mqttc.publish(topicName, message);
	time.sleep(1);

time.sleep(1)