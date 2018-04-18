import json
import paho.mqtt.client as mqtt
import datetime

def on_message(mqttc, obj, msg):
    message = msg.payload.decode()
    date = str(datetime.datetime.now())
    print("%s %s" % (message, date))
    data ={}
    data['probe'] = {'temp': message, 'datum': date}
    with open('data.json', 'a') as outfile:
        json.dump(data, outfile)

mqttc = mqtt.Client()
mqttc.on_message = on_message
mqttc.connect("172.17.4.95", 1883, 60)
mqttc.subscribe("topic/1f04", 0)
mqttc.loop_forever()

