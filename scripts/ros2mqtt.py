#!/usr/bin/env python3
import rospy
from sensor_msgs.msg import JointState

import random
import time
import json
from paho.mqtt import client as mqtt_client

client = None
broker = 'localhost'
port = 8080
topic = "topic"
client_id = f'python-mqtt-{random.randint(0, 1000)}'


def callback(msg):
    print(msg.name)
    now = rospy.Time.now()
    rospy.loginfo("now: %f", now.to_sec())

    data = {
        "eId": "",
        "time": 0,
        "joints": {
            msg.name[0]: msg.position[0],
            msg.name[1]: msg.position[1],
            msg.name[2]: msg.position[2],
            msg.name[3]: msg.position[3]
        }
    }

    mqtt_msg = json.dumps(data)
    result = client.publish(topic, mqtt_msg)
    status = result[0]
    if status == 0:
        print(f"Send `{mqtt_msg}` to topic `{topic}`")
    else:
        print(f"Failed to send message to topic {topic}")


def connect_mqtt():
    def on_connect(client, userdata, flags, rc):
        if rc == 0:
            print("Connected to MQTT Broker!")
        else:
            print("Failed to connect, return code %d\n", rc)

    client = mqtt_client.Client(client_id, transport="websockets")
    client.on_connect = on_connect
    client.connect(broker, port)
    return client


if __name__ == '__main__':
    client = connect_mqtt()
    client.loop_start()

    rospy.init_node('listener')
    rospy.Subscriber("joint_states", JointState, callback)
    rospy.spin()