#!/usr/bin/env python
# coding: utf-8

import os
import sys
import yaml
import paho.mqtt.client as mqtt


if __name__ == '__main__':

    config_file_path = os.path.normpath(os.path.join(os.path.dirname(os.path.abspath(__file__)), '../../mqtt_config.yml'))
    f = open(config_file_path, "r+")
    mqtt_config = yaml.load(f)
    print("[MQTT BROKER] ADDRESS: " + mqtt_config['mqtt']['ADDRESS'] + ", PORT: " + str(mqtt_config['mqtt']['PORT']))

    client = mqtt.Client(protocol=mqtt.MQTTv311)
    client.connect(mqtt_config['mqtt']['ADDRESS'], port=mqtt_config['mqtt']['PORT'], keepalive=60)

    print("0: red, 1: green, 5: exit")

    while True:
        signal_color = raw_input('>> ')
        if int(signal_color) == 5:
            client.disconnect()
            sys.exit()
        elif int(signal_color) in [0, 1]:
            client.publish("vehicle/" + str(mqtt_config['mqtt']['VEHICLEID']) + '/signal', signal_color)
        else:
            print("Invalid Input.")
