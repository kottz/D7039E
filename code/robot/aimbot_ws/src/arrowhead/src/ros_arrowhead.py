#!/usr/bin/env python3
import rospy
import threading
from arm_control.srv import *
import json
import time
from flask import Flask, jsonify, request
from arrowhead_core_systems import Arrowhead_system
from requests_pkcs12 import get, post

# Fix path to file (config_template_json)
with open("config_template.json") as json_file:
    config = json.load(json_file)[0]
    consumer_json = config["consumer_json"]
    consumer_name = config["consumer_json"]["systemName"]
    provider_json = config["provider_json"]
    provider_name = config["provider_json"]["systemName"]

# Fix path to file (sysop.p12)
test_provider = Arrowhead_system(
    "/home/albin/Documents/core-java-spring/certificates/testcloud2/sysop.p12", "123456")

test_provider.register_system(provider_json)


class Arrowhead_response:
    def __init__(self):
        self.direction = ""
        self.ready = False


ah_resp = Arrowhead_response()
# creating a Flask app
app = Flask(__name__)


@app.route('/pick-up', methods=['POST'])
def ready_for_pick_up():
    ready = {
        'ready': request.json['ready']
    }
    return jsonify({'ready': ready}), 201


@ app.route('/direction', methods=['POST'])
def setDirection(ros_request):
    direction_for_robot = request.json['direction']
    print(direction_for_robot)
    ah_resp.direction = direction_for_robot
    return jsonify({'direction': direction_for_robot}), 201


def arrowhead_direction(request):
    qr_data = request
    print(qr_data)
    return ah_resp.direction

def arrowhead_factory_ready(request):
    qr_data = request
    print(qr_data)
    return ah_resp.ready

# def setDirection(request):
#     qr_data = request
#     print(qr_data)
#     direction_for_robot = request.json['direction']
#    # print(direction_for_robot)
#     return direction_for_robot


def get_keyboard_input():
    while(True):
        inp = input("type something ")
        if inp == "follow":
            pass
        elif inp == "forward":
            pass
        elif inp == "left":
            pass
        elif inp == "right":
            pass
        elif inp == "back":
            pass
        elif inp == "pickup":
            pass
        print("key: " + inp)
        if(inp == "quit"):
            return
        self.n = int(inp)


def arrowhead_spoof(request):
    qr_data = request
    print(qr_data)
    inp = input("arrowhead input requested. What to do? ")
    # sry for ugly
    print("key: " + inp)
    return inp


if __name__ == '__main__':
    app.run(host="0.0.0.0", port="2342", debug=False)
    rospy.init_node('arrowhead', anonymous=True)

    # x = threading.Thread(target=get_keyboard_input)
    # x.start()
    # x.join()
    # rospy.Service('ah_req', ah_request, arrowhead_spoof)
    rospy.Service('ah_req', ah_request, arrowhead_factory_ready)
    rospy.spin()
