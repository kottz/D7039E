import json
import time
from flask import Flask, jsonify, request
from arrowhead_core_systems import Arrowhead_system
from requests_pkcs12 import get, post

with open("config_template.json") as json_file:
    config = json.load(json_file)[0]
    consumer_json = config["consumer_json"]
    consumer_name = config["consumer_json"]["systemName"]
    provider_json = config["provider_json"]
    provider_name = config["provider_json"]["systemName"]

test_provider = Arrowhead_system(
    "/home/albin/Documents/core-java-spring/certificates/testcloud2/sysop.p12", "123456")

test_provider.register_system(provider_json)

# creating a Flask app
app = Flask(__name__)


@app.route('/', methods=['GET'])
def home():
    ip_address = request.environ["REMOTE_ADDR"] + ":" + request.environ["SERVER_PORT"] 
    print(ip_address)
    return ip_address


@app.route('/pick-up', methods=['POST'])
def ready_for_pick_up():
    ready = {
        'ready': request.json['ready']
    }
    return jsonify({'ready': ready}), 201


@app.route('/place', methods=['GET'])
def ready_for_place():
        clock = time.localtime()
        data = str(clock.tm_hour) + ":" + \
                   str(clock.tm_min) + ":" + str(clock.tm_sec)
        return jsonify({'data': data})

@app.route('/direction', methods=['POST'])
def setDirection():
    direction = {
        'direction': request.json['direction']}
    direction_for_robot = request.json['direction']
    print(direction_for_robot)
    return jsonify({'direction': direction}), 201


 
if __name__ == "__main__":
    app.run(host="0.0.0.0", port="2342", debug=False)
