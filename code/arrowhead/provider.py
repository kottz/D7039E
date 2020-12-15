import json
from flask import Flask, jsonify, request
from requests_pkcs12 import get, post
from arrowhead_core_systems import Arrowhead_system

# creating a Flask app
app = Flask(__name__)

with open("config_template.json") as json_file:
    config = json.load(json_file)[0]
    provider_json = config["provider_json"]
    provider_name = config["provider_json"]["systemName"]

test_provider = Arrowhead_system(
    "/home/albin/Documents/core-java-spring/certificates/testcloud2/sysop.p12", "123456")

test_provider.register_system(provider_json)


@app.route('/', methods=['GET', 'POST'])
def home():
    if(request.method == 'GET'):

        data = "hello world"
        return jsonify({'data': data})


@app.route('/home/<int:num>', methods=['GET'])
def disp(num):

    return jsonify({'data': num**2})


@app.route('/pick-up', methods=['POST'])
def ready_for_pick_up():
    info = {
        'ready': request.json['ready']
    }

    return jsonify({'info': info}), 201

@app.route('/place', methods=['GET'])
def ready_for_place():
        data = "hello world"
        return jsonify({'data': data})
 
if __name__ == "__main__":
    app.run(host="0.0.0.0", port="2342")
