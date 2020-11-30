import json
from flask import Flask, jsonify, request
from requests_pkcs12 import get, post
from arrowhead_core_systems import Arrowhead_system



# TODO add code from core_systems.py to take care of all arrowhead logic.
with open("config_template.json") as json_file:
    config = json.load(json_file)[0]
    consumer_json = config["consumer_json"]
    consumer_name = config["consumer_json"]["systemName"]
    provider_json = config["provider_json"]
    provider_name = config["provider_json"]["systemName"]
    pick_up_service_json = config["pick_up_service_json"]
    pick_up_service_definition = config["pick_up_service_json"]["serviceDefinition"]
    place_service_json = config["place_service_json"]
    place_service_definition = config["place_service_json"]["serviceDefinition"]

test_provider = Arrowhead_system(
    "/home/albin/Documents/core-java-spring/certificates/testcloud2/sysop.p12", "123456")

test_provider.register_system(provider_json)
test_provider.register_service(pick_up_service_json)
print(test_provider.register_service(place_service_json).json())
test_provider.add_intracloud_authorization(pick_up_service_definition).json()
print(test_provider.create_orchestration_store_entry(pick_up_service_definition))
test_provider.add_intracloud_authorization(place_service_definition).json()
print(test_provider.create_orchestration_store_entry(place_service_definition))
print(test_provider.start_orchestration_based_on_id(consumer_name, provider_name).json())




# creating a Flask app
app = Flask(__name__)


@app.route('/', methods=['GET', 'POST'])
def home():
    if(request.method == 'GET'):

        data = "hello world"
        return jsonify({'data': data})


@app.route('/home/<int:num>', methods=['GET'])
def disp(num):

    return jsonify({'data': num**2})


@app.route('/Ready-for-pickup', methods=['POST'])
def service1():
    info = {
        'ready': request.json['ready']
    }

    return jsonify({'info': info}), 201


if __name__ == "__main__":
    app.run()
