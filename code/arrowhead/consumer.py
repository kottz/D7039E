import json
from arrowhead_core_systems import Arrowhead_system
from requests_pkcs12 import get, post
import time

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
    direction_service_json = config["direction_service_json"]
    direction_service_definition = config["direction_service_json"]["serviceDefinition"]


test_consumer = Arrowhead_system(
    "/home/albin/Documents/core-java-spring/certificates/testcloud2/sysop.p12", "123456")
test_consumer.register_system(consumer_json)
test_consumer.register_service(pick_up_service_json)
test_consumer.add_intracloud_authorization(pick_up_service_definition)
test_consumer.create_orchestration_store_entry(pick_up_service_definition)
test_consumer.register_service(place_service_json)
test_consumer.add_intracloud_authorization(place_service_definition)
test_consumer.create_orchestration_store_entry(place_service_definition)
test_consumer.register_service(direction_service_json)
test_consumer.add_intracloud_authorization(direction_service_definition)
test_consumer.create_orchestration_store_entry(direction_service_definition)
test_consumer.start_orchestration_based_on_id(
    consumer_name, provider_name)


provider_ip = test_consumer.start_orchestration_based_on_id(
    consumer_name, provider_name).json()["response"][0]["provider"]["address"]
proivder_port = test_consumer.start_orchestration_based_on_id(
    consumer_name, provider_name).json()["response"][0]["provider"]["port"]


service_uri_pick_up = test_consumer.start_orchestration_based_on_id(
    consumer_name, provider_name).json()["response"][0]["serviceUri"]
service_uri_place = test_consumer.start_orchestration_based_on_id(
    consumer_name, provider_name).json()["response"][1]["serviceUri"]
service_uri_direction = test_consumer.start_orchestration_based_on_id(
    consumer_name, provider_name).json()["response"][2]["serviceUri"]

url = 'http://' + provider_ip + ":" + str(proivder_port)

while(True):
    factory_ready_json = {
        'ready': False
    }
    print(post(url + service_uri_pick_up,
               verify=False, json=factory_ready_json
               ).json())
    time.sleep(10)
    factory_ready_json = {
        'ready': True
    }
    print(post(url + service_uri_pick_up,
               verify=False, json=factory_ready_json
               ).json())
    time.sleep(10)
