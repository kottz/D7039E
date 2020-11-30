import json
from arrowhead_core_systems import Arrowhead_system
from requests_pkcs12 import get, post


# TODO add code from core_systems.py to take care of all arrowhead logic.

with open("config_template.json") as json_file:
    config = json.load(json_file)[0]
    consumer_json = config["consumer_json"]
    consumer_name = config["consumer_json"]["systemName"]
    provider_name = config["provider_json"]["systemName"]

test_consumer = Arrowhead_system(
    "/home/albin/Documents/core-java-spring/certificates/testcloud2/sysop.p12", "123456")
test_consumer.register_system(consumer_json)
test_consumer.start_orchestration_based_on_id(consumer_name, provider_name)


#TODO flask api
# json = {
#     'ready': True
# }

# print(post('http://localhost:5000/Ready-for-pickup',
#            verify=False, json=json
#            ).json())
