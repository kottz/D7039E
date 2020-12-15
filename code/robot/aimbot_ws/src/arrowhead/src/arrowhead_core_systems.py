import json
from requests_pkcs12 import get, post

with open("config_template.json") as json_file:
    config = json.load(json_file)[0]
    pkcs12_filename = config["certificate_info"]["pkcs12_filename"]
    pkcs12_password = config["certificate_info"]["pkcs12_password"]
    service_regisrty_url = config["urls"]["service_regisrty_url"]
    authorization_url = config["urls"]["authorization_url"]
    orchestration_url = config["urls"]["orchestration_url"]
    consumer_json = config["consumer_json"]
    consumer_name = config["consumer_json"]["systemName"]
    provider_json = config["provider_json"]
    provider_name = config["provider_json"]["systemName"]
    pick_up_service_json = config["pick_up_service_json"]
    pick_up_service_definition = config["pick_up_service_json"]["serviceDefinition"]
    place_service_json: config["place_service_json"]
    place_service_definition = config["place_service_json"]["serviceDefinition"]

def get_services():
    url = service_regisrty_url + "/mgmt?direction=ASC"
    return get(url,
                verify=False, pkcs12_filename=pkcs12_filename, pkcs12_password=pkcs12_password
                )

def get_systems():
    url = service_regisrty_url + "/mgmt/systems?direction=ASC&sort_field=id"
    return get(url,
                verify=False, pkcs12_filename=pkcs12_filename, pkcs12_password=pkcs12_password
                )

def find_system_ids(consumer_name, provider_name):
    response = get_systems()
    for system in response.json()["data"]:
        if system["systemName"] == consumer_name:
            consumer_id = system["id"]
        elif system["systemName"] == provider_name:
            provider_id = system["id"]
    return [consumer_id, provider_id]

def find_service_id(service_definition):
    
    response = get_services()
    for service in response.json()["data"]:
        if service["serviceDefinition"]["serviceDefinition"] == service_definition:
            interface_id = service["interfaces"][0]["id"]
            service_definition_id = service["serviceDefinition"]["id"]
    return [service_definition_id, interface_id]

def create_authorization_json(service_definition):
    idarray = find_system_ids(consumer_name, provider_name) + \
        find_service_id(service_definition)
    authorization_json = {
        "consumerId": idarray[0],
        "interfaceIds": [
            idarray[3]
        ],
        "providerIds": [
            idarray[1]
        ],
        "serviceDefinitionIds": [
            idarray[2]
        ]
    }
    return authorization_json

def create_orchestration_json(service_definition):
    idarray = find_system_ids(consumer_name, provider_name)
    orchestration_json = [
        {
            "serviceDefinitionName": service_definition,
            "consumerSystemId": idarray[0],
            "providerSystem": {
                "systemName": "test_provider",
                "address": "192.168.43.204",
                "port": 2342,
                "authenticationInfo": ""
            },
            "cloud": {
                "operator": "aitia",
                "name": "testcloud2"
            },
            "serviceInterfaceName": "HTTPS-SECURE-JSON",
            "priority": 1
        }
    ]
    return orchestration_json


class Arrowhead_system:

    def __init__(self, pkcs12_filename, pkcs12_password):
        self.pkcs12_filename = pkcs12_filename
        self.pkcs12_password = pkcs12_password

    def register_system(self, json):
        url = service_regisrty_url + "/mgmt/systems"
        return post(url,
                    verify=False, pkcs12_filename=pkcs12_filename, pkcs12_password=pkcs12_password, json=json
                    )

    def register_service(self, json):
        url = service_regisrty_url + "/mgmt"
        return post(url,
                    verify=False, pkcs12_filename=pkcs12_filename, pkcs12_password=pkcs12_password, json=json
                    )

    def add_intracloud_authorization(self, service_definition):
        url = authorization_url + "/mgmt/intracloud"
        authorization_json = create_authorization_json(service_definition)
        return post(url,
                    verify=False, pkcs12_filename=pkcs12_filename, pkcs12_password=pkcs12_password, json=authorization_json
                    )

    def create_orchestration_store_entry(self, service_definition):
        url = orchestration_url + "/mgmt/store"
        orchestration_json = create_orchestration_json(service_definition)
        return post(url,
                    verify=False, pkcs12_filename=pkcs12_filename, pkcs12_password=pkcs12_password, json=orchestration_json
                    )

    def start_orchestration_based_on_id(self, consumer_name, provider_name):
        consumer_id = find_system_ids(consumer_name, provider_name)[0]
        url = orchestration_url + "/orchestration/" + str(consumer_id)
        return get(url,
                verify=False, pkcs12_filename=pkcs12_filename, pkcs12_password=pkcs12_password
                )
