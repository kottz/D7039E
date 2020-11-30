import json
from flask import Flask, jsonify, request
from requests_pkcs12 import get, post

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
