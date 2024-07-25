import zmq
import pickle
import numpy as np


def process_client_inputs(model, verbose: int=0):
    context = zmq.Context()
    socket = context.socket(zmq.REP)
    socket.bind("tcp://*:69420")

    while True:
        client_input = socket.recv()
        client_input = pickle.loads(client_input)
        if verbose:
            print(f"Received request: {client_input}")

        output = client_input * 2
        # output = model.predict(client_input)
        to_send = pickle.dumps(output)
        socket.send(to_send)
        if verbose:
            print("Sent a response.")
