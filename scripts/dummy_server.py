#!/usr/bin/env python3
import socket
import json


# a dummy server that accepts any command and sents back dummy state

HOST = '127.0.0.1'  # Standard loopback interface address (localhost)
PORT = 5001       # Port to listen on (non-privileged ports are > 1023)
CASE = "2DDis" # case for dummy state

with socket.socket(socket.AF_INET, socket.SOCK_STREAM) as s:
    s.bind((HOST, PORT))
    s.listen()
    conn, addr = s.accept()
    with conn:
        print('Connected by', addr)
        while True:
            data = conn.recv(8096)
            if data:
                print(data)
                j = json.loads(data)
                if CASE == "1D":
                    states = {"xs": [[0.0,0.0],[0.0,0.0]], "zs": [[0.0,0.0],[0.0,0.0]]}
                elif CASE == "2D":
                    states = {"xs": [[0.0,0.0, 0.0, 0.0, 0.0],[0.0,0.0, 0.0, 0.0, 0.0],[0.0,0.0, 0.0, 0.0, 0.0],[0.0,0.0, 0.0, 0.0, 0.0]], "zs": [{"c":[0.0,0.0],"d":1},{"c":[0.0,0.0],"d":1},{"c":[0.0,0.0],"d":1},{"c":[0.0,0.0],"d":1}]}
                elif CASE == "1DDis":
                    states = {"x": [0.0,0.0], "z": [0.0,0.0], "msgs": j["msgs"]}#"msgs": [{"\xce\xb4x":{"time":-3,"value":[0.0,0.0]},"z":{"time":-2,"value":[0.0,0.0]}},{"\xce\xb4x":{"time":-3,"value":[0.0,0.0]},"z":{"time":-2,"value":[0.0,0.0]}}]}
                elif CASE == "2DDis":
                    states = {"x": [0.0,0.0, 0.0, 0.0, 0.0], "z": {"c":[0.0,0.0],"d":1}, "msgs": j["msgs"]}#"msgs": [{"\xce\xb4x":{"time":-3,"value":[0.0,0.0]},"z":{"time":-2,"value":[0.0,0.0]}},{"\xce\xb4x":{"time":-3,"value":[0.0,0.0]},"z":{"time":-2,"value":[0.0,0.0]}}]}

                x = json.dumps(states) + "\n"
                print("Sending {}".format(x))
                conn.sendall(x.encode())
print("connection closed")