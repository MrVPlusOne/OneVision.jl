#!/usr/bin/env python3
import socket
import json


# a dummy server that accepts any command and sents back dummy state

HOST = '127.0.0.1'  # Standard loopback interface address (localhost)
PORT = 5001       # Port to listen on (non-privileged ports are > 1023)

with socket.socket(socket.AF_INET, socket.SOCK_STREAM) as s:
    s.bind((HOST, PORT))
    s.listen()
    conn, addr = s.accept()
    with conn:
        print('Connected by', addr)
        while True:
            data = conn.recv(1024)
            if data:
                print(data)
                states = {"xs": [[0.0,0.0],[0.0,0.0]], "zs": [[0.0,0.0],[0.0,0.0]]}
                x = json.dumps(states) + "\n"
                print("Sending {}".format(x))
                conn.sendall(x.encode())
print("connection closed")