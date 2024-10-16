import time
import zmq

class ServerComm(object):
    def __init__(self, port="tcp://localhost:5555"):
        self.context = zmq.Context()
        self.socket = self.context.socket(zmq.REP)
        self.socket.bind(port)

    def recv_send_message(self):
        message = self.socket.recv()
        decoded_message = message.decode('utf-8')
        print("Client message recieved")
        time.sleep(1)

        self.socket.send(b"Recieved")
        print("Message sent to client")
        return decoded_message 

class ClientComm(object):
    def __init__(self, port="tcp://localhost:5555"): 

        self.context = zmq.Context()
        self.socket = self.context.socket(zmq.REQ)
        self.socket.connect(port)

    def recv_send_message(self, outgoing_message):
        print("Sending client message")
        self.socket.send(outgoing_message.encode("utf-8"))
        print("Client message sent to server")
        incoming_message = self.socket.recv()
        print("Server message recieved")

