import threading
from time import sleep 
import zmq

class Status(object):
    
    def __init__(self):
        self.timeout = 10 
        self.completed = False 
        self.agents_status = {}
        self.lock = threading.Lock() 
        self.actor_ids = []
        self.context = zmq.Context()

    def reset_status(self):
        print("Starting to reset dictionary")
        self.lock.acquire()
        for actor_id in self.actor_ids:
            self.agents_status[actor_id] = False
            print(f"Actor ID {actor_id} is now set to {self.agents_status[actor_id]}")
        self.lock.release()
        print("Finished reseting dictionary")

    def update_dictionary(self, key, value):
        self.lock.acquire()
        self.agents_status[key] = value 
        self.lock.release()
        print(f"Added {key}: {value}")

    def check_completion(self):
        self.lock.acquire()
        if all(actor_id in self.agents_status for actor_id in self.actor_ids):
            print("All keys exist")
            if all(self.agents_status.values()):
                print("All values are true")
                self.completed = True 
        else: 
            self.completed = False
        self.lock.release() 