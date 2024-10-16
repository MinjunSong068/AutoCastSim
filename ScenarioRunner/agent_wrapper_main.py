from __future__ import print_function

import argparse
from argparse import RawTextHelpFormatter
import carla
import sys
import importlib
import os
import time 
from srunner.tools.route_parser_distributed import RouteParser
from srunner.autoagents.agent_wrapper_distributed import AgentWrapper
from srunner.tools.route_manipulation_distributed import interpolate_trajectory
import zmq 
from comm import ClientComm

class AgentWrapperMain(object): 

    def __init__(self, args):
        
        self._args = args
        self.client = carla.Client(args.host, int(args.port))
        self.client_timeout = 8000
        self.client.set_timeout(self.client_timeout)
        self.world = self.client.get_world()
        self.map = self.world.get_map()
        self.route_config = self.read_config()[0]
        self.route = self._get_route(self.route_config)
        self.ego_vehicle = None
        self.agent_instance = None 
        self.ego_vehicle = None 
        self.agent = None

        self.id = None
        wrapper_port = "tcp://" + args.host + ":" + args.AgentWrapperPort
        self.clientcomm_wrapper = ClientComm(port=wrapper_port)
        status_port = "tcp://" + args.host + ":" + args.AgentControlPort
        self.clientcomm_control = ClientComm(port=status_port)

    def read_config(self): 
        return RouteParser.parse_routes_file(self._args.route, self._args.route_id)
    
    def _get_route(self, config):        
        gps_route, route = interpolate_trajectory(self.map, self.world, config.keypoints)
        if config.agent is not None:
            config.agent.set_global_plan(gps_route, route)
        return route

    def create_agent_instance(self): 
        agent_type = self.route_config.agent_type
        if agent_type is not None:
            module_name = os.path.basename(agent_type ).split('.')[0]
            sys.path.insert(0, os.path.dirname(agent_type ))
            self.module_agent = importlib.import_module(module_name)

        if agent_type :
            agent_class_name = self.module_agent.__name__.title().replace('_', '')
            temp_agent_instance = getattr(self.module_agent, agent_class_name)(self._args.agentConfig)
            self.route_config.agent = temp_agent_instance
            self.agent_instance = temp_agent_instance

    def spawn_ego_vehicle(self): 
        blueprint_library = self.world.get_blueprint_library()
        elevate_transform = self.route[0][0]
        elevate_transform.location.z += 0.5
        vehicle = self.route_config.ego_vehicles[0]
        vehicle_bp = blueprint_library.filter(vehicle.model)[0]
        role_name = self.route_config.ego_vehicles[0].rolename
        vehicle_bp.set_attribute("role_name", role_name)
        
        self.ego_vehicle = self.world.try_spawn_actor(vehicle_bp, elevate_transform)
        if self.ego_vehicle:
            print(f"Vehicle spawned successfully at {elevate_transform.location}")
            self.id = str(self.ego_vehicle.id)

        else:
            print("Failed to spawn vehicle")

    def load_agent_wrapper(self): 
        self.agent  = AgentWrapper(self.agent_instance, self.world) if self.agent_instance else None
        if self.agent is not None: 
            self._sync_mode = True 
            self.agent.setup_sensors(self.ego_vehicle, debug_mode=self._args.debug)
            self.clientcomm_wrapper.recv_send_message(self.id)
        print("Sensors setup completed")
    
    def run_on_tick(self): 
        ego_action = self.agent()
        print(f"Applying control for agent {self.ego_vehicle}")
        self.ego_vehicle.apply_control(ego_action)
        print(f"Finished control for agent {self.ego_vehicle}")

    def run(self): 
        self.create_agent_instance()

        self.spawn_ego_vehicle()
        
        print("Loading Agent Wrapper")
        self.load_agent_wrapper()

        print("Starting agent while loop")
        last_frame_id = self.agent.sensor_interface._new_data_buffers.get()[1]
        while True:
            self.run_on_tick()
            current_frame_id = self.world.get_snapshot().frame
            print(f"Current frame: {current_frame_id}")
            outgoing_message = self.id + "_" + str(current_frame_id)
            self.clientcomm_control.recv_send_message(outgoing_message) 
            print("Message sent")
            while current_frame_id == last_frame_id: 
                time.sleep(0.001)
                current_frame_id = self.agent.sensor_interface._new_data_buffers.get()[1]
            last_frame_id = current_frame_id
            
def main(): 
    description = ("Setting up client separately from scenario_runner_distributed.py")

    # pylint: disable=line-too-long
    parser = argparse.ArgumentParser(description=description,
                                    formatter_class=RawTextHelpFormatter)
    parser.add_argument('--host',
                        help='IP of the host server')
    parser.add_argument('--port', default='2000',
                        help='TCP port to listen to (default: 2000)')
    parser.add_argument('--timeout', default="10.0",
                        help='Set the CARLA client timeout value in seconds')
    parser.add_argument('--AgentWrapperPort', default='5555',
                        help='Port to update to Agent Wrapper Status (default: 5555)')
    parser.add_argument('--AgentControlPort', default='5557',
                        help='Port to update to Agent Control Status (default: 5557)')
    parser.add_argument('--route', help='Run a route as a scenario', type=str, default="srunner/examples/DistributedMultiEgoVehicle.xml")
    parser.add_argument('--route_id', help='Run a specific route inside that \'route\' file', default='0', type=str)
    parser.add_argument('--agentConfig', type=str, help="Path to Agent's configuration file", default="")
    parser.add_argument('--debug', action="store_true", help='Run with debug output', default=False)
    arguments = parser.parse_args()

    agent_wrapper_main = AgentWrapperMain(arguments)
    agent_wrapper_main.run()

if __name__ == "__main__":
    sys.exit(main())
