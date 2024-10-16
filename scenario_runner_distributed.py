from __future__ import print_function

import glob
import traceback
import argparse
from argparse import RawTextHelpFormatter
from datetime import datetime
from distutils.version import LooseVersion
import importlib
import inspect
import os
import signal
import sys
import time
import json
import pkg_resources
import zmq 
import carla
import numpy as np
from numpy import random
from ScenarioRunner.comm import ServerComm
from ScenarioRunner.synchronization import Status
from ScenarioRunner.srunner.scenariomanager.carla_data_provider import CarlaDataProvider
from ScenarioRunner.srunner.scenariomanager.scenario_manager_distributed import ScenarioManager
from ScenarioRunner.srunner.scenarios.route_scenario_distributed import RouteScenario
from ScenarioRunner.srunner.tools.scenario_parser import ScenarioConfigurationParser
from ScenarioRunner.srunner.tools.route_parser_distributed import RouteParser


from AVR import Utils
from srunner.scenarioconfigs.openscenario_configuration import OpenScenarioConfiguration
from srunner.scenarioconfigs.route_scenario_configuration import RouteScenarioConfiguration
from srunner.scenariomanager.carla_data_provider import CarlaDataProvider #CarlaActorPool
from srunner.scenariomanager.scenario_manager import ScenarioManager
from AVR.DataLogger import DataLogger

from AVR import HUD

# Version of scenario_runner
VERSION = '0.9.13'

class ScenarioRunner(object):

    """
    This is the core scenario runner module. It is responsible for
    running (and repeating) a single scenario or a list of scenarios.

    Usage:
    scenario_runner = ScenarioRunner(args)
    scenario_runner.run()
    del scenario_runner
    """

    ego_vehicles = []

    # Tunable parameters
    client_timeout = 10.0  # in seconds
    wait_for_world = 20.0  # in seconds
    frame_rate = 20.0      # in Hz

    # CARLA world and scenario handlers
    world = None
    manager = None

    finished = False

    additional_scenario_module = None

    agent_instance = None
    module_agent = None

    def __init__(self, args):
        """
        Setup CARLA client and world
        Setup ScenarioManager
        """
        self._args = args
        self._agent_instances = []

        # AVR
        arguments = self._args
        Utils.mqtt_port = arguments.mqttport
        Utils.parse_config_flags(arguments)
        if arguments.eval:
            Utils.EvalEnv.parse_eval_args(arguments.eval)

        Utils.BACKGROUND_TRAFFIC = arguments.bgtraffic
        
        record_args = deepcopy(arguments)
        Utils.environment_config = record_args

        print("&&&&&&&&&&&&&&&&&&&&Initialization")
        print("Util.mqtt_port:",Utils.mqtt_port)

        if args.timeout:
            self.client_timeout = float(args.timeout)

        # First of all, we need to create the client that will send the requests
        # to the simulator. Here we'll assume the simulator is accepting
        # requests in the localhost at port 2000.
        self.client = carla.Client(args.host, int(args.port))
        self.client.set_timeout(10)
        
        dist = pkg_resources.get_distribution("carla")
        if LooseVersion(dist.version) < LooseVersion('0.9.15'):
            raise ImportError("CARLA version 0.9.15 or newer required. CARLA version found: {}".format(dist))

        print("Traffic manager port {}".format(self._args.trafficManagerPort))
        self.traffic_manager = self.client.get_trafficmanager(int(self._args.trafficManagerPort))

        # Load additional scenario definitions, if there are any
        # If something goes wrong an exception will be thrown by importlib (ok here)
        if self._args.additionalScenario != '':
            module_name = os.path.basename(args.additionalScenario).split('.')[0]
            sys.path.insert(0, os.path.dirname(args.additionalScenario))
            self.additional_scenario_module = importlib.import_module(module_name)

        # Load agent if requested via command line args
        # If something goes wrong an exception will be thrown by importlib (ok here)
        if self._args.agent is not None:
            module_name = os.path.basename(args.agent).split('.')[0]
            sys.path.insert(0, os.path.dirname(args.agent))
            self.module_agent = importlib.import_module(module_name)


        # Create the ScenarioManager
        self.manager = ScenarioManager(self._args.debug, self._args.sync, 10)

        # Create signal handler for SIGINT
        self._shutdown_requested = False
        if sys.platform != 'win32':
            signal.signal(signal.SIGHUP, self._signal_handler)
        if hasattr(signal, 'SIGHUP'):
            signal.signal(signal.SIGHUP, self._signal_handler)
        signal.signal(signal.SIGINT, self._signal_handler)
        signal.signal(signal.SIGTERM, self._signal_handler)

        self._start_wall_time = datetime.now()
         
        self.agents_wrapper_status = Status() 
        wrapper_port = "tcp://" + args.host + ":" + args.AgentWrapperPort
        self.servercomm_wrapper = ServerComm(port=wrapper_port)

        self.agents_control_status = Status()
        status_port = "tcp://" + args.host + ":" + args.AgentControlPort
        self.servercomm_control = ServerComm(port=status_port)

    def destroy(self):
        """
        Cleanup and delete actors, ScenarioManager and CARLA world
        """

        self._cleanup()
        if self.manager is not None:
            del self.manager
        if self.world is not None:
            del self.world
        if self.client is not None:
            del self.client

    def _signal_handler(self, signum, frame):
        """
        Terminate scenario ticking when receiving a signal interrupt
        """
        self._shutdown_requested = True
        if self.manager:
            self.manager.stop_scenario()
            self._cleanup()
            if not self.manager.get_running_status():
                raise RuntimeError("Timeout occurred during scenario execution")

    def _get_scenario_class_or_fail(self, scenario):
        """
        Get scenario class by scenario name
        If scenario is not supported or not found, exit script
        """

        # Path of all scenario at "srunner/scenarios" folder + the path of the additional scenario argument
        scenarios_list = glob.glob("{}/srunner/scenarios/*.py".format(os.getenv('SCENARIO_RUNNER_ROOT', "./")))
        scenarios_list.append(self._args.additionalScenario)

        for scenario_file in scenarios_list:

            # Get their module
            module_name = os.path.basename(scenario_file).split('.')[0]
            sys.path.insert(0, os.path.dirname(scenario_file))
            scenario_module = importlib.import_module(module_name)

            # And their members of type class
            for member in inspect.getmembers(scenario_module, inspect.isclass):
                if scenario in member:
                    return member[1]

            # Remove unused Python paths
            sys.path.pop(0)

        print("Scenario '{}' not supported ... Exiting".format(scenario))
        sys.exit(-1)

    def _cleanup(self):
        """
        Remove and destroy all actors
        """
        if self.finished:
            return

        self.finished = True

        # Simulation still running and in synchronous mode?
        if self.world is not None and self._args.sync:
            try:
                # Reset to asynchronous mode
                settings = self.world.get_settings()
                settings.synchronous_mode = False
                settings.fixed_delta_seconds = None
                self.world.apply_settings(settings)
                self.client.get_trafficmanager(int(self._args.trafficManagerPort)).set_synchronous_mode(False)
            except RuntimeError:
                sys.exit(-1)

        self.manager.cleanup()

        CarlaDataProvider.cleanup()

        for i, _ in enumerate(self.ego_vehicles):
            if self.ego_vehicles[i]:
                if not self._args.waitForEgo and self.ego_vehicles[i] is not None and self.ego_vehicles[i].is_alive:
                    print("Destroying ego vehicle {}".format(self.ego_vehicles[i].id))
                    self.ego_vehicles[i].destroy()
                self.ego_vehicles[i] = None
        self.ego_vehicles = []

        for agent_instance in self._agent_instances:
            if agent_instance:
                agent_instance.destroy()
                agent_instance = None

    def _analyze_scenario(self, config):
        """
        Provide feedback about success/failure of a scenario
        """

        # Create the filename
        current_time = str(datetime.now().strftime('%Y-%m-%d-%H-%M-%S'))
        junit_filename = None
        json_filename = None
        config_name = config.name
        if self._args.outputDir != '':
            config_name = os.path.join(self._args.outputDir, config_name)

        if self._args.junit:
            junit_filename = config_name + current_time + ".xml"
        if self._args.json:
            json_filename = config_name + current_time + ".json"
        filename = None
        if self._args.file:
            filename = config_name + current_time + ".txt"

        if not self.manager.analyze_scenario(self._args.output, filename, junit_filename, json_filename):
            print("All scenario tests were passed successfully!")
        else:
            print("Not all scenario tests were successful")
            if not (self._args.output or filename or junit_filename):
                print("Please run with --output for further information")

    def _record_criteria(self, criteria, name):
        """
        Filter the JSON serializable attributes of the criterias and
        dumps them into a file. This will be used by the metrics manager,
        in case the user wants specific information about the criterias.
        """
        file_name = name[:-4] + ".json"

        # Filter the attributes that aren't JSON serializable
        with open('temp.json', 'w', encoding='utf-8') as fp:

            criteria_dict = {}
            for criterion in criteria:

                criterion_dict = criterion.__dict__
                criteria_dict[criterion.name] = {}

                for key in criterion_dict:
                    if key != "name":
                        try:
                            key_dict = {key: criterion_dict[key]}
                            json.dump(key_dict, fp, sort_keys=False, indent=4)
                            criteria_dict[criterion.name].update(key_dict)
                        except TypeError:
                            pass

        os.remove('temp.json')

        # Save the criteria dictionary into a .json file
        with open(file_name, 'w', encoding='utf-8') as fp:
            json.dump(criteria_dict, fp, sort_keys=False, indent=4)

    def _load_and_wait_for_world(self, town):
        """
        Load a new CARLA world and provide data to CarlaDataProvider
        """

        if self._args.reloadWorld:
            self.world = self.client.load_world(town)
        else:
            # if the world should not be reloaded, wait at least until all ego vehicles are ready
            ego_vehicle_found = False
            if self._args.waitForEgo:
                while not ego_vehicle_found and not self._shutdown_requested:
                    vehicles = self.client.get_world().get_actors().filter('vehicle.*')
                    for ego_vehicle in ego_vehicles:
                        ego_vehicle_found = False
                        for vehicle in vehicles:
                            if vehicle.attributes['role_name'] == ego_vehicle.rolename:
                                ego_vehicle_found = True
                                break
                        if not ego_vehicle_found:
                            print("Not all ego vehicles ready. Waiting ... ")
                            time.sleep(1)
                            break

        self.world = self.client.get_world()

        if self._args.sync:
            settings = self.world.get_settings()
            settings.synchronous_mode = True
            settings.fixed_delta_seconds = 1.0 / self.frame_rate
            self.world.apply_settings(settings)


        self.traffic_manager.set_synchronous_mode(True)

        if self._args.eval and len(self._args.eval)==5:
            eval_seed = gen_seeds(self._args.eval, self._args.seed)
            random.seed(eval_seed)
        print("process control seed: {}".format(eval_seed)) 
        tm_seed = int(self._args.trafficManagerSeed)
        if tm_seed == 0:
            tm_seed = random.randint(0,10000)
        print("Setting traffic manager seed: {}".format(tm_seed))
        self.traffic_manager.set_random_device_seed(tm_seed)
        # CarlaActorPool.set_seed(tm_seed)
        CarlaDataProvider._rng = random.RandomState(tm_seed)

        """ TM AutoCast setting """
        self.traffic_manager.set_hybrid_physics_mode(True)
        if Utils.EvalEnv.collider_speed_kmph is not None:
            speed_diff = (20.0 - Utils.EvalEnv.collider_speed_kmph) / 20.0
            self.traffic_manager.global_percentage_speed_difference(speed_diff)
        

        CarlaDataProvider.set_client(self.client)
        CarlaDataProvider.set_world(self.world)
        CarlaDataProvider.set_traffic_manager_port(int(self._args.trafficManagerPort))        
        
        if self._args.route:
            self.manager.set_hud_world(self.world)

        # Wait for the world to be ready
        if CarlaDataProvider.is_sync_mode():
            self.world.tick()
        else:
            self.world.wait_for_tick()

        map_name = CarlaDataProvider.get_map().name.split('/')[-1]
        if map_name not in (town, "OpenDriveMap"):
            print("The CARLA server uses the wrong map: {}".format(map_name))
            print("This scenario requires to use map: {}".format(town))
            return False

        return True
    
    def _load_and_run_scenario(self, configs):
        """
        Load and run the scenario given by config
        """
        
        if not self._load_and_wait_for_world(configs[0].town):
            self._cleanup()
            return False

        CarlaDataProvider.set_traffic_manager_port(int(self._args.trafficManagerPort))
        tm = self.client.get_trafficmanager(int(self._args.trafficManagerPort))
        tm.set_random_device_seed(int(self._args.trafficManagerSeed))
        if self._args.sync:
            tm.set_synchronous_mode(True)

        # Prepare scenario
        print("Preparing routes")
        try:
            config = configs[0]
            if self._args.route:
                scenario = RouteScenario(world=self.world,
                                            configs=configs,
                                            agents_wrapper_status=self.agents_wrapper_status,
                                            servercomm=self.servercomm_wrapper,
                                            debug_mode=self._args.debug)
        except Exception as exception:                  # pylint: disable=broad-except
            print("The scenario cannot be loaded")
            traceback.print_exc()
            print(exception)
            self._cleanup()
            return False

        try:
            if self._args.record:
                recorder_name = "{}/{}/{}.log".format(
                    os.getenv('SCENARIO_RUNNER_ROOT', "./"), self._args.record, config.name)
                self.client.start_recorder(recorder_name, True)

            # Load scenario and run it
            self.manager.load_scenario(scenario, self.world)
            self.manager.run_scenario(self.agents_control_status, self.servercomm_control)

            # Provide outputs if required
            self._analyze_scenario(config)

            # Remove all actors, stop the recorder and save all criterias (if needed)
            scenario.remove_all_actors()
            if self._args.record:
                self.client.stop_recorder()
                self._record_criteria(self.manager.scenario.get_criteria(), recorder_name)

            result = True

        except Exception as e:              # pylint: disable=broad-except
            traceback.print_exc()
            print(e)
            result = False

        self._cleanup()
        return result

    def _run_route(self):
        """
        Run the route scenario
        """
        result = False

        # retrieve routes
        route_configurations = RouteParser.parse_routes_file(self._args.route, self._args.route_id)

        result = self._load_and_run_scenario(route_configurations)

        self._cleanup()
        return result

    def _run_openscenario(self):
        """
        Run a scenario based on OpenSCENARIO
        """

        # Load the scenario configurations provided in the config file
        if not os.path.isfile(self._args.openscenario):
            print("File does not exist")
            self._cleanup()
            return False

        config = OpenScenarioConfiguration(self._args.openscenario, self.client)
        result = self._load_and_run_scenario(config)
        self._cleanup()
        return result


    def run(self):
        """
        Run all scenarios according to provided commandline args
        """
        print("Util.mqtt_port:",Utils.mqtt_port)
        self.bindmqtt()        
        result = True
        print(self._args.route)
        if self._args.openscenario:
            result = self._run_openscenario()
        elif self._args.route:
            result = self._run_route()
        else:
            result = self._run_scenarios()


        print("No more scenarios .... Exiting")
        return result
    
    def bindmqtt(self):
        import os
        print("Binding Mqtt...", self._args.mqttport)
        print("Background Traffic", self._args.bgtraffic)
        os.system('mosquitto -p {} -d'.format(self._args.mqttport))

def gen_seeds(eval_params, seed=0):
    x = eval_params
    #if x[3]>20:
    #    return int(x[0]*100+x[1]*7+x[2]*20+x[3]*13+x[4]+11*seed)
    #else:
    #    return int(x[0]*100+x[1]*7+x[2]*20+x[3]+x[4]+11*seed)
    return int(x[0]*13+x[1]*97+x[2]*113+x[3]*107+x[4]*29+11*seed)

def main():
    """
    main function
    """
    description = ("CARLA Scenario Runner: Setup, Run and Evaluate scenarios using CARLA\n"
                   "Current version: " + VERSION)

    # pylint: disable=line-too-long
    parser = argparse.ArgumentParser(description=description,
                                     formatter_class=RawTextHelpFormatter)
    parser.add_argument('-v', '--version', action='version', version='%(prog)s ' + VERSION)
    parser.add_argument('--host',
                        help='IP of the host server')
    parser.add_argument('--port', default='2000',
                        help='TCP port to listen to (default: 2000)')
    parser.add_argument('--AgentWrapperPort', default='5555',
                        help='Port to update to Agent Wrapper Status (default: 5555)')
    parser.add_argument('--AgentControlPort', default='5557',
                        help='Port to update to Agent Control Status (default: 5557)')
    
    parser.add_argument('--trafficManagerPort', default='8000',
                        help='Port to use for the TrafficManager (default: 8000)')
    parser.add_argument('--trafficManagerSeed', default='0',
                        help='Seed used by the TrafficManager (default: 0)')
    parser.add_argument('--sync', action='store_true',
                        help='Forces the simulation to run synchronously')
    parser.add_argument('--list', action="store_true", help='List all supported scenarios and exit')

    parser.add_argument('--route', help='Run a route as a scenario', type=str, default="srunner/examples/DistributedMultiEgoVehicle.xml")
    parser.add_argument('--route-id', help='Run a specific route inside that \'route\' file', default='', type=str)

    parser.add_argument('--output', action="store_true", help='Provide results on stdout')
    parser.add_argument('--file', action="store_true", help='Write results into a txt file')
    parser.add_argument('--junit', action="store_true", help='Write results into a junit file')
    parser.add_argument('--json', action="store_true", help='Write results into a JSON file')
    parser.add_argument('--outputDir', default='', help='Directory for output files (default: this directory)')

    parser.add_argument('--configFile', default='', help='Provide an additional scenario configuration file (*.xml)')
    parser.add_argument('--additionalScenario', default='', help='Provide additional scenario implementations (*.py)')

    parser.add_argument('--debug', action="store_true", help='Run with debug output')
    parser.add_argument('--record', type=str, default='',
                        help='Path were the files will be saved, relative to SCENARIO_RUNNER_ROOT.\nActivates the CARLA recording feature and saves to file all the criteria information.')
    parser.add_argument('--randomize', action="store_true", help='Scenario parameters are randomized')
    parser.add_argument('--repetitions', default=1, type=int, help='Number of scenario executions')
    parser.add_argument('--waitForEgo', action="store_true", help='Connect the scenario to an existing ego vehicle')

    arguments = parser.parse_args()
    # pylint: enable=line-too-long

    if arguments.list:
        print("Currently the following scenarios are supported:")
        print(*ScenarioConfigurationParser.get_list_of_scenarios(arguments.configFile), sep='\n')
        return 1

    scenario_runner = None
    result = True
    try:
        scenario_runner = ScenarioRunner(arguments)
        result = scenario_runner.run()
    except Exception:   # pylint: disable=broad-except
        traceback.print_exc()

    finally:
        if scenario_runner is not None:
            scenario_runner.destroy()
            del scenario_runner
    return not result


if __name__ == "__main__":
    sys.exit(main())