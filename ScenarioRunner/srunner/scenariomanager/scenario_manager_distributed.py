#!/usr/bin/env python

# Copyright (c) 2018-2020 Intel Corporation
#
# This work is licensed under the terms of the MIT license.
# For a copy, see <https://opensource.org/licenses/MIT>.

"""
This module provides the ScenarioManager implementation.
It must not be modified and is for reference only!
"""

from __future__ import print_function
import sys
import time

import py_trees

from AutoCastSim.ScenarioRunner.srunner.scenariomanager.carla_data_provider import CarlaDataProvider
from AutoCastSim.ScenarioRunner.srunner.scenariomanager.result_writer import ResultOutputProvider
from AutoCastSim.ScenarioRunner.srunner.scenariomanager.timer import GameTime
from AutoCastSim.ScenarioRunner.srunner.scenariomanager.watchdog import Watchdog
from AutoCastSim.ScenarioRunner.synchronization import Status

class ScenarioManager(object):

    """
    Basic scenario manager class. This class holds all functionality
    required to start, and analyze a scenario.

    The user must not modify this class.

    To use the ScenarioManager:
    1. Create an object via manager = ScenarioManager()
    2. Load a scenario via manager.load_scenario()
    3. Trigger the execution of the scenario manager.run_scenario()
       This function is designed to explicitly control start and end of
       the scenario execution
    4. Trigger a result evaluation with manager.analyze_scenario()
    5. If needed, cleanup with manager.stop_scenario()
    """

    def __init__(self, debug_mode=False, sync_mode=False, timeout=2.0):
        """
        Setups up the parameters, which will be filled at load_scenario()

        """
        self.scenario = None
        self.scenario_tree = None
        self.ego_vehicles = None
        self.other_actors = None
        self.world = None 
        self.actor_ids = [] 

        self._debug_mode = debug_mode
        self._sync_mode = sync_mode
        self._watchdog = None
        self._timeout = timeout

        self._running = False
        self._timestamp_last_run = 0.0
        self.scenario_duration_system = 0.0
        self.scenario_duration_game = 0.0
        self.start_system_time = None
        self.end_system_time = None

    def _reset(self):
        """
        Reset all parameters
        """
        self._running = False
        self._timestamp_last_run = 0.0
        self.scenario_duration_system = 0.0
        self.scenario_duration_game = 0.0
        self.start_system_time = None
        self.end_system_time = None
        GameTime.restart()

    def cleanup(self):
        """
        This function triggers a proper termination of a scenario
        """

        if self._watchdog is not None:
            self._watchdog.stop()
            self._watchdog = None

        if self.scenario is not None:
            self.scenario.terminate()

        # #TODO: Move this to AgentWrapperMain
        # for ego_agent in self._agents: 
        #     ego_agent.cleanup()

        CarlaDataProvider.cleanup()

    def load_scenario(self, scenario, world):
        """
        Load a new scenario
        """
        self._reset()

        self.scenario = scenario
        self.scenario_tree = self.scenario.scenario_tree
        self.ego_vehicles = scenario.ego_vehicles
        self.other_actors = scenario.other_actors
        self.world = world 
        self.actor_ids = [str(ego_vehicle.id) for ego_vehicle in self.ego_vehicles]
        print(f"Actors ids before starting while loop: {self.actor_ids}")
        
    def run_scenario(self, agents_control_status, servercomm):
        """
        Trigger the start of the scenario and wait for it to finish/fail
        """
        print("ScenarioManager: Running scenario {}".format(self.scenario_tree.name))
        self.start_system_time = time.time()
        start_game_time = GameTime.get_time()

        self._watchdog = Watchdog(float(self._timeout))
        self._watchdog.start()
        self._running = True
        agents_control_status.actor_ids = self.actor_ids

        #TODO: Debug this part     
        while self._running:
            print("Entering new interation in run_scenario while loop")
            timestamp = None
            world = self.world
            if world:
                snapshot = world.get_snapshot()
                if snapshot:
                    timestamp = snapshot.timestamp
            if timestamp:
                self._tick_scenario(timestamp, agents_control_status, servercomm)
                agents_control_status.reset_status() 
                agents_control_status.completed = False

        servercomm.socket.close()
        self.cleanup()

        self.end_system_time = time.time()
        end_game_time = GameTime.get_time()

        self.scenario_duration_system = self.end_system_time - \
            self.start_system_time
        self.scenario_duration_game = end_game_time - start_game_time

        if self.scenario_tree.status == py_trees.common.Status.FAILURE:
            print("ScenarioManager: Terminated due to failure")

    def _tick_scenario(self, timestamp, agents_control_status, servercomm):
        """
        Run next tick of scenario and the agent.
        If running synchornously, it also handles the ticking of the world.
        """
        print("Starting tick")
        if self._timestamp_last_run < timestamp.elapsed_seconds and self._running:
            self._timestamp_last_run = timestamp.elapsed_seconds

            self._watchdog.update()

            if self._debug_mode:
                print("\n--------- Tick ---------\n")

            # Update game time and actor information
            GameTime.on_carla_tick(timestamp)
            CarlaDataProvider.on_carla_tick()
            
            print("Updating agents control")

            agents_control_completed = agents_control_status.completed 
            while not agents_control_completed:
                print("Waiting for agent wrapper to apply control")
                message = servercomm.recv_send_message()
                list_message = message.split("_")
                actor_id = list_message[0]
                frame_id = list_message[1]
                print(f"Current frame: {frame_id}")
                agents_control_status.update_dictionary(actor_id, True)
                agents_control_status.check_completion()
                agents_control_completed = agents_control_status.completed 
                print(agents_control_completed)
    
            print("All agents applied control")
            # Tick scenario
            self.scenario_tree.tick_once()

            if self._debug_mode:
                print("\n")
                py_trees.display.print_ascii_tree(self.scenario_tree, show_status=True)
                sys.stdout.flush()

            # if self.scenario_tree.status != py_trees.common.Status.RUNNING:
            #     self._running = False

        if self._sync_mode and self._running and self._watchdog.get_status():
            self.world.tick()

    def get_running_status(self):
        """
        returns:
           bool:  False if watchdog exception occured, True otherwise
        """
        return self._watchdog.get_status()

    def stop_scenario(self):
        """
        This function is used by the overall signal handler to terminate the scenario execution
        """
        self._running = False

    def analyze_scenario(self, stdout, filename, junit, json):
        """
        This function is intended to be called from outside and provide
        the final statistics about the scenario (human-readable, in form of a junit
        report, etc.)
        """

        failure = False
        timeout = False
        result = "SUCCESS"

        criteria = self.scenario.get_criteria()
        if len(criteria) == 0:
            print("Nothing to analyze, this scenario has no criteria")
            return True

        for criterion in criteria:
            if (not criterion.optional and
                    criterion.test_status != "SUCCESS" and
                    criterion.test_status != "ACCEPTABLE"):
                failure = True
                result = "FAILURE"
            elif criterion.test_status == "ACCEPTABLE":
                result = "ACCEPTABLE"

        if self.scenario.timeout_node.timeout and not failure:
            timeout = True
            result = "TIMEOUT"

        output = ResultOutputProvider(self, result, stdout, filename, junit, json)
        output.write()

        return failure or timeout
