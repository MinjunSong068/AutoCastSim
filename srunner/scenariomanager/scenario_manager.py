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
import os
import sys
import time

import py_trees

from srunner.autoagents.agent_wrapper import AgentWrapper
from srunner.scenariomanager.carla_data_provider import CarlaDataProvider
from srunner.scenariomanager.result_writer import ResultOutputProvider
from srunner.scenariomanager.timer import GameTime

from AutoCastSim.AVR.HUD import HUD
from AutoCastSim.AVR.DataLogger import DataLogger
from AutoCastSim.AVR import Utils, Collaborator


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

    def __init__(self, route_mode=True, debug_mode=False, sync_mode=False, timeout=2.0, recording=False, sharing=False, prefix=''):
        """
        Setups up the parameters, which will be filled at load_scenario()

        """
        self.scenario = None
        self.scenario_tree = None
        self.ego_vehicles = None
        self.other_actors = None

        self._debug_mode = debug_mode
        self._agent = None
        self._sync_mode = sync_mode
        self._watchdog = None
        self._timeout = timeout

        self._running = False
        self._timestamp_last_run = 0.0
        self.scenario_duration_system = 0.0
        self.scenario_duration_game = 0.0
        self.start_system_time = None
        self.end_system_time = None

        self._route_mode = route_mode

        """AVR"""
        Utils.RecordingOutput = os.path.join(Utils.RecordingOutput, prefix) + '/'

        self._temp_hud = True
        self._hud = None
        self._hud_debug = False
        if self._route_mode and self._temp_hud:
            self._hud = HUD(recording=recording, debug_mode=self._hud_debug)
        self.sharing_session = sharing
        print("Finished initializing scenario manager")


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

        if self._agent is not None:
            self._agent.cleanup()
            self._agent = None

        CarlaDataProvider.cleanup()

    def load_scenario(self, scenario, agent=None):
        """
        Load a new scenario
        """
        self._reset()
        self._agent = AgentWrapper(agent) if agent else None
        if self._agent is not None:
            self._sync_mode = True
        self.scenario = scenario
        self.scenario_tree = self.scenario.scenario_tree
        self.ego_vehicles = scenario.ego_vehicles
        self.other_actors = scenario.other_actors

        # To print the scenario tree uncomment the next line
        # py_trees.display.render_dot_tree(self.scenario_tree)

        if self._agent is not None:
            self._agent.setup_sensors(self.ego_vehicles[0], self._debug_mode)

    def check_sensors_on_other_actors(self, dist_thresh=50):
        """
        Keep actors in vicinity with sensors.
        """
        vehicles = CarlaDataProvider.get_actors()
        for id, vehicle in vehicles:
            """ filter ego vehicles """
            is_ego = False
            for ego_vehicle in self.ego_vehicles:
                # print("EGO ID: {}".format(ego_vehicle.id))
                if id == ego_vehicle.id:
                    is_ego = True
                    break
            if is_ego:
                continue
            if vehicle.attributes['role_name'] == Utils.PASSIVE_ACTOR_ROLENAME:
                continue
            """ decide distance """
            nearby = False
            vehicle_location = CarlaDataProvider.get_location(vehicle)
            for ego_vehicle in self.ego_vehicles:
                ego_location = CarlaDataProvider.get_location(ego_vehicle)
                if (ego_location is not None) and (vehicle_location is not None) and (ego_location.distance(vehicle_location) < dist_thresh):
                    nearby = True
                    break
            sensor_id = str(id) + Collaborator.LidarSensorName
            # print("Sensor ID: {}".format(sensor_id))
            existing_sensor = self._agent.has_sensor(sensor_id)
            if nearby and (not existing_sensor):
                # self._agent.try_setup_lidar_on_other_vehicle(vehicle, debug_mode=self._debug_mode)
                self._agent.setup_sensors(vehicle, debug_mode=self._debug_mode)
                self._agent.setup_collaborator(vehicle, self.sharing_session)
            if (not nearby) and existing_sensor:
                self._agent.destroy_sensors(id)
                self._agent.destroy_collaborator(id)        

    def run_scenario(self):
        """
        Trigger the start of the scenario and wait for it to finish/fail
        """
        print("ScenarioManager: Running scenario {}".format(self.scenario_tree.name))
        self.start_system_time = time.time()
        start_game_time = GameTime.get_time()

        self._watchdog = Watchdog(float(self._timeout))
        self._watchdog.start()
        self._running = True

        while self._running:
            print("=================================================================================================")
            time_start = time.time()
            timestamp = None
            world = CarlaDataProvider.get_world()
            if world:
                snapshot = world.get_snapshot()
                if snapshot:
                    timestamp = snapshot.timestamp
            if timestamp:
                """AVR share before action"""

                if self._route_mode and self.sharing_session:
                    self.check_sensors_on_other_actors()
                sensor_update = time.time()
                if Utils.TIMEPROFILE: print("Sensor Update: {} s, Total {} s".format(sensor_update-time_start, sensor_update-time_start))
                if self._agent is not None:
                    if not Utils.TIMEPROFILE:
                        """parallel sync version"""
                        self._agent.tick_collaborators()
                        self._agent.join_collaborators()
                        if self.sharing_session:
                            self._agent.tick_beacon()
                    else:
                        """single thread version"""
                        for c in self._agent._collaborator_dict.values():
                            if c:
                                c.tick()
                                c.tick_join()
                        if self.sharing_session:
                            self._agent.tick_beacon()
                collab_update = time.time()
                if Utils.TIMEPROFILE: print("Collaborator: {} s, Total {} s".format(collab_update - sensor_update, collab_update-time_start))
                    
                self._tick_scenario(timestamp)
                scen_update = time.time()
                if Utils.TIMEPROFILE: print("Scenario Tick: {} s, Total {} s".format(scen_update - collab_update, scen_update - time_start))
                """AVR Visualization"""
                if self._temp_hud:
                    if self._hud.tick():
                        break
                hud_update = time.time()
                if Utils.TIMEPROFILE: print("HUD update: {} s, Total {} s".format(hud_update - scen_update, hud_update - time_start))

        self.cleanup()

        self.end_system_time = time.time()
        end_game_time = GameTime.get_time()

        self.scenario_duration_system = self.end_system_time - \
            self.start_system_time
        self.scenario_duration_game = end_game_time - start_game_time

        if self.scenario_tree.status == py_trees.common.Status.FAILURE:
            print("ScenarioManager: Terminated due to failure")

    def _tick_scenario(self, timestamp):
        """
        Run next tick of scenario and the agent.
        If running synchornously, it also handles the ticking of the world.
        """

        if self._timestamp_last_run < timestamp.elapsed_seconds and self._running:
            self._timestamp_last_run = timestamp.elapsed_seconds

            self._watchdog.update()

            if self._debug_mode:
                print("\n--------- Tick ---------\n")

            # Update game time and actor information
            GameTime.on_carla_tick(timestamp)
            CarlaDataProvider.on_carla_tick()

            if self._route_mode and self._temp_hud:
                self._hud.on_carla_tick(timestamp)

            if self._agent is not None:
                ego_action = self._agent()  # pylint: disable=not-callable
                if Utils.HUMAN_AGENT:
                    self.ego_vehicles[0].apply_control(self._hud.human_control)
                else:
                    self.ego_vehicles[0].apply_control(ego_action)

            # Tick scenario
            self.scenario_tree.tick_once()

            if self._debug_mode:
                print("\n")
                py_trees.display.print_ascii_tree(self.scenario_tree, show_status=True)
                sys.stdout.flush()

            if self.scenario_tree.status != py_trees.common.Status.RUNNING:
                self._running = False

        if self._sync_mode and self._running and self._watchdog.get_status():
            CarlaDataProvider.get_world().tick()

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
    
    def set_hude_agent(self, agent):
        if self._hud:
            self._hud.set_agent(agent, self._agent)

    def set_hud_world(self, world):
        if self._hud:
            self._hud.set_world(world)
