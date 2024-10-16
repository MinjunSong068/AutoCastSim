#!/usr/bin/env python

# Copyright (c) 2019-2020 Intel Corporation
#
# This work is licensed under the terms of the MIT license.
# For a copy, see <https://opensource.org/licenses/MIT>.

"""
This module provides Challenge routes as standalone scenarios
"""

from __future__ import print_function

import glob
import os
import sys
import importlib
import inspect
import traceback
import py_trees

from numpy import random
import carla

from AutoCastSim.ScenarioRunner.agents.navigation.local_planner import RoadOption

from AutoCastSim.ScenarioRunner.srunner.scenarioconfigs.scenario_configuration import ActorConfigurationData
from AutoCastSim.ScenarioRunner.srunner.scenariomanager.carla_data_provider import CarlaDataProvider

from AutoCastSim.ScenarioRunner.srunner.scenariomanager.scenarioatomics.atomic_behaviors import ScenarioTriggerer, Idle
from AutoCastSim.ScenarioRunner.srunner.scenariomanager.scenarioatomics.atomic_trigger_conditions import WaitForBlackboardVariable
from AutoCastSim.ScenarioRunner.srunner.scenariomanager.scenarioatomics.atomic_criteria import (CollisionTest,
                                                                     InRouteTest,
                                                                     RouteCompletionTest,
                                                                     OutsideRouteLanesTest,
                                                                     RunningRedLightTest,
                                                                     RunningStopTest,
                                                                     ActorBlockedTest,
                                                                     MinimumSpeedRouteTest)

from AutoCastSim.ScenarioRunner.srunner.scenarios.basic_scenario_distributed import BasicScenario
from AutoCastSim.ScenarioRunner.srunner.scenarios.background_activity import BackgroundBehavior
from AutoCastSim.ScenarioRunner.srunner.scenariomanager.weather_sim import RouteWeatherBehavior
from AutoCastSim.ScenarioRunner.srunner.scenariomanager.lights_sim import RouteLightsBehavior
from AutoCastSim.ScenarioRunner.srunner.scenariomanager.timer import RouteTimeoutBehavior

from AutoCastSim.ScenarioRunner.srunner.tools.route_parser_distributed import RouteParser, DIST_THRESHOLD
from AutoCastSim.ScenarioRunner.srunner.tools.route_manipulation_distributed import interpolate_trajectory
from AutoCastSim.ScenarioRunner.synchronization import Status
import zmq

SECONDS_GIVEN_PER_METERS = 0.4


class RouteScenario(BasicScenario):

    """
    Implementation of a RouteScenario, i.e. a scenario that consists of driving along a pre-defined route,
    along which several smaller scenarios are triggered
    """

    def __init__(self, world, configs, agents_wrapper_status, servercomm, debug_mode=False, criteria_enable=True, timeout=300):
        """
        Setup all relevant parameters and create scenarios along route
        """

        self.configs = configs
        self.world = world
        self.routes = self._get_route(configs)

        ego_vehicles_detected = 0
        while ego_vehicles_detected < len(configs):
            ego_vehicles = self.get_ego_vehicles()
            ego_vehicles_detected = len(ego_vehicles)
        print("All ego vehicles spawned")

        actor_ids = [str(ego_vehicle.id) for ego_vehicle in ego_vehicles]
        agents_wrapper_status.actor_ids = actor_ids

        agents_wrapper_completed = agents_wrapper_status.completed 
        while not agents_wrapper_completed:
            message = servercomm.recv_send_message()
            print(message)
            agents_wrapper_status.update_dictionary(message, True)
            agents_wrapper_status.check_completion()
            agents_wrapper_completed = agents_wrapper_status.completed 
        print("Agent wrappers completed")
        servercomm.socket.close() 

        max_timeout = 0
        for route in self.routes:
            timeout = self._estimate_route_timeout(route) 

            if debug_mode:
                self._draw_waypoints(world, route, vertical_shift=0.1, size=0.1, persistency=timeout, downsample=5)
            
            if timeout > max_timeout: 
                max_timeout = timeout
        
        self.timeout = max_timeout

        self._build_scenarios(
            world, ego_vehicles, configs, timeout=self.timeout, debug=debug_mode > 0
        )

        super(RouteScenario, self).__init__(
            "Routes", ego_vehicles, configs, world, debug_mode > 1, False, criteria_enable
        )

    def _get_route(self, configs):
        """
        Gets the route from the configuration, interpolating it to the desired density,
        saving it to the CarlaDataProvider and sending it to the agent

        Parameters:
        - world: CARLA world
        - config: Scenario configuration (RouteConfiguration)
        - debug_mode: boolean to decide whether or not the route poitns are printed
        """
        # prepare route's trajectory (interpolate and add the GPS route)
        routes = []
        map = self.world.get_map()
        for config in configs:
            gps_route, route = interpolate_trajectory(map, self.world, config.keypoints)
            if config.agent is not None:
                config.agent.set_global_plan(gps_route, route)
            routes.append(route)
        return routes

    def _filter_scenarios(self, scenario_configs):
        """
        Given a list of scenarios, filters out does that don't make sense to be triggered,
        as they are either too far from the route or don't fit with the route shape

        Parameters:
        - scenario_configs: list of ScenarioConfiguration
        """
        new_scenarios_config = []
        for scenario_config in scenario_configs:
            trigger_point = scenario_config.trigger_points[0]
            if not RouteParser.is_scenario_at_route(trigger_point, self.route):
                print("WARNING: Ignoring scenario '{}' as it is too far from the route".format(scenario_config.name))
                continue

            new_scenarios_config.append(scenario_config)

        return new_scenarios_config
    
    def get_ego_vehicles(self): 
        all_actors = self.world.get_actors()
        all_vehicles = all_actors.filter("vehicle.*")
        ego_vehicles = [] 

        for vehicle in all_vehicles:
            full_role_name = vehicle.attributes.get("role_name") 
            type = full_role_name.split("_")[0]
            if type == "egovehicle":
                ego_vehicles.append(vehicle)
                CarlaDataProvider._carla_actor_pool[vehicle.id] = vehicle
                spawn_point = vehicle.get_transform()
                CarlaDataProvider.register_actor(vehicle, spawn_point)
        return ego_vehicles

    def _estimate_route_timeout(self, route):
        """
        Estimate the duration of the route, as a proportinal value of its length
        """
        route_length = 0.0  # in meters

        prev_point = route[0][0]
        for current_point, _ in route[1:]:
            dist = current_point.location.distance(prev_point.location)
            route_length += dist
            prev_point = current_point

        return int(SECONDS_GIVEN_PER_METERS * route_length)

    def _draw_waypoints(self, world, waypoints, vertical_shift, size, persistency=-1, downsample=1):
        """
        Draw a list of waypoints at a certain height given in vertical_shift.
        """
        for i, w in enumerate(waypoints):
            if i % downsample != 0:
                continue

            wp = w[0].location + carla.Location(z=vertical_shift)

            if w[1] == RoadOption.LEFT:  # Yellow
                color = carla.Color(128, 128, 0)
            elif w[1] == RoadOption.RIGHT:  # Cyan
                color = carla.Color(0, 128, 128)
            elif w[1] == RoadOption.CHANGELANELEFT:  # Orange
                color = carla.Color(128, 32, 0)
            elif w[1] == RoadOption.CHANGELANERIGHT:  # Dark Cyan
                color = carla.Color(0, 32, 128)
            elif w[1] == RoadOption.STRAIGHT:  # Gray
                color = carla.Color(64, 64, 64)
            else:  # LANEFOLLOW
                color = carla.Color(0, 128, 0)  # Green

            world.debug.draw_point(wp, size=0.1, color=color, life_time=persistency)

        world.debug.draw_point(waypoints[0][0].location + carla.Location(z=vertical_shift), size=2*size,
                               color=carla.Color(0, 0, 128), life_time=persistency)
        world.debug.draw_point(waypoints[-1][0].location + carla.Location(z=vertical_shift), size=2*size,
                               color=carla.Color(128, 128, 128), life_time=persistency)

    def _scenario_sampling(self, potential_scenarios, random_seed=0):
        """Sample the scenarios that are going to happen for this route."""
        # Fix the random seed for reproducibility, and randomly sample a scenario per trigger position.
        rng = random.RandomState(random_seed)

        sampled_scenarios = []
        for trigger in list(potential_scenarios):
            scenario_list = potential_scenarios[trigger]
            sampled_scenarios.append(rng.choice(scenario_list))

        return sampled_scenarios

    def get_all_scenario_classes(self):

        # Path of all scenario at "srunner/scenarios" folder
        # scenarios_list = glob.glob("{}/srunner/scenarios/*.py".format(os.getenv('SCENARIO_RUNNER_ROOT', "./")))
        dir_path = os.path.dirname(os.path.realpath(__file__))
        scenarios_list = glob.glob("{}/*.py".format(dir_path))
        
        all_scenario_classes = {}

        for scenario_file in scenarios_list:

            # Get their module
            module_name = os.path.basename(scenario_file).split('.')[0]
            sys.path.insert(0, os.path.dirname(scenario_file))
            scenario_module = importlib.import_module(module_name)

            # And their members of type class
            for member in inspect.getmembers(scenario_module, inspect.isclass):
                # TODO: Filter out any class that isn't a child of BasicScenario
                all_scenario_classes[member[0]] = member[1]

        return all_scenario_classes

    def _build_scenarios(self, world, ego_vehicles, configs, scenarios_per_tick=5, timeout=300, debug=False):
        """
        Initializes the class of all the scenarios that will be present in the route.
        If a class fails to be initialized, a warning is printed but the route execution isn't stopped
        """
        all_scenario_classes = self.get_all_scenario_classes()
        self.list_scenarios = []
        ego_data = []
        for ego_vehicle in ego_vehicles:
            ego_data.append(ActorConfigurationData(ego_vehicle.type_id, ego_vehicle.get_transform(), ego_vehicle.attributes["role_name"]))

        if debug:
            tmap = self.world.get_map()
            for config in configs:
                scenario_config = config.scenario_configs[0]
                scenario_loc = scenario_config.trigger_points[0].location
                debug_loc = tmap.get_waypoint(scenario_loc).transform.location + carla.Location(z=0.2)
                world.debug.draw_point(debug_loc, size=0.2, color=carla.Color(128, 0, 0), life_time=timeout)
                world.debug.draw_string(debug_loc, str(scenario_config.name), draw_shadow=False,
                                        color=carla.Color(0, 0, 128), life_time=timeout, persistent_lines=True)

        # for scenario_number, config in enumerate(configs):
        scenario_number = 0 
        scenario_config = configs[0].scenario_configs[0]
        scenario_config.ego_vehicles = [ego_data]
        scenario_config.route_var_name = "ScenarioRouteNumber{}".format(scenario_number)
        #TODO change back to self.routes[scenario_number]
        scenario_config.route = self.routes[scenario_number]

        try:
            scenario_class = all_scenario_classes[scenario_config.type]
            #TODO change back to [ego_vehicles[scenario_number]]
            scenario_instance = scenario_class(world, ego_vehicles, scenario_config, timeout=timeout)
            self.list_scenarios.append(scenario_instance)
            # Do a tick every once in a while to avoid spawning everything at the same time
            if scenario_number % scenarios_per_tick == 0:
                world.tick()

        except Exception as e:
            if not debug:
                print("Skipping scenario '{}' due to setup error: {}".format(scenario_config.type, e))
            else:
                traceback.print_exc()


    # pylint: enable=no-self-use
    def _initialize_actors(self, config):
        """
        Set other_actors to the superset of all scenario actors
        """
        # Add all the actors of the specific scenarios to self.other_actors
        for scenario in self.list_scenarios:
            self.other_actors.extend(scenario.other_actors)

    def _create_behavior(self):
        """
        Creates a parallel behavior that runs all of the scenarios part of the route.
        These subbehaviors have had a trigger condition added so that they wait until
        the agent is close to their trigger point before activating.

        It also adds the BackgroundActivity scenario, which will be active throughout the whole route.
        This behavior never ends and the end condition is given by the RouteCompletionTest criterion.
        """
        scenario_trigger_distance = DIST_THRESHOLD  # Max trigger distance between route and scenario

        behavior = py_trees.composites.Parallel(name="Route Behavior",
                                                policy=py_trees.common.ParallelPolicy.SUCCESS_ON_ALL)

        scenario_behaviors = []
        blackboard_list = []

        for scenario in self.list_scenarios:
            if scenario.behavior_tree is not None:
                scenario_behaviors.append(scenario.behavior_tree)
                blackboard_list.append([scenario.config.route_var_name,
                                        scenario.config.trigger_points[0].location])

        # Add the behavior that manages the scenario trigger conditions
        scenario_triggerer = ScenarioTriggerer(
            self.ego_vehicles[0], self.routes[0], blackboard_list, scenario_trigger_distance)
        behavior.add_child(scenario_triggerer)  # Tick the ScenarioTriggerer before the scenarios

        # Add the Background Activity
        behavior.add_child(BackgroundBehavior(self.ego_vehicles[0], self.routes[0], name="BackgroundActivity"))

        behavior.add_children(scenario_behaviors)
        return behavior

    def _create_test_criteria(self):
        """
        Create the criteria tree. It starts with some route criteria (which are always active),
        and adds the scenario specific ones, which will only be active during their scenario
        """
        criteria = py_trees.composites.Parallel(name="Criteria",
                                                policy=py_trees.common.ParallelPolicy.SUCCESS_ON_ONE)

        # End condition
        criteria.add_child(RouteCompletionTest(self.ego_vehicles[0], route=self.routes[0]))

        # 'Normal' criteria
        criteria.add_child(OutsideRouteLanesTest(self.ego_vehicles[0], route=self.routes[0]))
        criteria.add_child(CollisionTest(self.ego_vehicles[0], name="CollisionTest"))
        criteria.add_child(RunningRedLightTest(self.ego_vehicles[0]))
        criteria.add_child(RunningStopTest(self.ego_vehicles[0]))
        criteria.add_child(MinimumSpeedRouteTest(self.ego_vehicles[0], route=self.routes[0], checkpoints=4, name="MinSpeedTest"))

        # These stop the route early to save computational time
        criteria.add_child(InRouteTest(
            self.ego_vehicles[0], route=self.routes[0], offroad_max=30, terminate_on_failure=True))
        criteria.add_child(ActorBlockedTest(
            self.ego_vehicles[0], min_speed=0.1, max_time=180.0, terminate_on_failure=True, name="AgentBlockedTest")
        )

        for scenario in self.list_scenarios:
            scenario_criteria = scenario.get_criteria()
            if len(scenario_criteria) == 0:
                continue  # No need to create anything

            criteria.add_child(
                self._create_criterion_tree(scenario, scenario_criteria)
            )

        return criteria

    def _create_weather_behavior(self):
        """
        Create the weather behavior
        """
        # if len(self.config.weather) == 1:
        return  # Just set the weather at the beginning and done
        # return RouteWeatherBehavior(self.ego_vehicles[0], self.route, self.config.weather)

    def _create_lights_behavior(self):
        """
        Create the street lights behavior
        """
        return RouteLightsBehavior(self.ego_vehicles[0], 100)

    def _create_timeout_behavior(self):
        """
        Create the timeout behavior
        """
        return RouteTimeoutBehavior(self.ego_vehicles[0], self.routes[0])

    def _initialize_environment(self, world):
        """
        Set the weather
        """
        # Set the appropriate weather conditions
        world.set_weather(self.configs[0].weather[0][1])

    def _create_criterion_tree(self, scenario, criteria):
        """
        We can make use of the blackboard variables used by the behaviors themselves,
        as we already have an atomic that handles their (de)activation.
        The criteria will wait until that variable is active (the scenario has started),
        and will automatically stop when it deactivates (as the scenario has finished)
        """
        scenario_name = scenario.name
        var_name = scenario.config.route_var_name
        check_name = "WaitForBlackboardVariable: {}".format(var_name)

        criteria_tree = py_trees.composites.Sequence(name=scenario_name)
        criteria_tree.add_child(WaitForBlackboardVariable(var_name, True, False, name=check_name))

        scenario_criteria = py_trees.composites.Parallel(name=scenario_name,
                                                policy=py_trees.common.ParallelPolicy.SUCCESS_ON_ONE)
        for criterion in criteria:
            scenario_criteria.add_child(criterion)
        scenario_criteria.add_child(WaitForBlackboardVariable(var_name, False, None, name=check_name))

        criteria_tree.add_child(scenario_criteria)
        criteria_tree.add_child(Idle())  # Avoid the indivual criteria stopping the simulation
        return criteria_tree


    def __del__(self):
        """
        Remove all actors upon deletion
        """
        self.remove_all_actors()
