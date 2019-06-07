#!/usr/bin/env python

# Copyright (c) 2018 Intel Labs.
# authors: German Ros (german.ros@intel.com)
#
# This work is licensed under the terms of the MIT license.
# For a copy, see <https://opensource.org/licenses/MIT>.

""" This module implements an agent that roams around a track following random
waypoints and avoiding other vehicles.
The agent also responds to traffic lights. """

from enum import Enum
import math
import numpy as np

import carla
from agents.tools.misc import is_within_distance_ahead, compute_magnitude_angle


class AgentState(Enum):
    """
    AGENT_STATE represents the possible states of a roaming agent
    """
    NAVIGATING = 1
    BLOCKED_BY_VEHICLE = 2
    BLOCKED_BY_PEDESTRIAN = 3
    BLOCKED_RED_LIGHT = 4


class Agent(object):
    """
    Base class to define agents in CARLA
    """

    DISTANCE_LIGHT = 10  # m

    def __init__(self, vehicle):
        """

        :param vehicle: actor to apply to local planner logic onto
        """
        self._vehicle = vehicle
        self._proximity_threshold = 10.0  # meters
        self._local_planner = None
        self._world = self._vehicle.get_world()
        self._map = self._vehicle.get_world().get_map()
        self._last_red_light_id = None
        self._list_traffic_lights = []

        all_actors = self._world.get_actors()
        for _actor in all_actors:
            if 'traffic_light' in _actor.type_id:
                center, area = self.get_traffic_light_area(_actor)
                waypoints = []
                for pt in area:
                    waypoints.append(self._map.get_waypoint(pt))
                self._list_traffic_lights.append((_actor, center, area, waypoints))


    def run_step(self, debug=False):
        """
        Execute one step of navigation.
        :return: control
        """
        control = carla.VehicleControl()

        if debug:
            control.steer = 0.0
            control.throttle = 0.0
            control.brake = 0.0
            control.hand_brake = False
            control.manual_gear_shift = False

        return control

    def _is_light_red(self):
        """
        Method to check if there is a red light affecting us. This version of
        the method is compatible with both European and US style traffic lights.

        :param:
        :return: a tuple given by (bool_flag, traffic_light), where
                 - bool_flag is True if there is a traffic light in RED
                   affecting us and False otherwise
                 - traffic_light is the object itself or None if there is no
                   red traffic light affecting us
        """
        location = self._vehicle.get_transform().location
        if location is None:
            return (False, None)

        ego_waypoint = self._map.get_waypoint(location)

        for traffic_light, center, area, waypoints in self._list_traffic_lights:
            # logic
            center_loc = carla.Location(center)
            if self._last_red_light_id and self._last_red_light_id == traffic_light.id:
                continue
            if center_loc.distance(location) > self.DISTANCE_LIGHT:
                continue
            if traffic_light.state != carla.TrafficLightState.Red:
                continue

            for wp in waypoints:
                if ego_waypoint.road_id == wp.road_id and ego_waypoint.lane_id == wp.lane_id:
                    # this light is red and is affecting our lane!
                    # is the vehicle traversing the stop line?
                    self._last_red_light_id = traffic_light.id
                    return (True, traffic_light)

        return (False, None)

    def rotate_point(self, pt, angle):
        x_ = math.cos(math.radians(angle))*pt.x - math.sin(math.radians(angle))*pt.y
        y_ = math.sin(math.radians(angle))*pt.x - math.cos(math.radians(angle))*pt.y
        return carla.Vector3D(x_, y_, pt.z)

    def get_traffic_light_area(self, tl):
        base_transform = tl.get_transform()
        base_rot = base_transform.rotation.yaw

        area_loc = base_transform.transform(tl.trigger_volume.location)

        wpx = self._map.get_waypoint(area_loc)
        while not wpx.is_intersection:
            next = wpx.next(1.0)[0]
            if next:
                wpx = next
            else:
                break
        wpx_location = wpx.transform.location
        area_ext = tl.trigger_volume.extent

        area = []
        # why the 0.9 you may ask?... because the triggerboxes are set manually and sometimes they
        # cross to adjacent lanes by accident
        x_values = np.arange(-area_ext.x*0.9, area_ext.x*0.9, 1.0)
        for x in x_values:
            pt = self.rotate_point(carla.Vector3D(x, 0, area_ext.z), base_rot)
            area.append(wpx_location + carla.Location(x=pt.x, y=pt.y))

        return area_loc, area

    def _is_vehicle_hazard(self, vehicle_list):
        """
        Check if a given vehicle is an obstacle in our way. To this end we take
        into account the road and lane the target vehicle is on and run a
        geometry test to check if the target vehicle is under a certain distance
        in front of our ego vehicle.

        WARNING: This method is an approximation that could fail for very large
         vehicles, which center is actually on a different lane but their
         extension falls within the ego vehicle lane.

        :param vehicle_list: list of potential obstacle to check
        :return: a tuple given by (bool_flag, vehicle), where
                 - bool_flag is True if there is a vehicle ahead blocking us
                   and False otherwise
                 - vehicle is the blocker object itself
        """

        ego_vehicle_location = self._vehicle.get_location()
        ego_vehicle_waypoint = self._map.get_waypoint(ego_vehicle_location)

        for target_vehicle in vehicle_list:
            # do not account for the ego vehicle
            if target_vehicle.id == self._vehicle.id:
                continue

            # if the object is not in our lane it's not an obstacle
            target_vehicle_waypoint = self._map.get_waypoint(target_vehicle.get_location())
            if target_vehicle_waypoint.road_id != ego_vehicle_waypoint.road_id or \
                    target_vehicle_waypoint.lane_id != ego_vehicle_waypoint.lane_id:
                continue

            loc = target_vehicle.get_location()
            if is_within_distance_ahead(loc, ego_vehicle_location,
                                        self._vehicle.get_transform().rotation.yaw,
                                        self._proximity_threshold):
                return (True, target_vehicle)

        return (False, None)

    def _is_pedestrian_hazard(self, pedestrian_list):
        """
        Check if a given pedestrian is an obstacle in our way.

        :param pedestrian_list: list of potential pedestrians to check
        :return: a tuple given by (bool_flag, pedestrian), where
                 - bool_flag is True if there is a pedestrian ahead is nearby
                   and False otherwise
                 - pedestrian is the blocker object itself
        """

        MIN_DISTANCE_PEDESTRIAN = 20.0 # meters

        ego_vehicle_location = self._vehicle.get_location()
        for target_actor in pedestrian_list:
            # do not account for the ego vehicle
            if target_actor.id != self._vehicle.id:
                loc = target_actor.get_location()
                if loc.distance(ego_vehicle_location) <= MIN_DISTANCE_PEDESTRIAN:
                    if is_within_distance_ahead(loc, ego_vehicle_location,
                                                self._vehicle.get_transform().rotation.yaw,
                                                self._proximity_threshold):
                        return (True, target_actor)

        return (False, None)

    def emergency_stop(self):
        """
        Send an emergency stop command to the vehicle
        :return:
        """
        control = carla.VehicleControl()
        control.steer = 0.0
        control.throttle = 0.0
        control.brake = 1.0
        control.hand_brake = False

        return control
