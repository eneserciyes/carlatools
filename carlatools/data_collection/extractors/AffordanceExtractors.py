from .DataExtractor import DataExtractor
from ..data.Data import Data
from carlatools.utils.utils import _numpy, gnss_to_carla_coord

from carla import Location
import numpy as np

from agents.tools.misc import is_within_distance


class TrafficLightExtractor(DataExtractor):
    def __init__(self, name, vehicle, world, map, required_sensors=None):
        super().__init__(name, required_sensors=required_sensors)
        self.vehicle = vehicle
        self.world = world
        self.opendrive_map = map

    def extract(self):
        # 0 for no light, 1 for light detected
        tl_exists = 0
        # 0 represents green or nonexistent light. 1 represents yellow or red light
        tl_state = 0
        # 100 if no light detected
        tl_dist = 100
        tl_list = self.world.get_actors().filter("*traffic_light*")

        vehicle_loc = self.vehicle.get_location()
        vehicle_yaw = self.vehicle.get_transform().rotation.yaw
        vehicle_wp = self.opendrive_map.get_waypoint(vehicle_loc)

        for tl in tl_list:
            tl_transform = tl.get_transform()
            tl_loc = tl_transform.location
            tl_waypoint = self.opendrive_map.get_waypoint(tl_loc)

            if tl_waypoint.road_id != vehicle_wp.road_id:
                continue

            if is_within_distance(tl_loc, vehicle_loc, vehicle_yaw, 20, 60):
                tl_exists = 1
                tl_dist = np.linalg.norm(_numpy(vehicle_loc) - _numpy(tl_loc))
                tl_state = int(self.vehicle.get_traffic_light_state())
                tl_state = int(tl_state == 0 or tl_state == 1)

        return {"tl_exists": Data(tl_exists), "tl_state": Data(tl_state), "tl_dist": Data(tl_dist)}


class LaneAffordancesExtractor(DataExtractor):
    def __init__(self, name, vehicle, world, map, autopilot, required_sensors=None):
        super().__init__(name, required_sensors=required_sensors)
        self.vehicle = vehicle
        self.world = world
        self.opendrive_map = map
        self.autopilot = autopilot

    def extract(self):
        vehicle_loc = self.vehicle.get_location()
        vehicle_loc_np = _numpy(vehicle_loc)

        prev = self.autopilot._global_plan[self.autopilot.waypoint_idx-1][0]
        target = self.autopilot._global_plan[self.autopilot.waypoint_idx][0]

        # Unit vector pointing in vehicle's direction
        forward_vec = self.vehicle.get_transform().rotation.get_forward_vector()

        carla_coord_prev = gnss_to_carla_coord(
            prev["lat"], prev["lon"], prev["z"])
        carla_coord_target = gnss_to_carla_coord(
            target["lat"], target["lon"], target["z"])

        prev_wp = self.opendrive_map.get_waypoint(Location(*carla_coord_prev))
        target_wp = self.opendrive_map.get_waypoint(
            Location(*carla_coord_target))

        # Generate a fake waypoint if we don't have a target
        lane_wp = self.opendrive_map.get_waypoint(vehicle_loc)
        if target_wp is None:
            lane_wp_loc = lane_wp.transform.location
            lane_wp_loc += forward_vec + forward_vec
            target_wp = self.opendrive_map.get_waypoint(lane_wp_loc)

        # If at an intersection or CARLA gave us a wp on the wrong lane or the target waypoint is
        # at a new road section, treat a straight line between prev_wp and target_wp as our lane
        # and calculate features.
        if (target_wp.is_junction or lane_wp.is_junction or lane_wp.lane_id != target_wp.lane_id
                or lane_wp.road_id != target_wp.road_id):
            lane_dist, lane_forward_vec_np = self.interpolate_optimal_wp(
                vehicle_loc_np, prev_wp, target_wp)
        else:
            lane_wp_loc_np = _numpy(lane_wp.transform.location)
            lane_forward_vec_np = _numpy(
                lane_wp.transform.rotation.get_forward_vector())
            lane_dist = np.linalg.norm(vehicle_loc_np - lane_wp_loc_np)

        is_junction = lane_wp.is_junction
        forward_vec_np = _numpy(forward_vec)

        lane_angle = np.arctan2(lane_forward_vec_np[0], lane_forward_vec_np[1])
        forward_vec_angle = np.arctan2(forward_vec_np[0], forward_vec_np[1])
        angle_deviation = forward_vec_angle - lane_angle
        if angle_deviation > np.pi:
            angle_deviation = 2 * np.pi - angle_deviation
        elif angle_deviation < - np.pi:
            angle_deviation = 2 * np.pi + angle_deviation
        lane_angle = angle_deviation / np.pi

        return {"center_distance": Data(lane_dist), "relative_angle": Data(lane_angle), "is_junction": is_junction}

    def interpolate_optimal_wp(self, vehicle_loc_np, prev_wp, target_wp):
        if prev_wp is None:
            prev_wp_loc_np = vehicle_loc_np
        else:
            prev_wp_loc_np = _numpy(prev_wp.transform.location)
        target_wp_loc_np = _numpy(target_wp.transform.location)

        # Get the vector to target_wp in the coordinate system with prev_wp as origin
        opt_vec = target_wp_loc_np - prev_wp_loc_np
        opt_vec /= (np.linalg.norm(opt_vec) + 1e-6)

        # Move vehicle_loc_np to the coordinate system with prev_wp as origin
        rel_vehicle_loc = vehicle_loc_np - prev_wp_loc_np

        # https://en.wikipedia.org/wiki/Distance_from_a_point_to_a_line#Line_defined_by_two_points
        # This link shows the equation used to get the distance to the optimal line
        dist_to_optimal = abs(
            ((opt_vec[0]) * (-rel_vehicle_loc[1]) -
             (opt_vec[1]) * (-rel_vehicle_loc[0]))
            / (np.linalg.norm(opt_vec) + 1e-6)
        )
        if np.isnan(dist_to_optimal):
            print("dist is nan!")
            print("dist: ", dist_to_optimal)
            print("opt_vec: ", opt_vec)
        if np.isnan(np.sum(opt_vec)):
            print("opt_vec is nan!")
            print("dist: ", dist_to_optimal)
            print("opt_vec: ", opt_vec)

        return dist_to_optimal, opt_vec

    def project_vehicle_to_target_lane(self, vehicle_loc, target_wp, carla_map):
        """Returns the closest waypoint to the vehicle_loc on the target lane. This is not exact, it
        does its best given the limitations of the CARLA API."""
        vehicle_loc_np = _numpy(vehicle_loc)
        lane_wp = carla_map.get_waypoint(vehicle_loc)

        count = 0
        # If we are close enough to our lane, we shouldn't even enter this loop.
        while lane_wp.lane_id != target_wp.lane_id:
            count += 1
            # If the lane_wp is going the opposite direction, or is to the right of our target lane
            if (lane_wp.lane_id * target_wp.lane_id < 0
                    or abs(lane_wp.lane_id) > abs(target_wp.lane_id)):
                lane_wp = lane_wp.get_left_lane()
            # If the lane_wp is to the left of our target_lane
            elif abs(lane_wp.lane_id) < abs(target_wp.lane_id):
                lane_wp = lane_wp.get_right_lane()

            # We are failing to get the right lane for one reason or another
            if lane_wp is None or count > 20:
                target_wp_loc_np = _numpy(target_wp.transform.location)
                target_dist_np = np.linalg.norm(
                    target_wp_loc_np - vehicle_loc_np)
                lane_wp = carla_map.get_waypoint_xodr(
                    target_wp.road_id, target_wp.lane_id,
                    target_wp.s - target_dist_np)
                lane_wp_loc_np = _numpy(lane_wp.transform.location)
                if lane_wp is None or np.linalg.norm(lane_wp_loc_np - vehicle_loc_np) > 4.0:
                    # If target wp is close enough, use it.
                    if target_dist_np < 2.0:
                        lane_wp = target_wp
                    # Otherwise just get what CARLA gives you, even if it's the wrong lane.
                    else:
                        lane_wp = carla_map.get_waypoint(vehicle_loc)
                break
        return lane_wp


class LeadVehicleDistanceExtractor(DataExtractor):
    def __init__(self, name, vehicle, world, required_sensors=None):
        super().__init__(name, required_sensors=required_sensors)
        self.vehicle = vehicle
        self.world = world


class HazardStopExtractor(DataExtractor):
    pass
