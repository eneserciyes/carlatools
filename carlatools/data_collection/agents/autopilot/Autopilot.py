import numpy as np
import os

from carla import Color, Location, VehicleControl
import carla.libcarla
from .RoutePlanner import RoutePlanner

try:
    DEBUG = int(os.environ["DEBUG"]) == 1
except:
    DEBUG = False

def get_collision(p1, v1, p2, v2):
    A = np.stack([v1, -v2], 1)
    b = p2 - p1

    if abs(np.linalg.det(A)) < 1e-3:
        return False, None

    x = np.linalg.solve(A, b)
    collides = all(x >= 0) and all(x <= 1)
    return collides, p1 + x[0] * v1


def _numpy(carla_vector, normalize=False):
    result = np.float32([carla_vector.x, carla_vector.y])

    if normalize:
        return result / (np.linalg.norm(result) + 1e-4)

    return result


def _location(x, y, z):
    return Location(x=float(x), y=float(y), z=float(z))


def _orientation(yaw):
    return np.float32([np.cos(np.radians(yaw)), np.sin(np.radians(yaw))])

class Autopilot:
    def __init__(self, steer_controller, speed_controller):
        """
        Creates route planners and controllers for autopilot functions 

        Params:
         - steer_controller: PID controller for calculating steer command
         - speed_controller: PID controller for calculating throttle and brake command

        """
        self.steer_controller = steer_controller
        self.speed_controller = speed_controller  
        self.waypoint_planner = RoutePlanner(4.0, 50)
        self.command_planner = RoutePlanner(7.5, 25.0, 257)
        self._traffic_lights = list()
        self.waypoint_idx = 1 # starting from 1 to be able to interpolate to prev_waypoint

    def init(self, vehicle, world, map):
        """
        Initialize when the agent's run_step is first called
        """
        self.vehicle = vehicle
        self.world = world
        self.open_drive_map = map

    def set_global_plan(self,global_plan_gps, global_plan):
        self.waypoint_planner.set_route(global_plan_gps, True)
        self.command_planner.set_route(global_plan, True)
        self._global_plan = global_plan

    def run_step(self, input_gps, input_compass, input_speed):
        """
        Gets the input gps, compass and speed data. Using CARLA Autopilot functions, 
        returns a control output, highlevel command and target speed.  
        """
        self._traffic_lights = self.get_nearby_lights(self.vehicle, self.world.get_actors().filter("*traffic_light*"))
        self.update_plan(input_gps, input_speed)
        gps = self._get_position(input_gps)
        near_node, highlevel_command = self.waypoint_planner.run_step(gps)
        far_node, _ = self.command_planner.run_step(gps)

        steer, throttle, brake, target_speed = self._get_control(
            near_node, far_node, gps, input_compass, input_speed)

        control = VehicleControl()
        control.steer = steer + 1e-2 * np.random.randn()
        control.throttle = throttle
        control.brake = float(brake)

        return control, highlevel_command, target_speed
    
    def get_nearby_lights(self, vehicle, lights, pixels_per_meter=5.5, size=512, radius=5):
        """
        Return the list of nearby traffic lights.
        Taken from carla_project by Brady Zhou: 
        https://github.com/bradyz/carla_project/blob/ac791fcf7e59ad80b6908dadc00eb4f26147c065/src/carla_env.py
        """
        result = list()
        transform = vehicle.get_transform()
        pos = transform.location
        theta = np.radians(90 + transform.rotation.yaw)
        R = np.array([
            [np.cos(theta), -np.sin(theta)],
            [np.sin(theta),  np.cos(theta)],
            ])

        for light in lights:
            delta = light.get_transform().location - pos

            target = R.T.dot([delta.x, delta.y])
            target *= pixels_per_meter
            target += size // 2

            if min(target) < 0 or max(target) >= size:
                continue

            trigger = light.trigger_volume
            light.get_transform().transform(trigger.location)
            dist = trigger.location.distance(vehicle.get_location())
            a = np.sqrt(
                    trigger.extent.x ** 2 +
                    trigger.extent.y ** 2 +
                    trigger.extent.z ** 2)
            b = np.sqrt(
                    vehicle.bounding_box.extent.x ** 2 +
                    vehicle.bounding_box.extent.y ** 2 +
                    vehicle.bounding_box.extent.z ** 2)

            if dist > a + b:
                continue

            result.append(light)

        return result
    
    def _get_angle_to(self, pos, theta, target):
        R = np.array([
            [np.cos(theta), -np.sin(theta)],
            [np.sin(theta), np.cos(theta)],
        ])

        aim = R.T.dot(target - pos)
        angle = -np.degrees(np.arctan2(-aim[1], aim[0]))
        angle = 0.0 if np.isnan(angle) else angle

        return angle

    def update_plan(self, gps, speed):
        lookahead_distance = 0.5e-5 + (4.5e-5 * (speed/7.1))

        current_dist = np.linalg.norm(np.array([
                self._global_plan[self.waypoint_idx][0]["lat"] - gps[0],
                self._global_plan[self.waypoint_idx][0]["lon"] - gps[1]]))
        lookahead_idx = self.waypoint_idx
        for i in range(self.waypoint_idx + 1, len(self._global_plan)):
            if current_dist >= lookahead_distance:
                break
            current_dist += np.linalg.norm(np.array([
                self._global_plan[i][0]["lat"] - self._global_plan[i-1][0]["lat"],
                self._global_plan[i][0]["lon"] - self._global_plan[i-1][0]["lon"]]))
            lookahead_idx = i
        self.waypoint_idx = lookahead_idx
    
    def _get_position(self, gps):
        return (gps - self.command_planner.mean) * self.command_planner.scale
    
    def _get_control(self, target, far_target, pos, theta, speed):
            # Steering.
            angle_unnorm = self._get_angle_to(pos, theta, target)
            angle = angle_unnorm / 90

            steer = self.steer_controller.step(angle)
            steer = np.clip(steer, -1.0, 1.0)
            steer = round(steer, 3)

            # Acceleration.
            angle_far_unnorm = self._get_angle_to(pos, theta, far_target)
            should_slow = abs(angle_far_unnorm) > 45.0 or abs(angle_unnorm) > 5.0
            target_speed = 4 if should_slow else 7.0

            brake = self._should_brake()
            target_speed = target_speed if not brake else 0.0

            delta = np.clip(target_speed - speed, 0.0, 0.25)
            throttle = self.speed_controller.step(delta)
            throttle = np.clip(throttle, 0.0, 0.75)

            if brake:
                steer *= 0.5
                throttle = 0.0

            return steer, throttle, brake, target_speed

    def _should_brake(self):
        actors = self.world.get_actors()

        vehicle = self._is_vehicle_hazard(actors.filter("*vehicle*"))
        light = self._is_light_red(actors.filter("*traffic_light*"))
        walker = self._is_walker_hazard(actors.filter("*walker*"))

        return any(x is not None for x in [vehicle, light, walker])

    def _draw_line(self, p, v, z, color=(255, 0, 0)):
        if not DEBUG:
            return

        p1 = _location(p[0], p[1], z)
        p2 = _location(p[0] + v[0], p[1] + v[1], z)
        color = Color(*color)

        self.world.debug.draw_line(p1, p2, 0.25, color, 0.01)

    def _is_light_red(self, lights_list):
        if self.vehicle.get_traffic_light_state() != carla.libcarla.TrafficLightState.Green:
            affecting = self.vehicle.get_traffic_light()

            for light in self._traffic_lights:
                if light.id == affecting.id:
                    return affecting

        return None

    def _is_walker_hazard(self, walkers_list):
        z = self.vehicle.get_location().z
        p1 = _numpy(self.vehicle.get_location())
        v1 = 10.0 * _orientation(self.vehicle.get_transform().rotation.yaw)

        self._draw_line(p1, v1, z + 2.5, (0, 0, 255))

        for walker in walkers_list:
            v2_hat = _orientation(walker.get_transform().rotation.yaw)
            s2 = np.linalg.norm(_numpy(walker.get_velocity()))

            if s2 < 0.05:
                v2_hat *= s2

            p2 = -3.0 * v2_hat + _numpy(walker.get_location())
            v2 = 8.0 * v2_hat

            self._draw_line(p2, v2, z + 2.5)

            collides, _ = get_collision(p1, v1, p2, v2)

            if collides:
                return walker

        return None

    def _is_vehicle_hazard(self, vehicle_list):
        z = self.vehicle.get_location().z

        o1 = _orientation(self.vehicle.get_transform().rotation.yaw)
        p1 = _numpy(self.vehicle.get_location())
        s1 = max(7.5, 2.0 * np.linalg.norm(_numpy(self.vehicle.get_velocity())))
        v1_hat = o1
        v1 = s1 * v1_hat

        self._draw_line(p1, v1, z + 2.5, (255, 0, 0))

        for target_vehicle in vehicle_list:
            if target_vehicle.id == self.vehicle.id:
                continue

            o2 = _orientation(target_vehicle.get_transform().rotation.yaw)
            p2 = _numpy(target_vehicle.get_location())
            s2 = max(
                5.0, 2.0 * np.linalg.norm(_numpy(target_vehicle.get_velocity())))
            v2_hat = o2
            v2 = s2 * v2_hat

            p2_p1 = p2 - p1
            distance = np.linalg.norm(p2_p1)
            p2_p1_hat = p2_p1 / (distance + 1e-4)

            self._draw_line(p2, v2, z + 2.5, (255, 0, 0))

            angle_to_car = np.degrees(np.arccos(v1_hat.dot(p2_p1_hat)))
            angle_between_heading = np.degrees(np.arccos(o1.dot(o2)))

            if angle_between_heading > 60.0 and not (angle_to_car < 15 and distance < s1):
                continue
            elif angle_to_car > 30.0:
                continue
            elif distance > s1:
                continue

            return target_vehicle

        return None
