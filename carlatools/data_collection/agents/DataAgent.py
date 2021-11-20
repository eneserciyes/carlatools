import yaml

from leaderboard.autoagents import autonomous_agent
from srunner.scenariomanager.carla_data_provider import CarlaDataProvider

from carlatools.data_collection.data.DataCollector import DataCollector
from .autopilot.Autopilot import Autopilot
from .autopilot.pid import PIDController

def get_entry_point():
    return "AutoDataAgent"

class AutoDataAgent(autonomous_agent.AutonomousAgent):
    def setup(self, path_to_conf_file):
        self.track = autonomous_agent.Track.SENSORS
        self.step = -1
        with open(path_to_conf_file) as stream:
            self.configs = yaml.safe_load(stream)

        self.initialized = False

        self.data_collector = DataCollector(save_dir=self.configs["save_root"])
        self.autopilot = Autopilot(PIDController(
            K_P=1.25, K_I=0.75, K_D=0.3, n=40), PIDController(K_P=5.0, K_I=0.5, K_D=1.0, n=40))

    def init(self):
        self.vehicle = CarlaDataProvider.get_hero_actor()
        self.world = self.vehicle.get_world()
        self.map = self.world.get_map()
        
        self.autopilot.init(self.vehicle, self.world, self.map)

    def sensors(self):
        autopilot_sensors = [
            {
                "type": "sensor.other.imu",
                "x": 0.0,
                "y": 0.0,
                "z": 0.0,
                "roll": 0.0,
                "pitch": 0.0,
                "yaw": 0.0,
                "sensor_tick": 0.05,
                "id": "imu"
            },
            {
                "type": "sensor.other.gnss",
                "x": 0.0,
                "y": 0.0,
                "z": 0.0,
                "roll": 0.0,
                "pitch": 0.0,
                "yaw": 0.0,
                "sensor_tick": 0.01,
                "id": "gps"
            },
            {
                "type": "sensor.speedometer",
                "reading_frequency": 20,
                "id": "speed"
            }]

        return self.data_collector.required_sensors() + autopilot_sensors

    def run_step(self, input_data, timestamp):
        if not self.initialized:
            self.init()
            self.initialized = True
        print(self.autopilot.vehicle)
        control, highlevel_command, target_speed = self.autopilot.run_step(
            input_data["gps"][1][:2], input_data['imu'][1][-1], input_data['speed'][1]["speed"])
        input_data["highlevel_command"] = highlevel_command
        input_data["target_speed"] = target_speed

        self.data_collector.tick(input_data)
        return control

    def set_global_plan(self, global_plan_gps, global_plan_world_coord):
        super().set_global_plan(global_plan_gps, global_plan_world_coord)
        self.autopilot.set_global_plan(global_plan_gps, self._global_plan)
