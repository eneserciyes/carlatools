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

        self._vehicle = CarlaDataProvider.get_hero_actor()

        self.autopilot = Autopilot(PIDController(
            K_P=1.25, K_I=0.75, K_D=0.3, n=40), PIDController(K_P=5.0, K_I=0.5, K_D=1.0, n=40))

        self.data_collector = DataCollector(save_dir=self.configs["save_root"])

    def init(self):
        pass

    def sensors(self):
        self.data_collector.required_sensors()

    def run_step(self, input_data, timestamp):
        control, highlevel_command, target_speed = self.autopilot.run_step(
            input_data)
        input_data["highlevel_command"] = highlevel_command
        input_data["target_speed"] = target_speed

        self.data_collector.tick(input_data)
        return control

    def set_global_plan(self, global_plan_gps, global_plan_world_coord):
        super().set_global_plan(global_plan_gps, global_plan_world_coord)
        self.autopilot.set_global_plan(global_plan_gps, self._global_plan)
