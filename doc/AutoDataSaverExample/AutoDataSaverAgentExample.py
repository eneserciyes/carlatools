from carlatools.data_collection.agents.DataAgent import AutoDataAgent
from carlatools.data_collection.extractors.Extractors import RGBExtractor
from carlatools.data_collection.extractors.AffordanceExtractors import TrafficLightExtractor

def get_entry_point():
    return "AutoDataSaverAgent"

class AutoDataSaverAgent(AutoDataAgent):
    def setup(self, path_to_conf_file):
        super().setup(path_to_conf_file)
        self.data_collector.add_extractor(RGBExtractor('rgbs',
                                                       {
                                                           "type": "sensor.camera.rgb",
                                                           "x": 1.3,
                                                           "y": 0.0,
                                                           "z": 1.3,
                                                           "roll": 0.0,
                                                           "pitch": 0.0,
                                                           "yaw": 0.0,
                                                           "width": 288,
                                                           "height": 288,
                                                           "fov": 90,
                                                           "id": "rgb"
                                                       }
                                                       ))
        self.data_collector.start_episode(
            'example_rgb_save', self.configs["save_root"])

    def init(self):
        super().init()
        # self.data_collector.add_extractor(TrafficLightExtractor(
        #     'traffic_light', self.vehicle, self.world, self.map))

    def destroy(self):
        self.data_collector.end_episode()
