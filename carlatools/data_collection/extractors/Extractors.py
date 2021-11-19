from .DataExtractor import DataExtractor
from ..data.Data import Image

class RGBExtractor(DataExtractor):
    def __init__(self, name, *required_sensors, preprocess=lambda x: x[1][:, :, :3]):
        """
        Params:
         - name: DataExtractor name
         - required_sensor: sensor id that the extractor will collect
        """
        super().__init__(name, required_sensors)
        self.preprocess = preprocess
    
    def extract(self, input_data):
        return {sensor['id']: Image(self.preprocess(input_data[sensor['id']]), sensor['id']) for sensor in self.required_sensors}


        

