from DataExtractor import DataExtractor
from carlatools.data_collection.data.Data import Image

class RGBExtractor(DataExtractor):
    def __init__(self, name, required_sensors, preprocess=lambda x: x):
        """
        Params:
         - name: DataExtractor name
         - required_sensor: sensor id that the extractor will collect
        """
        super.__init__(name, required_sensors)
        self.preprocess = preprocess
    
    def extract(self, input_data):
        return Image([self.preprocess(input_data[sensor['id']]) for sensor in self.required_sensors], \
            [sensor['id'] for sensor in self.required_sensors])


        

