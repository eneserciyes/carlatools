from abc import ABC, abstractmethod

class DataExtractor(ABC):
    def __init__(self, name, required_sensors=None):
        """
        Params:
         - name: a unique string defining the ground truth that will be extracted.
        """
        self.name = name
        self.required_sensors = required_sensors
    
    @abstractmethod
    def extract(self, input_data):
        """
        Returns the data to be used. If a dictionary is returned, data collector does not keep
        a nested dictionary but merges it with others.
        """
        pass
