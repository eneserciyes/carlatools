from abc import ABC, abstractmethod
from pathlib import Path
from PIL import Image

class Data(ABC):
    def __init__(self, data):
        self.data = data
        self.frame_id = None # to be set when added to an episode
        pass

    @abstractmethod
    def save(self, path):
        return self.data

class Image(Data):
    def __init__(self, data, sensor_names, ext="png"):
        """
        Data type for camera data from CARLA.
        
        Params:
         - data: list of images
         - sensor_names: sensors that images in `data` list come from in the same order
         - ext: image format (png by default)
        """
        super().__init__(data)
        self.sensor_names = sensor_names
        self.ext = ext
    
    def save(self, path):
        """
        Saves images in different folders under the path root folder. Returns a dictionary of save paths.
       
        Params:
         - path: Root folder to save images
        
        Returns:
         - save_paths: Dictionary of paths where the images are saved
        """
        save_paths = {}
        for i in range(len(self.data)):
            sensor_name = self.sensor_names[i]
            data = self.data[i] 
            (path / Path(sensor_name)).mkdir(exist_ok=True)
            save_path = (path / Path(sensor_name) / f"{self.frame_id:05d}.{self.ext}").resolve()
            Image.save(data, save_path)
            save_paths[sensor_name] = save_path
        return save_paths