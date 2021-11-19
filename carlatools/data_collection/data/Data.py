from abc import ABC, abstractmethod
from pathlib import Path
import cv2
from numpy.lib.npyio import save

class Data(ABC):
    def __init__(self, data):
        self.data = data
        self.frame_id = None # to be set when added to an episode
        pass

    @abstractmethod
    def save(self, path):
        return self.data

class Image(Data):
    def __init__(self, data, sensor_name, ext="png"):
        """
        Data type for camera data from CARLA.
        
        Params:
         - data: list of images
         - sensor_names: sensors that images in `data` list come from in the same order
         - ext: image format (png by default)
        """
        super().__init__(data)
        self.sensor_name = sensor_name
        self.ext = ext
    
    def save(self, path):
        """
        Saves image in different folder under the path root folder. Returns a dictionary of save paths.
       
        Params:
         - path: Root folder to save images
        
        Returns:
         - save_paths: Dictionary of paths where the images are saved
        """
        (path / Path(self.sensor_name)).mkdir(exist_ok=True)
        save_path = str((path / Path(self.sensor_name) / f"{self.frame_id:05d}.{self.ext}").resolve())

        cv2.imwrite(save_path, self.data)
        # return the local path for saving to csv file
        return str((Path(self.sensor_name) / f"{self.frame_id:05d}.{self.ext}").resolve())