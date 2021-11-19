from pathlib import Path
import pandas as pd

class Episode:
    def __init__(self, name, save_dir):
        self.name = name
        self.data = []
        self.save_dir = save_dir
        self.opened = False
    
    def open(self):
        self.opened = True
    def close(self):
        self.opened = False
    def is_open(self):
        return self.opened
    
    def add_data(self, data_dict):
        for keys in data_dict:
            data_dict[keys].frame_id = len(self.data)
        self.data.append(data_dict)
    
    def save_to_disk(self):
        #TODO: dont forget to change exist_ok
        (Path(self.save_dir) / Path(self.name)).mkdir(exist_ok=True) # create a folder in root dir
        save_data = [{k:v.save(Path(self.save_dir) / Path(self.name)) for k,v in data_dict.items()} for data_dict in self.data]
        df = pd.DataFrame(save_data)
        print(df['rgb'])
        with open(str((Path(self.save_dir) / Path(self.name) / Path(self.name + ".csv")).resolve()), 'w') as csvfile:
            df.to_csv(csvfile)

    def visualize(self):
        #TODO: implement a visualizer that shows all data in the episode beautifully
        pass