import warnings
from .Episode import Episode

class DataCollector:
    """
    Container class for data extractors. Signals extractors at each tick and returns a dictionary of ground
    truth information.
    """

    def __init__(self, *args, save_dir, fps_divisor=1):
        self.save_dir = save_dir
        self.extractors = args
        self.episodes = []
        self.episode_count = 0
        self.active_episode = None

        self.fps_divisor = fps_divisor
        self.tick_counter = 0

    def add_extractor(self, *extractors):
        self.extractors += extractors

    def remove_extractor(self, data_extractor):
        self.extractors.remove(data_extractor)
    
    def required_sensors(self):
        """
        Return the list of required sensors from extractors.
        """
        return [item for sublist in \
            list(filter(lambda x: x is not None, [extractor.required_sensors for extractor in self.extractors])) \
            for item in sublist]

    # def required_sensors(self, sensors_json):
    #     """
    #     Read required sensors from a json file.
    #     Params:
    #      - sensors_json: path to the sensor json file
    #     """
    #     import json
    #     with open(sensors_json, 'r') as sensor_file:
    #         return json.load(sensor_file)

    def start_episode(self, episode_name=None, save_dir=None):
        """
        Starts a data collection episode. 
        """
        if self.active_episode is not None \
                and self.active_episode.is_open():
            raise Exception(
                "Active sequence must be closed before another sequence is started.")

        episode_name = episode_name if episode_name is not None else "Episode" + \
            self.episode_count
        # save to the data collector directory if not specified elsewhere
        save_dir = save_dir if save_dir is not None else self.save_dir

        self.active_episode = Episode(episode_name, save_dir)
        self.active_episode.open()

        self.episode_count += 1

    def end_episode(self):
        """
        Closes the active sequence and writes the collected data to disk. 
        """
        if not self.active_episode.is_open() or self.active_episode is None:
            warnings.warn("There is no active sequence, doing nothing.")

        self.active_episode.save_to_disk()
        self.active_episode.close()

    # def save(self):
    #     if self.active_episode.is_open():
    #         raise Exception(
    #             "Active sequence must be closed before saving the data to the disk.")
    #     for sequence in self.episodes:
    #         sequence.save_to_disk(self.save_dir)

    def tick(self, input_data):
        """
        Method to be called in each tick of the agent.
        If there is an active episode, adds the data to the episode.

        Params:
         - input_data: sensor data provided in tick method of the agent.

        Returns:
         - data_dict: a dictionary of data with keys as ids of data extractors.
        """
        self.tick_counter+=1
        if self.tick_counter % self.fps_divisor == 0:
            self.tick_counter = 0

            data_dict = {}

            for extractor in self.extractors:
                extracted_data = extractor.extract(input_data)
                if type(extracted_data) is dict:
                    data_dict.update(extracted_data)
                else:
                    data_dict[extractor.name] = extracted_data

            if self.active_episode is not None \
                    and self.active_episode.is_open():
                # if there is an active episode, add data to episode.
                self.active_episode.add_data(data_dict)

            return data_dict

    def visualize(self, data_dict):
        pass

