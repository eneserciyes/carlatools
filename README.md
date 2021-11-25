# carlatools

carlatools is an extensible library for CARLA Simulator utilities.


## Why carlatools?

CARLA Simulator is the most widely used simulator in autonomous driving research. NoCrash or the recent CARLA Leaderboard are the standard benchmarks in this area. I had used CARLA for SPARK for a while in the past, however, we switched to SVL Simulator later. Lately, I had the chance to use the myriad APIs and features CARLA offers in my research in end-to-end autonomous driving at AVG. CARLA provides a rich suite of sensors, ground truth information and a rule-based Autopilot with access to privileged information. However, there are three problems I encountered working with CARLA:

1. No recent tool for data collection. (data_collector from CARLA is for version 0.8.4)
2. Inextensible interface when you try to extract more than CARLA offers
3. Diverse methods of collecting data across papers.

So, I decided to brush up on my design pattern knowledge from software engineering courses to build a lightweight tool library with extensibility in mind. Main intention of this library is to provide a clean and flexible structure that can be easily extended by inheriting base classes for the most common tasks in CARLA - data collection, RL environment setup, etc. carlatools currently provides an extensible data collection structure along with the most common ground truth data extractors including some "affordances" like traffic light existence and centerline distance. It also offers a base DataSaverAgent class that collects data with CARLA Autopilot. 

## AutoRGBDataSaver: A Demo

The most basic data that one can collect from CARLA is RGBCamera data. The following is a demo of creating an agent that automatically collects RGB camera data using [CARLA Leaderboard](https://leaderboard.carla.org/get_started/). 

#### Importing AutoDataAgent base class and RGBExtractor
```python
from carlatools.data_collection.agents.DataAgent import AutoDataAgent
from carlatools.data_collection.extractors.Extractors import RGBExtractor
```

#### Creating an agent and adding the RGBExtractor

```python
class AutoDataSaverAgent(AutoDataAgent):
    def setup(self, path_to_conf_file):
        super().setup(path_to_conf_file)
```
When initiated, `AutoDataSaverAgent` has an empty `data_collector` field. Adding the RGBExtractor to the data_collector.

```python
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

```

RGBExtractor accepts a name and the list of cameras as parameters. RGBExtractor also has an optional preprocess parameter that is applied to raw sensor data prior to any other function.

#### Starting the episode to save data

DataCollector is a general purpose data extractor that does not save the data to disk, yet return it in each `tick` in default mode. To save data, DataCollector needs to have an active episode. Starting the episode.

```python
self.data_collector.start_episode(
            'example_rgb_save', self.configs["save_root"])
```

`start_episode()` accepts an episode name and save root as parameters.

To save to disk in default mode, episode must be ended. Episode can be ended when agent is destroyed by overriding the `destroy()` method.

```python
def destroy(self):
        self.data_collector.end_episode()
```

That's it! In less than 20 lines, carlatools allows one to create an autonomous data collector agent. This gets even better with all the complex affordance extractors provided in the `data_collection` module.

## Design
### Observer Pattern
Due to the one to many dependence between data extraction processes and CARLA Python API, data_collection module embraces the Observer pattern at its core. Each DataExtractor holds the data extraction logic encapsulated and exposes the data it extracts through the extract method. By subscribing to the DataCollector object, data extractors do their job at every tick independent of other extractors. 

### Extensibility

This design grants easier extensibility and code reuse as extractors become independent plug-and-play modules. When another data is needed from the simulator, one only needs to inherit from the DataExtractor base class and add it to the DataCollector instance of the agent. 

## Future Work

### data_collection
* Enhance DataExtractor library with all sensors, affordances from recent autonomous driving papers and BEV ground truths.
* Create a generic visualizer for visualizing the collected data online and offline. 
* Create DataLoader factories for PyTorch and TensorFlow. 
* Add the ability to scale data collection to many CARLA instances. 

### RL
After data_collection module, I aim to solve CARLA's excruciating RL setup process by integrating CARLA with OpenAI Gym. 
## Setup

You can install carlatools library using pip.

```shell
git clone https://github.com/eneserciyes/carlatools
pip install -e carlatools
```