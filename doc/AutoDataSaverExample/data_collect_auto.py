import subprocess
import os
import carlatools.server_utils.server_utils as server

if __name__ == "__main__":
    server_manager = server.CarlaServerManager("/home/fguney/CarlaUE4.sh")
    server_manager.start()

    for route in list(os.scandir("/home/fguney/projects/e2e-drive/leaderboard/data/routes_devtest")):
        print(os.path.abspath(route))
        os.environ["ROUTES"] = os.path.abspath(route)
        subprocess.call("./data_collect_auto.sh", shell=True)
