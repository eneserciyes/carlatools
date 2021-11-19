import subprocess
import os

if __name__ == "__main__":
    for route in list(os.scandir("/home/fguney/projects/e2e-drive/leaderboard/data/routes_devtest")):
        print(os.path.abspath(route))
        os.environ["ROUTES"] = os.path.abspath(route)
        subprocess.call("./data_collect_auto.sh", shell=True)
