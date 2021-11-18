import subprocess
import os

if __name__ == "__main__":
    for route in list(os.scandir("/home/enes/avg/e2e-drive/leaderboard/data/routes_devtest")):
        print(os.path.abspath(route))
        os.environ["ROUTES"] = os.path.abspath(route)
        subprocess.call("/home/enes/avg/carlatools/carlatools/doc/data_collect_auto.sh", shell=True)