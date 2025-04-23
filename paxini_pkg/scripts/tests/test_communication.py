import os
import sys
import time
from pathlib import Path
from typing import List

import numpy as np
import open3d as o3d
import serial
import yaml

parent_dir = str(Path(__file__).resolve().parent.parent)
sys.path.append(parent_dir)
from paxini_driver import Paxini


def test_sensing():
    """
    A simple communication test
    """
    paxini = Paxini(con_ids=[1])  # list of port id on the control box
    paxini.sensors_initialize()
    while True:
        t1 = time.time()
        sensing_data = paxini.get_all_module_sensing_data()
        print("sensing all sensors time cost: ", time.time() - t1)
        print(sensing_data.mean(axis=1))


if __name__ == "__main__":
    test_sensing()
