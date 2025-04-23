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


def test_vis_forces_dynamic():
    """
    Visualize the sensed forces dynamically.
    """
    paxini = Paxini(con_ids=[1])  # list of port id on the control box

    # Initialize the Open3D visualizer
    vis = o3d.visualization.Visualizer()
    vis.create_window()

    # paxini.vis_taxels_positions_normals()
    paxini.sensors_initialize()

    point_cloud = o3d.geometry.PointCloud()
    point_cloud.points = o3d.utility.Vector3dVector(paxini.taxel_positions)
    vis.add_geometry(point_cloud)

    # Create a list of 120 arrows with random positions and directions
    arrow_lines_list = []
    for i in range(paxini.n_taxels):
        arrow_lines = o3d.geometry.LineSet()
        start_point = end_point = paxini.taxel_positions[i]
        arrow_lines.points = o3d.utility.Vector3dVector([start_point, end_point])
        arrow_lines.lines = o3d.utility.Vector2iVector([[0, 1]])
        arrow_lines.paint_uniform_color([1.0, 0.0, 0.0])  # Set arrow shaft color to red
        arrow_lines_list.append(arrow_lines)
        vis.add_geometry(arrow_lines)

    while True:
        sensing_data = paxini.get_all_module_sensing_data()
        global_forces = paxini.transform_forces(sensing_data)
        for i in range(120):
            start_point = paxini.taxel_positions[i]
            end_point = start_point + global_forces[0, i, :]
            arrow_lines_list[i].points = o3d.utility.Vector3dVector([start_point, end_point])
            vis.update_geometry(arrow_lines_list[i])

        vis.poll_events()
        vis.update_renderer()


if __name__ == "__main__":
    test_vis_forces_dynamic()
