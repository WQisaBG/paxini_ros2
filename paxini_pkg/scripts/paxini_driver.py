import os
import time
from typing import List

import numpy as np
import open3d as o3d
import serial
import yaml


def extract_low_high_byte(d):
    """
    Args:
        d: decimal_number
    """
    high_byte = (d >> 8) & 0xFF
    low_byte = d & 0xFF
    return [low_byte, high_byte]


def decimal_to_hex(d):
    h = d & 0xFF
    return h


class Paxini:
    def __init__(self, con_ids: List[int]):
        # ---------- hyper-parameters ----------
        self.config_file = os.path.join(
            os.path.dirname(os.path.dirname(os.path.abspath(__file__))), "configs/configs.yaml"
        )
        self.n_taxels = 120     # 单个的传感器数量 
        self.n_modules = len(con_ids)   # 模块数量
        # ------------------------------

        print("Please execute: 'sudo chmod 666 /dev/ttyACM0'")

        self.ser = serial.Serial("/dev/ttyACM0", baudrate=460800, timeout=1.0)     # 串口
        self.control_box_mode = 0x05  # GEN2-DP-M2826's mode is 5  # GEN2-DP-M2826's mode is 5
        self.con_ids = con_ids  # thumb, index, middle, ring

        # load the config yaml file
        with open(self.config_file, "r") as file:
            self.configs = yaml.safe_load(file)
        self.taxel_positions, self.taxel_normals = self._calculate_taxel_normals()

    def _calculate_taxel_normals(self):
        taxel_positions = np.asarray(self.configs["taxel_positions"])
        point_cloud = o3d.geometry.PointCloud()
        point_cloud.points = o3d.utility.Vector3dVector(taxel_positions)
        k_neighbors = 8
        point_cloud.estimate_normals(search_param=o3d.geometry.KDTreeSearchParamKNN(knn=k_neighbors))
        point_cloud.orient_normals_consistent_tangent_plane(k_neighbors)
        taxel_normals = np.asarray(point_cloud.normals)
        taxel_normals = taxel_normals / np.linalg.norm(taxel_normals, axis=-1).reshape(self.n_taxels, 1)
        return taxel_positions, taxel_normals

    def sensors_initialize(self):
        control_box_version = self.get_control_box_version()
        print("Received control box version: ", control_box_version)
        _ = self.set_control_box_mode()
        control_box_mode = self.get_control_box_mode()
        print("Received control box mode: ", control_box_mode)
        _ = self.recalibrate_all_module()
        print("Paxini sensors initialization done.")

    def calculate_lrc(self, data):
        checksum = sum(data) & 0xFF
        lrc = (~checksum + 1) & 0xFF
        return lrc

    def build_protocol(self, fix_id, index, main_cmd, sub_cmd, length, data):
        head = [0x55, 0xAA, 0x7B, 0x7B]
        tail = [0x55, 0xAA, 0x7D, 0x7D]

        lrc_packet = [fix_id, index, main_cmd] + sub_cmd + length + data
        lrc = self.calculate_lrc(lrc_packet)
        packet = head + [fix_id, index, main_cmd] + sub_cmd + length + data + [lrc]
        packet += tail
        return bytes(packet)

    def check_response(self, res):
        if len(res) == 0:
            raise ValueError("Empty response.")
        if self.extract_error(res) != 0:
            raise ValueError(f"Reponse error code: {hex(self.extract_error(res))}")
        return True

    def extract_data(self, res):
        return res[12:-5]

    def extract_error(self, res):
        return res[9]

    def get_control_box_version(self):
        """
        Return the version of the module control box.
        """
        fix_id = 0x0E
        index = 0x00
        main_cmd = 0x60
        sub_cmd = [0xA0, 0x01]
        length = [0x00, 0x00]
        data = []
        packet = self.build_protocol(fix_id, index, main_cmd, sub_cmd, length, data)
        self.ser.write(packet)
        time.sleep(1e-2)  # necessary

        response = self.ser.read(self.ser.in_waiting)  # read all waiting data
        self.check_response(response)
        return self.extract_data(response).decode("utf-8", errors="ignore")

    def set_control_box_mode(self):
        """
        Set the mode of the module control box
        """
        fix_id = 0x0E
        index = 0x00
        main_cmd = 0x70
        sub_cmd = [0xC0, 0x0C]
        length = [0x01, 0x00]
        data = [self.control_box_mode]
        packet = self.build_protocol(fix_id, index, main_cmd, sub_cmd, length, data)
        self.ser.write(packet)

        response = self.ser.read(17)
        success = self.check_response(response)
        print(f"Set control box mode: {success}")
        return success

    def get_control_box_mode(self):
        """
        Return the mode of the module control box
        """
        fix_id = 0x0E
        index = 0x00
        main_cmd = 0x70
        sub_cmd = [0xC0, 0x0D]
        length = [0x00, 0x00]
        data = []
        packet = self.build_protocol(fix_id, index, main_cmd, sub_cmd, length, data)
        self.ser.write(packet)

        response = self.ser.read(17 + 1)
        self.check_response(response)
        return self.extract_data(response).hex()

    def set_module_port(self, con_id):
        """
        Args:
            con_id: con id on the control box
        """
        fix_id = 0x0E
        index = 0x00
        main_cmd = 0x70
        sub_cmd = [0xB1, 0x0A]
        length = [0x01, 0x00]
        data = [decimal_to_hex(con_id * 3 - 1)]
        packet = self.build_protocol(fix_id, index, main_cmd, sub_cmd, length, data)
        self.ser.write(packet)

        response = self.ser.read(17)
        success = self.check_response(response)
        # print(f"Set module port. con_id: {con_id}, success: {success}")
        return success

    def recalibrate_one_module(self):
        fix_id = 0x0E
        index = 0x00
        main_cmd = 0x70
        sub_cmd = [0xB0, 0x02]
        length = [0x02, 0x00]
        data = [0x03, 0x01]
        packet = self.build_protocol(fix_id, index, main_cmd, sub_cmd, length, data)
        self.ser.write(packet)

        response = self.ser.read(17 + 1)
        success = len(response) != 0 and response[9] == 6  # The error code in reponse is wrong.
        # print(f"Recaliberate the sensors. {success}")
        return success

    def recalibrate_all_module(self):
        for i, con_id in enumerate(self.con_ids):
            success = self.set_module_port(con_id)
            self.recalibrate_one_module()
            print(f"Recalibrate sensors at CON {con_id}. Success: {success}")

    def get_one_module_sensing_data(self):
        """
        Return:
            sensing_data_array: shape (120 ,3)
        """
        fix_id = 0x0E
        index = 0x00
        main_cmd = 0x70
        sub_cmd = [0xC0, 0x06]
        length = [0x05, 0x00]
        addr_begin = 1038
        addr_end = 1397
        num_bytes = addr_end - addr_begin + 1
        data = [0x7B] + extract_low_high_byte(addr_begin) + extract_low_high_byte(num_bytes)
        packet = self.build_protocol(fix_id, index, main_cmd, sub_cmd, length, data)
        self.ser.write(packet)    # 写指令

        response = self.ser.read(17 + 6 + num_bytes)    #读读数据
        self.check_response(response)

        data = self.extract_data(response)
        sensing_data = data[6:]
        # 重塑为120行3列的矩阵，表示120个触觉单元的3D力值（x, y, z）。
        sensing_data_array = np.asarray(list(map(int, sensing_data))).reshape(120, 3)

        # Data of x/y axis is two's complement. Convert them from 0~255 to -128~127
        xy = sensing_data_array[:, 0:2]    #硬件返回的x/y轴数据为单字节（0-255），需转换为有符号整数（-128~127）。
        xy = np.where(xy >= 128, xy - 256, xy)
        sensing_data_array[:, 0:2] = xy   
        return sensing_data_array    #确保x/y轴数据范围正确，避免数值溢出。

    def get_all_module_sensing_data(self):
        """
        Return:
            sensing_data_all: local forces; shape (n_modules, n_taxels, 3)
        """
        sensing_data_all = np.zeros((len(self.con_ids), 120, 3))
        for i, con_id in enumerate(self.con_ids):
            self.set_module_port(con_id)
            sensing_data_all[i, ...] = self.get_one_module_sensing_data()
        return sensing_data_all

    def vis_taxels_positions_normals(self):
        taxel_positions, taxel_normals = self._calculate_taxel_normals()
        point_cloud = o3d.geometry.PointCloud()
        point_cloud.points = o3d.utility.Vector3dVector(taxel_positions)
        point_cloud.normals = o3d.utility.Vector3dVector(taxel_normals)
        o3d.visualization.draw_geometries([point_cloud], point_show_normal=True)

    def transform_forces(self, local_forces):
        """
        Transform taxel local forces (defined in the taxel frame) to global forces (defined in the sensor frame).
        Args:
            local_forces: shape (n_modules, n_taxels, 3)
        Return:
            global_forces: shape (n_modules, n_taxels, 3)
        """

        print("Warning: currently only consider the local normal forces and ignore the tangential forces.")
        local_normal_force = local_forces[..., 2].reshape(-1, 1)  # shape (n_m * n_t, 1)
        global_forces = local_normal_force * np.tile(self.taxel_normals, (self.n_modules, 1))
        return global_forces.reshape(self.n_modules, self.n_taxels, 3)


def test_sensing():
    """
    A simple communication test
    """
    paxini = Paxini(con_ids=[1])  # list of port id on the control box
    paxini.sensors_initialize()
    while True:
        t1 = time.time()
        sensing_data = paxini.get_all_module_sensing_data()
        print("Sensing all sensors time cost: ", time.time() - t1)
        print("Average sensing forces on each module:", sensing_data.mean(axis=1))


if __name__ == "__main__":
    test_sensing()
