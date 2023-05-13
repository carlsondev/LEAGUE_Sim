import dronekit as dk
import numpy as np
from typing import Dict, List, Any, Optional, Callable
from pymavlink.dialects.v20.ardupilotmega import MAVLink_message
from custom_battery import CustomBattery
from enum import Enum
import matplotlib
from matplotlib.axes import Axes
from matplotlib import animation
import os, signal
import time

#matplotlib.use('TkAgg')
# HERELINK_TELEM

class OffloadingMethod(Enum):
    NONE = "none"
    ONBOARD = "onboard"
    PARTIAL_OFFLOAD = "partial"
    FULL_OFFLOAD = "full"


class EnergyVehicle(dk.Vehicle):



    def __init__(self, *args):
        super(EnergyVehicle, self).__init__(*args)

        self.messages_dict : Dict[str, List[MAVLink_message]] = {}
        self.queue_method : Optional[Callable[[List[float], List[float]]]] = None

        # SIM_BATTERY_VOLTAGE
        # TODO: Update these regularly
        self._avg_battery_voltage = 12.6
        self._custom_battery : Optional[CustomBattery] = None

        self._last_sample_time : int = 0

        self._offloading_method = OffloadingMethod.NONE
        self._video_data_pairs : List[Dict[str, Any]] = []

        self._curr_pair_idx = 0
        self._curr_bin_idx = 0

        self._graph_battery_percents : List[float] = []
        self._graph_cpu_utils : List[float] = []
        
        @self.on_message('*')
        def listener(self, name, message : MAVLink_message):
            msg_type : str = message.get_type()
            #print(f"Message: {msg_type}")
            self.messages_dict[msg_type] = self.messages_dict.get(msg_type, []) + [message]

        #Expected to send 1 time per second (1 Hz), should work either way
        @self.on_message('HEARTBEAT')
        def beat_heart(self, name : str,  message : MAVLink_message):
            
            self.sample_battery()

            self._last_sample_time = time.time()

        @self.on_attribute('battery')
        def main_battery_callback(self, attr_name : str, value : dk.Battery):
            if self._custom_battery is None:
                # 4,000 mAh default capacity
                self._custom_battery = CustomBattery(value, 4000)

            self._custom_battery.update(value)

            if len(self._custom_battery.pairs_lin_reg_params) == 0:
                self._custom_battery.pairs_lin_reg_params = [pair_dict["regression"] for pair_dict in self._video_data_pairs]

        @self.parameters.on_attribute('BATT_CAPACITY')
        def update_battery_capacity(self, attr_name, value):
            if self._vehicle._custom_battery is None:
                return
            self._vehicle._custom_battery.update_cap_mah(value)

    def set_sim_data(self, data : Dict[str, Any], offloading_method : OffloadingMethod, drone_idx : int):
        self.offloading_method = offloading_method
        self.drone_idx = drone_idx

        # Includes all data for all videos
        for key, value in data.items():
            self._video_data_pairs.append(value)

        if self._custom_battery is not None:
            self._custom_battery.pairs_lin_reg_params = [pair_dict["regression"] for pair_dict in self._video_data_pairs]

        

    def print_msg_dict(self, full : bool):
        for msg_type,msg_list in self.messages_dict.items():
            print(f"{msg_type} ({len(msg_list)}):")
            if not full:
                continue
            for msg in msg_list:
                print(f"    {msg}")

    def get_current_cpu_util(self) -> float:

        try:
            curr_pair_dict = self._video_data_pairs[self._curr_pair_idx]
        except IndexError:
            return 0
        
        curr_bin_ordering = curr_pair_dict["bin_ordering"]
        curr_bins =  curr_pair_dict["cpu_bins"]

        if len(curr_bins) == 0 or len(curr_bin_ordering) == 0:
            return 0

        # Get Bin for time
        

        current_cpu_bin = curr_bin_ordering[self._curr_bin_idx]

        curr_bin_data = curr_bins[str(current_cpu_bin)]
        rand_cpu_util = np.random.normal(curr_bin_data["mean"], curr_bin_data["std"])
        if rand_cpu_util < 0:
            rand_cpu_util = 0
        if rand_cpu_util > 100:
            rand_cpu_util = 100
        
        return rand_cpu_util


    def sample_battery(self):

        if self.battery is None:
            return

        self._custom_battery.update(self.battery)

        if len(self._video_data_pairs) == 0:
            return
        
        J_s_delta : float = 0

        curr_cpu_util = self.get_current_cpu_util()
        

        J_s_util = self._custom_battery.get_js_for_util(curr_cpu_util, self._curr_pair_idx)

        J_s_delta += J_s_util

        J_delta = J_s_delta * (time.time() - self._last_sample_time)

        self._custom_battery.update_cap_j(self._custom_battery.capacity_J - J_delta)
        capacity_percent = self._custom_battery.get_capacity_percentage() * 100

        print(f"Battery decreased {J_delta:.2f}J to {self._custom_battery.capacity_J:.2f}J ({capacity_percent:.2f}%)")

        if capacity_percent <= 0:
            print("Battery is dead, ending simulation")
            os.kill(os.getpid(), signal.SIGUSR1)
        self._graph_battery_percents.append(capacity_percent)
        self._graph_cpu_utils.append(curr_cpu_util)

        if self.queue_method is not None:
            self.queue_method(np.array(self._graph_battery_percents), np.array(self._graph_cpu_utils))

        # Cycles bins and (if necsessary) cycle pairs
        self._curr_bin_idx += 1
        curr_pair_dict = self._video_data_pairs[self._curr_pair_idx]
        if self._curr_bin_idx >= len(curr_pair_dict["bin_ordering"]):
            self._curr_bin_idx = 0
            self._curr_pair_idx += 1
            if self._curr_pair_idx >= len(self._video_data_pairs):
                self._curr_pair_idx = 0