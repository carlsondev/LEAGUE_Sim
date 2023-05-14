import dronekit as dk
import numpy as np
from typing import Dict, List, Any, Optional, Callable
from pymavlink.dialects.v20.ardupilotmega import MAVLink_message
from custom_battery import CustomBattery
from enum import Enum
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
    '''
    Vehicle class with CPU Workload Energy Simulation Capabilities
    '''


    def __init__(self, *args):
        super(EnergyVehicle, self).__init__(*args)

        # Collected MAVLink Messages
        self.messages_dict : Dict[str, List[MAVLink_message]] = {}

        # Method to queue the generated CPU and battery data
        self.queue_method : Optional[Callable[[List[float], List[float]]]] = None

        # Custom Battery object
        self._custom_battery : Optional[CustomBattery] = None

        # Last time (in seconds) the workload was sampled
        self._last_sample_time : int = 0


        self._curr_pair_idx = 0
        self._curr_bin_idx = 0
        self._video_data_pairs : List[Dict[str, Any]] = []


        # Generated data for graphing
        self._graph_battery_percents : List[float] = []
        self._graph_cpu_utils : List[float] = []
        

        # Message listener. Collects all messages for future analysis
        @self.on_message('*')
        def listener(self, name, message : MAVLink_message):
            msg_type : str = message.get_type()
            self.messages_dict[msg_type] = self.messages_dict.get(msg_type, []) + [message]


        @self.on_message('HEARTBEAT')
        def beat_heart(self, name : str,  message : MAVLink_message):
            '''
            HEARTBEAT message listener. Used to sample the CPU workload and battery.
            Expected to send 1 time per second (1 Hz), should work either way
            '''
            self.sample_battery()

            self._last_sample_time = time.time()

        @self.on_attribute('battery')
        def main_battery_callback(self, attr_name : str, value : dk.Battery):
            '''
            Drone battery updated callback. Used to update the custom battery object with external battery data
            '''
            if self._custom_battery is None:
                # 1,000 mAh default capacity
                self._custom_battery = CustomBattery(value, 1000)

            # Update battery with new data
            self._custom_battery.update(value)

            # If linear regression data has not been set, set it
            if len(self._custom_battery.pairs_lin_reg_params) == 0:
                self._custom_battery.pairs_lin_reg_params = [pair_dict["regression"] for pair_dict in self._video_data_pairs]

        @self.parameters.on_attribute('BATT_CAPACITY')
        def update_battery_capacity(self, attr_name, value):
            '''
            BATT_CAPACITY MAVLink message callback. Updates custom battery capacity with new value
            '''

            if self._vehicle._custom_battery is None:
                return
            self._vehicle._custom_battery.update_cap_mah(value)


    def set_sim_data(self, data : Dict[str, Any], offloading_method : OffloadingMethod, drone_idx : int):
        '''
        Sets the simulation data for the vehicle from JSON.

        :param data: JSON data from the simulation
        :type: Dict[str, Any]
        :param offloading_method: Offloading method used for the simulation
        :type: OffloadingMethod
        :param drone_idx: Index of the drone in the simulation
        :type: int
        '''

        self.offloading_method = offloading_method
        self.drone_idx = drone_idx

        # Includes all data for all videos
        for key, value in data.items():
            self._video_data_pairs.append(value)

        if self._custom_battery is not None:
            self._custom_battery.pairs_lin_reg_params = [pair_dict["regression"] for pair_dict in self._video_data_pairs]

        

    def print_msg_dict(self, full : bool):
        '''
        Print collected messages, either full or just the type and length
        
        :param full: Whether to print the full message list or just the type and length
        :type: bool
        '''
        for msg_type,msg_list in self.messages_dict.items():
            print(f"{msg_type} ({len(msg_list)}):")
            if not full:
                continue
            for msg in msg_list:
                print(f"    {msg}")

    def get_current_cpu_util(self) -> float:
        '''
        Generates a CPU utilization for the current time based on the current CPU bin

        :return: Generated CPU utilization
        :rtype: float
        '''


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
        '''
        Generates a CPU utilization for the current time and updates the custom battery object based on the corresponding power consumption
        '''

        if self.battery is None:
            return

        self._custom_battery.update(self.battery)

        if len(self._video_data_pairs) == 0:
            return
        
        
        # Get current CPU utilization and corresponding J/s consumption
        curr_cpu_util = self.get_current_cpu_util()
        
        J_s_util = self._custom_battery.get_js_for_util(curr_cpu_util, self._curr_pair_idx)

        # Overall J/s consumption difference
        J_s_delta : float = 0
        J_s_delta += J_s_util

        # Convert J/s to J
        J_delta = J_s_delta * (time.time() - self._last_sample_time)
 
        # Decrease battery capacity by J_delta and calculate capacity percentage
        self._custom_battery.update_cap_j(self._custom_battery.capacity_J - J_delta)
        capacity_percent = self._custom_battery.get_capacity_percentage() * 100

        print(f"Battery decreased {J_delta:.2f}J to {self._custom_battery.capacity_J:.2f}J ({capacity_percent:.2f}%)")

        if capacity_percent <= 0:
            print("Battery is dead, ending simulation")
            os.kill(os.getpid(), signal.SIGUSR1)

        # Add to graph data
        self._graph_battery_percents.append(capacity_percent)
        self._graph_cpu_utils.append(curr_cpu_util)

        # Add graph data to main thread queue
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