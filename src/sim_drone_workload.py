#! /usr/bin/env python3.9

#https://docs.px4.io/main/en/ros/mavros_offboard_python.html

import dronekit as dk
import time
from energy_vehicle import EnergyVehicle, OffloadingMethod
import argparse
from os import path
import json
from typing import Dict, Any, List, Optional, Union
import sys
import signal
import matplotlib.pyplot as plt
import matplotlib.ticker as mtick
import matplotlib
from queue import Queue
import numpy as np
from geopy import distance

matplotlib.use("Qt5agg") # Use Qt4 backend for matplotlib

# HERELINK_TELEM

vehicle : Optional[EnergyVehicle] = None
data_queue : Queue = Queue()

ax1, ax2, fig = None, None, None

def exit_signal_handler(signal, frame):
    print("Exiting...")
    sys.exit(1)



def wait_update_graph(test_location : Optional[Union[dk.LocationGlobalRelative, dk.LocationGlobal]], sleep_time : Optional[float]):
    '''
    Wait and update the graph until the vehicle reaches the test location or until the sleep time has elapsed.
    

    :param test_location: The location to test for. If the vehicle reaches this location, the graph will stop updating.
    :type: test_location: Optional[Union[dk.LocationGlobalRelative, dk.LocationGlobal]]
    :param sleep_time: The time to wait before stopping the graph update.
    :type sleep_time: Optional[float]
    '''

    global data_queue

    if sleep_time is None and test_location is None:
        print("Must provide either a test location or a sleep time!")
        return

    start_time = time.time()
    while True:

        # Fetch battery percentages and CPU utilization from the queue
        graph_bat_percents, graph_cpu_utils = data_queue.get(block=True)

        # Update the graph
        _update_graph(graph_bat_percents, graph_cpu_utils)

        # Check if the vehicle has reached the test location
        if test_location is not None:
            real_loc = (vehicle.location.global_relative_frame.lat, vehicle.location.global_relative_frame.lon)
            test_loc = (test_location.lat, test_location.lon)
            real_dist = distance.distance(real_loc, test_loc).meters
            # print(f"Distance to test location: {real_dist} meters")
            if real_dist < 1.5:
                break

        # Check if the sleep time has elapsed
        if sleep_time is not None:
            if time.time() - start_time > sleep_time:
                break

def _update_queue(graph_bat_percents : np.ndarray, graph_cpu_utils : np.ndarray):
    '''
    Add the battery percentages and CPU utilization to the queue.

    :param graph_bat_percents: The battery percentages to add to the queue, must be a value not reference
    :type graph_bat_percents: np.ndarray
    :param graph_cpu_utils: The CPU utilization to add to the queue, must be a value not reference
    :type graph_cpu_utils: np.ndarray
    '''

    global data_queue
    data_queue.put((graph_bat_percents, graph_cpu_utils))

def _update_graph(graph_battery_percents : np.ndarray, graph_cpu_utils : np.ndarray):
    '''
    Update the graph with the given battery percentages and CPU utilization.

    :param graph_battery_percents: The battery percentages to update the graph with
    :type graph_battery_percents: np.ndarray
    :param graph_cpu_utils: The CPU utilization to update the graph with
    :type graph_cpu_utils: np.ndarray
    '''


    global ax1, ax2, fig

    color_red = 'tab:red'
    color_blue = 'tab:blue'

    bat_percents = graph_battery_percents
    cpu_utils = graph_cpu_utils

    batt_percents_count = len(bat_percents)
    cpu_percents_count = len(cpu_utils)

    # If the graph has not been initialized yet, initialize it
    if batt_percents_count == 1 and cpu_percents_count == 1:
        plt.ion()

        fig, ax1 = plt.subplots()
        offload_method = vehicle.offloading_method
        plt.get_current_fig_manager().set_window_title(f"Drone {vehicle.drone_idx} Battery & CPU Graph ({offload_method.value})")

        method_name = offload_method.value.capitalize()
        if offload_method == OffloadingMethod.FULL_OFFLOAD or offload_method == OffloadingMethod.PARTIAL_OFFLOAD:
            method_name += " Offloading"

        plt.title(f"Drone {vehicle.drone_idx}, {method_name} Method")

        ax1.set_xlabel('Running Time')
        ax1.set_ylabel('Battery Percentage', color=color_blue)  # we already handled the x-label with ax1
        ax1.tick_params(axis='y', labelcolor=color_blue)
        ax1.yaxis.set_major_formatter(mtick.PercentFormatter(xmax=100, decimals=2))

        


        ax2 = ax1.twinx()  # instantiate a second axes that shares the same x-axis
        ax2.set_ylabel('CPU Utilization', color=color_red)
        ax2.tick_params(axis='y', labelcolor=color_red)
        ax2.yaxis.set_major_formatter(mtick.PercentFormatter(xmax=100, decimals=2))

        fig.tight_layout()  # otherwise the right y-label is slightly clipped
        plt.show()


    # Update the graph    
    ax1.plot(list(range(batt_percents_count)), bat_percents, color_blue)
    ax2.plot(list(range(cpu_percents_count)), cpu_utils, color_red)

    fig.canvas.draw()
    fig.canvas.flush_events()

def main(sim_data : Dict[str, Any], offloading_method : OffloadingMethod, drone_idx : int):   
    '''
    Main function for the simulation. Connects to the drone, sets the simulation data, and runs the simulation.

    :param sim_data: The simulation data to pass to the vehicle, loaded from the JSON file
    :type sim_data: Dict[str, Any]
    :param offloading_method: The offloading method to use for the simulation
    :type offloading_method: OffloadingMethod
    :param drone_idx: The index of the drone to connect to
    :type drone_idx: int
    '''

    global vehicle 

    drone_address = f"127.0.0.1:145{5+drone_idx}0" # 14550, 14560, 14570, etc.
    print(f"Connecting to drone at {drone_address}")
    vehicle = dk.connect(drone_address, wait_ready=True, vehicle_class=EnergyVehicle)

    # Pass JSON data to vehicle
    vehicle.set_sim_data(sim_data, offloading_method, drone_idx)

    # Set method for queueing graph data
    vehicle.queue_method = _update_queue


    # Set vehicle mode to guided and wait for the armable state
    vehicle.mode    = dk.VehicleMode("GUIDED")
    vehicle.wait_for_armable()

    vehicle.arm()
    vehicle.simple_takeoff(20) # Take off to 20m above ground

    # Perform graph updates and simulations for 50 seconds
    wait_update_graph(None, sleep_time=50)

    print("Set default/target airspeed to 3")
    vehicle.airspeed = 3

    print("Going towards first point for 30 seconds ...")
    offset = drone_idx * 0.0001
    point1 = dk.LocationGlobalRelative(-35.361354 + offset, 149.165218 + offset, 20)
    vehicle.simple_goto(point1)

    # Perform graph updates and simulations until the drone reaches the first point
    wait_update_graph(point1, None)

    print("Going towards second point for 30 seconds (groundspeed set to 10 m/s) ...")
    point2 = dk.LocationGlobalRelative(-35.363244 - offset, 149.168801 - offset, 20)
    vehicle.simple_goto(point2, groundspeed=10)

    # Perform graph updates and simulations until the drone reaches the second point
    wait_update_graph(point2, None)

    
    print("Returning to Launch")
    print(f"Home location: {vehicle.home_location}")
    vehicle.mode = dk.VehicleMode("RTL")
    

    # Perform graph updates and simulations until the drone reaches the home location (or 50 seconds)
    if vehicle.home_location is None:
        wait_update_graph(None, 50)
    else:
        wait_update_graph(vehicle.home_location, None)

    # Close vehicle object before exiting script
    vehicle.close()

    input("Press any key to exit...")
        

def validate_data_json_file(data_json_dict : Dict[str, Any]) -> bool:
    '''
    Validates the data JSON file to ensure that it has the correct format.
    :param data_json_dict: The dictionary representation of the data JSON file.
    :type: Dict[str, Any]
    :return: True if the data JSON file is valid, False otherwise.
    '''


    # Validate linear regression dictionary
    def validate_lin_reg_dict(lin_reg_dict : Dict[str, Any]) -> bool:
        if "coefs" not in lin_reg_dict:
            return False
        if "poly_stds" not in lin_reg_dict:
            return False
        if "r_2" not in lin_reg_dict:
            return False
        return True
    
    # Validate CPU bin/data dictionary
    def validate_cpu_bin_dict(cpu_bin_dict : Dict[str, Any]) -> bool:
        if not isinstance(cpu_bin_dict, dict):
                return False
        if "cpu_bins" not in cpu_bin_dict:
            print(f"CPU bins for {key} are not present!")
            return False
        if "bin_ordering" not in cpu_bin_dict:
            print(f"CPU bin ordering for {key} is not present!")
            return False
        if "regression" not in cpu_bin_dict:
            print(f"Regression for {key} is not present!")
            return False
        
        if not validate_lin_reg_dict(cpu_bin_dict["regression"]):
            print(f"Regression dictionary for {key} is not valid!")
            return False
        
        if not isinstance(cpu_bin_dict["cpu_bins"], dict):
            return False
        
        if not isinstance(cpu_bin_dict["bin_ordering"], list):
            return False
        
        for bin_key, bin_value in cpu_bin_dict["cpu_bins"].items():
            if not bin_key.isdigit():
                return False
            if not isinstance(bin_value, dict):
                return False
            if list(bin_value.keys()) != ["mean", "std", "n"]:
                print(f"At least one CPU bin ({bin_key}) does not include mean, std, or/and n keys for {key}!")
                return False
            
        return True
    

    for key, value in data_json_dict.items():
        if not validate_cpu_bin_dict(value):
            print(f"CPU bin dictionary for {key} is not valid!")
            return False
    
    return True
    

if __name__ == "__main__":

    parser = argparse.ArgumentParser(description="Executes drone")
    parser.add_argument("data_json_path", type=str, help="The path to the created json file containing simulation data such as CPU bin info and linear regression equations")
    parser.add_argument("--off-method", type=str, help="The method of offboarding to simulate. Options are: 'onboard', 'partial', 'full'")
    parser.add_argument("--drone-idx", type=int, help="The index of the drone to run the simulation on.")

    args = parser.parse_args()

    data_json_path : str = args.data_json_path
    off_method_str : str = args.off_method
    drone_idx : int = args.drone_idx

    off_method = OffloadingMethod.NONE

    # Currently used for describing the offloading method in the graph
    # Original goal was to index the JSON pairs
    if off_method_str not in ["onboard", "partial", "full"]:
        print(f"Offloading method {off_method_str} is not valid!")
        exit(1)

    if off_method_str == "onboard":
        off_method = OffloadingMethod.ONBOARD
    elif off_method_str == "partial":
        off_method = OffloadingMethod.PARTIAL_OFFLOAD
    elif off_method_str == "full":
        off_method = OffloadingMethod.FULL_OFFLOAD 


    signal.signal(signal.SIGUSR1, exit_signal_handler)

    if not path.exists(data_json_path):
        print(f"{data_json_path} does not exist!")
    
    try:
        sim_data_file = open(data_json_path, 'r')
        data_json_data = json.load(sim_data_file)

        if not validate_data_json_file(data_json_data):
            print("Data JSON file is not valid!")
            exit(1)

    except OSError as oe:
        print(f"Failed to open Data JSON file ({data_json_path}) with error: {oe}")
        exit(1)
    except json.JSONDecodeError as je:
        print(f"Failed to decode JSON in Data file. Is {data_json_path} a JSON file?")
        raise je

    main(data_json_data, off_method, drone_idx)
