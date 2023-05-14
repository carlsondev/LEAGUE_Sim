import dronekit as dk
from typing import Dict, List
import numpy as np
import random


class CustomBattery(dk.Battery):
    '''
    Custom Battery class that extends the dronekit Battery class
    '''
    
    def __init__(self, battery : dk.Battery, battery_cap_mah : int):
        '''
        Initialize the CustomBattery object

        :param battery: Battery object with the initial battery information
        :type: dk.Battery
        :param battery_cap_mah: Initial battery capacity in mAh (used unless otherwise updated)
        :type: int
        '''

        super().__init__(battery.voltage, battery.current, battery.level)

        # Convert battery capacity from mAh to Joules   
        self.capacity_J = self._joules(battery.voltage, battery_cap_mah)
        self._max_cap_j = 0
        
        self._all_data_lin_reg_params : Dict[str, float] = {}
        self.pairs_lin_reg_params : List[Dict[str, float]] = []

    def update_cap_mah(self, battery_cap_mah : int):
        '''
        Update the battery capacity in mAh (coverted to Joules)

        :param battery_cap_mah: New battery capacity in mAh
        :type: int
        '''
        print(f"Updating battery capacity to {battery_cap_mah} mAh")
        self.capacity_J = self._joules(self.voltage, battery_cap_mah)

    def update_cap_j(self, battery_cap_j : float):
        '''
        Update the battery capacity in Joules

        :param battery_cap_j: New battery capacity in Joules
        :type: float
        '''

        self.capacity_J = battery_cap_j
        self._max_cap_j = max(self._max_cap_j, self.capacity_J)

    def get_capacity_percentage(self) -> float:
        '''
        Get the battery capacity as a percentage of the maximum capacity
        '''
        return self.capacity_J / self._max_cap_j

    # Get Joules/s consumption based on the CPU utilization -> Energy consumption model
    # 1 Watt = 1 Joule/second
    def get_js_for_util(self, cpu_utilization : float, pair_idx) -> float:
        '''
        Get the Joules/second (Wattage) consumption for a given CPU utilization and pair (to index regression parameters)

        :param cpu_utilization: CPU utilization to get the Joules/second consumption for
        :type: float
        :param pair_idx: Index of the pair to get the regression parameters for
        :type: int
        :return: Joules/second consumption
        :rtype: float
        '''
        try:
            curr_lin_reg = self.pairs_lin_reg_params[pair_idx]
        except IndexError:
            return 0
        
        try:
            reg_coefs = curr_lin_reg["coefs"]
            regression = np.poly1d(reg_coefs)

            reg_watts = regression(cpu_utilization)

            # Add poly stds
            rand_poly_std_watt = 0
            if random.randint(0, 10) == 5:
                reg_poly_stds = curr_lin_reg["poly_stds"]
                cpu_std_idx = int(cpu_utilization) if cpu_utilization < 100 else 99
                rand_poly_std_watt = np.random.normal(0, reg_poly_stds[cpu_std_idx])

                print(f"    {reg_watts:.2f} + {rand_poly_std_watt:.2f} = {reg_watts + rand_poly_std_watt:.2f} W")

            return reg_watts + rand_poly_std_watt
        except KeyError:
            return 0

    def update(self, battery : dk.Battery):
        '''
        Update the battery information from the dk.Battery object

        :param battery: Updated battery object
        :type: dk.Battery
        '''
        self.voltage = battery.voltage
        self.current = battery.current
        self.level = battery.level


    def __str__(self):
        return f"Custom {super().__str__()},Capacity={self.capacity_J} J"
    
    def _joules(self, volt : float, mah : int) -> float:
        '''
        Convert voltage and mAh to Joules

        :param volt: Voltage
        :type: float
        :param mah: mAh
        :type: int
        :return: Joules
        :rtype: float
        '''
        return volt * mah * 3.6