import dronekit as dk
from typing import Dict, List
import numpy as np
import random
# HERELINK_TELEM

class CustomBattery(dk.Battery):
    
    def __init__(self, battery : dk.Battery, battery_cap_mah : int):
        super().__init__(battery.voltage, battery.current, battery.level)

        # Watts * 3600 seconds/hour
        self.capacity_J = self.voltage * battery_cap_mah * 3600
        self._max_cap_j = 0
        
        self._all_data_lin_reg_params : Dict[str, float] = {}
        self.pairs_lin_reg_params : List[Dict[str, float]] = []

    def update_cap_mah(self, battery_cap_mah : int):
        # Watts * 3600 seconds/hour
        print(f"Updating battery capacity to {battery_cap_mah} mAh")
        self.capacity_J = self.voltage * battery_cap_mah * 3.6

    def update_cap_j(self, battery_cap_j : float):
        #print(f"Updating battery capacity to {battery_cap_j} J")
        self.capacity_J = battery_cap_j
        self._max_cap_j = max(self._max_cap_j, self.capacity_J)

    def get_capacity_percentage(self) -> float:
        return self.capacity_J / self._max_cap_j

    # Get Joules/s consumption based on the CPU utilization -> Energy consumption model
    # 1 Watt = 1 Joule/second
    def get_js_for_util(self, cpu_utilization : float, pair_idx) -> float:
        #print(f"Fetching J/s for {cpu_utilization:.2f}% utilization")
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
        self.voltage = battery.voltage
        self.current = battery.current
        self.level = battery.level


    def __str__(self):
        return f"Custom {super().__str__()},Capacity={self.capacity_J} J"