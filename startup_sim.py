import os
import sys

if __name__ == "__main__":

    args = sys.argv[1:]

    if len(args) != 1:
        print("Usage: python startup_sim.py <num_drones>")
        exit(1)

    num_drones = int(args[0])
    if os.system("gnome-terminal -- roslaunch league_sim river.launch") != 0:
        print("Error launching river.launch")
        exit(1)

    for i in range(num_drones):
        if os.system(f'gnome-terminal --title="Drone {i} Console" -- sim_vehicle.py -v ArduCopter -f gazebo-iris --console -I{i}') != 0:
            print("Error launching sim_vehicle.py for drone " + str(i))
            exit(1)

    

    