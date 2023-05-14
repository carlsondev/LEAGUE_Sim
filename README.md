##  Project Setup Enviornment

`source ~/ardupilot_ws/devel/setup.bash`




### Packages
* `pip3 install --user -r requirements.txt`
* `sudo apt-get install ros-kinetic-jackal-simulator ros-kinetic-jackal-desktop ros-kinetic-jackal-navigation`

### Ardupilot

Ardupilot_Gazebo!
* Install Ardupilot
    - `git clone https://github.com/ArduPilot/ardupilot.git ~/`
    - `echo 'export PATH=$PATH:$HOME/ardupilot/Tools/autotest'`
### ROS Workspace
* `gazebo-iris.parm`
* `iqsim`
* `mavlink`
* `mavros`

## Simulation Environment Files

`echo 'export GAZEBO_MODEL_PATH=$GAZEBO_MODEL_PATH:~/ardupilot_ws/src/sim_proj/models' >> ~/.bashrc`
`echo 'export GAZEBO_MODEL_PATH=$GAZEBO_MODEL_PATH:~/ardupilot_gazebo/models' >> ~/.bashrc`

* `launch/river.launch`: Contains the launch data for the UAVs and the Jackal UGV
* `worlds/river.world`: River world file containing 3 UAVs 
* `models/droneN_with_camera`
    - `meshes`: Gazebo Iris UAV Meshes, same for all drones
    - `model.config`: Model information, same except for `<model><name>drone_with_camera</name></model>`
    - `model.sdf`: Model plugin information, same except for `fdm_port_in` and `fdm_port_out` in `arducopter_plugin` (difference of 10 between UAVs)
* `gazebo-iris.parm`: Parameters file for Gazebo Iris drone (where the BATT_CAPACITY is set). Originally located in `ardupilot/Tools/autotest/default_params/gazebo-iris.parm`


## Offboard_Node Information

## Usage

1. `python3 startup_sim.py 3`
    - Starts Gazebo simulator for `river.world`
    - Starts up three different MAVLink UAVs with a labeled terminal window for each one
2. `python3 src/sim_drone_workload.py final_jsons/partial.json --drone-idx=0 --off-method=partial`
    - Connects to drone **0** with **partial** offloading method
    - Reads the `partial.json` file for energy and cpu workload data
    - Performs a mission for drone **0** while generating CPU and energy data, presented to the specific graph