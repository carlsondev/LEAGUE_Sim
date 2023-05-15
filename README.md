# LEAGUE Sim simulation setup

Follow these steps in order

## Install Dependencies

Install ROS Melodic: [ROS Melodic Install](http://wiki.ros.org/melodic/Installation/Ubuntu)

```
sudo apt-get install python-wstool python-rosinstall-generator python-catkin-tools
sudo apt-get install ros-melodic-jackal-simulator ros-melodic-jackal-desktop ros-melodic-jackal-navigation
sudo apt-get install python-matplotlib python-serial python-wxgtk3.0 python-wxtools python-lxml python-scipy python-opencv ccache gawk python-pip python-pexpect python3-pyqt5 python3-pip
sudo pip install future pymavlink MAVProxy
```

##  Project Setup Enviornment

### Setup ArduPilot
1. ArduPilot Dev Env
```
cd ~ && git clone --recurse-submodules -j8 --branch Copter-4.3.3 https://github.com/ArduPilot/ardupilot.git
cd ~/ardupilot
./Tools/environment_install/install-prereqs-ubuntu.sh -y
echo 'export PATH=$PATH:$HOME/ardupilot/Tools/autotest' >> ~/.bashrc
echo 'export PATH=/usr/lib/ccache:$PATH' >> ~/.bashrc
```

2. Ardupilot Gazebo
```
cd ~ && git clone --branch dev https://github.com/khancyr/ardupilot_gazebo
cd ardupilot_gazebo
mkdir build
cd build
cmake ..
make -j4
sudo make install

echo 'source /usr/share/gazebo/setup.sh' >> ~/.bashrc
echo 'export GAZEBO_MODEL_PATH=~/ardupilot_gazebo/models:$GAZEBO_MODEL_PATH' >> ~/.bashrc
```

### Setup Workspace
1. `$ mkdir -p ~/league_ws/src && cd ~/league_ws`
2. Initialize workspace: `$ catkin init`
2. Install `mavlink` and `mavros` as dependencies
```
cd src
git clone --recurse-submodules -j8 https://github.com/mavlink/mavlink.git
git clone --recurse-submodules -j8 --branch 1.15.0 https://github.com/mavlink/mavros.git
cd ~/league_ws
rosdep install --from-paths src --ignore-src --rosdistro `echo $ROS_DISTRO` -y

catkin build
```
3. `$ cd src && git clone https://github.com/carlsondev/LEAGUE_Sim.git`
4. Install dependancies
    1. `$ cd ~/league_ws/src/LEAGUE_Sim`
    2. `$ pip3 install --user -r requirements.txt`
5. Build workspace: `$ cd ~/league_ws && catkin build`
6. Add setup script to bashrc: `$ echo 'source ~/league_ws/devel/setup.bash' >> ~/.bashrc`
7. Add local models to path: `echo 'export GAZEBO_MODEL_PATH=~/league_ws/src/LEAGUE_Sim/models:$GAZEBO_MODEL_PATH' >> ~/.bashrc`


Re-source bashrc
```
$ source ~/.bashrc
```

Restart
```
$ reboot
```

# Simulation Environment Files

* `launch/river.launch`: Contains the launch data for the UAVs and the Jackal UGV
* `worlds/river.world`: River world file containing 3 UAVs 
* `models/droneN_with_camera`
    - `meshes`: Gazebo Iris UAV Meshes, same for all drones
    - `model.config`: Model information, same except for `<model><name>drone_with_camera</name></model>`
    - `model.sdf`: Model plugin information, same except for `fdm_port_in` and `fdm_port_out` in `arducopter_plugin` (difference of 10 between UAVs)
* `models.*`: Supporting models for `river.world`
* `support_files/gazebo-iris.parm`: Parameters file for Gazebo Iris drone (where the BATT_CAPACITY is set). Originally located in `ardupilot/Tools/autotest/default_params/gazebo-iris.parm`


# LEAGUE Sim Information

## Usage

1. `python3 startup_sim.py 3`
    - Starts Gazebo simulator for `river.world`
    - Starts up three different MAVLink UAVs with a labeled terminal window for each one
2. `python3 src/sim_drone_workload.py final_jsons/partial.json --drone-idx=0 --off-method=partial`
    - Connects to drone **0** with **partial** offloading method
    - Reads the `partial.json` file for energy and cpu workload data
    - Performs a mission for drone **0** while generating CPU and energy data, presented to the specific graph


## Disclaimer

Project Structure, models, launch files, and worlds are based on the [IQ_SIM](https://github.com/Intelligent-Quads/iq_sim.git) repo
