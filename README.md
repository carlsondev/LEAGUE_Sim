##  Project Setup Enviornment


## Simulation Environment Files

* `launch/river.launch`: Contains the launch data for the UAVs and the Jackal UGV
* `worlds/river.world`: River world file containing 3 UAVs 
* `models/droneN_with_camera`
    - `meshes`: Gazebo Iris UAV Meshes, same for all drones
    - `model.config`: Model information, same except for `<model><name>drone_with_camera</name></model>`
    - `model.sdf`: Model plugin information, same except for `fdm_port_in` and `fdm_port_out` in `arducopter_plugin` (difference of 10 between UAVs)
    


## Offboard_Node Information