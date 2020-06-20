## IGVC Basic Course Gazebo Simulation

### Steps to configure
1. Install the models into the gazebo models folder.
 - `cd ~/caffeine-ws/src/Caffeine/worlds/gazebo_worlds/models`
 - `./install_models.sh`
2. Build src
 - `cd ~/caffeine-ws`
 - `catkin_make`

### Possible Actions
1. Launch only the world in gazebo
 - `gazebo ~/caffeine-ws/src/igvc_world/worlds/igvc_walls.world`
 - `gazebo ~/caffeine-ws/src/igvc_world/worlds/igvc.world`
2. Launch the world and caffeine
 - ex. `roslaunch description caffeine_gazebo.launch world:="full"`
 - world can take on values "walls" (default), "full"
