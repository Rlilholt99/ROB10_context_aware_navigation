# ROB10_context_aware_navigation


## Setting up the simulation

This repo uses the simulation provided by PAL Robotics. Follow the instructions found on the simulation repo https://git.hub.com/pal-robotics/tiago_simulation.
Not that the installation procedure places the source files in your home directory. These should be moved to the path of this repo using the following command

```bash
mv /tiago_public_sim /{path-to-repo}
```

If done corretly the the root of the repo should have the two folders src and tiago_public_sim

From this point simply build the repo

```bash
colcon build --symlink-install
```


## Launch

``` bash
source install/setup.bash
ros2 launch context_aware_nav_bringup startSim.launch.py
```