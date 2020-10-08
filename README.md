# Coppeliasim_ros_control

This is a ROS package for integrating the ros_control controller architecture with the [coppeliasim][] simulator. 

### Dependencies
- hardware_interface
- transmission_interface
- control_toolbox
- urdf

### Building 

The following instructions assume that a catkin workspace has been created at `$HOME/catkin_ws` and Coppeliasim directory is placed at the Home directory `$HOME/CoppeliaSim`. you always can update the paths based on your machine setup.

```bash
# change to the src directory in your Catkin workspace
 cd $HOME/catkin_ws/src

# Clone Coppeliasim_ros_control pkg 
 git clone https://github.com/tud-cor/coppeliasim_ros_control

# change to the main Catkin workspace
 cd ..

# build the workspace (using catkin_tools)
 catkin build

# After building, you can check if the plugin 'libsimExtRosControl.so' was successfuly by listing the content of the devel/lib
 ls devel/lib/
```

### Running
The generated plugin should be loaded while start coppeliasim, this can be done by copying the plugin 'libsimExtRosControl.so' to the main directory of Coppeliasim `$HOME/CoppeliaSim`
```
# Copy the generated plugin to CoppeliaSim directory
cp  $HOME/catkin_ws/devel/lib/libsimExtRosControl.so  ~/CoppeliaSim/
```
In a new terminal start a ros master before you start CoppelliaSim
```
roscore
```
In a nother terminal run CoppeliaSim
```
# change to Coppeliasim dir 
cd $HOME/CoppeliaSim

# run Coppeliasim
./coppeliaSim.sh
```
In the same terminal running Coppeliasim, you can see the following message indicating a successful loading of the RosControl plugin
```
Plugin 'RosControl': loading...
[DEBUG] [1602118275.276717674]:  initialize MyRobot hardware_interface!
Plugin 'RosControl': load succeeded.
```
By starting the simulation in coppeliasim, the plugin will wait for the robot description to be availabe in the parameter server.

### Example
Refer to [ur5_coppeliasim_roscontrol][] package on how you can use the coppeliasim_ros_control plugin to control ur5 robot. You can also create a similar configuration package for any robot. 



### Limitation
currenlty, both position and velocity controllers works properly. but the effort controller not properly working. The reason why effort control mode is not working -not implemented yet- is that Coppeliasim provides a direct APIs for only position/velocity control modes, and there is no direct API for the effort control mode. More information about how effort controllers is implemented in coppeliasm can be found in their website.

[coppeliasim]: https://www.coppeliarobotics.com/
[ur5_coppeliasim_roscontrol]: https://github.com/tud-cor/ur5_coppeliasim_roscontrol
# Coppeliasim_ros_control

This is a ROS package for integrating the ros_control controller architecture with the [coppeliasim][] simulator. 

### Dependencies
- hardware_interface
- transmission_interface
- control_toolbox
- urdf

### Building 

The following instructions assume that a catkin workspace has been created at `$HOME/catkin_ws` and Coppeliasim directory is placed at the Home directory `$HOME/CoppeliaSim`. you always can update the paths based on your machine setup.

```bash
# change to the src directory in your Catkin workspace
 cd $HOME/catkin_ws/src

# Clone Coppeliasim_ros_control pkg 
 git clone https://github.com/tud-cor/coppeliasim_ros_control

# change to the main Catkin workspace
 cd ..

# build the workspace (using catkin_tools)
 catkin build

# After building, you can check if the plugin 'libsimExtRosControl.so' was successfuly by listing the content of the devel/lib
 ls devel/lib/
```

### Running
The generated plugin should be loaded while start coppeliasim, this can be done by copying the plugin 'libsimExtRosControl.so' to the main directory of Coppeliasim `$HOME/CoppeliaSim`
```
# Copy the generated plugin to CoppeliaSim directory
cp  $HOME/catkin_ws/devel/lib/libsimExtRosControl.so  ~/CoppeliaSim/
```
In a new terminal start a ros master before you start CoppelliaSim
```
roscore
```
In a nother terminal run CoppeliaSim
```
# change to Coppeliasim dir 
cd $HOME/CoppeliaSim

# run Coppeliasim
./coppeliaSim.sh
```
In the same terminal running Coppeliasim, you can see the following message indicating a successful loading of the RosControl plugin
```
Plugin 'RosControl': loading...
[DEBUG] [1602118275.276717674]:  initialize MyRobot hardware_interface!
Plugin 'RosControl': load succeeded.
```
By starting the simulation in coppeliasim, the plugin will wait for the robot description to be availabe in the parameter server.

### Example
Refer to [ur5_coppeliasim_roscontrol][] package on how you can use the coppeliasim_ros_control plugin to control ur5 robot. You can also create a similar configuration package for any robot. 



### Limitation
currenlty, both position and velocity controllers works properly. but the effort controller not properly working. The reason why effort control mode is not working -not implemented yet- is that Coppeliasim provides a direct APIs for only position/velocity control modes, and there is no direct API for the effort control mode. More information about how effort controllers is implemented in coppeliasm can be found in their website.

[coppeliasim]: https://www.coppeliarobotics.com/
[ur5_coppeliasim_roscontrol]: https://github.com/tud-cor/ur5_coppeliasim_roscontrol
