# A Python-based Stewart Platform Simulation with Keyboard Control

![](https://github.com/daniel-s-ingram/stewart/blob/master/gif/stewart.gif)

This repo serves to present a Stewart platform simulation in Gazebo. This projected is [forked](https://github.com/daniel-s-ingram/stewart) and provides adaptaptions away from C++ and dulashock controller requirements, making it easier to get started.  

Because the Stewart platform is a closed loop manipulator, the description was written in SDF rather than URDF. However, ROS does not support SDF by default, so a plugin was written to make the joints in Gazebo visible to ROS.

## Usage
The code was written for and tested in Ubuntu 16.04 (ROS Kinetic). To get started, clone this repo to your catkin workspace src directory and build it:

```
cd ~/your_catkin_ws_src_path/  
git clone https://github.com/asmorgan24/stewart.git  
catkin_make stewart
source ~/your_catkin_ws/devel/setup.bash
```

To build the plugin:

```
cd plugin  
mkdir build  
cd build  
cmake ../  
make  
```

Create the SDF file from the ERB template:

```
cd ../../sdf/stewart/
erb model.sdf.erb > model.sdf
```

Now, to launch the package:

```
roslaunch stewart stewart_gazebo.launch
```

## Control
This should launch the gazebo simulation world with the stewart platform in the center. Keyboard control for the platform is currently only implemented for roll, pitch, and yaw of the platform (not translation, but can be easily implemented). The control is as follows.

Now launch the control package in a NEW terminal:

```
roslaunch stewart stewart_control.launch
```

<center>
NOTE: Directions are similar to video game controls. To exit the keyboard terminal control hit ```~``` on the keyboard.
  
  
  
| Key | Response   | Description                                                 |
|-----|------------|-------------------------------------------------------------|
| S   | Reset      | Resets the platform to its resting state                    |
| W   | +Pitch     | Increases pitch of platform by a single step                |
| X   | -Pitch     | Decreases pitch of platform by a single step                |
| A   | +Roll      | Increases roll of platform by a single step                 |
| D   | -Roll      | Decreases roll of platform by a single step                 |
| Q   | +Yaw       | Increases yaw of platform by a single step                  |
| E   | -Yaw       | Decreases yaw of platform by a single step                  |
| ~   | ESCAPE    | IMPORTANT: Escapes the keyboard control in termal           |
</center>


## Contribute
Feel free to openly use and adapt this code however you and your project sees fit. 
