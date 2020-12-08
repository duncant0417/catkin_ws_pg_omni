# omni_3wd

## A ROS Package for three-wheeled omnidirectional robots

### Getting Started

- `cd catkin_ws/src`
-  Clone this repo here : `git clone "https://github.com/YugAjmera/omni_3wd"`
- `cd ..` (Go back to catkin_ws/)
- `catkin_make`
- `source ./devel/setup.bash`
- `source ~/.bashrc`

### Run

- To view the model in Gazebo : ` roslaunch omni_3wd urdf_gazebo_view.launch `

- To view the model with controllers : `roslaunch omni_3wd velocity_controller.launch `

- To view RVIZ model : `roslaunch omni_3wd urdf_rviz_view.launch`

- Control the robot using keyboard keys: [teleop_keyboard_omni3](https://github.com/YugAjmera/teleop_keyboard_omni3)

![](screenshots/Screenshot%20from%202019-02-27%2000-17-33.png)


