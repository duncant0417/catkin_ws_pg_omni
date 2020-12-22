# catkin_ws_pg_omni
Hi All,<br><br>
## This repo is for the final project of Embedded Microprocessor(嵌入式微處理機).<br>
Use policy gradient to train a omni robot in order to reach the destination without crashing to obstacle.

The model of Omni robot is base on these two projects :<br>
>>https://github.com/GuiRitter/OpenBase<br>
>>https://github.com/YugAjmera/omni3ros_pkg

<br>
Some components comes from turtlebot3 ,including the lidar module ,some world enviroment and slam pakage. You can find some resources of the turtlebot from the below link:<br>

>>https://emanual.robotis.com/docs/en/platform/turtlebot3/overview/

<br>
And the model of Policy Gradient is base on :<br>

>>https://github.com/keon/policy-gradient/blob/master/pg.py



PART 1. Enviroment Setting:
=
<br>
Operation system : Ubuntu 18.04
<br>
ROS version : Melodic
<br>
<br>
1.First ,you need to install Ubuntu 18.04 and ROS melodic on your machine.<br>
You can get the information about ros installation from :
<br>

>>http://wiki.ros.org/melodic/Installation/Ubuntu

<br><br>
2.After you setup your ros ,please install these following pakage if you haven't install yet :

        sudo apt-get install ros-melodic-ros-control
        sudo apt-get install ros-melodic-effort-controllers
        sudo apt-get install ros-melodic-joint-state-controller
        sudo apt-get install ros-melodic-position-controllers
        sudo apt-get install ros-melodic-ros-control
        sudo apt-get install ros-melodic-velocity-controllers 
        sudo apt-get install ros-melodic-ros-controllers 
        sudo apt-get install ros-melodic-gazebo-ros 
        sudo apt-get install ros-melodic-gazebo-ros-control



PART 2.
=

You need to setup the enviroment for running ML.

1.Choose a proper version of anaconda and install it:<br>
>>https://repo.anaconda.com/archive/

<br><br>
2.Create a conda enviroment by :

        conda create -n ros-env python=2.7
<br>
4. Activate your conda enviroment:

        conda activate ros-env
<br>
4. Install ROS related pakages and tensorflow ,keras inside your conda enviroment which you already created:

        pip install rosinstall msgpack empy defusedxml netifaces
        pip install tensorflow==1.14
        pip install keras==2.1.5
        pip install pydot
<br>
5. Close all command line console.


PART 3. Clone the project
=
<br>
1. Move to your working space or any directory you like.<br>
2. clone the repo:

        git clone https://github.com/windlunar/catkin_ws_pg_omni



PART 4.Training
=

1.open a terminal ,and type the commands :

        conda activate ros-env
        source /opt/ros/melodic/setup.bash
<br>
2. Move to the catkin_ws_pg_omni folder , and type the following command.

        source ./devel/setup.bash
        roslaunch omni_3wd pg_env.launch
<br>

3. Open another terminal ,and type the same command to activate conda enviroment:

        conda activate ros-env
        source /opt/ros/melodic/setup.bash
<br>
4. For the new terminal ,move to :<br>catkin_ws_pg_omni/src/omni_pg/nodes
<br>
And type the following command to start tranning.

        python pg_main.py

![image](https://github.com/windlunar/catkin_ws_pg_omni/blob/main/picture/env.png)

<br>

PART 5.SLAM
=

Install gmapping:

        sudo sh -c 'echo "deb http://packages.ros.org/ros/ubuntu $(lsb_release -sc) main" > /etc/apt/sources.list.d/ros-latest.list'

        sudo apt-key adv --keyserver 'hkp://keyserver.ubuntu.com:80' --recv-key C1CF6E31E6BADE8868B172B4F42ED6FBAB17C654

        sudo apt update
        sudo apt-get install ros-melodic-slam-gmapping
<br>
Close all the terminal.<br><br>


<br>
1.Open a new terminal:

        source /opt/ros/melodic/setup.bash
<br>
Move to the catkin_ws_pg_omni folder and run:

        source ./devel/setup.bash
        roslaunch omni_3wd slam_env.launch
<br>
You can see the gazebo is open.
<br><br>
2.Open the second terminal:

        source /opt/ros/melodic/setup.bash
<br>
Move to the catkin_ws_pg_omni folder and run:

        source ./devel/setup.bash
        roslaunch turtlebot3_slam turtlebot3_slam.launch
<br>
Now rviz is open.
<br><br>
3.Open another terminal:

        source /opt/ros/melodic/setup.bash
<br>
Move to the catkin_ws_pg_omni folder

        source ./devel/setup.bash
<br>
move to "/catkin_ws_pg_omni/src/cmd_vel_keyboard/nodes" folder<br>
And run the command to control robot by keyboard:

        python omni_keyboard.py
<br>
or

        python cmd_vel_keyboard.py
         
<br>
Now you can use your keyboard to control the omniweel ,and the rviz will show the map which is created by gmapping.
<br>
<br>

![image](https://github.com/windlunar/catkin_ws_pg_omni/blob/main/picture/slam.png)

<br><br>
Cause I haven't map all the frame yet ,so you may see some componets of omnibot are missing in rviz.


Install map_saver to save the map:

        sudo sh -c 'echo "deb http://packages.ros.org/ros/ubuntu $(lsb_release -sc) main" > /etc/apt/sources.list.d/ros-latest.list'

        sudo apt-key adv --keyserver 'hkp://keyserver.ubuntu.com:80' --recv-key C1CF6E31E6BADE8868B172B4F42ED6FBAB17C654

        sudo apt update
        sudo apt-get install ros-melodic-map-server

Save your map:

        rosrun map_server map_saver -f ./map

Then you should see your map "map.pgm" and "map.yaml"  has been saved.



PART 6.AMCL
=

First, install amcl

        sudo sh -c 'echo "deb http://packages.ros.org/ros/ubuntu $(lsb_release -sc) main" > /etc/apt/sources.list.d/ros-latest.list'

        sudo apt-key adv --keyserver 'hkp://keyserver.ubuntu.com:80' --recv-key C1CF6E31E6BADE8868B172B4F42ED6FBAB17C654

        sudo apt update
        sudo apt-get install ros-melodic-amcl
<br>
Then, put your map(both "map.pgm" and "map.yaml") which was created by SLAM to the folder :
<br>/catkin_ws_pg_omni/src/turtlebot3_navigation/maps/
<br>
<br>
1.Open a new terminal:

        source /opt/ros/melodic/setup.bash
<br>
Move to the catkin_ws_pg_omni folder and run:

        source ./devel/setup.bash
        roslaunch omni_3wd slam_env.launch
<br>
You can see the gazebo is open.
<br>
<br>
2.Open another terminal:

        source /opt/ros/melodic/setup.bash
<br>
Move to the catkin_ws_pg_omni folder and run:

        source ./devel/setup.bash
        roslaunch turtlebot3_navigation turtlebot3_amcl.launch 

        roslaunch turtlebot3_navigation turtlebot3_navigation.launch
<br>
<br>
Now rviz is open again.
<br>

Click the "2D Pose Estimate" at the top of the rviz window ,And set the initial position of robot<br>

![image](https://github.com/windlunar/catkin_ws_pg_omni/blob/main/picture/amcl1.png)


You can now observe some particles around the robot.<br>
Use your keyboard drive your robot go through the enviroment ,<br>

![image](https://github.com/windlunar/catkin_ws_pg_omni/blob/main/picture/amcl2.png)

<br>Then you should observe that the particle converge to the real position of robot.<br>

![image](https://github.com/windlunar/catkin_ws_pg_omni/blob/main/picture/amcl3.png)



PART 7.Navigation
=

Install the navigation pakage:

        sudo sh -c 'echo "deb http://packages.ros.org/ros/ubuntu $(lsb_release -sc) main" > /etc/apt/sources.list.d/ros-latest.list'

        sudo apt-key adv --keyserver 'hkp://keyserver.ubuntu.com:80' --recv-key C1CF6E31E6BADE8868B172B4F42ED6FBAB17C654

        sudo apt update
        sudo apt-get install ros-melodic-navigation

<br>
1.Open a new terminal:

        source /opt/ros/melodic/setup.bash
<br>
Move to the catkin_ws_pg_omni folder and run:

        source ./devel/setup.bash
        roslaunch omni_3wd slam_env.launch
<br>
You can see the gazebo is open.
<br>
<br>
2.Open another terminal:

        source /opt/ros/melodic/setup.bash
<br>
Move to catkin_ws_pg_omni folder and run:

        source ./devel/setup.bash
        roslaunch turtlebot3_navigation turtlebot3_navigation.launch
<br>
<br>
Now rviz is open.

3.Converge the particle as you did at PART 6.AMCL
<br>
<br>
4.Click the "2D Nav Goal" at the top of rviz toolbar.
<br>
<br>
5.Choose a place as the destination of robot ,and then click the left button.
<br>
<br>
6.As you can see ,the robot auto navigate to the destination.