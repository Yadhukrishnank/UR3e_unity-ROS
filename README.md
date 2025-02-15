# Description

This is the ROS side repository that allows you to operate a ur3e robot with robotiq gripper 2f-85. Make sure the ROS work space is set up before running unity.

------

### 1. Download this repository

The _git clone_ command allows you to download files from a remote server to your local computer. Therefore, the _git clone_ command is specified first, followed by a link to the remote repository.

    git clone https://github.com/Yadhukrishnank/UR3e_unity.git    

### 2. Install Docker and VS Code

Install docker and VS Code on your local computer

    The installation of docker should complete correctly.

### 3. Open the work space in VS Code 

Navigate to ROS folder

    cd UR3e_unity/ROS/
     
Open the work_space in VS Code

    ~/UR3e_unity/ROS$ code .

### 4. Build the work_pace

Open a remote window from the bottom left corner and Select "Reopen in container". The docker image will start to build.
    
### 5. Open a new Terminal and enter the following commands

    catkin_make
    source devel/setup.bash
    
To launch the ROS-TCP connector and moveit launch file enter the following commands 

    roslaunch ur_robotiq pose_est.launch

Open an another terminal
To run the pick and place nodes run the following command

    rosrun ur_robotiq mover.py
------
 
