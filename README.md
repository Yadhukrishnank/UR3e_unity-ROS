# Description
This repository allows you to operate a ur3e robot with robotiq gripper 2f-85 in unity environment

------

### 1. Download the repository from an organization

The _git clone_ command allows you to download files from a remote server to your local computer. Therefore, the _git clone_ command is specified first, followed by a link to the remote repository.

    git clone git@github.com:THD-autonomous-system/ros-2023-thd.git

### 2. Install Docker

You need to go into the downloaded folder and go into the docker folder.


    cd ros-2023-thd/docker

If you don't have an Nvidia graphics card, run the command:

    ./install_docker.bash

    
The installation of docker should complete correctly. If docker does not install, it is most likely an error in your utility (apt) settings for downloading and updating from remote repositories. This is a problem you should solve on your own.
------

### 3. Building Docker
    
Docker is an open platform for developing, shipping, and running applications. Docker enables you to separate your applications from your infrastructure so you can deliver software quickly. With Docker, you can manage your infrastructure in the same ways you manage your applications. By taking advantage of Docker’s methodologies for shipping, testing, and deploying code quickly, you can significantly reduce the delay between writing code and running it in production.


This means that we must first create the environment in which our program will run. We already have a ready-made environment and we just need to building it:

    sudo ./build_docker.sh

Next will begin the process of building. If the process ends incorrectly or with an error, you must start building again.

But if you can't end the process correctly even after several uploads, open the Dockerfile with a text editor and comment out lines 59 and 61.

### 4. Run Docker

To execute the docker container use command:

    sudo ./run_docker.sh
    
If you need additional terminal inside of the Docker open new window in the terminal (Ctrl+Shift+T) and use command

    sudo ./into_docker.sh
    
### 5. Enter the following commands in the terminal 


    cd turtlebot3_ws
    catkin_make
    source devel/setup.bash
    
To launch the ROS-TCP connector and moveit launch file enter the following commands 

    roslaunch ur_robotiq pose_est.launch

Open an another terminal
To run the pick and place nodes run the following command

    rosrun ur_robotiq mover.py
