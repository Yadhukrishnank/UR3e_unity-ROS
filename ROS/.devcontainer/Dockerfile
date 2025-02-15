FROM osrf/ros:noetic-desktop-full

# User configuration
ARG USERNAME=vscode
ARG USER_UID=1000
ARG USER_GID=$USER_UID

# Create user and setup sudo
RUN if ! id -u $USER_UID >/dev/null 2>&1; then \
        groupadd --gid $USER_GID $USERNAME && \
        useradd -s /bin/bash --uid $USER_UID --gid $USER_GID -m $USERNAME; \
    fi && \
    apt-get update && \
    apt-get install -y sudo && \
    echo "$USERNAME ALL=(root) NOPASSWD:ALL" > /etc/sudoers.d/$USERNAME && \
    chmod 0440 /etc/sudoers.d/$USERNAME

# Switch to user
USER $USERNAME
WORKDIR /home/$USERNAME

# Install system dependencies
RUN sudo apt-get update && sudo apt-get install -y \
    git \
    vim \
    iputils-ping \
    net-tools \
    python3-pip \
    libgl1-mesa-glx \
    libgl1-mesa-dri \
    x11-apps \
    dbus-x11 \
    xauth \
    ros-noetic-robot-state-publisher \
    ros-noetic-moveit \
    ros-noetic-rosbridge-suite \
    ros-noetic-joy \
    ros-noetic-ros-control \
    ros-noetic-ros-controllers \
    ros-noetic-tf* \
    ros-noetic-gazebo-ros-pkgs \
    ros-noetic-joint-state-publisher \
    ros-noetic-soem \
    ros-noetic-ros-canopen \
    dos2unix

# ROS configuration
RUN rosdep update && \
    echo "source /opt/ros/${ROS_DISTRO}/setup.bash" >> ~/.bashrc && \
    echo "export QT_X11_NO_MITSHM=1" >> ~/.bashrc && \
    echo "export XAUTHORITY=/home/$USERNAME/.Xauthority" >> ~/.bashrc

# Python dependencies
RUN sudo -H pip3 --no-cache-dir install \
    rospkg \
    numpy \
    jsonpickle \
    scipy \
    easydict \
    torch==1.7.1+cu101 \
    torchvision==0.8.2+cu101 \
    torchaudio==0.7.2 -f https://download.pytorch.org/whl/torch_stable.html

# Add user to video group for hardware access
RUN sudo usermod -aG video $USERNAME

# Environment configuration
ENV DISPLAY=:0
ENV QT_X11_NO_MITSHM=1
ENV XAUTHORITY=/home/$USERNAME/.Xauthority

EXPOSE 10000