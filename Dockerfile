FROM osrf/ros:humble-desktop-full


# # NOTES #
# in generale puntare a modelli dentro models di turtlebot3, dentro cartella mdels di drl exploration 
# e fuel_models:
# per fare ciò, o esportare GAZEBO_MODEL_PATH in bashrc oppure mettere in launchfile
# ===> MOLTO MEGLIO METTERE IN .BASHRC <===
#
#  for hospital world, must install respective requirements.txt:
#     docopt>=0.6.2
#     requests
#     lxml

#  and run this scrpt:
#     #!/bin/bash
#     python3 -m pip install -r requirements.txt
#     python3 fuel_utility.py download \
#     -m XRayMachine -m IVStand -m BloodPressureMonitor -m BPCart -m BMWCart \
#     -m CGMClassic -m StorageRack -m Chair \
#     -m InstrumentCart1 -m Scrubs -m PatientWheelChair \
#     -m WhiteChipChair -m TrolleyBed -m SurgicalTrolley \
#     -m PotatoChipChair -m VisitorKidSit -m FemaleVisitorSit \
#     -m AdjTable -m MopCart3 -m MaleVisitorSit -m Drawer \
#     -m OfficeChairBlack -m ElderLadyPatient -m ElderMalePatient \
#     -m InstrumentCart2 -m MetalCabinet -m BedTable -m BedsideTable \
#     -m AnesthesiaMachine -m TrolleyBedPatient -m Shower \
#     -m SurgicalTrolleyMed -m StorageRackCovered -m KitchenSink \
#     -m Toilet -m VendingMachine -m ParkingTrolleyMin -m PatientFSit \
#     -m MaleVisitorOnPhone -m FemaleVisitor -m MalePatientBed \
#     -m StorageRackCoverOpen -m ParkingTrolleyMax \
#     -d fuel_models --verbose



# Create a non-root user
ARG USERNAME=ros
ARG USER_UID=1000
ARG USER_GID=$USER_UID

RUN groupadd --gid $USER_GID $USERNAME \
    && useradd -s /bin/bash --uid $USER_UID --gid $USER_GID -m $USERNAME \
    && mkdir /home/$USERNAME/.config && chown $USER_UID:$USER_GID /home/$USERNAME/.config


# Set up sudo
RUN apt-get update \
  && apt-get install -y sudo \
  && echo $USERNAME ALL=\(root\) NOPASSWD:ALL > /etc/sudoers.d/$USERNAME\
  && chmod 0440 /etc/sudoers.d/$USERNAME \
  && rm -rf /var/lib/apt/lists/*


# turtlebot3 dependencies
RUN apt-get update \
    && apt-get install -y \
    ros-humble-gazebo-* \
    ros-humble-cartographer \
    ros-humble-cartographer-ros \
    ros-humble-dynamixel-sdk \
    ros-humble-turtlebot3-msgs \
    ros-humble-turtlebot3 \
    && rm -rf /var/lib/apt/lists/*

# stable-baselines3 dependencies
RUN apt-get update \
    && apt-get install -y \
    python3 \
    python3-pip \
    python3-venv \
    && rm -rf /var/lib/apt/lists/*

RUN pip install --no-cache-dir --upgrade pip \
    &&  pip install --no-cache-dir \
    stable-baselines3[extra] \
    "numpy < 2" \
    wandb 



WORKDIR /ros_ws/src

RUN git clone -b humble-devel https://github.com/ROBOTIS-GIT/turtlebot3_simulations.git
# ${date} is used to be sure it will not be cached and changes are always built
RUN git clone https://github.com/RiccardoMengozzi/drl_exploration.git ${date} 

WORKDIR /ros_ws

USER ros
RUN rosdep update \ 
    && rosdep install -i --from-path src --rosdistro humble -y

USER root

RUN /bin/bash -c "source /opt/ros/humble/setup.bash && colcon build"

COPY drl_exploration_install.sh /drl_exploration_install.sh 
COPY bashrc /home/${USERNAME}/.bashrc
COPY bashrc /root/.bashrc

