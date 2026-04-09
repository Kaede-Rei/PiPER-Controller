[RoboStack Documentation](https://robostack.github.io/)

```bash
# Create a ros-noetic desktop environment
micromamba create -n ros_env -c conda-forge -c robostack-noetic ros-noetic-desktop-full
# Activate the environment
micromamba activate ros_env
# Add the robostack channel to the environemnt
micromamba config append channels robostack-noetic --env
micromamba install -c conda-forge ros-dev-tools
# Add the compilers to the environment
micromamba install -c conda-forge compilers cxx-compiler c-compiler binutils sysroot_linux-64
# Add MoveIt! to the environment
micromamba install -c conda-forge ros-noetic-moveit
# Add Serial to the environment
micromamba install -c conda-forge ros-noetic-rosserial ros-noetic-rosserial-python
# Add OrbbecSDK to the environment
cd ./ros_env/pyorbbecsdk && sudo chmod +x ./install_udev_rules.sh && sudo ./install_udev_rules.sh && sudo udevadm control --reload && sudo udevadm trigger && pip install pyorbbecsdk2
# Add Can and PiPER-SDK to the environment
pip install python-can piper_sdk
# Use the conda compilers to build the workspace, the cmake version depends on your system, here we use 3.5 as an example in Ubuntu 22.04
# note: before building piper_tomato, you need to build piper_ros and source the setup first, otherwise the new setup will overwrite the old setup causing the piper_ros be not found
. ./ros_env/use-mamba-gcc.sh
cd piper_ros
catkin_make -DCATKIN_ENABLE_TESTING=OFF -DCMAKE_POLICY_VERSION_MINIMUM=3.5
source devel/setup.bash
cd ../piper_tomato
catkin_make -DCATKIN_ENABLE_TESTING=OFF -DCMAKE_POLICY_VERSION_MINIMUM=3.5
```

```bash
# Quick Start
# create a ros env and install the dependencies
micromamba create -n ros_env -c conda-forge -c robostack-noetic \
    ros-noetic-desktop-full \
    ros-dev-tools \
    ros-noetic-moveit \
    ros-noetic-trac-ik-kinematics-plugin \
    ros-noetic-rosserial \
    ros-noetic-rosserial-python \
    compilers cxx-compiler c-compiler binutils sysroot_linux-64

# if you want to install others
micromamba activate ros_env
micromamba install -c conda-forge -c robostack-noetic \
    ros-noetic-<package-name>

# install the OrbbecSDK and add the udev rules
cd ./ros_env/pyorbbecsdk && sudo chmod +x ./install_udev_rules.sh && sudo ./install_udev_rules.sh && sudo udevadm control --reload && sudo udevadm trigger && pip install pyorbbecsdk2 && cd ../../
# install Can and PiPER-SDK
pip install python-can piper_sdk
# optional trimesh can be used to simplify robotic arm meshes
pip install fast-simplification trimesh
# build the workspace
. ./ros_env/use-mamba-gcc.sh && cd piper_ros && catkin_make -DCATKIN_ENABLE_TESTING=OFF -DCMAKE_POLICY_VERSION_MINIMUM=3.5 && source devel/setup.bash && cd ../piper_tomato && catkin_make -DCATKIN_ENABLE_TESTING=OFF -DCMAKE_POLICY_VERSION_MINIMUM=3.5 && source devel/setup.bash && cd ..
# or if you need clangd
. ./ros_env/use-mamba-gcc.sh && cd piper_ros && catkin_make -DCATKIN_ENABLE_TESTING=OFF -DCMAKE_POLICY_VERSION_MINIMUM=3.5 -DCMAKE_EXPORT_COMPILE_COMMANDS=ON && source devel/setup.bash && cd ../piper_tomato && catkin_make -DCATKIN_ENABLE_TESTING=OFF -DCMAKE_POLICY_VERSION_MINIMUM=3.5 -DCMAKE_EXPORT_COMPILE_COMMANDS=ON && source devel/setup.bash && cd .. && ln -sf ../piper_tomato/build/compile_commands.json ./build/compile_commands.json
# run the demo
cd .. && ./piper-start.sh
```