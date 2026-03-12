[RoboStack Documentation](https://moveit.picknik.ai/main/index.html)

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
# Add Can and PiPER-SDK to the environment
pip install python-can piper_sdk
# Use the conda compilers to build the workspace, the cmake version depends on your system, here we use 3.5 as an example in Ubuntu 22.04
# note: before building piper_controller, you need to build piper_ros and source the setup first, otherwise the new setup will overwrite the old setup causing the piper_ros be not found
. ./ros_env/use-mamba-gcc.sh
cd piper_ros
catkin_make -DCATKIN_ENABLE_TESTING=OFF -DCMAKE_POLICY_VERSION_MINIMUM=3.5
source devel/setup.bash
cd ../piper_controller
catkin_make -DCATKIN_ENABLE_TESTING=OFF -DCMAKE_POLICY_VERSION_MINIMUM=3.5
```