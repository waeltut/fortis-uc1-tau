# Source this file from Bash for interactive shells, builds and launch scripts.
source /opt/ros/humble/setup.bash
if [ -f "${ROS_WORKSPACE}/install/local_setup.bash" ]; then
    source "${ROS_WORKSPACE}/install/local_setup.bash"
fi
# Last so workspace package/library paths cannot select the affected apt TF.
source /opt/tf2_fix/local_setup.bash
