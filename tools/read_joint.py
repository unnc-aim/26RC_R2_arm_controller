import os
import subprocess

# Source the ROS2 workspace setup and echo the joint read topics.
base_cmd = "source ../../install/setup.bash && ros2 topic echo /joint{i}/read --once | grep position"
for i in range(1, 5):
    subprocess.run(base_cmd.format(i=i), shell=True, executable="/bin/bash")

