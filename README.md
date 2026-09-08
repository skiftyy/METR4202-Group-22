# METR4202-Group-22
**Open laptop:**
cd /path/to/your/repo

git pull origin main


**Close laptop:**
cd /path/to/your/repo

git status

git add .

git commit -m "Describe what you changed"

git push origin main

**Test:**

_Terminal 1:_

export TURTLEBOT3_MODEL=waffle_pi

ros2 launch turtlebot3_gazebo turtlebot3_world.launch.py

_Terminal 2:_

export TURTLEBOT3_MODEL=waffle_pi

ros2 launch turtlebot3_navigation2 navigation2.launch.py use_sim_time:=True slam:=True

_Terminal 3:_

cd /path/to/your/repo

colcon build --symlink-install --packages-select tb3_exploration

source install/setup.bash

ros2 run tb3_exploration waypoint_cycler