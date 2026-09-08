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
export TURTLEBOT3_MODEL=waffle_pi

ros2 launch turtlebot3_gazebo turtlebot3_world.launch.py


export TURTLEBOT3_MODEL=waffle_pi

ros2 launch turtlebot3_navigation2 navigation2.launch.py use_sim_time:=True slam:=True


colcon build --symlink-install --packages-select waypoint_commander

ros2 run waypoint_commander waypoint_cycler
