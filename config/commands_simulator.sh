

roslaunch neato_simulator neato_playground.launch

roslaunch my_pf test_my_pf.launch map_file:=`rospack find neato_2dnav`/maps/playground_smaller.yaml use_sim_time:=true

roslaunch turtlebot_rviz_launchers view_navigation.launch

rosrun teleop_twist_keyboard teleop_twist_keyboard.py teleop_twist_keyboard.py
