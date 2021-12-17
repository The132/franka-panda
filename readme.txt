roslaunch panda_gazebo panda_world.launch
roslaunch panda_sim_moveit sim_move_group.launch

roslaunch panda_simulator_examples demo_moveit.launch

catkin build -DCMAKE_BUILD_TYPE=Release -DFranka_DIR:PATH=/home/tianhai/libfranka/libfranka/build

