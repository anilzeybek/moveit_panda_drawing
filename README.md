# moveit_panda_drawing

Video: https://drive.google.com/file/d/1n9GwSBfWdlTY59WNZhWxDFojfeswAj_2/view?usp=sharing

To simulate:
1. Build the panda simulator: https://github.com/justagist/panda_simulator
   (follow the steps in that repo)
3. ```roslaunch moveit_panda_drawing simulation.launch``` (to start gazebo)
4. ```roslaunch panda_sim_moveit sim_move_group.launch``` (to start moveit server)
5. ```roslaunch moveit_panda_drawing demo.launch``` (to start rviz)
6. ```rosrun moveit_panda_drawing main``` (to start drawing)
