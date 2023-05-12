search_mode=OPTIMIZE_MAX_JOINT
srdf_filename=sawyer.srdf
robot_name_in_srdf=sawyer
moveit_config_pkg=sawyer_moveit_config
robot_name=sawyer
planning_group_name=right_arm
ikfast_plugin_pkg=sawyer_right_arm_ikfast_plugin
base_link_name=base
eef_link_name=right_motor_tip
ikfast_output_path=/home/dezhong/Desktop/helixBifurcation_robotic_parts_back/src/sawyer_right_arm_ikfast_plugin/src/sawyer_right_arm_ikfast_solver.cpp

rosrun moveit_kinematics create_ikfast_moveit_plugin.py\
  --search_mode=$search_mode\
  --srdf_filename=$srdf_filename\
  --robot_name_in_srdf=$robot_name_in_srdf\
  --moveit_config_pkg=$moveit_config_pkg\
  $robot_name\
  $planning_group_name\
  $ikfast_plugin_pkg\
  $base_link_name\
  $eef_link_name\
  $ikfast_output_path
