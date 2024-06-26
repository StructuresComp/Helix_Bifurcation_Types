#include <helix_test/demo_application.h>
#include <fstream>
#include <sstream>
#include <string>
#include <map>


/* MOVE HOME
  Goal:
    - Move the robot to from its current location to the first point in the robot path.
    - Execute the robot path.

  Hints:
    - Use the "first_point_ptr->getNominalJointPose(...)" method in order to get the joint values at that position.
    - Execute the robot path by sending it to the moveit_msgs::ExecuteKnownTrajectory server by calling the
      "moveit_run_path_client_.call(srv)" client method.

*/

namespace helix_test
{

void DemoApplication::runPath(const DescartesTrajectory& path)
{
  //ROS_ERROR_STREAM("Task '"<<__FUNCTION__ <<"' is incomplete. Exiting"); exit(-1);

  // creating move group to move the arm in free space
  moveit::planning_interface::MoveGroupInterface move_group(config_.group_name);
  move_group.setPlannerId(PLANNER_ID);

  // creating goal joint pose to start of the path
  /*  Fill Code:
   * Goal:
   * - Retrieve the first point in the path.
   * - Save the joint values of the first point into "start_pose".
   * Hint:
   * - The first argument to "getNominalJointPose()" is a "seed" joint pose, used to find a nearby IK solution
   * - The last argument to "getNominalJointPose()" is a "std::vector<double>" variable
   *    where the joint values are to be stored.
   */
  std::vector<double> seed_pose(robot_model_ptr_->getDOF());
  std::vector<double> start_pose;

  descartes_core::TrajectoryPtPtr first_point_ptr = path[0];
  first_point_ptr->getNominalJointPose(seed_pose,*robot_model_ptr_,start_pose);

  // moving arm to joint goal
  move_group.setJointValueTarget(start_pose);
  move_group.setPlanningTime(10.0f);
  moveit_msgs::MoveItErrorCodes result = move_group.move();
  if(result.val != result.SUCCESS)
  {
    ROS_ERROR_STREAM("Move to start joint pose failed");
    exit(-1);
  }

  // creating Moveit trajectory from Descartes Trajectory
  moveit_msgs::RobotTrajectory moveit_traj;

  fromDescartesToMoveitTrajectory(path,moveit_traj.joint_trajectory);
  moveit_traj.joint_trajectory.joint_names = config_.joint_names;

  // moveit_traj.joint_trajectory = toROSJointTrajectory(path, *robot_model_ptr_, config_.joint_names, 1.0);


  // const std::string[7] tmp = ["right_j0", "right_j1", "right_j2", "right_j3", "right_j4", "right_j5", "right_j6"];
  // std::cout <<"tdz1"<<std::endl;
  //
  // std::cout <<"abc: "<< moveit_traj.joint_trajectory.joint_names[0] << std::endl;
  // exit(0);


  // sending robot path to server for execution
  /*  Fill Code:
   * Goal:
   * - Complete the action goal by placing the "moveit_msgs::RobotTrajectory" trajectory in the goal object
   * - Use the action client to send the trajectory for execution.
   * Hint:
   * - The "goal.trajectory" can be assigned the Moveit trajectory.
   * - The "moveit_run_path_client_ptr_->sendGoalAndWait(goal)" sends a trajectory execution request.
   */
  moveit_msgs::ExecuteTrajectoryGoal goal;
  goal.trajectory = moveit_traj;
  // print(goal_trajecotry)
  std::cout << goal.trajectory << std::endl;

  // std::cout<<moveit_traj.joint_trajectory<<std::endl;

  // save data to files like joint_files
  std::vector<std::vector<double>> trajectory_pos(moveit_traj.joint_trajectory.points.size());
  std::ofstream outfile;
  outfile.open(config_.savefile);
  outfile.precision(10);

  for (int i = 0; i< moveit_traj.joint_trajectory.points.size(); i++)
  {
    trajectory_pos.at(i) = {moveit_traj.joint_trajectory.points[i].positions[0],
      moveit_traj.joint_trajectory.points[i].positions[1],
      moveit_traj.joint_trajectory.points[i].positions[2],
      moveit_traj.joint_trajectory.points[i].positions[3],
      moveit_traj.joint_trajectory.points[i].positions[4],
      moveit_traj.joint_trajectory.points[i].positions[5],
      moveit_traj.joint_trajectory.points[i].positions[6]};

    for (int c = 0; c < 7; c ++)
    {
      double temp = moveit_traj.joint_trajectory.points[i].positions[c];
      outfile<<temp<<" ";
    }
    outfile<<std::endl;
  }
  outfile.close();

  ROS_INFO_STREAM("Robot path sent for execution");
  if(moveit_run_path_client_ptr_->sendGoalAndWait(goal) == actionlib::SimpleClientGoalState::SUCCEEDED)
  {
    ROS_INFO_STREAM("Robot path execution completed");
  }
  else
  {
    ROS_ERROR_STREAM("Failed to run robot path with error "<<*moveit_run_path_client_ptr_->getResult());
    exit(-1);
  }

  ROS_INFO_STREAM("Task '"<<__FUNCTION__<<"' completed");

}

}
