
#include <helix_test/demo_application.h>

/* LOAD PARAMETERS
  Goal:
    - Load missing application parameters from the ros parameter server.
    - Use a private NodeHandle in order to load parameters defined in the node's namespace.

  Hints:
    - Look at how the 'config_' structure is used to save the parameters.
    - A private NodeHandle can be created by passing the "~" string to its constructor.
*/

namespace helix_test
{

void DemoApplication::loadParameters()
{
  ros::NodeHandle ph("~");

  // creating handle with public scope
  ros::NodeHandle nh;

  if(ph.getParam("group_name",config_.group_name) &&
      ph.getParam("tip_link",config_.tip_link) &&
      ph.getParam("base_link",config_.base_link) &&
      ph.getParam("world_frame",config_.world_frame) &&
      ph.getParam("trajectory/seed_pose",config_.seed_pose) &&
      ph.getParam("visualization/min_point_distance",config_.min_point_distance) &&
      ph.getParam("filename", config_.filename) &&
      ph.getParam("offset", config_.offset) &&
      ph.getParam("savefile", config_.savefile) &&
      nh.getParam("controller_joint_names",config_.joint_names)
     )
  {
    ROS_INFO_STREAM("Loaded application parameters");
  }
  else
  {
    ROS_ERROR_STREAM("Failed to load application parameters");
    exit(-1);
  }

  ROS_INFO_STREAM("Task '"<<__FUNCTION__<<"' completed");

}

}
