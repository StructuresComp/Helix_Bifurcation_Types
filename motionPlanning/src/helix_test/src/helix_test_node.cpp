/*
 * helix_test_node.cpp
 *
 *  Created on: Apr 10, 2015
 *      Author: Jorge Nicho
 */

#ifdef __i386__
  #pragma message("i386 Architecture detected, disabling EIGEN VECTORIZATION")
  #define EIGEN_DONT_VECTORIZE
  #define EIGEN_DISABLE_UNALIGNED_ARRAY_ASSERT
#else
  #pragma message("64bit Architecture detected, enabling EIGEN VECTORIZATION")
#endif

#include <helix_test/demo_application.h>

int main(int argc,char** argv)
{
  ros::init(argc,argv,"helix_test");
  ros::AsyncSpinner spinner(2); // use 2 threads
  spinner.start();

  // creating application
  helix_test::DemoApplication application;

  // loading parameters
  application.loadParameters();

  // initializing ros components
  application.initRos();

  // initializing descartes
  application.initDescartes();
  // moving to home position
  // application.moveHome();

  // generating trajectory
  helix_test::DescartesTrajectory traj;
  application.generateTrajectory(traj);
  //
  //
  //
  // // planning robot path
  helix_test::DescartesTrajectory output_path;
  application.planPath(traj,output_path);
  std::cout<<"tdz2"<<std::endl;
  //
  //
  // // running robot path
  application.runPath(output_path);

  // exiting ros node
  spinner.stop();



  return 0;
}
