# Helix_Bifurcation_Types

Codes for studying the pitch-fork bifurcations of a initially rod with helical centerline.

<p align="center">
<img src="images/introduction.png" alt>
<br>
<em> Figure 1. Studied systems and various types of pitch-fork bifurcations. </em>
</p>

***

## Simulation Codes
The ``simCodes`` folder contains the required codes for executing numeric experiments along various exploring directions in the parameter space as shown in Fig. 1(a).

### Dependencies
Install the following C++ dependencies:
- [Eigen](http://eigen.tuxfamily.org/index.php?title=Main_Page)
  - Eigen is used for various linear algebra operations.
    ```bash
    cd eigen-3.4.0 && mkdir build && cd build
    cmake ..
    sudo make install
    ```

- [OpenGL / GLUT](https://www.opengl.org/)
  - OpenGL / GLUT is used for rendering the knot through a simple graphic.
  - Simply install through apt package manager:
      ```bash
    sudo apt-get install libglu1-mesa-dev freeglut3-dev mesa-common-dev
    ```
- Lapack (*usually preinstalled on your computer*)

***
### Compiling
After completing all the necessary above steps, go to the folder of ``simCodes``.
```bash
mkdir build && cd build
cmake ..
make -j4
```

***

### Setting Parameters

All simulation parameters are set through a parameter file ```option.txt```. A template file ```template_option.txt``` is provided that can be used to construct ```option.txt```. All the parameters are defaulted in this project.

Specifiable parameters are as follows (we use SI units):
- ```RodLength``` - Contour length of the rod.
- ```youngM``` - Young's modulus of the rod's material.
- ```rodRadius``` - Cross-sectional radius of the rod.
- ```Poisson``` - Poisson's ratio of the rod's material.
- ```tol``` and ```stol``` - Small numbers used in solving the linear system. Fraction of a percent, e.g. 1.0e-3, is often a good choice.
- ```density``` - Density of the rod's material.
- ```maxIter``` - Maximum number of iterations allowed before the solver quits. 
- ```gVector``` - 3x1 vector specifying acceleration due to gravity.
- ```viscosity``` - Viscosity for applying damping forces.
- ```render (0 or 1) ```- Flag indicating whether OpenGL visualization should be rendered.
- ```saveData (0 or 1)``` - Flag indicating whether pull forces and rod end positions should be reocrded.
- ```v_constant``` - Rate of the boundary conditions along the exploring direction in the parameter space.
- ```c1``` and ```c2``` Define the exploring direction.
- ```kapB``` The curvature of the buckling point obtained from the theoretical analysis


***
### Running the Simulation
In this folder, we offer MatLab file ```ruSim.m``` for simple execution of the codes. Users can claim the exploring direction ``S`` and Poisson's ratio ``Poisson`` in the beginning of the matlab code. Then, the matlab file will execute the simulation program. Here, we have the variable ``experiment``, which is used to control how we would like to run the simulation. If ``experiment = 0``, simulation program will the executed and show the plot of exploring distance v.s. difference between rod's configuration and prescribed helical centerline. If ``experiment = 1``, simulation program will generate the input file for the following robotic motion planning.

If the user does not have license for MatLab, they can set parameters in the ``option.txt''. Once parameters are set to your liking, the simulation can be ran from the terminal by running the provided script:
```bash
cd simCodes
./simDER option.txt
```

***

## Motion Planning Codes
The ``motionPlanning`` folder is a ros workspace which contains the required codes for generate joint trajectory of a sawyer robot for doing robotic experiments along various exploring directions in the parameter space as shown in Fig. 1(a).

***
### Dependencies
Install the following dependencies:
- [ROS](http://wiki.ros.org/Installation/Ubuntu)
  - The Robot Operating System (ROS) is a set of software libraries and tools that help you build robot applications. 
  - The installation instructions of ROS in ubuntu are [Here](http://wiki.ros.org/Installation/Ubuntu). Following the instructions and install all the ROS in the computer.

- [User log-in file]
  - We need to add the environment variable to the ``.bashrc`` file in Ubuntu system.
  - Open a terminal and then do the following operations:
    ```bash
    vim .bashrc
    ```
    Adding two lines in the bottom of the ``.bashrc`` file:
    ```bash
       source /opt/ros/$ROS_VERSION/setup.bash
       source /$MOTIONPLANNINGPATH$/devel/setup.bash
    ```
    Here ``$ROS_VERSION$`` is the version of ros, i.e. ``$ROS_VERSION`` should be ``noetic`` for Ubuntu 20.04. ``$MOTIONPLANNINGPATH`` is the path to the 
    ``motionPlanning`` folder.

***
### Compiling
After completing all the necessary above steps, go to the folder of ``motionPlanning``.
```bash
catkin_make
```
If a error about ``Unable to find either executable 'empy' or Python module 'em'...`` occurs. Compile the workspace with
```bash
catkin_make -DPYTHON_EXECUTABLE=/usr/bin/python3
```
***

### Setting Parameters

All motion planning parameters are set through a launch file ```helix_run.launch```. The launch file locates in the subfolder ``motionPlanning/src/helix_text/launch/helix_run.launch``. There are a few parameters in the launch file and we justify the meaning of each parameters here.

- ```group_name``` - Name of planning group(robot).
- ```tip_link``` - The frame of the manipulator.
- ```base_link``` - The frame of the robot base (world frame).
- ```world_frame``` - The world frame.
- ```filename``` - The path of the generated input simulation data file.
- ```savefile``` - The path to save the planned joint trajectory.
- ```offset``` - 3x1 vector for define the position trajectory in the world frame.
- ```trajectory/seed_pose``` - A joint seed for solving the trajectory
- ```visualization/min_point_distance```- The distance of the visualized discrete trajectory in rviz.

When doing the motion planning, the main parameters need to be adjusted is ``filename``, ``savefile``, and ``offset``.

***
### Running the planning codes
After completeing the above steps, the user should open a terminal and go to the subfolder ``motionPlanning``, then running the provided script:
```bash
 roslaunch helix_test helix_setup.launch 
```
to establish and visualize the planning environment in rviz.

Then, the user should generate the input file with simulation by executing the simulation codes with the ``experiment = 1``. Then, the user should adjust ``filename`` in the ``helix_run.launch`` to the path of the input file and adjust ``savefile`` to the path to store the planned joint trajectory solutions. To do the motion planning, the user should run the provided script in another opened terminal:
```bash
 roslaunch helix_test helix_run.launch 
```

***

## Data Processing
The ``DataProcessing`` folder contains the codes and files for classify the buckling points from experimental data. 

### Dependencies
User should have `MaTLaB` installed in the computer. This script also uses the ``Robotics System Toolbox``
- [Robotics System Toolbox](https://www.mathworks.com/products/robotics.html)
  - Robotics System Toolbox rovides tools and algorithms for designing, simulating, testing, and deploying manipulator and mobile robot applications. 
  - Free trial of Robotics System Toolbox in `MatLab` are [Here](https://www.mathworks.com/campaigns/products/trials.html). Following the instructions and install them in the computer.

### Data Explanation
The ``Joints`` folder contains the joint trajectory inputted to the robots. The ``Simulations`` folder contains the simulation data. The ``Observations`` contains the raw experimental data. For the files in the ``Observation`` folder, each row is the sampled data, which is a 1 x 12 vector. The 1st to 7th elements are the joints positions expressing the robot's configuration; the 8th to 10th elements are the raw 3D position of the attached marker in the camera frame; the 11th to 12th are the pixel coordinate of the attached marker in the image domain.

### Running the Codes
The ``PlotFunc`` folder contains the main scripts and relevant functions. The main script is named as ``processAllData.m``. Users can adjust the boolean variable ``withTwist`` to ``True`` or ``False`` to see the processed results of the manipulated rod with external twisting moment or without external moment.

***