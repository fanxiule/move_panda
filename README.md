# move_group

This package was written so that users can utilize MoveIt! more easily to interact with the Franke Emika Panda robotic arm.

## Installation

Before building this package, users should download and build the ``panda_moveit_config`` package first. Then users can clone this package to the source folder of their ROS workspace. In this document, it is assumed that users save both of these packages in catkin_ws/src

## Demo

After building this package, users can take a look at move_demo.cpp and run the following command to see a demo:

    cd catkin_ws
    roslaunch panda_moveit_config demo.launch

In a new shell:

    rosrun move_panda move_demo

Then users can advance each step by using the ``Next`` button in the Rviz GUI tool.

## Using the Package

The following funcitons are provided by this package:

- **Arm Class**

  - Constructor to initialize a move group to move the arm with given planning group (usually it is `"panda_arm"`)
  
            Arm(const std::string PLANNING_GROUP)

  - Obtain the current pose of the arm

            geometry_msgs::Pose getCurrentPose()

  - Obtain the current joint values of the arm

            std::vector<double> getCurrentJointValues()

  - Change the maximum joint velocity to number specified in ``double factor``

            void setMaxVelScalingFactor(double factor)

  - Move to the target pose given by ``geometry_msgs::Pose target_pose``

            void moveTargetPose(geometry_msgs::Pose target_pose, bool step = true, bool execute = false)
 
  - Note that the default value for ``bool step`` is true and for ``bool execute`` is false. If these two arguments are not specified when calling the method, users need to use the Next button in the GUI tool to proceed to the next operation, and MoveIt! only plans for the motion but does not execute it. This is true for all methods in this package that can move the arm or the gripper
  - Move to the target joint values given by ``std::vector<double> target_joint``

            void moveTargetJoint(std::vector<double> target_joint, bool step = true, bool execute = false)

- **Hand Class**

  - Constructor to initialize a move group to move the arm with given planning group (usually it is `"hand"`)
  
            Hand(const std::string PLANNING_GROUP)

  - Obtain the current joint values of the gripper

            std::vector<double> getCurrentJointValues()

  - Change the maximum joint velocity to number specified in ``double factor``

            void setMaxVelScalingFactor(double factor)
   
  - Open the gripper completely

            void openGripper(bool step = true, bool execute = false)

  - Close the gripper completely

            void closeGripper(bool step = true, bool execute = false);

  - Move gripper to the location specified by ``std::vector<double> target_gripper_value``

            void moveGripper(double target_gripper_value, bool step = true, bool execute = false)
