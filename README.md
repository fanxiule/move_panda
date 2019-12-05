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

  - Change the planner for MoveIt! to the one specified by ``std::string planner``

            void setPlanner(std::string planner);

  - Move to the target pose given by ``geometry_msgs::Pose target_pose``

            void moveTargetPose(geometry_msgs::Pose target_pose, bool step = true, bool execute = false)
 
  - Note that the default value for ``bool step`` is true and for ``bool execute`` is false. If these two arguments are not specified when calling the method, users need to use the Next button in the GUI tool to proceed to the next operation, and MoveIt! only plans for the motion but does not execute it. This is true for all methods in this package that can move the arm or the gripper
  
  - Move to the target joint values given by ``std::vector<double> target_joint``

            void moveTargetJoint(std::vector<double> target_joint, bool step = true, bool execute = false)

  - Set current end effector orientation as the orienation constraint. ``std::array<double, 3> axis_tolerance`` specifies axis-angle error tolerance in x, y and z axis, respectively. The default tolerance for all thress axes is ``0.1``. ``double weight`` denotes the importance of this constraint relative to other constriants. Default value is ``1.0`` indicating the constraint is very important

            void setHandOrientConstraint(std::array<double, 3> axis_tolerance = {0.1, 0.1, 0.1}, double weight = 1.0)

  - Set current joint value of the given joint ``std::string joint_name`` as a constraint. ``double upper_tol`` and ``double lower_tol`` are the upper and lower tolernces of this constraint. Their default value is ``0.1``. ``double weight`` denotes the importance of this constraint relative to others
  
            void setJointConstraint(std::string joint_name, double upper_tol = 0.1, double lower_tol = 0.1, double weight = 1.0); 

  - Move to the target pose given by ``geometry_msgs::Pose target_pose`` with path constraint. ``double plan_time`` overrides the default planning time of the MoveIt! solver. Default value is ``10.0`` seconds

            void moveTargetPoseCon(geometry_msgs::Pose target_pose, bool step = true, bool execute = false, double plan_time = 10.0)

  - Clear all constraints and reset planning time back to the default value.

            void clearPathConstraint()

  - Set waypoints with the given intermediate pose by ``geometry_msgs::Pose inter_pose``. If this method is called the first time, the current pose will be set as the first waypoint when the intermediate pose will be set

            void setWaypoints(geometry_msgs::Pose inter_pose);

  - Compute a cartesian path as per the waypoints set by the previous method, and move the robot according to the path. The path is calculated based on the given ``double jump_threshold`` with default value ``0.0`` and ``double eef_step`` with default value

            void moveCartesianPath(double jump_threshold = 0.0, double eef_step = 0.01, bool step = true, bool execute = false);

- **Hand Class**

  - Constructor to initialize a move group to move the arm with given planning group (usually it is `"hand"`)
  
            Hand(const std::string PLANNING_GROUP)

  - Obtain the current joint values of the gripper

            std::vector<double> getCurrentJointValues()

  - Change the maximum joint velocity to number specified in ``double factor``

            void setMaxVelScalingFactor(double factor)

  - Change the planner for MoveIt! to the one specified by ``std::string planner``

            void setPlanner(std::string planner);

  - Open the gripper completely

            void openGripper(bool step = true, bool execute = false)

  - Close the gripper completely

            void closeGripper(bool step = true, bool execute = false);

  - Move gripper to the location specified by ``double target_gripper_value``

            void moveGripper(double target_gripper_value, bool step = true, bool execute = false)
