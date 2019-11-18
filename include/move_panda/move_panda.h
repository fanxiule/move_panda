#ifndef PANDA_MOTION_H
#define PANDA_MOTION_H

#include <moveit/move_group_interface/move_group_interface.h>
#include <moveit_msgs/DisplayRobotState.h>
#include <moveit_msgs/DisplayTrajectory.h>
#include <moveit_visual_tools/moveit_visual_tools.h>

namespace panda
{
class Arm
{
private:
    //private variables
    std::vector<std::string> link_names;                       //store link names
    moveit::planning_interface::MoveGroupInterface move_group; //move_group object storing joint and link info
    const robot_state::JointModelGroup *joint_model_group;     //pointer to joint_model_group
    moveit::planning_interface::MoveGroupInterface::Plan plan; //plan for motion planning
    bool success;                                              //if success == true, motion planning is successful

public:
    //public variables
    geometry_msgs::Pose home_pose;                       //store home pose
    std::vector<double> home_joint;                      //store joint value at home position
    moveit_visual_tools::MoveItVisualTools visual_tools; //visual tools to use the GUI in rviz

    //public methods
    //basic methods
    Arm(const std::string PLANNING_GROUP);       //constructor
    geometry_msgs::Pose getCurrentPose();        //get current pose
    std::vector<double> getCurrentJointValues(); //get current joint values
    void setMaxVelScalingFactor(double factor);  //set maximum velocity scaling factor

    //for all move methods: step == true: use Next in GUI to advance, execute == false: planning only
    void moveTargetPose(geometry_msgs::Pose target_pose, bool step = true, bool execute = false);   //move to a target pose
    void moveTargetJoint(std::vector<double> target_joint, bool step = true, bool execute = false); //move to target joint values
};

class Hand
{
private:
    //private variables
    std::vector<std::string> link_names;                       //store link names
    moveit::planning_interface::MoveGroupInterface move_group; //move_group object storing joint and link info
    moveit_visual_tools::MoveItVisualTools visual_tools;       //visual tools to use the GUI in rviz
    const robot_state::JointModelGroup *joint_model_group;     //pointer to joint_model_group
    moveit::planning_interface::MoveGroupInterface::Plan plan; //plan for motion planning
    bool success;                                              //if success == true, motion planning is successful

public:
    //public variables
    geometry_msgs::Pose home_pose;  //store home pose
    std::vector<double> home_joint; //store joint value at home position

    //public methods
    Hand(const std::string PLANNING_GROUP);      //constructor
    std::vector<double> getCurrentJointValues(); //get current joint values
    void setMaxVelScalingFactor(double factor);  //set maximum velocity scaling factor

    //for all move methods: step == true: use Next in GUI to advance, execute == false: planning only
    void openGripper(bool step = true, bool execute = false);                              //open gripper
    void closeGripper(bool step = true, bool execute = false);                             //close gripper
    void moveGripper(double target_gripper_value, bool step = true, bool execute = false); //move gripper to target value
};
} // namespace panda
#endif