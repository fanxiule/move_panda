#ifndef PANDA_MOTION_H
#define PANDA_MOTION_H

#include <ros/ros.h>
#include <moveit/move_group_interface/move_group_interface.h>
#include <moveit_msgs/DisplayRobotState.h>
#include <moveit_msgs/DisplayTrajectory.h>
#include <moveit_visual_tools/moveit_visual_tools.h>
#include <moveit/trajectory_processing/iterative_time_parameterization.h>
#include <moveit/robot_trajectory/robot_trajectory.h>
#include <moveit/robot_model_loader/robot_model_loader.h>
#include <moveit/robot_model/robot_model.h>
#include <moveit/robot_state/robot_state.h>

namespace panda
{
class Arm
{
private:
    //private variables
    moveit::planning_interface::MoveGroupInterface move_group; //move_group object storing joint and link info
    const robot_state::JointModelGroup *joint_model_group;     //pointer to joint_model_group
    moveit::planning_interface::MoveGroupInterface::Plan plan; //plan for motion planning
    moveit_msgs::OrientationConstraint ocm;                    //orientation constraint
    moveit_msgs::JointConstraint jcm;                          //joint value constraint
    moveit_msgs::Constraints path_constraints;                 //path constraint
    std::string planning_frame;                                //planning frame of the robot
    std::string group;
    std::vector<geometry_msgs::Pose> waypoints; //store waypoints
    bool success = false;                       //if success == true, motion planning is successful
    bool orient_constraint = false;             //if orient_constraint == true, orientation constraint is set
    bool joint_constraint = false;              //if joint_constraint == true, joint constraint is set
    bool set_waypoints = false;                 //if set_waypoints = true, waypoints have already been set
    double default_plan_time;                   //default planning time of the solver
    double vel_factor;                          //store volocity scaling factor

public:
    //public variables
    std::vector<std::string>
        link_names;                                      //store link names
    std::vector<std::string> joint_names;                //store joint names
    geometry_msgs::Pose home_pose;                       //store home pose
    std::vector<double> home_joint;                      //store joint value at home position
    moveit_visual_tools::MoveItVisualTools visual_tools; //visual tools to use the GUI in rviz

    //public methods
    //basic methods
    Arm(const std::string PLANNING_GROUP);        //constructor
    geometry_msgs::Pose getCurrentPose();         //get current pose
    std::vector<double> getCurrentJointValues();  //get current joint values
    double getJointValue(std::string joint_name); //get joint value of the given joint
    void setMaxVelScalingFactor(double factor);   //set maximum velocity scaling factor
    void setPlanner(std::string planner);         //set the planner for moveit

    //for all move methods: step == true: use Next in GUI to advance, execute == false: planning only
    void moveTargetPose(geometry_msgs::Pose target_pose, bool step = true, bool execute = false);                             //move to a target pose
    void moveTargetPoseCon(geometry_msgs::Pose target_pose, bool step = true, bool execute = false, double plan_time = 10.0); //move to a target pose with constraint
    void moveTargetJoint(std::vector<double> target_joint, bool step = true, bool execute = false);                           //move to target joint values
    void setHandOrientConstraint(std::array<double, 3> axis_tolerance = {0.1, 0.1, 0.1}, double weight = 1.0);                //set end effector orienation constraint to current end effector orientation
    void setJointConstraint(std::string joint_name, double upper_tol = 0.1, double lower_tol = 0.1, double weight = 1.0);     //set joint value constraint to the current joint value of a given joint
    void clearPathConstraint();                                                                                               //clear orientation constriant
    void setWaypoints(geometry_msgs::Pose inter_pose);                                                                        //set waypoints using intermediate poses
    void moveCartesianPath(double jump_threshold = 0.0, double eef_step = 0.01, bool step = true, bool execute = false);      //move robot according to cartesian path
};

class Hand
{
private:
    //private variables
    moveit::planning_interface::MoveGroupInterface move_group; //move_group object storing joint and link info
    moveit_visual_tools::MoveItVisualTools visual_tools;       //visual tools to use the GUI in rviz
    const robot_state::JointModelGroup *joint_model_group;     //pointer to joint_model_group
    moveit::planning_interface::MoveGroupInterface::Plan plan; //plan for motion planning
    bool success;                                              //if success == true, motion planning is successful

public:
    //public variables
    std::vector<std::string> link_names; //store link names
    geometry_msgs::Pose home_pose;       //store home pose
    std::vector<double> home_joint;      //store joint value at home position

    //public methods
    Hand(const std::string PLANNING_GROUP);      //constructor
    std::vector<double> getCurrentJointValues(); //get current joint values
    void setMaxVelScalingFactor(double factor);  //set maximum velocity scaling factor
    void setPlanner(std::string planner);        //set the planner for moveit

    //for all move methods: step == true: use Next in GUI to advance, execute == false: planning only
    void openGripper(bool step = true, bool execute = false);                              //open gripper
    void closeGripper(bool step = true, bool execute = false);                             //close gripper
    void moveGripper(double target_gripper_value, bool step = true, bool execute = false); //move gripper to target value
};
} // namespace panda
#endif