#include "move_panda/move_panda.h"

namespace panda
{
//Arm class
Arm::Arm(const std::string PLANNING_GROUP) : move_group(PLANNING_GROUP), visual_tools("")
{ //constructor
    joint_model_group = move_group.getCurrentState()->getJointModelGroup(PLANNING_GROUP);
    link_names = move_group.getLinkNames();
    ROS_INFO_NAMED("move_panda", "Arm reference frame: %s", move_group.getPlanningFrame().c_str());
    ROS_INFO_NAMED("move_panda", "Arm end effector link: %s", move_group.getEndEffectorLink().c_str());

    home_pose = move_group.getCurrentPose().pose;
    home_joint = move_group.getCurrentJointValues();

    moveit_visual_tools::MoveItVisualTools visual_tools_temp(link_names[0]);
    visual_tools = visual_tools_temp;
    visual_tools.loadRemoteControl();
}

geometry_msgs::Pose Arm::getCurrentPose()
{ //get current pose
    ROS_INFO_NAMED("move_panda", "Arm current pose recorded");
    return move_group.getCurrentPose().pose;
}

std::vector<double> Arm::getCurrentJointValues()
{ //get current joint values
    ROS_INFO_NAMED("move_panda", "Arm current joint values recorded");
    return move_group.getCurrentJointValues();
}

void Arm::setMaxVelScalingFactor(double factor)
{                     //set maximum velocity scaling factor
    if (factor > 1.0) //cannot exceed maximum velocity of the robot
    {
        factor = 1.0;
        move_group.setMaxVelocityScalingFactor(factor);
        ROS_INFO_NAMED("move_panda", "Velocity scaling factor greater than 1.0. 1.0 is used");
    }
    else
    {
        move_group.setMaxVelocityScalingFactor(factor);
        ROS_INFO_NAMED("move_panda", "Velocity scaling factor set as %.2f", factor);
    }
}

void Arm::moveTargetPose(geometry_msgs::Pose target_pose, bool step, bool execute)
{ //move to a defined pose, default step = true, default execute = false
    move_group.setPoseTarget(target_pose);
    success = (move_group.plan(plan) == moveit::planning_interface::MoveItErrorCode::SUCCESS);
    ROS_INFO_NAMED("move_panda", "Visualizing plan to target pose %s", success ? "" : "FAILED");
    visual_tools.trigger();

    if (step == true)
    {
        visual_tools.prompt("Press 'next' in the RvizVisualToolsGui window");
    }

    if (execute == true)
    {
        move_group.execute(plan);
        ROS_INFO_NAMED("move_panda", "Plan to target pose exexuted");
        if (step == true)
        {
            visual_tools.prompt("Press 'next' in the RvizVisualToolsGui window");
        }
    }
}

void Arm::moveTargetJoint(std::vector<double> target_joint, bool step, bool execute)
{ //move to target joint values, default step = true, default execute = false
    move_group.setJointValueTarget(target_joint);
    success = (move_group.plan(plan) == moveit::planning_interface::MoveItErrorCode::SUCCESS);
    ROS_INFO_NAMED("move_panda", "Visualizing plan to target joint values %s", success ? "" : "FAILED");
    visual_tools.trigger();

    if (step == true)
    {
        visual_tools.prompt("Press 'next' in the RvizVisualToolsGui window");
    }

    if (execute == true)
    {
        move_group.execute(plan);
        ROS_INFO_NAMED("move_panda", "Plan to target joint values exexuted");
        if (step == true)
        {
            visual_tools.prompt("Press 'next' in the RvizVisualToolsGui window");
        }
    }
}

//Hand class
Hand::Hand(const std::string PLANNING_GROUP) : move_group(PLANNING_GROUP), visual_tools("")
{ //constructor
    joint_model_group = move_group.getCurrentState()->getJointModelGroup(PLANNING_GROUP);
    link_names = move_group.getLinkNames();
    ROS_INFO_NAMED("move_panda", "Hand reference frame: %s", move_group.getPlanningFrame().c_str());

    home_joint = move_group.getCurrentJointValues();

    moveit_visual_tools::MoveItVisualTools visual_tools_temp(link_names[0]);
    visual_tools = visual_tools_temp;
    visual_tools.loadRemoteControl();
}

std::vector<double> Hand::getCurrentJointValues()
{ //get current joint values
    ROS_INFO_NAMED("move_panda", "Hand current joint values recorded");
    return move_group.getCurrentJointValues();
}

void Hand::setMaxVelScalingFactor(double factor)
{                     //set maximum velocity scaling factor
    if (factor > 1.0) //cannot exceed maximum velocity of the robot
    {
        factor = 1.0;
        move_group.setMaxVelocityScalingFactor(factor);
        ROS_INFO_NAMED("move_panda", "Velocity scaling factor greater than 1.0. 1.0 is used");
    }
    else
    {
        move_group.setMaxVelocityScalingFactor(factor);
        ROS_INFO_NAMED("move_panda", "Velocity scaling factor set as %.2f", factor);
    }
}

void Hand::openGripper(bool step, bool execute)
{ //open gripper
    moveit::core::RobotStatePtr current_state = move_group.getCurrentState();
    std::vector<double> hand_position;
    current_state->copyJointGroupPositions(joint_model_group, hand_position);

    hand_position[0] = 0.04;
    move_group.setJointValueTarget(hand_position);
    success = (move_group.plan(plan) == moveit::planning_interface::MoveItErrorCode::SUCCESS);
    ROS_INFO_NAMED("move_panda", "Visualizing plan to open gripper %s", success ? "" : "FAILED");

    if (step == true)
    {
        visual_tools.prompt("Press 'next' in the RvizVisualToolsGui window");
    }

    if (execute == true)
    {
        move_group.execute(plan);
        ROS_INFO_NAMED("move_panda", "Plan to open gripper executed");
        if (step == true)
        {
            visual_tools.prompt("Press 'next' in the RvizVisualToolsGui window");
        }
    }
}

void Hand::closeGripper(bool step, bool execute)
{ //close gripper
    moveit::core::RobotStatePtr current_state = move_group.getCurrentState();
    std::vector<double> hand_position;
    current_state->copyJointGroupPositions(joint_model_group, hand_position);

    hand_position[0] = 0;
    move_group.setJointValueTarget(hand_position);
    success = (move_group.plan(plan) == moveit::planning_interface::MoveItErrorCode::SUCCESS);
    ROS_INFO_NAMED("move_panda", "Visualizing plan to close gripper %s", success ? "" : "FAILED");

    if (step == true)
    {
        visual_tools.prompt("Press 'next' in the RvizVisualToolsGui window");
    }

    if (execute == true)
    {
        move_group.execute(plan);
        ROS_INFO_NAMED("move_panda", "Plan to close gripper executed");
        if (step == true)
        {
            visual_tools.prompt("Press 'next' in the RvizVisualToolsGui window");
        }
    }
}

void Hand::moveGripper(double target_gripper_value, bool step, bool execute)
{ //close gripper
    moveit::core::RobotStatePtr current_state = move_group.getCurrentState();
    std::vector<double> hand_position;
    current_state->copyJointGroupPositions(joint_model_group, hand_position);

    if (target_gripper_value > 0.04 || target_gripper_value < 0)
    {
        ROS_INFO_NAMED("move_panda", "Specified target gripper value exceeds limit. Maximum/minimum gripper position is used");
        if (target_gripper_value > 0.04)
        {
            target_gripper_value = 0.04;
        }
        else
        {
            target_gripper_value = 0;
        }
    }

    hand_position[0] = target_gripper_value;
    move_group.setJointValueTarget(hand_position);
    success = (move_group.plan(plan) == moveit::planning_interface::MoveItErrorCode::SUCCESS);
    ROS_INFO_NAMED("move_panda", "Visualizing plan to move gripper %s", success ? "" : "FAILED");

    if (step == true)
    {
        visual_tools.prompt("Press 'next' in the RvizVisualToolsGui window");
    }

    if (execute == true)
    {
        move_group.execute(plan);
        ROS_INFO_NAMED("move_panda", "Plan to move gripper executed");
        if (step == true)
        {
            visual_tools.prompt("Press 'next' in the RvizVisualToolsGui window");
        }
    }
}
} // namespace panda