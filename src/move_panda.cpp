#include "move_panda/move_panda.h"

namespace panda
{
//Arm class
Arm::Arm(const std::string PLANNING_GROUP) : move_group(PLANNING_GROUP), visual_tools(move_group.getLinkNames()[0])
{ //constructor
    joint_model_group = move_group.getCurrentState()->getJointModelGroup(PLANNING_GROUP);
    link_names = move_group.getLinkNames();
    joint_names = move_group.getJointNames();
    default_plan_time = move_group.getPlanningTime();
    planning_frame = move_group.getPlanningFrame();
    ROS_INFO_NAMED("move_panda", "Arm reference frame: %s", planning_frame.c_str());
    ROS_INFO_NAMED("move_panda", "Arm end effector link: %s", move_group.getEndEffectorLink().c_str());

    home_pose = move_group.getCurrentPose().pose;
    home_joint = move_group.getCurrentJointValues();
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

double Arm::getJointValue(std::string joint_name)
{ //get joint value of the joint given by joint_name
    std::vector<double> current_joint_values = getCurrentJointValues();
    if (joint_names.size() != current_joint_values.size())
    { //if there is mismatch between number of jaint names and number of jaint values
        ROS_INFO_NAMED("move_panda", "Number of joint names different from number of joint_values");
        return -1;
    }
    else
    {
        for (int joint_index = 0; joint_index < current_joint_values.size(); joint_index++)
        {
            if (joint_name == joint_names[joint_index])
            {
                ROS_INFO_NAMED("move_panda", "Cuurent %s value %f is obtained", joint_name.c_str(), current_joint_values[joint_index]);
                return current_joint_values[joint_index];
            }
        }
    }
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

void Arm::setPlanner(std::string planner)
{ //set the planner of moveit
    move_group.setPlannerId(planner);
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

void Arm::moveTargetPoseCon(geometry_msgs::Pose target_pose, bool step, bool execute, double plan_time)
{ //move to a target pose with constraint
    if (orient_constraint == false && joint_constraint == false)
    {
        ROS_INFO_NAMED("move_group", "No constraint set");
    }

    else
    {
        move_group.setPathConstraints(path_constraints);
        move_group.setPlanningTime(plan_time);

        if (orient_constraint == true)
        { //use orientation constraint as the target orientation, otherwise there will be error
            ROS_INFO_NAMED("move_panda", "Orientaiton constraint will be set as target orientation");
            target_pose.orientation.w = ocm.orientation.w;
            target_pose.orientation.x = ocm.orientation.x;
            target_pose.orientation.y = ocm.orientation.y;
            target_pose.orientation.z = ocm.orientation.z;
        }
        if (step == true)
        {
            visual_tools.prompt("Press 'next' in the RvizVisualToolsGui window");
        }

        moveTargetPose(target_pose, step, execute);
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

void Arm::setHandOrientConstraint(std::array<double, 3> axis_tolerance, double weight)
{                                                      //set orienation constraint to current end effector orientation
    ocm.link_name = link_names[link_names.size() - 2]; //assign panda_link7 as the ocm link name
    ocm.header.frame_id = planning_frame;              //assign the planning frame as the ocm header frame id
    ocm.orientation = getCurrentPose().orientation;    //assign current orientation to constriant
    ocm.absolute_x_axis_tolerance = axis_tolerance[0];
    ocm.absolute_y_axis_tolerance = axis_tolerance[1];
    ocm.absolute_z_axis_tolerance = axis_tolerance[2];
    ocm.weight = weight; //higher weight means this constraint is more important compared to others
    path_constraints.orientation_constraints.push_back(ocm);
    ROS_INFO_NAMED("move_panda", "End effector orientation constraint set");
    orient_constraint = true;
}

void Arm::setJointConstraint(std::string joint_name, double upper_tol, double lower_tol, double weight)
{ //set joint value constraint to the current joint value of a given joint
    jcm.joint_name = joint_name;
    jcm.position = getJointValue(joint_name);
    jcm.tolerance_above = upper_tol;
    jcm.tolerance_below = lower_tol;
    jcm.weight = weight;
    path_constraints.joint_constraints.push_back(jcm);
    ROS_INFO_NAMED("move_panda", "%s value constraint set", joint_name.c_str());
    joint_constraint = true;
}

void Arm::clearPathConstraint()
{
    move_group.clearPathConstraints();
    path_constraints.orientation_constraints.clear();
    move_group.setPlanningTime(default_plan_time);
    ROS_INFO_NAMED("move_panda", "Path constraint clear");
    orient_constraint = false;
}

//Hand class
Hand::Hand(const std::string PLANNING_GROUP) : move_group(PLANNING_GROUP), visual_tools(move_group.getLinkNames()[0])
{ //constructor
    joint_model_group = move_group.getCurrentState()->getJointModelGroup(PLANNING_GROUP);
    link_names = move_group.getLinkNames();
    ROS_INFO_NAMED("move_panda", "Hand reference frame: %s", move_group.getPlanningFrame().c_str());

    home_joint = move_group.getCurrentJointValues();
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

void Hand::setPlanner(std::string planner)
{ //set the planner of moveit
    move_group.setPlannerId(planner);
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