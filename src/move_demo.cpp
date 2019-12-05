#include <move_panda/move_panda.h>

int main(int argc, char **argv)
{
  ros::init(argc, argv, "move_demo");
  ros::NodeHandle node_handle;
  ros::AsyncSpinner spinner(1);
  spinner.start();

  panda::Arm arm("panda_arm");
  panda::Hand hand("hand");
  geometry_msgs::Pose arm_current_pose = arm.getCurrentPose();
  std::vector<double> arm_current_joint = arm.getCurrentJointValues();
  std::vector<double> hand_current_joint = hand.getCurrentJointValues();
  arm.visual_tools.prompt("Press 'next' in the RvizVisualToolsGui window");

  geometry_msgs::Pose target_pose1;
  target_pose1.orientation.w = 1.0;
  target_pose1.position.x = 0.28;
  target_pose1.position.y = -0.2;
  target_pose1.position.z = 0.5;

  //For all the move methods, the two boolean values are to specify:
  //1. if you want to advance throught planning and execution
  //using the Next button on Rviz GUI tool (first bool)
  //2. if you wnat to plan the motion or plan and execute it (second bool)
  //e.g. arm.moveTargetPose(target_pose1, true, true) means you want to
  //use Next to setp through the process AND you wand to execute the motion
  //after planning.
  //If you don't specify them, default is true, false. It means that you
  //will need to use Next and moveit will plan but not execute the motion for you

  arm.setMaxVelScalingFactor(0.1);                 //change joint speed
  arm.moveTargetPose(target_pose1, true, true);    //move to target pose
  arm.setMaxVelScalingFactor(1.2);                 //change joint speed, maximum is 1
  arm.moveTargetJoint(arm.home_joint, true, true); //move to desired joint values

  hand.setMaxVelScalingFactor(0.25);  //change gripper speed
  hand.openGripper(true, true);       //open gripper
  hand.setMaxVelScalingFactor(1);     //change gripper speed, maximum is 1
  hand.closeGripper(true, true);      //close gripper
  hand.moveGripper(0.02, true, true); //move gripper to target value

  arm_current_pose = arm.getCurrentPose();
  arm.setHandOrientConstraint(); //set current end effector orientatoin as the constraint
  arm_current_pose.position.x -= 0.1;
  arm_current_pose.position.y -= 0.1;
  arm_current_pose.position.z -= 0.1;
  arm_current_pose.orientation.w = 1;
  arm.moveTargetPoseCon(arm_current_pose, true, true); //move to target pose with orientation constraint
  arm.clearPathConstraint();

  arm.moveTargetJoint(arm.home_joint, true, true);
  arm.setJointConstraint(arm.joint_names[1]);      //set panda_joint2 value as a joint constraint
  arm.moveTargetPoseCon(target_pose1, true, true); //move to target pose with joint constraint
  arm.clearPathConstraint();
  arm.moveTargetJoint(arm.home_joint, true, true);

  geometry_msgs::Pose inter_pose1 = arm.getCurrentPose();
  arm.setMaxVelScalingFactor(0.1);
  inter_pose1.position.z -= 0.2;
  arm.setWaypoints(inter_pose1);
  inter_pose1.position.y -= 0.2;
  arm.setWaypoints(inter_pose1);
  inter_pose1.position.z += 0.2;
  inter_pose1.position.y += 0.2;
  inter_pose1.position.x -= 0.2;
  arm.setWaypoints(inter_pose1);
  arm.moveCartesianPath(0, 0.01, true, true);

  ros::shutdown();
  return 0;
}