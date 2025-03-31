#include <ros/ros.h>
#include <geometry_msgs/PoseStamped.h>
#include <sensor_msgs/Joy.h>

const static double kTriggerThreshold = 0.95;
bool is_hand_trigger_pressed = false;
bool is_index_trigger_pressed = false;
bool is_button_X_pressed_over_3s_ = false;
bool is_button_Y_pressed_ = false;

geometry_msgs::PoseStamped previous_left_joy_pose;
geometry_msgs::PoseStamped current_left_joy_pose;

geometry_msgs::PoseStamped initial_gripper_pose;
geometry_msgs::PoseStamped relative_gripper_pose;
geometry_msgs::PoseStamped target_gripper_pose;

void leftJoyPoseCallback(const geometry_msgs::PoseStamped::ConstPtr& msg)
{
  current_left_joy_pose.pose = msg->pose;
}

void leftJoyCallback(const sensor_msgs::Joy::ConstPtr& msg)
{
  // axes[0] and axes[1] are for mobile platform teleoperation
  switch (msg->axes.size()) {
    case 4:
      // [3] primary hand trigger
      is_hand_trigger_pressed = (msg->axes[3] > kTriggerThreshold);
    case 3:
      // [2] primary index trigger
      is_index_trigger_pressed = (msg->axes[2] > kTriggerThreshold);
      break;
    default:
      ROS_WARN_ONCE("Mismatch number of left joystick axes (%lu).", msg->axes.size());
      ROS_WARN_ONCE("  Please check your joystick(s) setup or rosbridge connection.");
      break;
  }

  switch (msg->buttons.size()) {
    case 2:
      // [1] Y
      is_button_Y_pressed_ = (msg->buttons[1] == 1);
    case 1:
      // [0] X
      static ros::Time button_X_pressed_time;
      if (msg->buttons[0] == 1) {
        if (button_X_pressed_time.isZero()) {
          button_X_pressed_time = ros::Time::now();
        } else if ((ros::Time::now() - button_X_pressed_time).toSec() > 3.0) {
          is_button_X_pressed_over_3s_ = true;
        }
      } else {
        button_X_pressed_time = ros::Time(0);
      }
      break;
    default:
      ROS_WARN_ONCE("Mismatch number of left joystick buttons (%lu).", msg->axes.size());
      ROS_WARN_ONCE("  Please check your joystick(s) setup or rosbridge connection.");
      break;
  }
}

int main(int argc, char **argv)
{
  ros::init(argc, argv, "test_cartesian_control");
  ros::NodeHandle nh;
  ros::Publisher target_frame_pub = nh.advertise<geometry_msgs::PoseStamped>("target_frame", 10);
  ros::Subscriber left_joy_pose_sub = nh.subscribe("/unity/joy_pose/left", 10, leftJoyPoseCallback);
  ros::Subscriber left_joy_sub = nh.subscribe("/unity/joy/left", 10, leftJoyCallback);
  ros::Rate loop_rate(25);

  initial_gripper_pose.header.frame_id = "base_link";
  initial_gripper_pose.pose.position.x = 0.712;
  initial_gripper_pose.pose.position.y = -0.122;
  initial_gripper_pose.pose.position.z = 1.043;
  initial_gripper_pose.pose.orientation.x = 0.5;
  initial_gripper_pose.pose.orientation.y = 0.5;
  initial_gripper_pose.pose.orientation.z = 0.5;
  initial_gripper_pose.pose.orientation.w = 0.5;
  target_gripper_pose = initial_gripper_pose;
  target_frame_pub.publish(initial_gripper_pose);

  bool begin_teleoperation = true;
  while (ros::ok()) {
    if (is_index_trigger_pressed && is_hand_trigger_pressed) {
      if (begin_teleoperation) {
        ROS_INFO_STREAM("Begin the teleoperation...");
        previous_left_joy_pose.pose = current_left_joy_pose.pose;
        begin_teleoperation = false;
      }

      relative_gripper_pose.pose.position.x = current_left_joy_pose.pose.position.x - previous_left_joy_pose.pose.position.x;
      relative_gripper_pose.pose.position.y = current_left_joy_pose.pose.position.y - previous_left_joy_pose.pose.position.y;
      relative_gripper_pose.pose.position.z = current_left_joy_pose.pose.position.z - previous_left_joy_pose.pose.position.z;

      target_gripper_pose.pose.position.x = initial_gripper_pose.pose.position.x + relative_gripper_pose.pose.position.x * 0.8;
      target_gripper_pose.pose.position.y = initial_gripper_pose.pose.position.y + relative_gripper_pose.pose.position.y * 0.8;
      target_gripper_pose.pose.position.z = initial_gripper_pose.pose.position.z + relative_gripper_pose.pose.position.z * 0.8;
      target_frame_pub.publish(target_gripper_pose);
    } else {
      begin_teleoperation = true;
      initial_gripper_pose = target_gripper_pose;
    }

    loop_rate.sleep();
    ros::spinOnce();
  }

  return 0;
}