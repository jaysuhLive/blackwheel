#include <ros/ros.h>
#include <std_msgs/String.h>
#include <std_msgs/Empty.h>
#include <std_msgs/Bool.h>
#include <nav_msgs/Path.h>
#include <math.h>
#include <sstream>
#include "geometry_msgs/Twist.h"
#include "actionlib_msgs/GoalStatusArray.h"
#include "freeway_msgs/DistanceTimeCalculator.h"
#include "freeway_joyfw/stm_fw_msg.h"
#include "freeway_joyfw/stm_fw_sonar_msg.h"
#include "freeway_joyfw/stm_am_msg.h"
#include "freeway_joyfw/stm_fw_srv.h"

#define RP_LIDAR_S2 true

class Freeway_Joy_Fw
{
  public:
    Freeway_Joy_Fw(ros::NodeHandle *n)
    {
      cmd_pub = n->advertise<geometry_msgs::Twist>("/cmd_vel/joy", 10);
      cmd_emer_pub = n->advertise<geometry_msgs::Twist>("/cmd_vel/emer", 10);
      am_mode_pub = n->advertise<freeway_joyfw::stm_am_msg>("freeway/am_status", 10);
      move_base_flex_cancel_pub = n->advertise<actionlib_msgs::GoalID>("move_base_flex/move_base/cancel", 10);
      range_right_pub = n->advertise<sensor_msgs::Range>("/freeway/ultrasound/right", 10);
      range_left_pub = n->advertise<sensor_msgs::Range>("/freeway/ultrasound/left", 10);
      diag_sub = n->subscribe("freeway/diagnostics", 10, &Freeway_Joy_Fw::get_diagnostics_cb, this);
      sonar_sub = n->subscribe("/freeway/ultrasound", 10, &Freeway_Joy_Fw::fw_sonar_cb, this);
      front_obstacle_sub = n->subscribe("/freeway/front_obstacle", 10, &Freeway_Joy_Fw::front_obstacle_cb, this);
      cmd_vel_ui_sub = n->subscribe("/cmd_vel/ui", 10, &Freeway_Joy_Fw::cmd_vel_ui_cb, this);
      dt_sub = n->subscribe("/freeway/distancetimecalculator", 10, &Freeway_Joy_Fw::dt_sub_cb, this);
      diagnostics_time;
      stm_msg;
      dt_msg;
      front_obstacle_detected = false;
    }

    bool am_mode_cb(freeway_joyfw::stm_fw_srv::Request &req, freeway_joyfw::stm_fw_srv::Response &res);

    void front_obstacle_cb(const std_msgs::Bool::ConstPtr& front_obstacle_msg)
    {
      front_obstacle_detected = front_obstacle_msg->data;
    }

    void dt_sub_cb(const freeway_msgs::DistanceTimeCalculator::ConstPtr& msg) {
        dt_msg = *msg;
    }

    void get_diagnostics_cb(const freeway_joyfw::stm_fw_msg &diag_msg) {
      geometry_msgs::Twist cmd_vel_msg;
      stm_msg = diag_msg;
      diagnostics_time = ros::Time::now();
      //if (stm_msg.am_status == true && stm_msg.e_stop_status == true)
      if (stm_msg.e_stop_status == true)
      {
        if(front_obstacle_detected) {
          if (diag_msg.cmd_vel_mcu.linear.x > 0.0) {
              cmd_vel_msg.linear.x = 0.0;
          }
          else {
            cmd_vel_msg.linear.x =  diag_msg.cmd_vel_mcu.linear.x;
          }
          cmd_vel_msg.angular.z = diag_msg.cmd_vel_mcu.angular.z;
          cmd_pub.publish(cmd_vel_msg);
        }
        else {
          cmd_vel_msg.linear.x = diag_msg.cmd_vel_mcu.linear.x;
          cmd_vel_msg.angular.z = diag_msg.cmd_vel_mcu.angular.z;
          cmd_pub.publish(cmd_vel_msg);
        }
      }
      else if (stm_msg.e_stop_status == false)
      {
        cmd_vel_msg.linear.x = 0.0;//diag_msg.cmd_vel_mcu.linear.x;
        cmd_vel_msg.angular.z = 0.0;//diag_msg.cmd_vel_mcu.angular.z;
        cmd_emer_pub.publish(cmd_vel_msg);
        if (dt_msg.status_info == 1) {
            actionlib_msgs::GoalID empty_goal;
            move_base_flex_cancel_pub.publish(empty_goal);
        }
      }
    }

  void fw_sonar_cb(const freeway_joyfw::stm_fw_sonar_msg::ConstPtr& fw_sonar_msg) {
	ros::Time stamp_now;
	stamp_now = ros::Time::now();
	sensor_msgs::Range right_msg;
	sensor_msgs::Range left_msg;
	right_msg = (fw_sonar_msg->range_right);
	left_msg = (fw_sonar_msg->range_left);

        right_msg.header.frame_id = "sonar_link_right";
	left_msg.header.frame_id = "sonar_link_left";

	right_msg.header.stamp = stamp_now;
	left_msg.header.stamp = stamp_now;

	right_msg.max_range = 4.0;
	left_msg.max_range = 4.0;

	right_msg.range = right_msg.range / 100.0;
	left_msg.range = left_msg.range / 100.0;

	range_right_pub.publish(right_msg);
	range_left_pub.publish(left_msg);
}

    void cmd_vel_ui_cb(const geometry_msgs::Twist &cmd_vel_ui_msg) {
        geometry_msgs::Twist cmd_vel_msg;
        ros::Duration timeout_duration(0.25);
        bool active_flag = false;
        if (ros::Time::now() - diagnostics_time < timeout_duration) active_flag = true;
        else active_flag = false;

        if (active_flag == false)
        {
            if(front_obstacle_detected) {
            if (cmd_vel_ui_msg.linear.x > 0.0) {
               cmd_vel_msg.linear.x = 0.0;
            }
            else {
             cmd_vel_msg.linear.x =  cmd_vel_ui_msg.linear.x;
            }

            cmd_vel_msg.angular.z = cmd_vel_ui_msg.angular.z;
            cmd_pub.publish(cmd_vel_msg);

            }
            else {
            cmd_vel_msg.linear.x = cmd_vel_ui_msg.linear.x;
            cmd_vel_msg.angular.z = cmd_vel_ui_msg.angular.z;
            cmd_pub.publish(cmd_vel_msg);
          }
       }

//       if (active_flag == true && stm_msg.e_stop_status == false)
//       {
//         cmd_vel_msg.linear.x = 0.0;//diag_msg.cmd_vel_mcu.linear.x;
//         cmd_vel_msg.angular.z = 0.0;//diag_msg.cmd_vel_mcu.angular.z;
//         cmd_emer_pub.publish(cmd_vel_msg);
//       }
     }

  private:
    ros::Publisher cmd_pub;
    ros::Publisher cmd_emer_pub;
    ros::Publisher move_base_flex_cancel_pub;
    ros::Publisher am_mode_pub;
    ros::Publisher range_right_pub;
    ros::Publisher range_left_pub;
    ros::Subscriber diag_sub;
    ros::Subscriber front_obstacle_sub;
    ros::Subscriber cmd_vel_ui_sub;
    ros::Subscriber dt_sub;
    ros::Subscriber sonar_sub;
    freeway_joyfw::stm_fw_msg stm_msg;
    freeway_msgs::DistanceTimeCalculator dt_msg;
    ros::Time diagnostics_time;
    bool front_obstacle_detected = false;
};

bool Freeway_Joy_Fw::am_mode_cb(freeway_joyfw::stm_fw_srv::Request &req, freeway_joyfw::stm_fw_srv::Response &res)
{
  freeway_joyfw::stm_am_msg am_msg;
  res.result = req.am_mode;
  am_msg.am_status2=res.result;
  am_mode_pub.publish(am_msg);
  ROS_INFO("res.result : %d", res.result);
  return true;
}

int main(int argc, char **argv)
{
  ros::init(argc, argv, "freeway_joy_node");
  ros::NodeHandle n;

  Freeway_Joy_Fw Fj = Freeway_Joy_Fw(&n);
  ros::ServiceServer service = n.advertiseService("am_mode", &Freeway_Joy_Fw::am_mode_cb, &Fj);
  
  ros::Rate loop_rate(20);

  while (ros::ok())
  {
    ros::spinOnce();

    loop_rate.sleep();
  }


  return 0;
}
