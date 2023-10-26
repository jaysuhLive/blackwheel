#include <ros/ros.h>
#include <std_msgs/String.h>
#include <std_msgs/Empty.h>
#include <std_msgs/Bool.h>
#include <std_msgs/Int8.h>
#include <std_msgs/Float32.h>
#include <sensor_msgs/LaserScan.h>
#include "geometry_msgs/Twist.h"
#include "nav_msgs/Odometry.h"
#include "actionlib_msgs/GoalStatusArray.h"
#include "std_msgs/Empty.h"
#include "std_srvs/Empty.h"
#include "freeway_msgs/DistanceTimeCalculator.h"
#include <boost/bind.hpp>
#include <string>
#include <sstream>
//#include <thread>
 
//#define rad_to_deg(rad) ((rad) * 180.0 / M_PI)

class Freeway_Safety
{
  public:
    Freeway_Safety(ros::NodeHandle *nh)
    {
      n = nh;
//      if(!n->getParam("kp", kp_)) {
//	      ROS_WARN("Could not get value of kp parameter, using default value.");
//	      kp_ = 10.0; // Use a default value if parameter is not set
//	      }
//      if(!n->getParam("ki", ki_)) {
//	      ROS_WARN("Could not get value of ki parameter, using default value.");
//	      ki_ = 5.0; // Use a default value if parameter is not set
//	      }
//      if(!n->getParam("kd", kd_)) {
//	      ROS_WARN("Could not get value of kd parameter, using default value.");
//	      kd_ = 1.0; // Use a default value if parameter is not set
//	      }
//      if(!n->getParam("ttc_desired", ttc_desired_)) {
//	      ROS_WARN("Could not get value of ttc_desired parameter, using default value.");
//	      ttc_desired_ = 5.0; // Use a default value if parameter is not set
//	      }
//      if(!n->getParam("ttc_limit", ttc_limit_)) {
//	      ROS_WARN("Could not get value of ttc_limit parameter, using default value.");
//	      ttc_limit_ = 5.0; // Use a default value if parameter is not set
//	      }
      if(!n->getParam("frequency", frequency_)) {
	      ROS_WARN("Could not get value of frequency parameter, using default value.");
	      frequency_ = 5.0; // Use a default value if parameter is not set
	      }
      if(!n->getParam("odom_topic", odom_topic_)) {
	      ROS_WARN("Could not get value of odom_topic parameter, using default value.");
	      odom_topic_ = "/odom"; // Use a default value if parameter is not set
	      }
      if(!n->getParam("goal_resume_time", goal_resume_time_)) {
          ROS_WARN("Could not get value of goal_resume_time parameter, using default value.");
          goal_resume_time_ = 2.7; // Use a default value if parameter is not set
              }


      cmd_sub = n->subscribe("/cmd_vel", 10, &Freeway_Safety::cmd_update_cb, this);
      ttc_sub = n->subscribe("/freeway/ttc", 10, &Freeway_Safety::ttc_update_cb, this);
      ttc_flag_sub = n->subscribe("/freeway/ttc_flag", 10, &Freeway_Safety::ttc_flag_update_cb, this);
      front_obstacle_sub = n->subscribe("/freeway/front_obstacle", 10, &Freeway_Safety::front_obstacle_update_cb, this);
      dtc_sub = n->subscribe("/freeway/distancetimecalculator", 10, &Freeway_Safety::dtc_update_cb, this);
      odom_sub = n->subscribe(odom_topic_, 10, &Freeway_Safety::odom_update_cb, this);
      resume_feedback_sub = n->subscribe("/freeway/resume/feedback", 10, &Freeway_Safety::resume_feedback_update_cb, this);
      cancel_flag_sub = n->subscribe("/freeway/cancel_status", 10, &Freeway_Safety::cancel_flag_cb,this);

      cmd_e_vel_pub = n->advertise<geometry_msgs::Twist>("/cmd_vel/emer", 10);
      resume_pub = n->advertise<std_msgs::Empty>("/freeway/resume", 10);
      move_base_cancel_pub = n->advertise<std_msgs::Empty>("freeway/emer_goal_cancel", 10);
      brake_stop_pub = n->advertise<std_msgs::Empty>("/freeway/initsetbrakestop", 1);
      clearcostmap_pub = n->serviceClient<std_srvs::Empty>("move_base_flex/clear_costmaps");

      odom_msg;
      cmd_msg;
      e_stop_timer;
      resume_timer;
      resume_feedback_msg.data = -1;
      cancel_flag = -1;
      //ttc_current_ = 10.0;
      //desired_vel_ = 0.0;
      //error = 0.0;
      //integral = 0.0;
      //derivative = 0.0;
      //prev_error = 0.0;
      //dt_ = 1.0/frequency_;

      ttc_time = 0.0;
      ttc_flag = false;
      ttc_activate_ = false;
      front_obstacle_detected = false;

      e_stop_flag = true;
    }

    void log_param() {

        ROS_INFO_STREAM("odom_topic: " << odom_topic_);
        ROS_INFO("frequency: %f", frequency_);
    	ROS_INFO("goal_resume_time: %f", goal_resume_time_);
//        ROS_INFO("kp: %f", kp_);
//        ROS_INFO("ki: %f", ki_);
//        ROS_INFO("kd: %f", kd_);
//        ROS_INFO("ttc_desired: %f", ttc_desired_);
    }

    float getFrequency() const {
      return frequency_;
    }

    float getTtcTime() const {
        return ttc_time;
    }

//    void resume_tim_cb(const ros::TimerEvent&, ros::Publisher& pub) {
//        std_msgs::Empty resume_msg;
//        pub.publish(resume_msg);
//    }

//    void e_stop_tim_cb(const ros::TimerEvent&, ros::Publisher& pub) {
//	ROS_INFO("e_stop_tim_cb called!!");
//	geometry_msgs::Twist zero_velocity;
//	zero_velocity = geometry_msgs::Twist();
//        if (cmd_msg.linear.x > 0.0 || odom_msg.twist.twist.linear.x > 0.0 ) {
//          cmd_e_vel_pub.publish(zero_velocity);
//          }
//        else {
//          e_stop_timer.stop();
//        }
//    }

    nav_msgs::Odometry getOdomData() const {
        return odom_msg;
    }



    void slow_down(float ttc_time, const nav_msgs::Odometry::ConstPtr& odom) {
      // Calculate the error between the desired ttc and the current ttc
      //ttc_current_ = ttc_time;
      //error = ttc_desired_ - ttc_current_;  
      // Calculate the integral and derivative terms
      //integral += error * dt_;
      //derivative = (error - prev_error) / dt_;  
      // Calculate the output of the PID controller
      //float output = kp_ * error + ki_ * integral + kd_ * derivative;  
      // Calculate the desired velocity based on the output
      //desired_vel_ = std::max(0.0f, static_cast<float>(odom.twist.twist.linear.x - output));
    
      // Apply the desired velocity as a linear acceleration to the robot
      geometry_msgs::Twist cmd_vel;
      cmd_vel = geometry_msgs::Twist();
      //cmd_vel.linear.x = desired_vel_;
      if (odom->twist.twist.linear.x != 0 && !e_stop_flag == false) {
        cmd_e_vel_pub.publish(cmd_vel);
      }
      // Update the previous error for the next iteration
      //prev_error = error;
    }
    
    void brake_robot(float frequency) {
      //actionlib_msgs::GoalID empty_goal;
      std_msgs::Empty empty_goal;
//      std_msgs::Empty brake_msg;
	std_msgs::Empty resume_msg;
      geometry_msgs::Twist zero_velocity;
      zero_velocity = geometry_msgs::Twist();
      move_base_cancel_pub.publish(empty_goal);
//      brake_stop_pub.publish(brake_msg);
//      timer = n->createTimer(ros::Duration(1.0/frequency), boost::bind(&Freeway_Safety::e_stop_tim_cb, this, _1, cmd_e_vel_pub));
       e_stop_timer = n->createTimer(ros::Duration(1.0/frequency), [&](const ros::TimerEvent& envet) {
	        if ((cmd_msg.linear.x > 0.0 || odom_msg.twist.twist.linear.x > 0.0) && this->ttc_time < 2.0 ) {
                cmd_e_vel_pub.publish(zero_velocity);
	        }
	        else {
		    //resume_pub.publish(resume_msg);
	            e_stop_timer.stop();
        	}
        });
    }

    void stop_logic(float ttc_time, float frequency, const nav_msgs::Odometry::ConstPtr& odom) {
      if (ttc_activate_) {
        // Slow down the robot to increase TTC to at least 2 seconds
//       slow_down(ttc_time, odom);
       if ( e_stop_flag == false && front_obstacle_detected == true) {
          e_stop_flag = true;
         // Stop the robot if a front obstacle is detected
    	  ROS_INFO("Safety Node, Brake Robot called!!!");
	  brake_robot(frequency);
        }
        else if (e_stop_flag == true && front_obstacle_detected == false) {
          // Resume normal operation if no front obstacle is detected
          std_srvs::Empty srv;
          clearcostmap_pub.call(srv);
          e_stop_flag = false;
//          resume_timer = n->createTimer(ros::Duration(goal_resume_time_), boost::bind(&Freeway_Safety::resume_tim_cb, this, _1, resume_pub), true);
          resume_timer = n->createTimer(ros::Duration(goal_resume_time_), [&](const ros::TimerEvent&){
	        ROS_INFO("Resume timer callback Started!!!");
            std_msgs::Empty resume_msg;
            if (cancel_flag == 1) resume_pub.publish(resume_msg);
            if (resume_feedback_msg.data == -1) {
	            ROS_INFO("Resume timer callback Stopped!!!, status_info: %d", resume_feedback_msg.data);
                resume_timer.stop();

               }
          });

	      if(odom->twist.twist.linear.x <= 0.0) ttc_activate_ = false;
        }
      }
    }

  private:
    ros::NodeHandle* n;
    ros::Subscriber cmd_sub;
    ros::Subscriber ttc_sub;
    ros::Subscriber ttc_flag_sub;
    ros::Subscriber front_obstacle_sub;
    ros::Subscriber dtc_sub;
    ros::Subscriber odom_sub;
    ros::Subscriber resume_feedback_sub;
    ros::Subscriber cancel_flag_sub;

    ros::Publisher cmd_e_vel_pub;
    ros::Publisher resume_pub;
    ros::Publisher move_base_cancel_pub;
    ros::Publisher brake_stop_pub;
    ros::ServiceClient clearcostmap_pub;

    ros::Timer e_stop_timer;
    ros::Timer resume_timer;
    
    std_msgs::Int8 resume_feedback_msg;
    std::string odom_topic_;
    nav_msgs::Odometry odom_msg;
    geometry_msgs::Twist cmd_msg;
    freeway_msgs::DistanceTimeCalculator dtc;

    //float kp_;
    //float ki_;
    //float kd_;
    //float ttc_desired_;
    //float ttc_limit_;
    //float ttc_current_;
    //float desired_vel_;
    //float error;
    //float integral;
    //float derivative;
    //float prev_error;
    //float dt_;

    float ttc_time;
    float goal_resume_time_;
    bool ttc_flag;
    bool front_obstacle_detected;
    bool e_stop_flag;
    bool ttc_activate_;
    int cancel_flag;
    float frequency_;

    void ttc_update_cb(const std_msgs::Float32::ConstPtr& msg) {
        ttc_time = msg->data;
        //if (ttc_time <= ttc_limit_) ttc_activate_ = true;
    }
    void ttc_flag_update_cb(const std_msgs::Bool::ConstPtr& msg) {
        ttc_flag = msg->data;
	if(ttc_flag == true) ttc_activate_ = true;
    }
    void front_obstacle_update_cb(const std_msgs::Bool::ConstPtr& msg) {
        front_obstacle_detected = msg->data;
    }
    void dtc_update_cb(const freeway_msgs::DistanceTimeCalculator::ConstPtr& msg) {
        dtc = *msg;
    }
    void odom_update_cb(const nav_msgs::Odometry::ConstPtr& msg) {
        odom_msg = *msg;
    }
    void cmd_update_cb(const geometry_msgs::Twist::ConstPtr& msg) {
        cmd_msg = *msg;
    }
    void resume_feedback_update_cb(const std_msgs::Int8::ConstPtr& msg) {
        resume_feedback_msg = *msg;
    }

    void cancel_flag_cb(const std_msgs::Int8::ConstPtr& msg) {
        cancel_flag = msg->data;
    }
};

int main(int argc, char **argv)
{
  ros::init(argc, argv, "freeway_lidar_ttc_node");
  ros::NodeHandle n;
  Freeway_Safety FS = Freeway_Safety(&n);  
  FS.log_param();
  float frequency = FS.getFrequency();

  ros::Rate r(frequency);

  while (ros::ok()) {
    nav_msgs::Odometry odom = FS.getOdomData();
    float ttc_time = FS.getTtcTime();
    FS.stop_logic(ttc_time, frequency, boost::make_shared<const nav_msgs::Odometry>(odom));

    ros::spinOnce();
    r.sleep();
    }
  return 0;
}
