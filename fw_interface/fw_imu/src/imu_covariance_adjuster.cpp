#include <ros/ros.h>
#include <sensor_msgs/Imu.h>
#include <std_msgs/Bool.h>

class ImuCovarianceAdjuster
{
public:
  ImuCovarianceAdjuster()
  {
    // Subscribe to the robot's status data
    status_sub_ = nh_.subscribe<std_msgs::Bool>("freeway/moving_check", 1, &ImuCovarianceAdjuster::statusCallback, this);

    // Subscribe to the IMU data
    imu_sub_ = nh_.subscribe<sensor_msgs::Imu>("imu/data", 1, &ImuCovarianceAdjuster::imuCallback, this);

    // Advertise the updated IMU data
    imu_pub_ = nh_.advertise<sensor_msgs::Imu>("imu/data/cov_adj", 1);

    // Set the timer interval to match the raw IMU data frequency
    timer_ = nh_.createTimer(ros::Duration(0.005), &ImuCovarianceAdjuster::timerCallback, this);
  }

  void statusCallback(const std_msgs::Bool::ConstPtr& msg)
  {
    moving_check_ = msg->data; // Update robot moving status based on the received message
  }

  void timerCallback(const ros::TimerEvent& event)
  {
    if (last_imu_msg_)
    {
      // Make a copy of the last received IMU message
      sensor_msgs::Imu imu_data = *last_imu_msg_;

      if (!moving_check_)
      {
	imu_data.orientation_covariance[0] = 0.0;
        imu_data.angular_velocity_covariance[0] = 0.01; // Set normal covariance value when robot is moving
        imu_data.angular_velocity_covariance[4] = 0.01; // Set high covariance value w$
        imu_data.angular_velocity_covariance[8] = 0.01; // Set high covariance value w$
        imu_data.linear_acceleration_covariance[0] = 1.0;
 	imu_data.linear_acceleration_covariance[4] = 1.0;
 	imu_data.linear_acceleration_covariance[8] = 1.0;
      }
      else
      {
	imu_data.orientation_covariance[0] = 9999.0;
        imu_data.angular_velocity_covariance[0] = 0.01; // Set normal covariance value when robot is moving
        imu_data.angular_velocity_covariance[4] = 0.01; // Set high covariance value w$
        imu_data.angular_velocity_covariance[8] = 0.01; // Set high covariance value w$
        imu_data.linear_acceleration_covariance[0] = 1.0;
 	imu_data.linear_acceleration_covariance[4] = 1.0;
 	imu_data.linear_acceleration_covariance[8] = 1.0;
      }

      // Publish the updated IMU data
      // Use the same timestamp as the original message
      imu_data.header.stamp = last_imu_msg_->header.stamp;
      imu_pub_.publish(imu_data);
    }
  }

  void imuCallback(const sensor_msgs::Imu::ConstPtr& msg)
  {
    last_imu_msg_ = msg; // Store the last received IMU message
  }

private:
  ros::NodeHandle nh_;
  ros::Subscriber status_sub_;
  ros::Subscriber imu_sub_;
  ros::Publisher imu_pub_;
  ros::Timer timer_;
  bool moving_check_ = true;
  sensor_msgs::Imu::ConstPtr last_imu_msg_;
};

int main(int argc, char **argv)
{
  ros::init(argc, argv, "fw_imu_node");
  ImuCovarianceAdjuster adjuster;
  ros::spin();
  return 0;
}

