#include <stdio.h>
 #include <iostream>
#include <stdlib.h>
#include <string>
#include <vector>
#include <ros/ros.h>
#include <serial/serial.h>
#include "geometry_msgs/Twist.h"
#include "std_msgs/Bool.h"
#include <fcntl.h> // for open()
#include <unistd.h> // for close()
#include <sys/ioctl.h> // for ioctl()
#include <linux/usbdevice_fs.h> // for USBDEVFS_RESET
#include <cstdlib>
#include <cstdio>
#include <sstream>
#include <array>

int get_pid(std::string port) {
    std::string command = "lsof -t " + port;
    std::array<char, 128> buffer;
    std::string result;
    FILE* pipe = popen(command.c_str(), "r");
    if (!pipe) {
        std::cerr << "Failed to run command\n";
        return -1;
    }
    while (fgets(buffer.data(), buffer.size(), pipe) != nullptr) {
        result += buffer.data();
    }
    pclose(pipe);
    int pid;
    if (sscanf(result.c_str(), "%d", &pid) != 1) {
        std::cerr << "No process is using the port\n";
        return -1;
    }
    return pid;
}

bool kill_process(int pid) {
    std::stringstream ss;
    ss << "kill -9 " << pid;
    return system(ss.str().c_str()) == 0;
}

bool reset_usb_device(std::string device_file, int pid) {
    // First, try to kill the process that's using the device.
   // if (!kill_process(pid)) {
   //     ROS_ERROR_STREAM(" Failed to kill the process. Return false to indicate failure...");
   //     return false;
   // }

    int fd = open(device_file.c_str(), O_WRONLY);
    if (fd == -1) {
        // Cannot open device file. Return false to indicate failure.
        return false;
    }

    int rc = ioctl(fd, USBDEVFS_RESET, 0);
    if (rc == -1) {
        // USBDEVFS_RESET failed. Close the device file and return false.
        close(fd);
        return false;
    }

    close(fd);
    return true; // Successfully reset USB device.
}

class MAIN_JOY {
	
	public:

		MAIN_JOY(ros::NodeHandle* nh) : n(nh)
		{
	        if(!n->getParam("fw_main_joy_node/device_name", device_name_)) {
			ROS_WARN("Could not get value of device_name parameter, using default value.");
			device_name_ = "/dev/MAINJOY";
		}
                if(!n->getParam("fw_main_joy_node/x_scale", x_scale_)) {
                        ROS_WARN("Could not get value of x_scale parameter, using default value.");
                        x_scale_ = 1.2;
                }
                if(!n->getParam("fw_main_joy_node/y_scale", y_scale_)) {
                        ROS_WARN("Could not get value of y_scale parameter, using default value.");
                        y_scale_ = 0.7;
                }

		if(!n->getParam("fw_main_joy_node/frequency", frequency_)) {
			ROS_WARN("Could not get value of frequency parameter, using default value.");
			frequency_ = 50.0;
		}
		if(!n->getParam("fw_main_joy_node/baudrate", baudrate_)) {
			ROS_WARN("Could not get value of baudrate parameter, using default value.");
			baudrate_ = 38400;
			}
		if(!n->getParam("fw_main_joy_node/cmd_vel_topic", cmd_vel_topic_)) {
			ROS_WARN("Could not get value of cmd_topic parameter, using default value.");
			cmd_vel_topic_ = "/cmd_vel";
			}
                if(!n->getParam("fw_main_joy_node/uart_debugmode", uart_debugmode_)) {
                        ROS_WARN("Could not get value of uart_debugmode parameter, using default value.");
                        uart_debugmode_ = true;
			}
                if(!n->getParam("fw_main_joy_node/front_obstacle_topic", front_obstacle_topic_)) {
                        ROS_WARN("Could not get value of front_obstacle_topic parameter, using default value.");
                        front_obstacle_topic_ = "/freeway/front_obstacle";
			}
    	   	cmd_pub = n->advertise<geometry_msgs::Twist>(cmd_vel_topic_, 10);
		front_obstacle_update_sub = n->subscribe(front_obstacle_topic_, 10, &MAIN_JOY::front_obstacle_update_cb, this);

		cmd_array;
		front_obstacle_detected = false;
		}

void parse(const std::string& input_serial) {
    // Reset cmd_array by clearing all elements
    cmd_array.clear();

    // Find the position of the first delimiter (',')
    size_t first_delimiter_pos = input_serial.find(',');
    if (first_delimiter_pos == std::string::npos) {
        // First delimiter not found, handle accordingly
        ROS_WARN("Invalid input_serial format. First delimiter not found.");
        return;
    }

    // Extract the first data before the first delimiter
    std::string first_data_str = input_serial.substr(0, first_delimiter_pos);
    try {
        float first_data = std::stof(first_data_str);
        cmd_array.push_back(first_data);
    } catch (const std::exception& e) {
        ROS_WARN("Failed to parse first_data: %s", e.what());
        return;
    }

    // Find the position of the second delimiter ('>')
    size_t second_delimiter_pos = input_serial.find('>', first_delimiter_pos + 1);
    if (second_delimiter_pos == std::string::npos) {
        // Second delimiter not found, handle accordingly
        ROS_WARN("Invalid input_serial format. Second delimiter not found.");
        return;
    }

    // Extract the second data between the first delimiter and the second delimiter
    std::string second_data_str = input_serial.substr(first_delimiter_pos + 1, second_delimiter_pos - first_delimiter_pos - 1);
    try {
        float second_data = std::stof(second_data_str);
        cmd_array.push_back(second_data);
    } catch (const std::exception& e) {
        ROS_WARN("Failed to parse second_data: %s", e.what());
        return;
    }
}

void pub_cmd_vel() {
    if (cmd_array.size() != 2) {
        ROS_WARN("cmd_array does not contain the required number of elements.");
        return;
    }

    geometry_msgs::Twist cmd_vel;
    float linear_x = x_scale_ * cmd_array[0];
    float angular_z = y_scale_ * cmd_array[1];
//    if (front_obstacle_detected)  {
//	if (linear_x > 0.0) {
//	  cmd_vel.linear.x = 0.0;
//	}
//        else {
//          cmd_vel.linear.x = linear_x;
//        }
//    }
//    else {
//        cmd_vel.linear.x = linear_x;
//    }
  cmd_vel.linear.x = linear_x;
  cmd_vel.angular.z = angular_z;
  if ( (linear_x >= 0.01 || linear_x <= -0.01) || (angular_z >= 0.01 || angular_z <= -0.01)) cmd_pub.publish(cmd_vel);
}

	std::string r_device_name() {
		return device_name_;
	}

	bool r_uart_debugmode() const {
		return uart_debugmode_;
	}

	std::string r_uart_payload() {
		std::string a1 = std::to_string(cmd_array[0]);
		std::string a2 = std::to_string(cmd_array[1]);
		return "axis_X: " + a1 + " " + "axis+Y: " + a2;
	}

	float r_frequency() const {
		return frequency_;
	}

	int r_baudrate() const {
		return baudrate_;
	}

private:
	ros::NodeHandle* n;
	ros::Publisher cmd_pub;
	ros::Subscriber front_obstacle_update_sub;
	std::string device_name_;
	std::string cmd_vel_topic_;
	std::string front_obstacle_topic_;
	std::vector<float> cmd_array;
	bool front_obstacle_detected;
	bool uart_debugmode_;

	float frequency_;
	float x_scale_;
	float y_scale_;
	int baudrate_;

        void front_obstacle_update_cb(const std_msgs::Bool::ConstPtr& front_obstacle_msg)
        {
          front_obstacle_detected = front_obstacle_msg->data;
        }

};

int main(int argc, char** argv) {
	ros::init(argc, argv, "fw_main_joy_node");
	ros::NodeHandle nh;
    	serial::Serial ser;
	MAIN_JOY mj = MAIN_JOY(&nh);

	float frequency = mj.r_frequency();
	int baudrate = mj.r_baudrate();
        std::string port = mj.r_device_name();

        // Try to open the port until successful, resetting the USB device and sleeping for 5 seconds after each failure.
        while (true) {
            try
            {
                ser.setPort(port);
                ser.setBaudrate(baudrate);
                serial::Timeout to = serial::Timeout::simpleTimeout(1000);
                ser.setTimeout(to);
                ser.open();
                break;  // if ser.open() succeeded, break the loop.
            }
            catch (serial::IOException& e)
            {
		int pid = get_pid(port);
                ROS_ERROR_STREAM("Unable to open port. Resetting and retrying...");
                if (!reset_usb_device(port, pid)) {
                    ROS_ERROR_STREAM("Failed to reset USB device");
                }
                ros::Duration(2.0).sleep();  // sleep for 2 seconds before retrying.
            }
        }
	if (ser.isOpen()) {
		ROS_INFO("Serial Port initialized");
	}
	else {
		return -1;
	}

	ros::Rate loop_rate(frequency);

	while (ros::ok()) {
	// This is the part where you're ready to receive data.
	std::string command = "a";
	ser.write(command);

        // Wait for a short moment to give the Arduino time to prepare the response.
        ros::Duration(0.001).sleep(); // Sleep for 10 milliseconds.

	if(ser.available()) {
    	    std::string input_serial = ser.read(ser.available());
    	    mj.parse(input_serial);
    	    if(mj.r_uart_debugmode()) ROS_INFO("Parsed data : %s", mj.r_uart_payload().c_str());
    	    mj.pub_cmd_vel();
    	}
		ros::spinOnce();
		loop_rate.sleep();
	}
	ser.close();
	return 0;
}
