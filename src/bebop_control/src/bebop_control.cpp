#include <ros/ros.h>
#include <std_msgs/String.h>
#include <std_msgs/Empty.h>
#include <nav_msgs/Odometry.h>
#include <geometry_msgs/Twist.h>
#include <termios.h>
#include <unistd.h>
#include <tf/transform_datatypes.h>
#include <Eigen/Dense>
#include <iostream>
#include <thread>
#include <atomic>
#include <cstdlib>
#include "../ROSUtilities/csv_logger.h"

struct DesiredPose
{
    double x;
    double y;
    double z;
    double yaw;
};
class bebop_control
{
public:
    // Class members
    ros::NodeHandle nh_;
    ros::Publisher takeoff_pub_;
    ros::Publisher land_pub_;
    ros::Publisher cmd_vel_pub_;
    ros::Subscriber position_sub_;
    //Parameters
    double amp_ident_x_;
    double amp_ident_y_;
    double amp_ident_z_;
    double amp_ident_yaw_;
    double temp_ident_;
    double period_duration_;
    bool control_ident_z_ = false;
    bool control_ident_y_ = false;
    bool control_ident_x_ = false;
    bool control_ident_yaw_ = false;
    bool enable_control_keyboard_ = true;
    bool enable_controller_ = false;
    std::atomic<bool> interrupt_control_{true};
    DesiredPose desired_pose_;
    //Messages Declaration
    std_msgs::Empty empty_msg;
    geometry_msgs::Twist twist_msg;
    nav_msgs::Odometry position_msg;
    Eigen::VectorXd x_estates = Eigen::VectorXd::Zero(12);
    Eigen::VectorXd integral_error = Eigen::VectorXd::Zero(4);
    Eigen::VectorXd error = Eigen::VectorXd::Zero(4);
    Eigen::Matrix<double, 4, 12> Kx;
    Eigen::Matrix<double, 4, 4> Ki;
    //Loggers
    std::unique_ptr<CSVLogger> csv_logger_u_control_;
    std::unique_ptr<CSVLogger> csv_logger_desired_trajectory_;
    //Constructor
    bebop_control(ros::NodeHandle& nh):nh_(nh)
    {
        // Initialize parameters
        initializeParameters();


        // Initialize publishers
        takeoff_pub_ = nh_.advertise<std_msgs::Empty>("bebop/takeoff", 10);
        land_pub_ = nh_.advertise<std_msgs::Empty>("bebop/land", 10);
        cmd_vel_pub_ = nh_.advertise<geometry_msgs::Twist>("bebop/cmd_vel", 10);
        // Initialize subscribers
        position_sub_ = nh_.subscribe("/filtered_pose", 10, &bebop_control::positionCallback, this);


        // Initialize services
        // service_ = nh.advertiseService("service_name", &bebop_control::serviceCallback, this);

        //Initialize Functions
        helpCommands();

        //Initialize Loggers
        const char* workspace_name = std::getenv("MY_WORKSPACE_NAME");
        if (workspace_name == nullptr)
        {
            ROS_ERROR("Environment variable MY_WORKSPACE_NAME is not set.");
            return;
        }

        std::vector<std::string> headertrajectory = std::vector<std::string>{"timestamp", 
                                                                                "topic", 
                                                                                "x",  
                                                                                "y",  
                                                                                "z", 
                                                                                "yaw"};
        csv_logger_desired_trajectory_ = std::make_unique<CSVLogger>(workspace_name, 
                                                                        "bebop_control", 
                                                                        "desired_trajectory", 
                                                                        headertrajectory);
        std::vector<std::string> header_u_control = std::vector<std::string>{"timestamp", 
                                                                                "topic", 
                                                                                "ux",  
                                                                                "uy",  
                                                                                "uz", 
                                                                                "uyaw"};
        csv_logger_u_control_ = std::make_unique<CSVLogger>(workspace_name, 
                                                                "bebop_control", 
                                                                "u_control", 
                                                                header_u_control);                                                                        
    }

    void spin()
    {
        // Start ROS spinning
        ros::spin();
    }


    // Function to initialize parameters
    void initializeParameters()
    {
        desired_pose_.x = 0.0;
        desired_pose_.y = 0.0;
        desired_pose_.z = 0.5;
        desired_pose_.yaw = 0.0;
        nh_.param("amp_ident_x", amp_ident_x_, 0.1);
        nh_.param("amp_ident_y", amp_ident_y_, 0.1);
        nh_.param("amp_ident_z", amp_ident_z_, 0.1);
        nh_.param("amp_ident_yaw", amp_ident_yaw_, 0.5);
        nh_.param("temp_ident", temp_ident_, 5.0);
        nh_.param("period_duration", period_duration_, 1.0);
        Kx << -0.0287, -0.0262, -0.7636, -0.1476, 0.0, 0.0, 0.0, 0.0, 0.0, 0.0, 0.0, 0.0,
                0.0, 0.0, 0.0, 0.0, 0.0199,0.0130,-0.5906,-0.1791, 0.0, 0.0, 0.0, 0.0,
                0.0, 0.0, 0.0, 0.0, 0.0, 0.0, 0.0, 0.0, -3.6558,-3.7091, 0.0, 0.0,
                0.0, 0.0, 0.0, 0.0, 0.0, 0.0, 0.0, 0.0, 0.0, 0.0, -3.0674,-3.3309;
        Ki << 0.00246, 0.0, 0.0, 0.0,
                0.0, 0.00278, 0.0, 0.0,
                0.0, 0.0, 0.01574, 0.0,
                0.0, 0.0, 0.0, 0.0388;
    }

    //Function to control drone from keyboad
    void teleopKeyboard()
    {
        char key;
        struct termios oldt, newt;

        // Get the terminal settings for stdin
        tcgetattr(STDIN_FILENO, &oldt);
        newt = oldt;
        // Disable canonical mode and echo
        newt.c_lflag &= ~(ICANON | ECHO);
        // Set the new terminal settings
        tcsetattr(STDIN_FILENO, TCSANOW, &newt);

        ros::Rate rate(200); // Set the loop frequency to 10 Hz

        while (ros::ok())
        {
            // Use select to check if a key is pressed
            fd_set set;
            struct timeval timeout;
            FD_ZERO(&set);
            FD_SET(STDIN_FILENO, &set);
            timeout.tv_sec = 0;
            timeout.tv_usec = 100000; // 0.1 seconds

            int rv = select(STDIN_FILENO + 1, &set, NULL, NULL, &timeout);

            if (rv == -1)
            {
                perror("select"); // Error occurred in select()
            }
            else if (rv == 0)
            {
                if (enable_control_keyboard_)
                {
                    // No key pressed, reset twist message
                    twist_msg.linear.x = 0.0;
                    twist_msg.linear.y = 0.0;
                    twist_msg.linear.z = 0.0;
                    twist_msg.angular.z = 0.0;
                }
            }
            else
            {
                // Key pressed, read it
                key = getchar();
                if (!interrupt_control_){ROS_INFO("Control law interrupted by keyboard input.");}
                interrupt_control_ = true; 
                // Check for arrow keys (escape sequences)
                if (key == 27)
                {
                    key = getchar();
                    if (key == 91)
                    {
                        key = getchar();
                        switch (key)
                        {
                            case 'A': // Arrow Up
                                twist_msg.linear.x = 1.0;
                                break;
                            case 'B': // Arrow Down
                                twist_msg.linear.x = -1.0;
                                break;
                            case 'C': // Arrow Right
                                twist_msg.linear.y = -1.0;
                                break;
                            case 'D': // Arrow Left
                                twist_msg.linear.y = 1.0;
                                break;
                        }
                    }
                }
                else
                {
                    initializeParameters();
                    switch (key)
                    {
                        case 'w':
                            twist_msg.linear.z = 1.0;
                            break;
                        case 's':
                            twist_msg.linear.z = -1.0;
                            break;
                        case 'a':
                            twist_msg.angular.z = 1.0;
                            break;
                        case 'd':
                            twist_msg.angular.z = -1.0;
                            break;
                        case 't':
                            ROS_INFO("Takeoff command received");
                            takeoff_pub_.publish(empty_msg);
                            helpCommands();
                            break;
                        case 'l':
                            ROS_INFO("Land command received");
                            land_pub_.publish(empty_msg);
                            break;
                        case '1':
                            control_ident_z_ = !control_ident_z_;
                            enable_control_keyboard_ = false;
                            // TO-DO: Make this parameters configurable
                            sendAlternatingStepCommand(amp_ident_z_, temp_ident_, period_duration_, "z", control_ident_z_);
                            enable_control_keyboard_ = true;
                            break;
                        case '2':
                            control_ident_y_ = !control_ident_y_;
                            enable_control_keyboard_ = false;
                            sendAlternatingStepCommand(amp_ident_y_, temp_ident_, period_duration_, "y", control_ident_y_);
                            enable_control_keyboard_ = true;
                            break;
                        case '3':
                            control_ident_x_ = !control_ident_x_;
                            enable_control_keyboard_ = false;
                            sendAlternatingStepCommand(amp_ident_x_, temp_ident_, period_duration_, "x", control_ident_x_);
                            enable_control_keyboard_ = true;
                            break;
                        case '4':
                            control_ident_yaw_ = !control_ident_yaw_;
                            enable_control_keyboard_ = false;
                            sendAlternatingStepCommand(amp_ident_yaw_, temp_ident_, period_duration_, "yaw", control_ident_yaw_);
                            enable_control_keyboard_ = true;
                            break;
                        case 'c': 
                            interrupt_control_ = false; // Reset interruption flag                           
                            ROS_INFO("Controller is enabled.");
                            break;
                        case 'h':
                            helpCommands();
                            break;
                        case '\x03': // CTRL+C
                            return;
                        default:
                            break;
                    }
                }
            }
            if (interrupt_control_)
            {
                publishCmdVel();
                // cmd_vel_pub_.publish(twist_msg);
            }            
            rate.sleep(); // Sleep to maintain the loop rate
        }

        // Restore the old terminal settings
        tcsetattr(STDIN_FILENO, TCSANOW, &oldt);
    }
    void helpCommands()
    {
        ROS_INFO("Help commands:");
        ROS_INFO("w: Move forward");
        ROS_INFO("s: Move backward");
        ROS_INFO("a: Move left");
        ROS_INFO("d: Move right");
        ROS_INFO("t: Takeoff");
        ROS_INFO("l: Land");
        ROS_INFO("1: Start control identification for z-axis");
        ROS_INFO("2: Start control identification for y-axis");
        ROS_INFO("3: Start control identification for x-axis");
        ROS_INFO("4: Start control identification for yaw-axis");
        ROS_INFO("c: Enable controller");
        ROS_INFO("h: Show help commands");
        ROS_INFO("CTRL+C: Exit program");
    }
    void sendAlternatingStepCommand(double amp, double temp, double period_duration_, const std::string& axis, bool control_ident)
    {
        if (control_ident)
        {
            ROS_INFO("Starting alternating step command with amplitude: %f, duration: %f seconds, axis: %s", amp, temp, axis.c_str());

            ros::Time start_time = ros::Time::now();
            ros::Duration duration(temp); 
            bool positive = true;
            int num_periods = static_cast<int>(temp);
            ros::Duration period_duration(period_duration_);

               
            while (ros::Time::now() - start_time < duration)
            {
                ros::Time period_start_time = ros::Time::now(); // Marca o tempo de início do período

                // Loop enquanto a diferença entre o tempo atual e o tempo de início do período for menor que a duração do período
                while (ros::Time::now() - period_start_time < period_duration)
                {
                    // Alterna o comando entre positivo e negativo
                    if (axis == "x")
                    {
                        twist_msg.linear.x = positive ? amp : -amp;
                    }
                    else if (axis == "y")
                    {
                        twist_msg.linear.y = positive ? amp : -amp;
                    }
                    else if (axis == "z")
                    {
                        twist_msg.linear.z = positive ? amp : -amp;
                    }
                    else if (axis == "yaw")
                    {
                        twist_msg.angular.z = positive ? amp : -amp;
                    }
                    else
                    {
                        ROS_WARN("Invalid axis specified: %s", axis.c_str());
                        return;
                    }
                    publishCmdVel();
                    // cmd_vel_pub_.publish(twist_msg); // Publica o comando

                    // Sleep for a short duration to avoid spamming the log
                    ros::Duration(0.1).sleep();
                }

                positive = !positive; // Alterna o sinal ao final do período
            }

            ROS_INFO("Finished alternating step command for axis: %s", axis.c_str());
        }
        else
        {
            ROS_WARN("Control identification is not enabled.");
        }

        // Reseta a mensagem twist após a duração
        twist_msg.linear.x = 0.0;
        twist_msg.linear.y = 0.0;
        twist_msg.linear.z = 0.0;
        twist_msg.angular.z = 0.0;
        publishCmdVel();
        // cmd_vel_pub_.publish(twist_msg);

        ROS_INFO("Reset twist message after duration.");
    }
    void positionCallback(const nav_msgs::Odometry::ConstPtr& msg)
    {
        // Your control law logic here
        // Check for interruption
        if (interrupt_control_)
        {
            // ROS_INFO("Control law interrupted by keyboard input.");            
            // Handle interruption (e.g., stop the control law)
            return;
        }
        position_msg = *msg;
        tf::Quaternion quaternion;      
        tf::quaternionMsgToTF(position_msg.pose.pose.orientation, quaternion);
        double roll, pitch, yaw;
        tf::Matrix3x3(quaternion).getRPY(roll, pitch, yaw);
        double x = position_msg.pose.pose.position.x;
        double dx = position_msg.twist.twist.linear.x;
        double pitch_ = pitch;
        double dpitch = position_msg.twist.twist.angular.y;
        double y = position_msg.pose.pose.position.y;
        double dy = position_msg.twist.twist.linear.y;
        double roll_ = roll;
        double droll = position_msg.twist.twist.angular.x;
        double z = position_msg.pose.pose.position.z;
        double dz = position_msg.twist.twist.linear.z;
        double yaw_ = yaw;
        double dyaw = position_msg.twist.twist.angular.z;
        x_estates << x, dx, pitch_, dpitch, y, dy, roll_, droll, z, dz, yaw_, dyaw;
        // Calculate error
        error << desired_pose_.x - x, desired_pose_.y - y, desired_pose_.z - z, desired_pose_.yaw - yaw;
        // Update integral error
        integral_error += error;

        // Log the values
        // ROS_INFO_STREAM("x_estates: " << x_estates.transpose());
        // ROS_INFO_STREAM("error: " << error.transpose());
        // ROS_INFO_STREAM("integral_error: " << integral_error.transpose());



        // Calculate control input

        Eigen::VectorXd u = Kx * x_estates + Ki * integral_error;
        twist_msg.linear.x = u(0);
        twist_msg.linear.y = u(1);
        twist_msg.linear.z = u(2);
        twist_msg.angular.z = u(3);
        // Publish the control input
        publishCmdVel();
        // ROS_INFO("Position received: x = %f, y = %f, z = %f", msg->pose.pose.position.x, msg->pose.pose.position.y, msg->pose.pose.position.z);
    }
    void publishCmdVel()
    {
        cmd_vel_pub_.publish(twist_msg);
        writeLogs();
    }
    void writeLogs()
    {
        double current_time = ros::Time::now().toSec();
        if (current_time == 0.0)
        {
            ROS_WARN("ROS Time is not initialized properly. Skipping log entry.");
            return;
        }
        std::vector<std::variant<std::string, double>> data_u_control = {current_time, 
                                                                            "cmd_vel", 
                                                                            twist_msg.linear.x, 
                                                                            twist_msg.linear.y,
                                                                            twist_msg.linear.z,
                                                                            twist_msg.angular.z}; 
        csv_logger_u_control_->writeCSV(data_u_control);
        std::vector<std::variant<std::string, double>> data_desired_trajectory = {current_time, 
                                                                                "desired_trajectory", 
                                                                                desired_pose_.x, 
                                                                                desired_pose_.y,
                                                                                desired_pose_.z,
                                                                                desired_pose_.yaw};
        csv_logger_desired_trajectory_->writeCSV(data_desired_trajectory);        
    }
};

int main(int argc, char** argv)
{
    ros::init(argc, argv, "bebop_control");
    ros::NodeHandle nh;

    bebop_control node(nh);

    // Start the teleopKeyboard function in a separate thread
    std::thread teleop_thread(&bebop_control::teleopKeyboard, &node);

    // node.teleopKeyboard();
    node.spin();

    teleop_thread.join(); // Wait for the teleop thread to finish
    return 0;
}