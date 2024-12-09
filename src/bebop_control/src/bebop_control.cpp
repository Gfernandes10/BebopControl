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
#include <bebop_control/SetReferencePose.h>
#include <bebop_control/SetCircPath.h>

struct DesiredPose
{
    double x;
    double y;
    double z;
    double yaw;
};
class BebopControl
{
public:
    // Class members
    ros::NodeHandle nh_;
    ros::Publisher takeoff_pub_;
    ros::Publisher land_pub_;
    ros::Publisher cmd_vel_pub_;
    ros::Subscriber position_sub_;
    ros::ServiceServer set_reference_pose_srv_;
    ros::ServiceServer set_circ_path_srv_; 
    
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
    double circ_radius_;
    double circ_angular_velocity_;
    double circ_height_;
    ros::Timer trajectory_timer_;
    bool trajectory_timer_started_ = false;
    //Messages Declaration
    std_msgs::Empty empty_msg;
    geometry_msgs::Twist twist_msg;
    nav_msgs::Odometry position_msg;
    Eigen::VectorXd x_estates = Eigen::VectorXd::Zero(12);
    Eigen::VectorXd integral_error = Eigen::VectorXd::Zero(4);    
    Eigen::VectorXd error = Eigen::VectorXd::Zero(4);
    Eigen::VectorXd u = Eigen::VectorXd::Zero(4);
    Eigen::Matrix<double, 4, 12> Kx;
    Eigen::Matrix<double, 4, 4> Ki;
    //Loggers
    std::unique_ptr<CSVLogger> csv_logger_u_control_;
    std::unique_ptr<CSVLogger> csv_logger_desired_trajectory_;
    std::unique_ptr<CSVLogger> csv_logger_error_;
    //Constructor
    BebopControl(ros::NodeHandle& nh):nh_(nh)
    {
        // Initialize parameters
        initializeParameters();
        integral_error << 0, 0, 250, 0; //250

        // Initialize publishers
        takeoff_pub_ = nh_.advertise<std_msgs::Empty>("bebop/takeoff", 10);
        land_pub_ = nh_.advertise<std_msgs::Empty>("bebop/land", 10);
        cmd_vel_pub_ = nh_.advertise<geometry_msgs::Twist>("bebop/cmd_vel", 10);
        // Initialize subscribers
        position_sub_ = nh_.subscribe("/filtered_pose", 10, &BebopControl::positionCallback, this);


        // Initialize services
        set_reference_pose_srv_ = nh_.advertiseService("SetReferencePose", &BebopControl::handleSetReferenceSrv, this);
        set_circ_path_srv_ = nh_.advertiseService("SetCircPath", &BebopControl::handleSetCircPathSrv, this);
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
                                                                 
        std::vector<std::string> header_error = std::vector<std::string>{"timestamp", 
                                                                        "topic", 
                                                                        "ex",
                                                                        "int_ex",  
                                                                        "ey",
                                                                        "int_ey",  
                                                                        "ez",
                                                                        "int_ez", 
                                                                        "eyaw",
                                                                        "int_eyaw"};
        csv_logger_error_ = std::make_unique<CSVLogger>(workspace_name, 
                                                                "bebop_control", 
                                                                "error", 
                                                                header_error);    
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
        desired_pose_.z = 1.0;
        desired_pose_.yaw = 0.0;
        nh_.param("amp_ident_x", amp_ident_x_, 0.1);
        nh_.param("amp_ident_y", amp_ident_y_, 0.1);
        nh_.param("amp_ident_z", amp_ident_z_, 0.1);
        nh_.param("amp_ident_yaw", amp_ident_yaw_, 0.5);
        nh_.param("temp_ident", temp_ident_, 5.0);
        nh_.param("period_duration", period_duration_, 1.0);
        // Kx <<  -0.7636, -0.1476, -0.0287, -0.0262, 0.0, 0.0, 0.0, 0.0, 0.0, 0.0, 0.0, 0.0,
        //         0.0, 0.0, 0.0, 0.0,-0.5906,-0.1791, 0.0199,0.0130, 0.0, 0.0, 0.0, 0.0,
        //         0.0, 0.0, 0.0, 0.0, 0.0, 0.0, 0.0, 0.0, -0.6558,-0.7091, 0.0, 0.0,
        //         0.0, 0.0, 0.0, 0.0, 0.0, 0.0, 0.0, 0.0, 0.0, 0.0, -3.0674,-3.3309;
        Kx <<  -0.7636, -0.1476, -0.0287, -0.0262, 0.0, 0.0, 0.0, 0.0, 0.0, 0.0, 0.0, 0.0, //-0.7636, -0.1476, -0.0287, -0.0262
                0.0, 0.0, 0.0, 0.0,-0.5906,-0.1791, 0.0199,0.0130, 0.0, 0.0, 0.0, 0.0,
                0.0, 0.0, 0.0, 0.0, 0.0, 0.0, 0.0, 0.0, -0.6558,-0.7091, 0.0, 0.0, //-0.4558,-0.4091 | -0.6558,-0.7091
                0.0, 0.0, 0.0, 0.0, 0.0, 0.0, 0.0, 0.0, 0.0, 0.0, -1.0674,-1.3309; //-3.0674,-3.3309
        Ki << 0.001846, 0.0, 0.0, 0.0,//0.00246
                0.0, 0.001778, 0.0, 0.0, //0.00278
                0.0, 0.0, 0.002574, 0.0, //0.002574
                0.0, 0.0, 0.0, 0.00388; //0.0388
        //Parametros ARDrone
        // Kx <<  -1.5088, -0.8481, -2.4496, -0.5186, 0.0, 0.0, 0.0, 0.0, 0.0, 0.0, 0.0, 0.0, //-0.7636, -0.1476, -0.0287, -0.0262
        //         0.0, 0.0, 0.0, 0.0,-1.4914, -0.8853, 2.9098, 0.9564, 0.0, 0.0, 0.0, 0.0,
        //         0.0, 0.0, 0.0, 0.0, 0.0, 0.0, 0.0, 0.0, -0.6558,-0.7091, 0.0, 0.0, //-0.4558,-0.4091 | -0.6558,-0.7091
        //         0.0, 0.0, 0.0, 0.0, 0.0, 0.0, 0.0, 0.0, 0.0, 0.0, -0.9184, -0.1469; //-3.0674,-3.3309
        // Ki << 0.2587, 0.0, 0.0, 0.0,//0.00246
        //         0.0, 0.2608, 0.0, 0.0, //0.00278
        //         0.0, 0.0, 0.002574, 0.0, //0.002574
        //         0.0, 0.0, 0.0, 0.4797; //0.0388
    }
    bool handleSetCircPathSrv(bebop_control::SetCircPath::Request& req, bebop_control::SetCircPath::Response& res)
    {
        if (req.radius < 0.0)
        {
            ROS_WARN("Invalid radius value specified: %f. Radius value must be greater than or equal to 0.0", req.radius);
            res.status = false;
            return false;
        }

        circ_radius_ = req.radius;
        circ_angular_velocity_ = req.angular_velocity;
        circ_height_ = req.height;

        // Stop the existing trajectory timer if it is running
        if (trajectory_timer_started_)
        {
            trajectory_timer_.stop();
            trajectory_timer_started_ = false;
        }

        // Start the trajectory timer
        trajectory_timer_ = nh_.createTimer(ros::Duration(0.1), &BebopControl::updateCircularTrajectory, this);
        trajectory_timer_started_ = true;
        res.status = true;
        ROS_INFO("Circular path set with radius = %f, angular velocity = %f, height = %f", circ_radius_, circ_angular_velocity_, circ_height_);
        return true;
    }
    void updateCircularTrajectory(const ros::TimerEvent&)
    {
        static double time = 0.0;
        time += 0.1;

        desired_pose_.x = circ_radius_ * cos(circ_angular_velocity_ * time);
        desired_pose_.y = circ_radius_ * sin(circ_angular_velocity_ * time);
        desired_pose_.z = circ_height_;
        // desired_pose_.yaw = atan2(desired_pose_.y, desired_pose_.x);
        desired_pose_.yaw = 0;
        // // Log the desired trajectory
        // std::vector<std::variant<std::string, double>> data_trajectory = {ros::Time::now().toSec(), 
        //                                                                   "desired_trajectory", 
        //                                                                   desired_pose_.x, 
        //                                                                   desired_pose_.y, 
        //                                                                   desired_pose_.z, 
        //                                                                   desired_pose_.yaw};
        // csv_logger_desired_trajectory_->writeCSV(data_trajectory);
    }
    bool handleSetReferenceSrv(bebop_control::SetReferencePose::Request& req, bebop_control::SetReferencePose::Response& res)
    {
        if (req.z < 0.0)
        {
            ROS_WARN("Invalid z value specified: %f. Z value must be greater than or equal to 0.0", req.z);
            res.status = false;
            return false;
        }
        desired_pose_.x = req.x;
        desired_pose_.y = req.y;
        desired_pose_.z = req.z;
        desired_pose_.yaw = req.heading;
        res.status = true;
        ROS_INFO("Reference pose set to x = %f, y = %f, z = %f, yaw = %f", desired_pose_.x, desired_pose_.y, desired_pose_.z, desired_pose_.yaw);
        return true;
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
                if (trajectory_timer_started_)
                {
                    trajectory_timer_started_ = false;  
                    trajectory_timer_.stop();
                }
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

        // Log errors
        std::vector<std::variant<std::string, double>> data_error = {ros::Time::now().toSec(), 
                                                                        "errors", 
                                                                        error[0], 
                                                                        integral_error[0],
                                                                        error[1],
                                                                        integral_error[1],
                                                                        error[2],
                                                                        integral_error[2],
                                                                        error[3],
                                                                        integral_error[3]};
        csv_logger_error_->writeCSV(data_error);

        // Calculate control input

        u = Kx * x_estates + Ki * integral_error;
        
        twist_msg.linear.x = u(0);
        twist_msg.linear.y = u(1);
        twist_msg.linear.z = u(2);
        twist_msg.angular.z = u(3);
        // twist_msg.linear.x = 0;
        // twist_msg.linear.y = 0;
        // twist_msg.linear.z = u(2);       
        // twist_msg.angular.z = 0;
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

    BebopControl node(nh);

    // Start the teleopKeyboard function in a separate thread
    std::thread teleop_thread(&BebopControl::teleopKeyboard, &node);

    // node.teleopKeyboard();
    node.spin();

    teleop_thread.join(); // Wait for the teleop thread to finish
    return 0;
}