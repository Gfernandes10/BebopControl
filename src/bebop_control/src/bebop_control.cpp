#include <ros/ros.h>
#include <std_msgs/String.h>
#include <std_msgs/Empty.h>
#include <sensor_msgs/Joy.h>
#include <geometry_msgs/Twist.h>
#include <termios.h>
#include <unistd.h>
#include <iostream>

// Define key codes for arrow keys and other controls
#define KEYCODE_L 0x44
#define KEYCODE_R 0x43
#define KEYCODE_U 0x41
#define KEYCODE_D 0x42
#define KEYCODE_A 0x61
#define KEYCODE_S 0x64

class bebop_control
{
public:
    // Class members
    ros::Publisher takeoff_pub_;
    ros::Publisher land_pub_;
    ros::Publisher cmd_vel_pub_;
    //Parameters
    int axis_roll_;
    int axis_pitch_;
    int axis_thrust_;
    int axis_yaw_;
    int button_takeoff_;
    int button_land_;
    int button_emergency_;
    int button_ident_z_;
    int button_ident_y_;
    int button_ident_x_;
    int button_ident_yaw_;
    bool control_ident_z_ = false;
    bool control_ident_y_ = false;
    bool control_ident_x_ = false;
    bool control_ident_yaw_ = false;
    bool enable_control_keyboard_ = true;
    //Messages Declaration
    std_msgs::Empty empty_msg;
    geometry_msgs::Twist twist_msg;
    bebop_control(ros::NodeHandle& nh)
    {
        // Initialize parameters
        initializeParameters(nh);

        // Initialize publishers
        takeoff_pub_ = nh.advertise<std_msgs::Empty>("bebop/takeoff", 10);
        land_pub_ = nh.advertise<std_msgs::Empty>("bebop/land", 10);
        cmd_vel_pub_ = nh.advertise<geometry_msgs::Twist>("bebop/cmd_vel", 10);
        // Initialize subscribers


        // Initialize services
        // service_ = nh.advertiseService("service_name", &bebop_control::serviceCallback, this);

    }

    void spin()
    {
        ros::spin();
    }


    // Function to initialize parameters
    void initializeParameters(ros::NodeHandle& nh)
    {
        
        nh.param("axis_roll", axis_roll_, 3);
        nh.param("axis_pitch", axis_pitch_, 4);
        nh.param("axis_thrust", axis_thrust_, 1);
        nh.param("axis_yaw", axis_yaw_, 0);
        nh.param("button_takeoff", button_takeoff_, 5);
        nh.param("button_land", button_land_, 7);
        nh.param("button_emergency", button_emergency_, 4);
        nh.param("button_ident_z", button_ident_z_, 0);
        nh.param("button_ident_y", button_ident_y_, 1);
        nh.param("button_ident_x", button_ident_x_, 2);
        nh.param("button_ident_yaw", button_ident_yaw_, 3);
        // Add more parameters as needed
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

        std::cout << "Reading from the keyboard and Publishing to Twist!" << std::endl;
        std::cout << "W: Up, S: Down, A: Yaw Left, D: Yaw Right" << std::endl;
        std::cout << "Arrow Up: Forward, Arrow Down: Backward, Arrow Left: Left, Arrow Right: Right" << std::endl;
        std::cout << "T: Takeoff, L: Land, CTRL+C to quit." << std::endl;

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
                    // // Reset twist message
                    // twist_msg.linear.x = 0.0;
                    // twist_msg.linear.y = 0.0;
                    // twist_msg.linear.z = 0.0;
                    // twist_msg.angular.z = 0.0;

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
                            break;
                        case 'l':
                            ROS_INFO("Land command received");
                            land_pub_.publish(empty_msg);
                            break;
                        case '1':
                            control_ident_z_ = !control_ident_z_;
                            enable_control_keyboard_ = false;
                            // TO-DO: Make this parameters configurable
                            sendAlternatingStepCommand(0.1, 5.0, 1.0, "z", control_ident_z_);
                            enable_control_keyboard_ = true;
                            break;
                        case '2':
                            control_ident_y_ = !control_ident_y_;
                            enable_control_keyboard_ = false;
                            sendAlternatingStepCommand(0.1, 5.0, 1.0, "y", control_ident_y_);
                            enable_control_keyboard_ = true;
                            break;
                        case '3':
                            control_ident_x_ = !control_ident_x_;
                            enable_control_keyboard_ = false;
                            sendAlternatingStepCommand(0.1, 5.0, 1.0, "x", control_ident_x_);
                            enable_control_keyboard_ = true;
                            break;
                        case '4':
                            control_ident_yaw_ = !control_ident_yaw_;
                            enable_control_keyboard_ = false;
                            sendAlternatingStepCommand(0.1, 5.0, 1.0, "yaw", control_ident_yaw_);
                            enable_control_keyboard_ = true;
                            break;
                        case '\x03': // CTRL+C
                            return;
                        default:
                            break;
                    }
                }
            }

            cmd_vel_pub_.publish(twist_msg);
            rate.sleep(); // Sleep to maintain the loop rate
        }

        // Restore the old terminal settings
        tcsetattr(STDIN_FILENO, TCSANOW, &oldt);
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
                        ROS_INFO("Setting linear.x to %f", twist_msg.linear.x);
                    }
                    else if (axis == "y")
                    {
                        twist_msg.linear.y = positive ? amp : -amp;
                        ROS_INFO("Setting linear.y to %f", twist_msg.linear.y);
                    }
                    else if (axis == "z")
                    {
                        twist_msg.linear.z = positive ? amp : -amp;
                        ROS_INFO("Setting linear.z to %f", twist_msg.linear.z);
                    }
                    else if (axis == "yaw")
                    {
                        twist_msg.angular.z = positive ? amp : -amp;
                        ROS_INFO("Setting angular.z to %f", twist_msg.angular.z);
                    }
                    else
                    {
                        ROS_WARN("Invalid axis specified: %s", axis.c_str());
                        return;
                    }

                    cmd_vel_pub_.publish(twist_msg); // Publica o comando

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
        cmd_vel_pub_.publish(twist_msg);

        ROS_INFO("Reset twist message after duration.");
    }
};

int main(int argc, char** argv)
{
    ros::init(argc, argv, "my_ros_node");
    ros::NodeHandle nh;
    
    bebop_control node(nh);
    node.teleopKeyboard();
    node.spin();

    return 0;
}