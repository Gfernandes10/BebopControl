#include <ros/ros.h>
#include <std_msgs/String.h>
#include <std_msgs/Empty.h>
#include <sensor_msgs/Joy.h>
#include <geometry_msgs/Twist.h>

class bebop_control
{
public:
    // Class members
    ros::Publisher takeoff_pub_;
    ros::Publisher land_pub_;
    ros::Publisher cmd_vel_pub_;
    ros::Subscriber joy_sub_;
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
        joy_sub_ = nh.subscribe("joy", 10, &bebop_control::joyCallback, this);

        // Initialize services
        // service_ = nh.advertiseService("service_name", &bebop_control::serviceCallback, this);

    }

    void spin()
    {
        ros::spin();
    }

private:
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

    // Callback for the subscriber
    void joyCallback(const sensor_msgs::Joy::ConstPtr& msg)
    {
        ROS_INFO("Received joystick input");
        // Process joystick commands here
        // Read joystick axes
        double roll = msg->axes[axis_roll_];
        double pitch = msg->axes[axis_pitch_];
        double thrust = msg->axes[axis_thrust_];
        double yaw = msg->axes[axis_yaw_];

        // Read joystick buttons
        bool takeoff = msg->buttons[button_takeoff_];
        bool land = msg->buttons[button_land_];
        bool emergency = msg->buttons[button_emergency_];
        bool ident_z = msg->buttons[button_ident_z_];
        bool ident_y = msg->buttons[button_ident_y_];
        bool ident_x = msg->buttons[button_ident_x_];
        bool ident_yaw = msg->buttons[button_ident_yaw_];

        // Log the received values
        // ROS_INFO("Roll: %f, Pitch: %f, Thrust: %f, Yaw: %f", roll, pitch, thrust, yaw);
        // ROS_INFO("Takeoff: %d, Land: %d, Emergency: %d", takeoff, land, emergency);
        // ROS_INFO("Ident Z: %d, Ident Y: %d, Ident X: %d, Ident Yaw: %d", ident_z, ident_y, ident_x, ident_yaw);

        if (takeoff)
        {
            ROS_INFO("Takeoff command received");
            takeoff_pub_.publish(empty_msg);
        }
        else if (land)
        {
            ROS_INFO("Land command received");
            land_pub_.publish(empty_msg);
        }
        else if (emergency)
        {
            ROS_INFO("Emergency command received");
            twist_msg.linear.x = 0.0;
            twist_msg.linear.y = 0.0;
            twist_msg.linear.z = 0.0;
            twist_msg.angular.x = 0.0;
            twist_msg.angular.y = 0.0;
            twist_msg.angular.z = 0.0;
            cmd_vel_pub_.publish(twist_msg);
        }
        else if (ident_z)
        {           
            ROS_INFO("Steps Z command received");
            control_ident_z_ = !control_ident_z_;
            sendAlternatingStepCommand(0.1, 5.0, "z", control_ident_z_);
            ROS_INFO("Steps Z command ended");
        }
        else if (ident_y)
        {
            ROS_INFO("Steps Y command received");
            control_ident_y_ = !control_ident_y_;
            sendAlternatingStepCommand(0.1, 5.0, "y", control_ident_y_);
            ROS_INFO("Steps Y command ended");
        }
        else if (ident_x)
        {
            ROS_INFO("Steps X command received");
            control_ident_x_ = !control_ident_x_;
            sendAlternatingStepCommand(0.1, 5.0, "x", control_ident_x_);
            ROS_INFO("Steps X command ended");
        }
        else if (ident_yaw)
        {
            ROS_INFO("Steps Yaw command received");
            control_ident_yaw_ = !control_ident_yaw_;
            sendAlternatingStepCommand(0.1, 5.0, "yaw", control_ident_yaw_);
            ROS_INFO("Steps Yaw command ended");
        }

    }
    
    void sendAlternatingStepCommand(double amp, double temp, const std::string& axis, bool control_ident)
    {
        if (control_ident)
        {
            ros::Rate rate(10); // 10 Hz
            ros::Time start_time = ros::Time::now();
            ros::Duration duration(temp);
            bool positive = true;

            while (ros::Time::now() - start_time < duration)
            {
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
                    ROS_WARN("Invalid axis specified");
                    return;
                }

                cmd_vel_pub_.publish(twist_msg);
                rate.sleep();
                positive = !positive;
            }
        }

        // Reset the twist message after the duration
        twist_msg.linear.x = 0.0;
        twist_msg.linear.y = 0.0;
        twist_msg.linear.z = 0.0;
        twist_msg.angular.z = 0.0;
        cmd_vel_pub_.publish(twist_msg);
    }
};

int main(int argc, char** argv)
{
    ros::init(argc, argv, "my_ros_node");
    ros::NodeHandle nh;

    bebop_control node(nh);
    node.spin();

    return 0;
}