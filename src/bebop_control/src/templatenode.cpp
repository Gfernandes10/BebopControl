#include <ros/ros.h>
#include <std_msgs/String.h>
#include <sensor_msgs/Joy.h>

class MyRosNode
{
public:
    MyRosNode(ros::NodeHandle& nh)
    {
        // Initialize parameters
        initializeParameters(nh);

        // Initialize publishers
        joy_pub_ = nh.advertise<sensor_msgs::Joy>("joy", 10);

        // Initialize subscribers
        joy_sub_ = nh.subscribe("joy", 10, &MyRosNode::joyCallback, this);

        // Initialize services
        // service_ = nh.advertiseService("service_name", &MyRosNode::serviceCallback, this);

        // Initialize timers
        timer_ = nh.createTimer(ros::Duration(1.0), &MyRosNode::timerCallback, this);
    }

    void spin()
    {
        ros::spin();
    }

private:
    // Function to initialize parameters
    void initializeParameters(ros::NodeHandle& nh)
    {
        int default_value = 0; // Define default_value with the same type as param_value_
        nh.param("param_name", param_value_, default_value);
        // Add more parameters as needed
    }

    // Callback for the subscriber
    void joyCallback(const sensor_msgs::Joy::ConstPtr& msg)
    {
        ROS_INFO("Received joystick input");
        // Process joystick commands here
    }

    // Callback for the service
    // bool serviceCallback(std_srvs::Empty::Request& req, std_srvs::Empty::Response& res)
    // {
    //     ROS_INFO("Service called");
    //     // Process service call here
    //     return true;
    // }

    // Callback for the timer
    void timerCallback(const ros::TimerEvent&)
    {
        ROS_INFO("Timer triggered");
        // Process timer events here
    }

    // Class members
    ros::Publisher joy_pub_;
    ros::Subscriber joy_sub_;
    // ros::ServiceServer service_;
    ros::Timer timer_;

    // Parameters
    int param_value_;
};

int main(int argc, char** argv)
{
    ros::init(argc, argv, "my_ros_node");
    ros::NodeHandle nh;

    MyRosNode node(nh);
    node.spin();

    return 0;
}