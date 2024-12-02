#include <ros/ros.h>
#include <std_msgs/Empty.h>
#include <std_msgs/Bool.h>
#include <ncurses.h>

class keybrd_bebop_control
{
public:
    // Class members

    ros::Publisher take_pub_;
    ros::Publisher land_pub_;
    ros::Publisher takecontrol_pub_;
    // Parameters
    int keybrd_rate_;
    bool takeoff_ = false;
    std_msgs::Empty empty_msg;
    std_msgs::Bool takecontrol_msg;

    keybrd_bebop_control(ros::NodeHandle& nh)
    {
        // Initialize parameters
        initializeParameters(nh);
        // Initialize publishers
        take_pub_ = nh.advertise<std_msgs::Empty>("bebop/takeoff", 10);
        land_pub_ = nh.advertise<std_msgs::Empty>("bebop/land", 10);
        takecontrol_pub_ = nh.advertise<std_msgs::Bool>("bebop/takecontrol", 10);

        // Initialize ncurses
        initscr();
        cbreak();
        noecho();
        keypad(stdscr, TRUE);
        timeout(100); // Non-blocking getch with 100ms timeout

            
    }

    ~keybrd_bebop_control()
    {
        // Restore terminal settings
        endwin();
    }

    void spin()
    {
        ros::Rate rate(keybrd_rate_); // 10 Hz
        while (ros::ok())
        {
            ros::spinOnce();
            processKeyboardInput();
            rate.sleep();
        }
    }

private:
    // Function to initialize parameters
    void initializeParameters(ros::NodeHandle& nh)
    {
        takecontrol_msg.data = false;
        int default_keybrd_rate = 100; // Define default_value with the same type as param_value_
        nh.param("keyboard_rate", keybrd_rate_, default_keybrd_rate);       
        // Add more parameters as needed
        
    }

    // Process keyboard input
    void processKeyboardInput()
    {
        int key = getch();
        if (key != ERR)
        {
            takecontrol_msg.data = true;
            // Process key press here
            // Example: Publish a Twist message based on key press
                        switch (key)
            {
                case ' ':
                    takeoff_ = !takeoff_;
                    if (takeoff_)
                    {
                        take_pub_.publish(empty_msg);
                    }
                    else
                    {
                        land_pub_.publish(empty_msg);
                    }
                    break;
                default:                   
                    break;
            } 
        }

    }
};



int main(int argc, char** argv)
{
    ros::init(argc, argv, "keybrd_bebop_control");
    ros::NodeHandle nh;

    keybrd_bebop_control node(nh);
    node.spin();

    return 0;
}