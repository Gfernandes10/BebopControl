#include <ros/ros.h>
#include <std_msgs/String.h>
#include <sensor_msgs/Joy.h>

class MyRosNode
{
public:
    MyRosNode(ros::NodeHandle& nh)
    {
        // Inicialize parâmetros
        initializeParameters(nh);

        // Inicialize publishers
        joy_pub_ = nh.advertise<sensor_msgs::Joy>("joy", 10);

        // Inicialize subscribers
        joy_sub_ = nh.subscribe("joy", 10, &MyRosNode::joyCallback, this);

        // Inicialize serviços
        // service_ = nh.advertiseService("service_name", &MyRosNode::serviceCallback, this);

        // Inicialize timers
        timer_ = nh.createTimer(ros::Duration(1.0), &MyRosNode::timerCallback, this);
    }

    void spin()
    {
        ros::spin();
    }

private:
    // Função para inicializar parâmetros
    void initializeParameters(ros::NodeHandle& nh)
    {
        nh.param("param_name", param_value_, default_value);
        // Adicione mais parâmetros conforme necessário
    }

    // Callback para o subscriber
    void joyCallback(const sensor_msgs::Joy::ConstPtr& msg)
    {
        ROS_INFO("Received joystick input");
        // Processar os comandos do joystick aqui
    }

    // Callback para o serviço
    // bool serviceCallback(std_srvs::Empty::Request& req, std_srvs::Empty::Response& res)
    // {
    //     ROS_INFO("Service called");
    //     // Processar a chamada do serviço aqui
    //     return true;
    // }

    // Callback para o timer
    void timerCallback(const ros::TimerEvent&)
    {
        ROS_INFO("Timer triggered");
        // Processar eventos do timer aqui
    }

    // Membros da classe
    ros::Publisher joy_pub_;
    ros::Subscriber joy_sub_;
    // ros::ServiceServer service_;
    ros::Timer timer_;

    // Parâmetros
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