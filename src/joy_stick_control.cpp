#include <ros/ros.h>
#include <ros/console.h>
#include <string>
#include <sensor_msgs/Joy.h>
#include <ackermann_msgs/AckermannDriveStamped.h>
#include <std_msgs/Bool.h>
#include <std_msgs/Int32.h>

class JOY_CONTROL
{
    public:
        JOY_CONTROL(ros::NodeHandle& n);
        ~JOY_CONTROL();
        void joy_callback(const sensor_msgs::Joy::ConstPtr& joy_msg);
        void auto_callback(const ackermann_msgs::AckermannDriveStampedConstPtr& msg);
        void visual_callback(const ackermann_msgs::AckermannDriveStampedConstPtr& msg);
        void etri_mobile_callback(const sensor_msgs::Joy::ConstPtr& joy_msg);
        void etri_manual_command_callback(const ackermann_msgs::AckermannDriveStampedConstPtr &msg);


        void run();

    private:        
        ros::NodeHandle nh;
        ros::Subscriber SubJoyCommand, SubAutoCommand, SubVisualCommand, SubEtriMobileCommand, SubEtriManualCommand;
        ros::Publisher PubVehCommand, pubCameraMotorInit, PubOperationMode;

        ackermann_msgs::AckermannDriveStamped VehCommand;
        ackermann_msgs::AckermannDriveStamped m_AutoCommand, m_VisualCommand, m_EtriManualCommand;

        bool AutonmousMode, VisualMode, bManualMode, bEtriMode;
        double maximum_speed; //kph
        double maximum_steering_angle;

        bool reverse_angle;
        bool init_camera_motor;

};

JOY_CONTROL::JOY_CONTROL(ros::NodeHandle& n) : nh(n), maximum_steering_angle(20), maximum_speed(5.0),
                                               AutonmousMode(false), init_camera_motor(false), bManualMode(false), bEtriMode(false)
{
    // Subscribers
    n.param("/etri_joy_controller_node/maximum_speed", maximum_speed, double(5.0));
    n.param("/etri_joy_controller_node/maximum_steering_angle", maximum_steering_angle, double(20));
    n.param("/etri_joy_controller_node/reverse_angle", reverse_angle, bool(true));

    SubJoyCommand = nh.subscribe<sensor_msgs::Joy>("/joy", 10, &JOY_CONTROL::joy_callback, this);   
    SubEtriMobileCommand = nh.subscribe<sensor_msgs::Joy>("/joy/etri_mobile", 10, &JOY_CONTROL::etri_mobile_callback, this);   

    SubVisualCommand = nh.subscribe<ackermann_msgs::AckermannDriveStamped>("/Ackermann/command/visual", 10, &JOY_CONTROL::visual_callback, this);
    SubAutoCommand = nh.subscribe<ackermann_msgs::AckermannDriveStamped>("/Ackermann/command/auto", 10, &JOY_CONTROL::auto_callback, this);
    SubEtriManualCommand = nh.subscribe<ackermann_msgs::AckermannDriveStamped>("/Ackermann/command/etri", 10, 
                                        &JOY_CONTROL::etri_manual_command_callback, this);

    PubVehCommand = n.advertise<ackermann_msgs::AckermannDriveStamped>("/Ackermann/command/joy", 10);
    pubCameraMotorInit = n.advertise<std_msgs::Bool>("/Bool/CameraMotorInit", 10);
    PubOperationMode = n.advertise<std_msgs::Int32>("/Int/OperationMode", 10);
};

JOY_CONTROL::~JOY_CONTROL() 
{    
    ROS_INFO("JOY_CONTROL DESTRUCTOR.");
};

void JOY_CONTROL::visual_callback(const ackermann_msgs::AckermannDriveStampedConstPtr &msg)
{
    m_VisualCommand = *msg;
}

void JOY_CONTROL::auto_callback(const ackermann_msgs::AckermannDriveStampedConstPtr &msg)
{
    m_AutoCommand = *msg;
}

void JOY_CONTROL::etri_manual_command_callback(const ackermann_msgs::AckermannDriveStampedConstPtr &msg)
{
    m_EtriManualCommand = *msg;
}
void JOY_CONTROL::etri_mobile_callback(const sensor_msgs::Joy::ConstPtr& joy_msg)
{    
    if(!bEtriMode)
        return;

    VehCommand.drive.speed =0;
    VehCommand.drive.steering_angle =0;
    init_camera_motor = false;
    bManualMode = false;

    if(joy_msg->buttons.at(4) == 1)
    {
        init_camera_motor = true;
    }
    std_msgs::Bool init_camera_motor_msg;
    init_camera_motor_msg.data = init_camera_motor;
    pubCameraMotorInit.publish(init_camera_motor_msg);

    //Visual-servoing Mode Toggle    
    if( joy_msg->buttons.at(3) == 1 && VisualMode == false){
        VisualMode = true;
    }
    else if ( joy_msg->buttons.at(0) == 1 && VisualMode == true){
        VisualMode = false;
    }

    //E-STOP    
    if( joy_msg->buttons.at(1) == 1 && AutonmousMode == false){
        AutonmousMode = true;
    }
    else if ( joy_msg->buttons.at(0) == 1 && AutonmousMode == true){
        AutonmousMode = false;
    }

    if( joy_msg->buttons.at(1) == 1 && VisualMode == true){
        VisualMode = false;
    }

    // Steering angle reverse
    double reverse = 1;
    if(reverse_angle)
    {
        reverse = -1;
    }

    if(AutonmousMode == true || VisualMode == true){
        //Manual Control
        if( joy_msg->buttons.at(5) == 1){
            VehCommand.drive.speed = m_EtriManualCommand.drive.speed;
            VehCommand.drive.steering_angle =  reverse * m_EtriManualCommand.drive.steering_angle;
            bManualMode = true;
        }
        //Visual servoing control
        else if(VisualMode)
        {
            VehCommand.drive.speed = m_VisualCommand.drive.speed;
            VehCommand.drive.steering_angle = reverse * m_VisualCommand.drive.steering_angle;
        }
        //Auto Control
        else
        {
            VehCommand.drive.speed = m_AutoCommand.drive.speed;
            VehCommand.drive.steering_angle = reverse * m_AutoCommand.drive.steering_angle;
        }
    }
    else
    {
        VehCommand.drive.speed = 0;
        VehCommand.drive.steering_angle = 0;
    }
};


void JOY_CONTROL::joy_callback(const sensor_msgs::Joy::ConstPtr& joy_msg)
{    
    if(joy_msg->buttons.at(8) == 1 && bEtriMode == false)
    {
        bEtriMode = true;
        VisualMode = false;
        AutonmousMode = false;
    }
    else if(joy_msg->buttons.at(9) == 1 && bEtriMode == true)
    {
        bEtriMode = false;
    }
    if(bEtriMode)
        return;

    VehCommand.drive.speed =0;
    VehCommand.drive.steering_angle =0;
    init_camera_motor = false;
    bManualMode = false;

    if(joy_msg->buttons.at(4) == 1)
    {
        init_camera_motor = true;
    }
    std_msgs::Bool init_camera_motor_msg;
    init_camera_motor_msg.data = init_camera_motor;
    pubCameraMotorInit.publish(init_camera_motor_msg);


    //Visual-servoing Mode Toggle    
    if( joy_msg->buttons.at(3) == 1 && VisualMode == false){
        VisualMode = true;
    }
    else if ( joy_msg->buttons.at(0) == 1 && VisualMode == true){
        VisualMode = false;
    }

    //E-STOP    
    if( joy_msg->buttons.at(1) == 1 && AutonmousMode == false){
        AutonmousMode = true;
    }
    else if ( joy_msg->buttons.at(0) == 1 && AutonmousMode == true){
        AutonmousMode = false;
    }

    if( joy_msg->buttons.at(1) == 1 && VisualMode == true){
        VisualMode = false;
    }

    // Steering angle reverse
    double reverse = 1;
    if(reverse_angle)
    {
        reverse = -1;
    }

    if(AutonmousMode == true || VisualMode == true){
        //Manual Control
        if( joy_msg->buttons.at(5) == 1){
            VehCommand.drive.speed = maximum_speed * joy_msg->axes.at(4);
            VehCommand.drive.steering_angle =  reverse * maximum_steering_angle * joy_msg->axes.at(0);
            bManualMode = true;
        }
        //Visual servoing control
        else if(VisualMode)
        {
            VehCommand.drive.speed = m_VisualCommand.drive.speed;
            VehCommand.drive.steering_angle = reverse * m_VisualCommand.drive.steering_angle;
        }
        //Auto Control
        else
        {
            VehCommand.drive.speed = m_AutoCommand.drive.speed;
            VehCommand.drive.steering_angle = reverse * m_AutoCommand.drive.steering_angle;
        }
    }
    else
    {
        VehCommand.drive.speed = 0;
        VehCommand.drive.steering_angle = 0;
    }
};

void JOY_CONTROL::run()
{
    PubVehCommand.publish(VehCommand);
    std_msgs::Int32 operationModeMsg;
    if (AutonmousMode || VisualMode)
    {
        if(!bEtriMode)
            std::cout << "-----------KAIST_MODE----------" << std::endl;
        else
        {
            std::cout << "-----------ETRI_MODE----------" << std::endl;
        }   
        if(VisualMode)
        {
            std::cout << "VISUAL MODE" << std::endl;
            operationModeMsg.data = 2;
        }
            
        else if(bManualMode)
        {
            std::cout << "MANUAL MODE" << std::endl;
            operationModeMsg.data = 1;
        }
            
        else
        {
            std::cout << "AUTONOMOUS MODE" << std::endl;
            operationModeMsg.data = 3;
        }
        
        std::cout << "Speed: " << VehCommand.drive.speed << std::endl;
        std::cout << "Steering: " << VehCommand.drive.steering_angle << std::endl; 
    }

    else
    {
        ROS_INFO("Stop Mode");
        if(!bEtriMode)
            std::cout << "-----------KAIST_MODE----------" << std::endl;
        else
        {
            std::cout << "-----------ETRI_MODE----------" << std::endl;
        }
        operationModeMsg.data = 4;
    }
    PubOperationMode.publish(operationModeMsg);


}

int main(int argc, char** argv)
{
    ros::init(argc, argv, "etri_joy_controller_node");
    ros::NodeHandle nh;   
    JOY_CONTROL joy_control(nh);
    ros::Rate loop_rate(40);

    while (ros::ok()) {
        ros::spinOnce();
        joy_control.run();
        loop_rate.sleep();
    }
    return 0;
}
