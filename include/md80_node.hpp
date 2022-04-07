#include "ros/ros.h"
#include "candle.hpp"

#include "ros1_mab_md80/ImpedanceCommand.h"
#include "ros1_mab_md80/MotionCommand.h"
#include "ros1_mab_md80/VelocityPidCommand.h"
#include "ros1_mab_md80/PositionPidCommand.h"

#include "ros1_mab_md80/AddMd80s.h"
#include "ros1_mab_md80/GenericMd80Msg.h"
#include "ros1_mab_md80/SetModeMd80s.h"
#include "ros1_mab_md80/SetLimitsMd80.h"

#include "sensor_msgs/JointState.h"

class Md80Node
{
public:
    Md80Node();
    ~Md80Node();

private:

    mab::Candle*candle;

    ros::NodeHandle n;
    ros::Timer pubTimer;
    
    ros::ServiceServer addMd80Service;
    ros::ServiceServer zeroMd80Service;
    ros::ServiceServer setModeMd80Service;
    ros::ServiceServer enableMd80Service;
    ros::ServiceServer disableMd80Service;
    ros::ServiceServer setLimitsMd80Service;

    ros::Publisher jointStatePub;
        
    ros::Subscriber impedanceCommandSub;
    ros::Subscriber motionCommandSub;
    ros::Subscriber velocityCommandSub;
    ros::Subscriber positionCommandSub;

    bool service_addMd80(ros1_mab_md80::AddMd80s::Request& request, ros1_mab_md80::AddMd80s::Response& response);
    bool service_zeroMd80(ros1_mab_md80::GenericMd80Msg::Request& request, ros1_mab_md80::GenericMd80Msg::Response& response);
    bool service_setModeMd80(ros1_mab_md80::SetModeMd80s::Request& request, ros1_mab_md80::SetModeMd80s::Response& response);
    bool service_enableMd80(ros1_mab_md80::GenericMd80Msg::Request& request,ros1_mab_md80::GenericMd80Msg::Response& response);
    bool service_disableMd80(ros1_mab_md80::GenericMd80Msg::Request& request, ros1_mab_md80::GenericMd80Msg::Response& response);
    bool service_setLimitsMd80(ros1_mab_md80::SetLimitsMd80::Request& request, ros1_mab_md80::SetLimitsMd80::Response& response);

    void publishJointStates();
    
    void motionCommandCallback(const ros1_mab_md80::MotionCommand::ConstPtr& msg);
    void impedanceCommandCallback(const ros1_mab_md80::ImpedanceCommand::ConstPtr& msg);
    void velocityCommandCallback(const ros1_mab_md80::VelocityPidCommand::ConstPtr& msg);
    void positionCommandCallback(const ros1_mab_md80::PositionPidCommand::ConstPtr& msg);

};
