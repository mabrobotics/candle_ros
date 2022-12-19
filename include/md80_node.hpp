#include "candle.hpp"
#include "candle_ros/AddMd80s.h"
#include "candle_ros/GenericMd80Msg.h"
#include "candle_ros/ImpedanceCommand.h"
#include "candle_ros/MotionCommand.h"
#include "candle_ros/PositionPidCommand.h"
#include "candle_ros/SetLimitsMd80.h"
#include "candle_ros/SetModeMd80s.h"
#include "candle_ros/VelocityPidCommand.h"
#include "ros/ros.h"
#include "sensor_msgs/JointState.h"

class Md80Node
{
   public:
	Md80Node();
	Md80Node(int argc, char** argv);
	~Md80Node();

   private:
	std::vector<mab::Candle*> candleInstances;

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

	bool service_addMd80(candle_ros::AddMd80s::Request& request, candle_ros::AddMd80s::Response& response);
	bool service_zeroMd80(candle_ros::GenericMd80Msg::Request& request, candle_ros::GenericMd80Msg::Response& response);
	bool service_setModeMd80(candle_ros::SetModeMd80s::Request& request, candle_ros::SetModeMd80s::Response& response);
	bool service_enableMd80(candle_ros::GenericMd80Msg::Request& request, candle_ros::GenericMd80Msg::Response& response);
	bool service_disableMd80(candle_ros::GenericMd80Msg::Request& request, candle_ros::GenericMd80Msg::Response& response);

	mab::Candle* findCandleByMd80Id(uint16_t md80Id);
	void publishJointStates();
	void motionCommandCallback(const candle_ros::MotionCommand::ConstPtr& msg);
	void impedanceCommandCallback(const candle_ros::ImpedanceCommand::ConstPtr& msg);
	void velocityCommandCallback(const candle_ros::VelocityPidCommand::ConstPtr& msg);
	void positionCommandCallback(const candle_ros::PositionPidCommand::ConstPtr& msg);
};
