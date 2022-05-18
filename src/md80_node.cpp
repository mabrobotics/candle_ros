#include "md80_node.hpp"

Md80Node::Md80Node()
{
	candle = new mab::Candle(mab::CANdleBaudrate_E::CAN_BAUD_1M, true);

	addMd80Service = n.advertiseService("/add_md80s", &Md80Node::service_addMd80, this);
    zeroMd80Service = n.advertiseService("/zero_md80s", &Md80Node::service_zeroMd80, this);
    setModeMd80Service = n.advertiseService("/set_mode_md80s", &Md80Node::service_setModeMd80, this);
    enableMd80Service = n.advertiseService("/enable_md80s", &Md80Node::service_enableMd80, this);
    disableMd80Service = n.advertiseService("/disable_md80s", &Md80Node::service_disableMd80, this);

	motionCommandSub = n.subscribe("md80/motion_command", 1000, &Md80Node::motionCommandCallback, this);
	impedanceCommandSub = n.subscribe("md80/impedance_command", 1000, &Md80Node::impedanceCommandCallback, this);
	velocityCommandSub = n.subscribe("md80/velocity_pid_command", 1000, &Md80Node::velocityCommandCallback, this);
	positionCommandSub = n.subscribe("md80/position_pid_command", 1000, &Md80Node::positionCommandCallback, this);

	jointStatePub = n.advertise<sensor_msgs::JointState>("md80/joint_states", 1000);

	pubTimer = n.createTimer(ros::Duration(0.1), std::bind(&Md80Node::publishJointStates, this));
	pubTimer.stop();
	
	ROS_INFO("candle_ros_node has started.");
}

Md80Node::~Md80Node()
{
	delete candle;
	ROS_INFO("candle_ros_node finished.");
}

bool Md80Node::service_addMd80(candle_ros::AddMd80s::Request& request, candle_ros::AddMd80s::Response& response)
{ 
	for(auto&id : request.drive_ids)
		response.drives_success.push_back(candle->addMd80(id));

	response.total_number_of_drives = candle->md80s.size();

	return true;
}

bool Md80Node::service_zeroMd80(candle_ros::GenericMd80Msg::Request& request, candle_ros::GenericMd80Msg::Response& response)
{
	for(auto&id : request.drive_ids)
		response.drives_success.push_back(candle->controlMd80SetEncoderZero(id));

	return true;
}

bool Md80Node::service_setModeMd80(candle_ros::SetModeMd80s::Request& request, candle_ros::SetModeMd80s::Response& response)
{
	if(request.drive_ids.size() != request.mode.size())
	{
		for(auto&id : request.drive_ids)
		{
			(void)id;
			response.drives_success.push_back(false);
		}
		ROS_WARN("SetMode request incomplete. Sizes of arrays do not match!");
		return 0;
	}
	for(int i = 0; i < (int)request.drive_ids.size(); i++)
	{
		mab::Md80Mode_E mode;
		
		if(request.mode[i] == "IMPEDANCE")
			mode = mab::Md80Mode_E::IMPEDANCE;
		else if(request.mode[i] == "POSITION_PID")
			mode = mab::Md80Mode_E::POSITION_PID;
		else if(request.mode[i] == "VELOCITY_PID")
			mode = mab::Md80Mode_E::VELOCITY_PID;
		else if(request.mode[i] == "TORQUE")
			mode = mab::Md80Mode_E::TORQUE;
		else
		{
			ROS_WARN("MODE %s not recognized, setting IDLE for driveID = %d", request.mode[i].c_str(), request.drive_ids[i]);
			mode = mab::Md80Mode_E::IDLE;
		}
		response.drives_success.push_back(candle->controlMd80Mode(request.drive_ids[i], mode));
	}

	return true;
}

bool Md80Node::service_enableMd80(candle_ros::GenericMd80Msg::Request& request,candle_ros::GenericMd80Msg::Response& response)
{
	for(auto&id : request.drive_ids)
		response.drives_success.push_back(candle->controlMd80Enable(id,true));
	candle->begin();
	pubTimer.start();

	return true;
}

bool Md80Node::service_disableMd80(candle_ros::GenericMd80Msg::Request& request, candle_ros::GenericMd80Msg::Response& response)
{
	candle->end();
	pubTimer.stop();
	for(auto&id : request.drive_ids)
		response.drives_success.push_back(candle->controlMd80Enable(id,false));
	
	return true;
}

void Md80Node::publishJointStates()
{
	sensor_msgs::JointState jointStateMsg;
	jointStateMsg.header.stamp = ros::Time::now();
	for(auto&md : candle->md80s)
	{
		jointStateMsg.name.push_back(std::string("Joint " + std::to_string(md.getId())));
		jointStateMsg.position.push_back(md.getPosition());
		jointStateMsg.velocity.push_back(md.getVelocity());
		jointStateMsg.effort.push_back(md.getTorque());
	}
	this->jointStatePub.publish(jointStateMsg);
}

void Md80Node::motionCommandCallback(const candle_ros::MotionCommand::ConstPtr& msg)
{
	if(msg->drive_ids.size() != msg->target_position.size() || msg->drive_ids.size() != msg->target_velocity.size() ||
		msg->drive_ids.size() != msg->target_torque.size())
	{
		ROS_WARN("Motion Command message incomplete. Sizes of arrays do not match! Ignoring message.");
		return;
	}
	for(int i = 0; i < (int)msg->drive_ids.size(); i++)
	{
		try
		{
			auto&md = candle->getMd80FromList(msg->drive_ids[i]);
			md.setTargetPosition(msg->target_position[i]);
			md.setTargetVelocity(msg->target_velocity[i]);
			md.setTorque(msg->target_torque[i]);
		}
		catch(const char* eMsg)
		{
			ROS_WARN(eMsg);
		}

	}
}

void Md80Node::impedanceCommandCallback(const candle_ros::ImpedanceCommand::ConstPtr& msg)
{
	if(msg->drive_ids.size() != msg->kp.size() || msg->drive_ids.size() != msg->kd.size())
	{
		ROS_WARN("Impedance Command message incomplete. Sizes of arrays do not match! Ignoring message.");
		return;
	}

	for(int i = 0; i < (int)msg->drive_ids.size(); i++)
	{
		try
		{
			auto&md = candle->getMd80FromList(msg->drive_ids[i]);
			md.setImpedanceControllerParams(msg->kp[i], msg->kd[i]);
			md.setMaxTorque(msg->max_output[i]);
		}
		catch(const char* eMsg)
		{
			ROS_WARN(eMsg);
		}

	}
}

void Md80Node::velocityCommandCallback(const candle_ros::VelocityPidCommand::ConstPtr& msg)
{
	if(msg->drive_ids.size() != msg->velocity_pid.size())
	{
		ROS_WARN("Velocity Command message incomplete. Sizes of arrays do not match! Ignoring message.");
		return;
	}
	for(int i = 0; i < (int)msg->drive_ids.size(); i++)
	{
		try
		{
			auto&md = candle->getMd80FromList(msg->drive_ids[i]);
			md.setVelocityControllerParams(msg->velocity_pid[i].kp, msg->velocity_pid[i].ki, msg->velocity_pid[i].kd, msg->velocity_pid[i].i_windup);
			md.setMaxTorque(msg->velocity_pid[i].max_output);
		}
		catch(const char* eMsg)
		{
			ROS_WARN(eMsg);
		}

	}
}

void Md80Node::positionCommandCallback(const candle_ros::PositionPidCommand::ConstPtr& msg)
{
	if(msg->drive_ids.size() != msg->position_pid.size())
	{
		ROS_WARN("Position Command message incomplete. Sizes of arrays do not match! Ignoring message.");
		return;
	}
	for(int i = 0; i < (int)msg->drive_ids.size(); i++)
	{
		try
		{
			auto&md = candle->getMd80FromList(msg->drive_ids[i]);
			md.setPositionControllerParams(msg->position_pid[i].kp, msg->position_pid[i].ki, msg->position_pid[i].kd, msg->position_pid[i].i_windup);
			md.setMaxVelocity(msg->position_pid[i].max_output);
			if(i < (int)msg->velocity_pid.size())
				md.setVelocityControllerParams(msg->velocity_pid[i].kp, msg->velocity_pid[i].ki, msg->velocity_pid[i].kd, msg->velocity_pid[i].i_windup);
				md.setMaxTorque(msg->velocity_pid[i].max_output);
		}
		catch(const char* eMsg)
		{
			ROS_WARN(eMsg);
		}

	}
}

int main(int argc, char **argv)
{
	ros::init(argc, argv, "candle_ros_node");
	Md80Node n;
	ros::spin();

    return 0;
}