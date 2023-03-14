#include "md80_node.hpp"

const std::string version = "v1.2.1";

Md80Node::Md80Node(int argc, char** argv)
{
	if (argc < 3 || argc > 4)
	{
		std::cout << "Wrong arguments specified, please see candle_ros candle_ros_node --help" << std::endl;
		return;
	}

	if (strcmp(argv[1], "--help") == 0)
	{
		std::cout << "usage: candle_ros candle_ros_node <bus> <baud> [<device>] [--help]" << std::endl;
		std::cout << "<bus> can be SPI/USB/UART" << std::endl;
		std::cout << "<baud> can be 1M/2M/5M/8M" << std::endl;
		std::cout << "<device> SPI or UART device if default is not suitable" << std::endl;
		std::cout << "[--help] - displays help message" << std::endl;
		return;
	}

	mab::BusType_E bus = mab::BusType_E::USB;
	mab::CANdleBaudrate_E baud = mab::CAN_BAUD_1M;

	if (strcmp(argv[1], "SPI") == 0)
		bus = mab::BusType_E::SPI;
	else if (strcmp(argv[1], "USB") == 0)
		bus = mab::BusType_E::USB;
	else if (strcmp(argv[1], "UART") == 0)
		bus = mab::BusType_E::UART;
	else
	{
		std::cout << "bus parameter not recognised!" << std::endl;
		return;
	}

	if (strcmp(argv[2], "1M") == 0)
		baud = mab::CAN_BAUD_1M;
	else if (strcmp(argv[2], "2M") == 0)
		baud = mab::CAN_BAUD_2M;
	else if (strcmp(argv[2], "5M") == 0)
		baud = mab::CAN_BAUD_5M;
	else if (strcmp(argv[2], "8M") == 0)
		baud = mab::CAN_BAUD_8M;
	else
	{
		std::cout << "baud parameter not recognised!" << std::endl;
		return;
	}

	while (bus == mab::BusType_E::USB)
	{
		try
		{
			auto candle = new mab::Candle(baud, true, bus);
			std::cout << "[CANDLE] Found CANdle with ID: " << candle->getDeviceId() << std::endl;
			candleInstances.push_back(candle);
		}
		catch (const char* eMsg)
		{
			break;
		}
	}

	if (bus != mab::BusType_E::USB)
	{
		mab::Candle* candle = nullptr;

		if (argc == 4 && argv[3] != 0)
			candle = new mab::Candle(baud, true, bus, argv[3]);
		else
			candle = new mab::Candle(baud, true, bus);

		candleInstances.push_back(candle);
	}

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

	ROS_INFO("candle_ros_node %s has started.", version.c_str());
}

Md80Node::~Md80Node()
{
	for (auto candle : candleInstances)
		delete candle;
	ROS_INFO("candle_ros_node finished.");
}

mab::Candle* Md80Node::findCandleByMd80Id(uint16_t md80Id)
{
	for (auto candle : candleInstances)
	{
		for (auto id : candle->md80s)
		{
			if (id.getId() == md80Id) return candle;
		}
	}
	return NULL;
}

bool Md80Node::service_addMd80(candle_ros::AddMd80s::Request& request, candle_ros::AddMd80s::Response& response)
{
	for (auto& id : request.drive_ids)
	{
		unsigned int md80NotFound = 0;
		unsigned int md80Found = 0;

		for (auto candle : candleInstances)
		{
			if (candle->addMd80(id, false) == true)
			{
				response.drives_success.push_back(true);
				md80Found++;
			}
			else
				md80NotFound++;
		}
		/* if the id was found on multiple CANdle devices */
		if (md80Found > 1)
			ROS_WARN("Drive with ID %d seem to be duplicated", id);
		/* if the drive was not found on any of CANdle devices */
		else if (md80NotFound == candleInstances.size())
			response.drives_success.push_back(false);
	}

	int totalNumberOfDrives = 0;

	/* collect total number of drives from all CANdle devices */
	for (auto candle : candleInstances)
		totalNumberOfDrives += candle->md80s.size();

	response.total_number_of_drives = totalNumberOfDrives;

	return true;
}

bool Md80Node::service_zeroMd80(candle_ros::GenericMd80Msg::Request& request, candle_ros::GenericMd80Msg::Response& response)
{
	for (auto& id : request.drive_ids)
	{
		auto candle = findCandleByMd80Id(id);
		if (candle != NULL)
			response.drives_success.push_back(candle->controlMd80SetEncoderZero(id));
		else
		{
			response.drives_success.push_back(false);
			ROS_WARN("Drive with ID: %d is not added!", id);
		}
	}
	return true;
}

bool Md80Node::service_setModeMd80(candle_ros::SetModeMd80s::Request& request, candle_ros::SetModeMd80s::Response& response)
{
	if (request.drive_ids.size() != request.mode.size())
	{
		for (auto& id : request.drive_ids)
		{
			(void)id;
			response.drives_success.push_back(false);
		}
		ROS_WARN("SetMode request incomplete. Sizes of arrays do not match!");
		return false;
	}
	for (int i = 0; i < (int)request.drive_ids.size(); i++)
	{
		mab::Md80Mode_E mode;

		if (request.mode[i] == "IMPEDANCE")
			mode = mab::Md80Mode_E::IMPEDANCE;
		else if (request.mode[i] == "POSITION_PID")
			mode = mab::Md80Mode_E::POSITION_PID;
		else if (request.mode[i] == "VELOCITY_PID")
			mode = mab::Md80Mode_E::VELOCITY_PID;
		else if (request.mode[i] == "DEPRECATED")
			mode = mab::Md80Mode_E::DEPRECATED;
		else
		{
			ROS_WARN("MODE %s not recognized, setting IDLE for driveID = %d", request.mode[i].c_str(), request.drive_ids[i]);
			mode = mab::Md80Mode_E::IDLE;
		}
		auto candle = findCandleByMd80Id(request.drive_ids[i]);
		if (candle != NULL)
			response.drives_success.push_back(candle->controlMd80Mode(request.drive_ids[i], mode));
		else
		{
			response.drives_success.push_back(false);
			ROS_WARN("Drive with ID: %d is not added!", request.drive_ids[i]);
		}
	}

	return true;
}

bool Md80Node::service_enableMd80(candle_ros::GenericMd80Msg::Request& request, candle_ros::GenericMd80Msg::Response& response)
{
	std::vector<mab::Candle*> candlesToBegin;

	for (auto& id : request.drive_ids)
	{
		auto candle = findCandleByMd80Id(id);
		if (candle != NULL)
		{
			response.drives_success.push_back(candle->controlMd80Enable(id, true));
			/* this is to ensure only one copy of CANdle object is present for the begin procedure */
			if (std::find(candlesToBegin.begin(), candlesToBegin.end(), candle) == candlesToBegin.end())
				candlesToBegin.push_back(candle);
		}
		else
		{
			response.drives_success.push_back(false);
			ROS_WARN("Drive with ID: %d is not added!", id);
		}
	}

	/* begin the communication on each CANdle */
	for (auto candle : candlesToBegin)
	{
		candle->begin();
	}

	pubTimer.start();

	return true;
}

bool Md80Node::service_disableMd80(candle_ros::GenericMd80Msg::Request& request, candle_ros::GenericMd80Msg::Response& response)
{
	std::vector<mab::Candle*> candlesToEnd;

	/* just to find which CANdle devices are commanded to be disabled */
	for (auto& id : request.drive_ids)
	{
		auto candle = findCandleByMd80Id(id);
		if (candle != NULL)
		{
			if (std::find(candlesToEnd.begin(), candlesToEnd.end(), candle) == candlesToEnd.end())
				candlesToEnd.push_back(candle);
		}
	}

	/* ending CANdles */
	for (auto candle : candlesToEnd)
	{
		candle->end();
	}
	pubTimer.stop();

	/* Actually disabling individual MD80s */
	for (auto& id : request.drive_ids)
	{
		auto candle = findCandleByMd80Id(id);
		if (candle != NULL)
			response.drives_success.push_back(candle->controlMd80Enable(id, false));
		else
		{
			response.drives_success.push_back(false);
			ROS_WARN("Drive with ID: %d is not added!", id);
		}
	}

	return true;
}

void Md80Node::publishJointStates()
{
	sensor_msgs::JointState jointStateMsg;
	jointStateMsg.header.stamp = ros::Time::now();
	for (auto candle : candleInstances)
	{
		for (auto& md : candle->md80s)
		{
			jointStateMsg.name.push_back(std::string("Joint " + std::to_string(md.getId())));
			jointStateMsg.position.push_back(md.getPosition());
			jointStateMsg.velocity.push_back(md.getVelocity());
			jointStateMsg.effort.push_back(md.getTorque());
		}
	}

	this->jointStatePub.publish(jointStateMsg);
}

void Md80Node::motionCommandCallback(const candle_ros::MotionCommand::ConstPtr& msg)
{
	if (msg->drive_ids.size() != msg->target_position.size() || msg->drive_ids.size() != msg->target_velocity.size() ||
		msg->drive_ids.size() != msg->target_torque.size())
	{
		ROS_WARN("Motion Command message incomplete. Sizes of arrays do not match! Ignoring message.");
		return;
	}
	for (int i = 0; i < (int)msg->drive_ids.size(); i++)
	{
		try
		{
			auto candle = findCandleByMd80Id(msg->drive_ids[i]);
			if (candle != NULL)
			{
				auto& md = candle->getMd80FromList(msg->drive_ids[i]);
				md.setTargetPosition(msg->target_position[i]);
				md.setTargetVelocity(msg->target_velocity[i]);
				md.setTorque(msg->target_torque[i]);
			}
			else
				ROS_WARN("Drive with ID: %d is not added!", msg->drive_ids[i]);
		}
		catch (const char* eMsg)
		{
			ROS_WARN(eMsg);
		}
	}
}

void Md80Node::impedanceCommandCallback(const candle_ros::ImpedanceCommand::ConstPtr& msg)
{
	if (msg->drive_ids.size() != msg->kp.size() || msg->drive_ids.size() != msg->kd.size())
	{
		ROS_WARN("Impedance Command message incomplete. Sizes of arrays do not match! Ignoring message.");
		return;
	}
	for (int i = 0; i < (int)msg->drive_ids.size(); i++)
	{
		try
		{
			auto candle = findCandleByMd80Id(msg->drive_ids[i]);
			if (candle != NULL)
			{
				auto& md = candle->getMd80FromList(msg->drive_ids[i]);
				md.setImpedanceControllerParams(msg->kp[i], msg->kd[i]);
				md.setMaxTorque(msg->max_output[i]);
			}
			else
				ROS_WARN("Drive with ID: %d is not added!", msg->drive_ids[i]);
		}
		catch (const char* eMsg)
		{
			ROS_WARN(eMsg);
		}
	}
}

void Md80Node::velocityCommandCallback(const candle_ros::VelocityPidCommand::ConstPtr& msg)
{
	if (msg->drive_ids.size() != msg->velocity_pid.size())
	{
		ROS_WARN("Velocity Command message incomplete. Sizes of arrays do not match! Ignoring message.");
		return;
	}
	for (int i = 0; i < (int)msg->drive_ids.size(); i++)
	{
		try
		{
			auto candle = findCandleByMd80Id(msg->drive_ids[i]);
			if (candle != NULL)
			{
				auto& md = candle->getMd80FromList(msg->drive_ids[i]);
				md.setVelocityControllerParams(msg->velocity_pid[i].kp, msg->velocity_pid[i].ki, msg->velocity_pid[i].kd, msg->velocity_pid[i].i_windup);
				md.setMaxTorque(msg->velocity_pid[i].max_output);
			}
			else
				ROS_WARN("Drive with ID: %d is not added!", msg->drive_ids[i]);
		}
		catch (const char* eMsg)
		{
			ROS_WARN(eMsg);
		}
	}
}

void Md80Node::positionCommandCallback(const candle_ros::PositionPidCommand::ConstPtr& msg)
{
	if (msg->drive_ids.size() != msg->position_pid.size())
	{
		ROS_WARN("Position Command message incomplete. Sizes of arrays do not match! Ignoring message.");
		return;
	}
	for (int i = 0; i < (int)msg->drive_ids.size(); i++)
	{
		try
		{
			auto candle = findCandleByMd80Id(msg->drive_ids[i]);
			if (candle != NULL)
			{
				auto& md = candle->getMd80FromList(msg->drive_ids[i]);
				md.setPositionControllerParams(msg->position_pid[i].kp, msg->position_pid[i].ki, msg->position_pid[i].kd, msg->position_pid[i].i_windup);
				md.setMaxVelocity(msg->position_pid[i].max_output);
				if (i < (int)msg->velocity_pid.size())
				{
					md.setVelocityControllerParams(msg->velocity_pid[i].kp, msg->velocity_pid[i].ki, msg->velocity_pid[i].kd, msg->velocity_pid[i].i_windup);
					md.setMaxTorque(msg->velocity_pid[i].max_output);
				}
			}
			else
				ROS_WARN("Drive with ID: %d is not added!", msg->drive_ids[i]);
		}
		catch (const char* eMsg)
		{
			ROS_WARN(eMsg);
		}
	}
}

int main(int argc, char** argv)
{
	ros::init(argc, argv, "candle_ros_node");
	Md80Node n(argc, argv);
	ros::spin();

	return 0;
}
