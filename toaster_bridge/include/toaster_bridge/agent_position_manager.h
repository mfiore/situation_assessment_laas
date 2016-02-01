#ifndef AGENT_POSITION_MANAGER_H
#define AGENT_POSITION_MANAGER_H

#include <ros/ros.h>
#include <toaster_msgs/RobotList.h>
#include <toaster_msgs/HumanList.h>
#include <toaster_msgs/Agent.h>

#include <boost/thread/mutex.hpp>
#include <boost/thread/locks.hpp> 

#include <string>
#include <vector>
#include <iostream>
#include <map>

using namespace std;

class AgentPositionManager {
public:
	AgentPositionManager(ros::NodeHandle node_handle_);
	toaster_msgs::Agent getAgentPosition(string name);

	private:
		void humanCallback(const toaster_msgs::HumanList& msg);
		void robotCallback(const toaster_msgs::RobotList& msg);

		map<string,toaster_msgs::Agent> agent_position_map_;
		ros::NodeHandle node_handle_;

		ros::Subscriber human_sub_,robot_sub_;

		boost::mutex mutex_human_;
		boost::mutex mutex_robot_;
};


#endif