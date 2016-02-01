#include <toaster_bridge/agent_position_manager.h>

AgentPositionManager::AgentPositionManager(ros::NodeHandle node_handle):node_handle_(node_handle) {
	human_sub_=node_handle_.subscribe("/pdg/humanList",1000,&AgentPositionManager::humanCallback,this);
	robot_sub_=node_handle_.subscribe("/pdg/robotList",1000,&AgentPositionManager::robotCallback,this);

	ros::Rate r(3);
	ROS_INFO("TOASTER_BRIDGE waiting for human and robot position publisher");
	while (human_sub_.getNumPublishers()==0 || robot_sub_.getNumPublishers()==0) {
		r.sleep();
	}
	ROS_INFO("TOASTER_BRIDGE connected");	
}

void AgentPositionManager::humanCallback(const toaster_msgs::HumanList& msg) {
	boost::lock_guard<boost::mutex> lock(mutex_human_);
	for (int i=0; i<msg.humanList.size();i++) {
		toaster_msgs::Agent entity=msg.humanList[i].meAgent;
		agent_position_map_[entity.meEntity.name]=entity;
	}
}
void AgentPositionManager::robotCallback(const toaster_msgs::RobotList& msg){
	boost::lock_guard<boost::mutex> lock(mutex_robot_);
	for (int i=0;i<msg.robotList.size();i++) {
		toaster_msgs::Agent entity=msg.robotList[i].meAgent;
		agent_position_map_[entity.meEntity.name]=entity;
	}
}

toaster_msgs::Agent AgentPositionManager::getAgentPosition(string name) {
	boost::lock_guard<boost::mutex> lock(mutex_robot_);
	boost::lock_guard<boost::mutex> lock2(mutex_human_);
	if(agent_position_map_.find(name)!=agent_position_map_.end())
		return agent_position_map_[name];
	else {
		toaster_msgs::Agent null_agent;
		null_agent.meEntity.name="null_entity";
		return null_agent;
	}
}