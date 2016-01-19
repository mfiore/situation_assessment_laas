#include <ros/ros.h>

#include <tf/LinearMath/Matrix3x3.h>
#include <tf/transform_datatypes.h>

#include <toaster_msgs/PutInHand.h>
#include <toaster_msgs/RemoveFromHand.h>
#include <toaster_msgs/SetEntityPose.h>

#include <toaster_bridge/agent_position_manager.h>

#include <situation_assessment_msgs/PlaceObject.h>
#include <situation_assessment_msgs/PutObjectInHand.h>


using namespace std;

ros::ServiceClient toaster_put_in_hand_client_;
ros::ServiceClient toaster_remove_from_hand_client_;
ros::ServiceClient toaster_set_entity_pose_client_;

AgentPositionManager* agent_position_manager_;


bool simulation_;



bool putObjectInHand(situation_assessment_msgs::PutObjectInHand::Request &req,
					 situation_assessment_msgs::PutObjectInHand::Response &res ) 
{
	if (!simulation_) {
		if (req.has_object==true) {
			toaster_msgs::PutInHand toaster_srv;
			toaster_srv.request.objectId=req.object;
			toaster_srv.request.agentId=req.agent;
			toaster_srv.request.jointName="right_arm";
			if (toaster_put_in_hand_client_.call(toaster_srv)) {
				res.result=true;
			} 
			else {
				ROS_ERROR("TOASTER_BRIDGE Could not contact toaster");
				return false;
			}
		}
		else {
			toaster_msgs::RemoveFromHand toaster_srv;
			toaster_srv.request.objectId=req.object;
			if (toaster_put_in_hand_client_.call(toaster_srv)) {
				res.result=true;
			} 
			else {
				ROS_ERROR("TOASTER_BRIDGE Could not contact toaster");
				return false;
			}
		}
	}
	else {
		if (req.has_object==true) {
			toaster_msgs::SetEntityPose toaster_srv;
			toaster_srv.request.id=req.object;
			toaster_srv.request.x=0;
			toaster_srv.request.y=0;
			toaster_srv.request.z=0;
			if (toaster_set_entity_pose_client_.call(toaster_srv)) {
				return true;
			}
			else {
				ROS_ERROR("TOASTER_BRIDGE Could not contact toaster");
			}
		}
	}
	return true;
}
bool placeObject(situation_assessment_msgs::PlaceObject::Request &req,
					 situation_assessment_msgs::PlaceObject::Response &res ) 
{
	toaster_msgs::SetEntityPose toaster_srv;
	toaster_srv.request.id=req.name;
	toaster_srv.request.x=req.pose.position.x;
	toaster_srv.request.y=req.pose.position.y;
	toaster_srv.request.z=req.pose.position.z;

	tf::Quaternion quat;
	tf::quaternionMsgToTF(req.pose.orientation,quat);
	tf::Matrix3x3 m(quat);
	double roll, pitch, yaw;
	m.getRPY(roll, pitch, yaw);

	toaster_srv.request.roll=roll;
	toaster_srv.request.pitch=pitch;
	toaster_srv.request.yaw=yaw;

	if (toaster_set_entity_pose_client_.call(toaster_srv)) {
		res.result=toaster_srv.response.answer;
		return true;
	} 
	else {
		ROS_ERROR("TOASTER_BRIDGE Could not contact toaster");
		return false;
	}
}


int main(int argc, char** argv) {
	ros::init(argc,argv,"toaster_bridge");

	ros::NodeHandle node_handle;
	simulation_=false;
	node_handle.getParam("/situation_assessment/simulation",simulation_);

	ROS_INFO("TOASTER_BRIDGE Started toaster_bridge");
	ROS_INFO("TOASTER_BRIDGE Parameters:");
	ROS_INFO("TOASTER_BRIDGE simulation - %d",simulation_);


	agent_position_manager_=new AgentPositionManager(node_handle);

	string module_name;
	if (simulation_) {
		module_name="/toaster_simu/";
	}
	else {
		module_name="/pdg/";
	}
	toaster_put_in_hand_client_=node_handle.serviceClient<toaster_msgs::PutInHand>(module_name+"put_in_hand");
	toaster_set_entity_pose_client_=node_handle.serviceClient<toaster_msgs::SetEntityPose>(module_name+"set_entity_pose");
	toaster_remove_from_hand_client_=node_handle.serviceClient<toaster_msgs::RemoveFromHand>(module_name+"remove_from_hand");

	ROS_INFO("TOASTER_BRIDGE waiting for toaster services");
	toaster_set_entity_pose_client_.waitForExistence();
	if (!simulation_) {
		toaster_put_in_hand_client_.waitForExistence();
		toaster_remove_from_hand_client_.waitForExistence();
	}
	ROS_INFO("TOASTER_BRIDGE connected to toaster services");



	ros::ServiceServer put_object_in_hand_server=node_handle.advertiseService("situation_assessment/put_object_in_hand",putObjectInHand);
	ros::ServiceServer place_object_server=node_handle.advertiseService("situation_assessment/place_object",placeObject);
	ROS_INFO("TOASTER_BRIDGE advertising services");
	ROS_INFO("TOASTER_BRIDGE Ready");

	ros::spin();
	delete agent_position_manager_;
	return 0;
}