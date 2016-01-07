#include <ros/ros.h>

#include <toaster_msgs/PutInHand.h>
#include <toaster_msgs/RemoveFromHand.h>
#include <toaster_msgs/SetEntityPose.h>

#include <situation_assessment_msgs/PlaceObject.h>
#include <situation_assessment_msgs/PutObjectInHand.h>


using namespace std;

ros::ServiceClient toaster_put_in_hand_client_;
ros::ServiceClient toaster_remove_from_hand_client_;
ros::ServiceClient toaster_set_entity_pose_client_;

bool putObjectInHand(situation_assessment_msgs::PutObjectInHand::Request &req,
					 situation_assessment_msgs::PutObjectInHand::Response &res ) 
{
	if (req.has_object==true) {
		toaster_msgs::PutInHand toaster_srv;
		toaster_srv.req.obect_id=req.object;
		toaster_srv.req.agent_id=agent;
		toaster_srv.req.jointName="right_arm";
		if (toaster_put_in_hand_client_.call(toaster_srv)) {
			res.result=true;
		} 
		else {
			ROS_ERROR("Could not contact toaster");
			return false;
		}
	}
	else {
		toaster_msgs::RemoveFromHand toaster_srv;
		toaster_srv.req.object_id=req.object;
		if (toaster_put_in_hand_client_.call(toaster_srv)) {
			res.result=true;
		} 
		else {
			ROS_ERROR("Could not contact toaster");
			return false;
		}
	}
	return true;
}
bool placeObject(situation_assessment_msgs::PlaceObject::Request &req,
					 situation_assessment_msgs::PlaceObject::Response &res ) 
{

	return true;
}

int main(int argc, char** argv) {
	ros::init(argc,argv,"toaster_bridge");

	ros::NodeHandle node_handle;

	toaster_put_in_hand_client_=node_handle.serviceClient<toaster_msgs::PutInHand>("/pdg/put_in_hand");
	toaster_set_entity_pose_client_=node_handle.serviceClient<toaster_msgs::SetEntityPose>("/pdg/set_entity_pose");
	toaster_remove_from_hand_client_=node_handle.serviceClient<toaster_msgs::RemoveFromHand>("/pdg/remove_from_hand");

	ros::ServiceServer put_object_in_hand_server=node_handle.advertiseService("situation_assessment/put_object_in_hand",putObjectInHand);
	ros::ServiceServer place_object_server=node_handle.advertiseService("situation_assessment/place_object",placeObject);

	ros::spin();
}