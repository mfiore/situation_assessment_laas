#include <ros/ros.h>

#include <situation_assessment_msgs/DatabaseRequest.h>
#include <situation_assessment_msgs/QueryDatabase.h>
#include <toaster_msgs/GetFacts.h>
#include <toaster_msgs/RemoveFactToAgent.h>
#include <toaster_msgs/AddFactToAgent.h>

#include <string>
#include <vector>
using namespace std;

ros::ServiceClient add_fact_toaster, remove_fact_toaster, query_toaster;

std::vector<std::string> & stringSplitElems(const std::string &s, char delim, std::vector<std::string> &elems) {
    std::stringstream ss(s);
    std::string item;
    while (std::getline(ss, item, delim)) {
        elems.push_back(item);
    }
    return elems;
}

std::vector<std::string> stringSplit(const std::string &s, char delim) {
    std::vector<std::string> elems;
    stringSplitElems(s, delim, elems);
    return elems;
}


void getPredicateString(vector<string> predicate, string *property, string *target) {
	string s="";
	vector<string> property_predicate;
	if (predicate.size()>1){
		property_predicate.insert(property_predicate.begin(),predicate.begin(),predicate.end()-1);
		target=predicate[predicate.size()-1];
	}
	for (int i=0; i<property_predicate.size()-1;i++) {
		s=s+predicate[i]+"_";
	}
	if (property_predicate.size()>0) {
		s=s+predicate[i];
	}
	return s;
}
vector<string> getPredicateVector(string toaster_property, string toaster_target) {
	vector<string> temp=stringSplit(toaster_property);
	if (toaster_target!="") {
		temp.push_back(toaster_target);
	}
	return temp;
}

toaster_msgs::Fact convertToToasterFact(situation_assessment_msgs::Fact fact) {
	toaster_msgs::Fact toaster_fact;
	toaster_fact.subjectId=fact.subject;
	getPredicateString(fact.predicate,&toaster_fact.property,&toaster_fact.targetId);
	if (f.value.size()>0) {
		toaster_fact.stringValue=f.value[0];
	}
	if (f.value.size()>1) {
		ROS_WARN("TOASTER_DATABASE_BRIDGE toaster_database_bridge doesn't support multiple values. Ignoring values after the first");
	}
	return toaster_fact;
}

situation_assessment_msgs::Fact convertToSharyFact(toaster_msgs::Fact toaster_fact) {
	situation_assessment_msgs::Fact result;
	result.subject=toaster_fact.subjectId;
	result.predicate=getPredicateVector(toaster_fact.property,toaster_fact.target);
	if (toaster_fact.doubleValue!=0) {
		result.value.push_back(boost::lexical_cast<string>(toaster_fact.doubleValue));
	}
	else {
		result.value.push_back(toaster_fact.stringValue);
	}
	return result;
}


bool addFacts(situation_assessment_msgs::DatabaseRequest::Request &req,
        situation_assessment_msgs::DatabaseRequest::Response &res) {
	for (int i=0; i<req.fact_list.size();i++) {
		situation_assessment_msgs::Fact f=req.fact_list[i];
		toaster_msgs::AddFactToAgent srv;
		srv.req.agentId=fact.model;
		srv.req.fact=convertToToasterFact(f);
		if (add_fact_toaster.call(srv)) {
			if (!srv.res.answer) {
				res.report=false;
				break;
			}
		}
		else {
			ROS_ERROR("TOASTER_DATABASE_BRIDGE can't contact the toaster database");
		}
	}
	return true;
}


bool removeFacts(situation_assessment_msgs::DatabaseRequest::Request &req,
        situation_assessment_msgs::DatabaseRequest::Response &res) { 
	for (int i=0; i<req.fact_list.size();i++) {
		situation_assessment_msgs::Fact f=req.fact_list[i];
		toaster_msgs::AddFactToAgent srv;
		srv.req.agentId=fact.model;
		srv.req.fact=convertToToasterFact(f);
		if (remove_fact_toaster.call(srv)) {
			if (!srv.res.answer) {
				res.report=false;
				break;
			}
		}
		else {
			ROS_ERROR("TOASTER_DATABASE_BRIDGE can't contact the toaster database");
		}
	}
	return true;
}


bool query(situation_assessment_msgs::QueryDatabase::Request &req,
        situation_assessment_msgs::QueryDatabase::Response &res) {
	toaster_msgs::GetFacts srv;
	srv.req.agentId=req.query.model;
	srv.req.reqFact=convertToToasterFact(req.query);

	if (query_toaster.call(srv)) {
		vector<situation_assessment_msgs::Fact> result;
		vector<toaster_msgs::Fact> toaster_facts=srv.res.resFactList.fact_list;
		for (int i=0; i<toaster_facts.size();i++) {
			situation_assessment_msgs::Fact f=convertToSharyFact(toaster_facts[i]);
			result.push_back(f);
		}
		res.result=result;
	}
	else {
		ROS_ERROR("TOASTER_DATABASE_BRIDGE can't contact the toaster database");
	}
	return true;
}


int main(int argc,char** argv) {
	ros::init(argc,argv,"toaster_database_bridge");
	ros::NodeHandle node_handle;

	add_fact_toaster=node_handle.serviceClient<toaster_msgs/AddFactToAgent>("database/add_faact_to_agent");
	remove_fact_toaster=node_handle.serviceClient<toaster_msgs/RemoveFactToAgent>("database/add_faact_to_agent");
	query_toaster=node_handle.serviceClient<toaster_msgs/GetFacts>("database/add_faact_to_agent");

	ros::ServiceServer service_add = node_handle.advertiseService("situation_assessment/add_facts", addFacts);
	ros::ServiceServer service_remove = node_handle.advertiseService("situation_assessment/remove_facts", removeFacts);
	ros::ServiceServer service_query = node_handle.advertiseService("situation_assessment/query_database", query);

	ros::spin();
	return 0;
	
}