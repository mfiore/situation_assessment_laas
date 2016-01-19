/*This file acts as a bridge for the toaster database
Facts are represented in shary as subject predicate[] value[] 
        		  while in toaster as subject predicate target_id stringValue and other stuff we don't care
so we need to convert them 
the problem is that sometimes the target in toaster is represented as a value in shary, and sometimes it's a part of
the predicate. We take this into account saving this information in a yaml file 
*/

#include <ros/ros.h>

#include <situation_assessment_msgs/DatabaseRequest.h>
#include <situation_assessment_msgs/QueryDatabase.h>
#include <toaster_msgs/ExecuteSQL.h>
#include <toaster_msgs/RemoveFactToAgent.h>
#include <toaster_msgs/AddFactToAgent.h>

#include <string>
#include <vector>
using namespace std;

//toaster services
ros::ServiceClient add_fact_toaster, remove_fact_toaster, query_toaster;

map<string,string> target_use_map; //links a predicate (at least the first part of a predicate) to a target use
string AS_PREDICATE="predicate"; //values of target_use
string AS_VALUE="value";

string robot_name_;
//utility functions to split a string
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


//Split a predicate vector into a string. Saves also the target if map_target_use[predicate]=predicate
void getPredicateString(vector<string> predicate, string *property, string *target) {
	string s="";
	vector<string> property_predicate;

	if (predicate.size()>1 && target_use_map[predicate[0]]==AS_PREDICATE){
		property_predicate.insert(property_predicate.begin(),predicate.begin(),predicate.end()-1);
		*target=predicate[predicate.size()-1];
	}
	else {
		property_predicate=predicate;
	}
	for (int i=0; i<(property_predicate.size()-1);i++) {
		s=s+property_predicate[i]+"_";
	}
	if (property_predicate.size()>0) {
		s=s+property_predicate[property_predicate.size()-1];
	}
	*property=s;
}
//splits a predicate string into a vector. takes into account the use of target
vector<string> getPredicateVector(string toaster_property, string toaster_target) {
	vector<string> temp=stringSplit(toaster_property,'_');

	if (temp.size()>0 && toaster_target!="" && target_use_map[temp[0]]==AS_PREDICATE) {
		temp.push_back(toaster_target);
	}
	return temp;
}

toaster_msgs::Fact convertToToasterFact(situation_assessment_msgs::Fact fact) {
	toaster_msgs::Fact toaster_fact;
	toaster_fact.subjectId=fact.subject;
	ROS_INFO("TOASTER_DATABASE_BRIDGE in convertToToasterFact");
	getPredicateString(fact.predicate,&toaster_fact.property,&toaster_fact.targetId);
	ROS_INFO("TOASTER_DATABASE_BRIDGE got predicate string");
	if (fact.predicate.size()>0 && fact.value.size()>0) {
		if (target_use_map[fact.predicate[0]]==AS_VALUE){
			toaster_fact.targetId=fact.value[0];
		}
		else  {
			toaster_fact.stringValue=fact.value[0];
		}
	}
	if (fact.value.size()>1) {
		ROS_WARN("TOASTER_DATABASE_BRIDGE toaster_database_bridge doesn't support multiple values. Ignoring values after the first");
	}
	return toaster_fact;
}

situation_assessment_msgs::Fact convertToSharyFact(toaster_msgs::Fact toaster_fact) {
	situation_assessment_msgs::Fact result;
	result.subject=toaster_fact.subjectId;
	result.predicate=getPredicateVector(toaster_fact.property,toaster_fact.targetId);
	if (target_use_map[result.predicate[0]]==AS_VALUE && toaster_fact.targetId!="") {
		result.value.push_back(toaster_fact.targetId);
	}
	if (toaster_fact.doubleValue!=0) {
		result.value.push_back(boost::lexical_cast<string>(toaster_fact.doubleValue));
	}
	else if (toaster_fact.stringValue!=""){
		result.value.push_back(toaster_fact.stringValue);
	}
	
	return result;
}

string getQueryString(situation_assessment_msgs::Fact fact) {
	string subject_sql, predicate_sql, target_sql, stringValue_sql;
	string predicate_string;
	string target_string;
	toaster_msgs::Fact toaster_fact=convertToToasterFact(fact);

	int n=0;
	if (toaster_fact.subjectId!=""){
		subject_sql=toaster_fact.subjectId;
	 	n++;	
	}
	if (toaster_fact.property!="") {
		predicate_sql=toaster_fact.property; 
		n++;
	}
	if (toaster_fact.targetId!="") {
		target_sql=toaster_fact.targetId; 
		n++;
	}
	if (toaster_fact.stringValue!="") {
		stringValue_sql=toaster_fact.stringValue; 
		n++;
	}
	string query_string;
	if (subject_sql!="" || predicate_sql!="" || target_sql!="" || stringValue_sql!="") {
		query_string=" WHERE ";
	}
	if (subject_sql!="") {
		query_string=query_string+"subject_id='"+subject_sql+"' ";
		if (n>1) query_string=query_string+" and ";
		n--;
	}
	if (predicate_sql!="") {
		query_string=query_string+"predicate='"+predicate_sql+"' ";
		if (n>1) query_string=query_string+" and ";
		n--;
	}
	if (target_sql!="") {
		query_string=query_string+"target_id='"+target_sql+"' ";
		if (n>1) query_string=query_string+" and ";
		n--;
	}
	if (stringValue_sql!="") {
		query_string=query_string+"valueString='"+stringValue_sql+"' ";
	}


	return query_string;
}


bool addFacts(situation_assessment_msgs::DatabaseRequest::Request &req,
        situation_assessment_msgs::DatabaseRequest::Response &res) {
	for (int i=0; i<req.fact_list.size();i++) {
		situation_assessment_msgs::Fact f=req.fact_list[i];
		toaster_msgs::AddFactToAgent srv;
		srv.request.agentId=f.model;
		srv.request.fact=convertToToasterFact(f);
		if (add_fact_toaster.call(srv)) {
			if (!srv.response.answer) {
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
		srv.request.agentId=f.model;
		srv.request.fact=convertToToasterFact(f);
		if (remove_fact_toaster.call(srv)) {
			if (!srv.response.answer) {
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
	toaster_msgs::ExecuteSQL srv;
		ROS_INFO("TOASTER_DATABASE_BRIDGE got query");

//	srv.request.agentId=req.query.model;
//	srv.request.reqFact=convertToToasterFact(req.query);
	string order="SELECT subject_id, predicate, target_id, valueString from fact_table_"+robot_name_;
	order=order+getQueryString(req.query);
	srv.request.order=order;
	ROS_INFO("Query is %s",order.c_str());

	if (query_toaster.call(srv)) {
		ROS_INFO("response size is %ld",srv.response.results.size());
		for (int i=0; i<srv.response.results.size();i++) {
			ROS_INFO("%s",srv.response.results[i].c_str());
		}

		int i=0;
		vector<situation_assessment_msgs::Fact> result;		
		while (i<srv.response.results.size()) {
			toaster_msgs::Fact toaster_fact;
			toaster_fact.subjectId=srv.response.results[i];
			toaster_fact.property=srv.response.results[i+1];
			toaster_fact.targetId=srv.response.results[i+2];
			toaster_fact.stringValue=srv.response.results[i+3];
			situation_assessment_msgs::Fact shary_fact=convertToSharyFact(toaster_fact);
			shary_fact.model=robot_name_;
			result.push_back(shary_fact);
			i=i+4;
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

	vector<string> predicate_list;
	node_handle.getParam("/situation_assessment/robot_name",robot_name_);
	ROS_INFO("TOASTER_DATABASE_BRIDGE robot name is %s",robot_name_.c_str());
	node_handle.getParam("/toaster_bridge/predicate_list",predicate_list);
	ROS_INFO("TOASTER_DATABASE_BRIDGE got %ld predicates",predicate_list.size());
	for (int i=0; i<predicate_list.size();i++) {
		string this_target_use;
		string param_name="/toaster_bridge/target_use/"+predicate_list[i];
		node_handle.getParam(param_name,this_target_use);
		target_use_map[predicate_list[i]]=this_target_use;
		ROS_INFO("TOASTER_DATABASE_BRIDGE predicate %s is of type %s",predicate_list[i].c_str(),this_target_use.c_str());
	}

	add_fact_toaster=node_handle.serviceClient<toaster_msgs::AddFactToAgent>("database/add_fact_to_agent");
	remove_fact_toaster=node_handle.serviceClient<toaster_msgs::RemoveFactToAgent>("database/remove_fact_to_agent");
	query_toaster=node_handle.serviceClient<toaster_msgs::ExecuteSQL>("database/execute_SQL");

	ROS_INFO("TOASTER_DATABASE_BRIDGE waiting for toaster services");
	add_fact_toaster.waitForExistence();
	remove_fact_toaster.waitForExistence();
	query_toaster.waitForExistence();
	ROS_INFO("TOASTER_DATABASE_BRIDGE connected to toaster services");

	ros::ServiceServer service_add = node_handle.advertiseService("situation_assessment/add_facts", addFacts);
	ros::ServiceServer service_remove = node_handle.advertiseService("situation_assessment/remove_facts", removeFacts);
	ros::ServiceServer service_query = node_handle.advertiseService("situation_assessment/query_database", query);

	ros::spin();
	return 0;
	
}