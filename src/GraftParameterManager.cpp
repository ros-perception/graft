/*
 * Copyright (c) 2013, Willow Garage, Inc.
 * All rights reserved.
 *
 * Redistribution and use in source and binary forms, with or without
 * modification, are permitted provided that the following conditions are met:
 *
 *     * Redistributions of source code must retain the above copyright
 *       notice, this list of conditions and the following disclaimer.
 *     * Redistributions in binary form must reproduce the above copyright
 *       notice, this list of conditions and the following disclaimer in the
 *       documentation and/or other materials provided with the distribution.
 *     * Neither the name of the Willow Garage, Inc. nor the names of its
 *       contributors may be used to endorse or promote products derived from
 *       this software without specific prior written permission.
 *
 * THIS SOFTWARE IS PROVIDED BY THE COPYRIGHT HOLDERS AND CONTRIBUTORS "AS IS"
 * AND ANY EXPRESS OR IMPLIED WARRANTIES, INCLUDING, BUT NOT LIMITED TO, THE
 * IMPLIED WARRANTIES OF MERCHANTABILITY AND FITNESS FOR A PARTICULAR PURPOSE
 * ARE DISCLAIMED. IN NO EVENT SHALL THE COPYRIGHT OWNER OR CONTRIBUTORS BE
 * LIABLE FOR ANY DIRECT, INDIRECT, INCIDENTAL, SPECIAL, EXEMPLARY, OR
 * CONSEQUENTIAL DAMAGES (INCLUDING, BUT NOT LIMITED TO, PROCUREMENT OF
 * SUBSTITUTE GOODS OR SERVICES; LOSS OF USE, DATA, OR PROFITS; OR BUSINESS
 * INTERRUPTION) HOWEVER CAUSED AND ON ANY THEORY OF LIABILITY, WHETHER IN
 * CONTRACT, STRICT LIABILITY, OR TORT (INCLUDING NEGLIGENCE OR OTHERWISE)
 * ARISING IN ANY WAY OUT OF THE USE OF THIS SOFTWARE, EVEN IF ADVISED OF THE
 * POSSIBILITY OF SUCH DAMAGE.
 */

/* 
 * Author: Chad Rockey
 */

 #include <graft/GraftParameterManager.h>




GraftParameterManager::GraftParameterManager(ros::NodeHandle n, ros::NodeHandle pnh): n_(n), pnh_(pnh),
                                                                                      include_pose_(false){

}

GraftParameterManager::~GraftParameterManager(){

}

void GraftParameterManager::parseNavMsgsOdometryParameters(ros::NodeHandle& tnh, boost::shared_ptr<GraftOdometryTopic>& odom){
	// Check how to use this sensor
	bool absolute_pose, delta_pose, use_velocities;
	double timeout;
	tnh.param<bool>("absolute_pose", absolute_pose, false);
	tnh.param<bool>("delta_pose", delta_pose, false);
	tnh.param<bool>("use_velocities", use_velocities, false);
	tnh.param<double>("timeout", timeout, 1.0);

	// Check for incompatible usage
	if(absolute_pose == true){
		delta_pose = false; // Should not use for both absolute position and velocity
	}
	if(delta_pose == true){
		use_velocities = false; // Should not use both estimated and reported velocities
	}

	// Apply to global state
	include_pose_ = include_pose_ || absolute_pose;

	// Apply to sensor
	//odom->useAbsolutePose(absolute_pose);
	odom->useDeltaPose(delta_pose);
	//odom->useVelocities(use_velocities);
  odom->setTimeout(timeout);

  //ROS_INFO("Abs pose: %d\nDelta pose: %d\nUse Vel: %d\nTimeout: %.3f", absolute_pose, delta_pose, use_velocities, timeout);

  // Set covariances
  XmlRpc::XmlRpcValue xml_pose_covariance;
  if (tnh.getParam("override_pose_covariance", xml_pose_covariance)){
  	if(xml_pose_covariance.size() == 36){
  		boost::array<double, 36> pose_covariance;
	    for(size_t i = 0; i < xml_pose_covariance.size(); i++){
	      std::stringstream ss; // Convert the list element into doubles
	      ss << xml_pose_covariance[i];
	      ss >> pose_covariance[i] ? pose_covariance[i] : 0;
	    }
	    odom->setPoseCovariance(pose_covariance);
    } else {
    	ROS_WARN("%s/override_pose_covariance parameter requires 36 elements, skipping.", tnh.getNamespace().c_str());
    }
  }

  XmlRpc::XmlRpcValue xml_twist_covariance;
  if (tnh.getParam("override_twist_covariance", xml_twist_covariance)){
  	if(xml_twist_covariance.size() == 36){
  		boost::array<double, 36> twist_covariance;
	    for(size_t i = 0; i < xml_twist_covariance.size(); i++){
	      std::stringstream ss; // Convert the list element into doubles
	      ss << xml_twist_covariance[i];
	      ss >> twist_covariance[i] ? twist_covariance[i] : 0;
	    }
	    odom->setTwistCovariance(twist_covariance);
    } else {
    	ROS_WARN("%s/override_twist_covariance parameter requires 36 elements, skipping.", tnh.getNamespace().c_str());
    }
  }
}

void GraftParameterManager::parseSensorMsgsIMUParameters(ros::NodeHandle& tnh, boost::shared_ptr<GraftImuTopic>& imu){
	// Check how to use this sensor
	bool absolute_orientation, delta_orientation, use_velocities, use_accelerations;
	double timeout;
	tnh.param<bool>("absolute_orientation", absolute_orientation, false);
	tnh.param<bool>("delta_orientation", delta_orientation, false);
	tnh.param<bool>("use_velocities", use_velocities, false);
	tnh.param<bool>("use_accelerations", use_accelerations, false);
	tnh.param<double>("timeout", timeout, 1.0);

	// Check for incompatible usage
	if(absolute_orientation == true){
		delta_orientation = false; // Should not use for both absolute position and velocity
	}
	if(delta_orientation == true){
		use_velocities = false; // Should not use both estimated and reported velocities
	}

	// Apply to global state
	include_pose_ = include_pose_ || absolute_orientation;

	// Apply to sensor
	//imu->useAbsoluteOrientation(absolute_orientation);
	imu->useDeltaOrientation(delta_orientation);
	//imu->useVelocities(use_velocities);
	//imu->useAccelerations(use_accelerations);
  imu->setTimeout(timeout);

  ROS_INFO("Abs orientation: %d\nDelta orientation: %d\nUse Vel: %d\nTimeout: %.3f", absolute_orientation, delta_orientation, use_velocities, timeout);

  // Set covariances
  XmlRpc::XmlRpcValue xml_orientation_covariance;
  if (tnh.getParam("override_orientation_covariance", xml_orientation_covariance)){
  	if(xml_orientation_covariance.size() == 9){
  		boost::array<double, 9> orientation_covariance;
	    for(size_t i = 0; i < xml_orientation_covariance.size(); i++){
	      std::stringstream ss; // Convert the list element into doubles
	      ss << xml_orientation_covariance[i];
	      ss >> orientation_covariance[i] ? orientation_covariance[i] : 0;
	    }
	    imu->setOrientationCovariance(orientation_covariance);
    } else {
    	ROS_WARN("%s/override_orientation_covariance parameter requires 9 elements, skipping.", tnh.getNamespace().c_str());
    }
  }

  XmlRpc::XmlRpcValue xml_angular_velocity_covariance;
  if (tnh.getParam("override_angular_velocity_covariance", xml_angular_velocity_covariance)){
  	if(xml_angular_velocity_covariance.size() == 9){
  		boost::array<double, 9> angular_velocity_covariance;
	    for(size_t i = 0; i < xml_angular_velocity_covariance.size(); i++){
	      std::stringstream ss; // Convert the list element into doubles
	      ss << xml_angular_velocity_covariance[i];
	      ss >> angular_velocity_covariance[i] ? angular_velocity_covariance[i] : 0;
	    }
	    imu->setAngularVelocityCovariance(angular_velocity_covariance);
    } else {
    	ROS_WARN("%s/override_angular_velocity_covariance parameter requires 9 elements, skipping.", tnh.getNamespace().c_str());
    }
  }

  XmlRpc::XmlRpcValue xml_linear_acceleration_covariance;
  if (tnh.getParam("override_linear_acceleration_covariance", xml_linear_acceleration_covariance)){
  	if(xml_linear_acceleration_covariance.size() == 9){
  		boost::array<double, 9> linear_acceleration_covariance;
	    for(size_t i = 0; i < xml_linear_acceleration_covariance.size(); i++){
	      std::stringstream ss; // Convert the list element into doubles
	      ss << xml_linear_acceleration_covariance[i];
	      ss >> linear_acceleration_covariance[i] ? linear_acceleration_covariance[i] : 0;
	    }
	    imu->setLinearAccelerationCovariance(linear_acceleration_covariance);
    } else {
    	ROS_WARN("%s/override_linear_acceleration_covariance parameter requires 9 elements, skipping.", tnh.getNamespace().c_str());
    }
  }
}

void GraftParameterManager::loadParameters(std::vector<boost::shared_ptr<GraftSensor> >& topics, std::vector<ros::Subscriber>& subs){
	// Filter behavior parameters
	pnh_.param<std::string>("filter_type", filter_type_, "EKF");
	pnh_.param<bool>("planar_output", planar_output_, true);

	pnh_.param<std::string>("parent_frame_id", parent_frame_id_, "odom");
	pnh_.param<std::string>("child_frame_id", child_frame_id_, "base_link");

	pnh_.param<double>("freq", update_rate_, 50.0);
	pnh_.param<double>("update_rate", update_rate_, update_rate_); // Overrides 'freq'
	pnh_.param<double>("dt_override", dt_override_, 0.0);

	pnh_.param<int>("queue_size", queue_size_, 1);

	// Process noise covariance
	XmlRpc::XmlRpcValue xml_Q;
  if (pnh_.getParam("Q", xml_Q)){
  	if(xml_Q.size() == 4){ // Only supporting velocity for now
	    for(size_t i = 0; i < xml_Q.size(); i++){
	      std::stringstream ss; // Convert the list element into doubles
	      ss << xml_Q[i];
	      ss >> Q_[i] ? Q_[i] : 0;
	    }
    } else {
    	ROS_WARN("%s/Q (process noise covariance) parameter requires 4 elements, skipping.", pnh_.getNamespace().c_str());
    }
  }

	// Read each topic config
	try{
	  XmlRpc::XmlRpcValue topic_list;
	  if(!pnh_.getParam("topics", topic_list)){
      ROS_FATAL("XmlRpc Error parsing parameters.  Make sure parameters were loaded into this node's namespace.");
    	ros::shutdown();
    }

    ROS_INFO("Number of topics: %i", topic_list.size());

    // Iterate over the map of each joint and its parameters
    std::map<std::string, XmlRpc::XmlRpcValue>::iterator i;
    for (i = topic_list.begin(); i != topic_list.end(); i++)
    {
    	// Get the name of this topic.  ex: base_odometry
      std::string topic_name = i->first;

      // Set up a nodehandle in this namespace (so we don't have to deal with the XmlRpc object)
      ros::NodeHandle tnh("~/topics/" + topic_name);

      ROS_INFO("Topic name: %s", topic_name.c_str());

      // Parse parameters according to topic type
      std::string type;
      if(!tnh.getParam("type", type)){
      	ROS_ERROR("Could not get topic type for %s, skipping.", topic_name.c_str());
      	continue;
      }

      ROS_INFO("Type: %s", type.c_str());

      if(type == "nav_msgs/Odometry"){
      	std::string full_topic;
      	if(!tnh.getParam("topic", full_topic)){
      		ROS_ERROR("Could not get full topic for %s, skipping.", topic_name.c_str());
      		continue;
      	}

      	// Create Odometry object
      	boost::shared_ptr<GraftOdometryTopic> odom(new GraftOdometryTopic(topic_name));
        odom->setName(topic_name);
      	topics.push_back(odom);	

      	// Subscribe to topic
      	ros::Subscriber sub = n_.subscribe(full_topic, queue_size_, &GraftOdometryTopic::callback, odom);
      	subs.push_back(sub);

      	// Parse rest of parameters
      	parseNavMsgsOdometryParameters(tnh, odom);
      } else if(type == "sensor_msgs/Imu"){
      	std::string full_topic;
      	if(!tnh.getParam("topic", full_topic)){
      		ROS_ERROR("Could not get full topic for %s, skipping.", topic_name.c_str());
      		continue;
      	}

      	// Create Odometry object
      	boost::shared_ptr<GraftImuTopic> imu(new GraftImuTopic(topic_name));
        imu->setName(topic_name);
      	topics.push_back(imu);	

      	// Subscribe to topic
      	ros::Subscriber sub = n_.subscribe(full_topic, queue_size_, &GraftImuTopic::callback, imu);
      	subs.push_back(sub);

      	// Parse rest of parameters
      	parseSensorMsgsIMUParameters(tnh, imu);
      } else {
      	ROS_WARN("Unknown type: %s  Not parsing configuration.", type.c_str());
      }
    }

	} catch(...){
    ROS_FATAL("XmlRpc Error parsing parameters.  Cannot start.");
    ros::shutdown();
  }

}


std::string GraftParameterManager::getFilterType(){
	return filter_type_;
}

bool GraftParameterManager::getPlanarOutput(){
	return planar_output_;
}

std::string GraftParameterManager::getParentFrameID(){
	return parent_frame_id_;
}

std::string GraftParameterManager::getChildFrameID(){
	return child_frame_id_;
}

double GraftParameterManager::getUpdateRate(){
	return update_rate_;
}

std::string GraftParameterManager::getUpdateTopic(){
	return update_topic_;
}

double GraftParameterManager::getdtOveride(){
	return dt_override_;
}

int GraftParameterManager::getQueueSize(){
	return queue_size_;
}

bool GraftParameterManager::getIncludePose(){
	return include_pose_;
}

boost::array<double, 4> GraftParameterManager::getProcessNoise(){
	return Q_;
}