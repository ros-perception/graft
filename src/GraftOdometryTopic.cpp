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

 #include <graft/GraftOdometryTopic.h>


GraftOdometryTopic::GraftOdometryTopic(): absolute_pose_(false), delta_pose_(false),
                                          use_velocities_(false), timeout_(1.0){

}

GraftOdometryTopic::~GraftOdometryTopic(){

}

void GraftOdometryTopic::callback(const nav_msgs::Odometry::ConstPtr& msg){
	msg_ = msg;
}

void GraftOdometryTopic::setName(const std::string& name){
	name_ = name;
}

std::string GraftOdometryTopic::getName(){
	return name_;
}

graft::GraftSensorResidual::Ptr GraftOdometryTopic::h(const graft::GraftState& state){
	graft::GraftSensorResidual::Ptr out(new graft::GraftSensorResidual());
	out->header = state.header;
	out->name = name_;
	out->pose = state.pose;
	out->twist = state.twist;
  return out;
}

graft::GraftSensorResidual::Ptr GraftOdometryTopic::z(){
	if(msg_ == NULL){ ///< @TODO If timeout, return NULL
		return graft::GraftSensorResidual::Ptr();
	}
	graft::GraftSensorResidual::Ptr out(new graft::GraftSensorResidual());
	out->header = msg_->header;
	out->name = name_;
	out->pose = msg_->pose.pose;
	out->twist = msg_->twist.twist;
	if(std::accumulate(pose_covariance_.begin(),pose_covariance_.end(),0.0) > 1e-15){ // Override message
		out->pose_covariance = pose_covariance_;
	} else { // Use from message
		out->pose_covariance = msg_->pose.covariance;
	}
	if(std::accumulate(twist_covariance_.begin(),twist_covariance_.end(),0.0) > 1e-15){ // Override message
		out->twist_covariance = twist_covariance_;
	} else { // Use from message
		out->twist_covariance = msg_->twist.covariance;
	}
  return out;
}

void GraftOdometryTopic::useDeltaPose(bool delta_pose){
	delta_pose_ = delta_pose;
}

void GraftOdometryTopic::setTimeout(double timeout){
	timeout_ = timeout;
}

void GraftOdometryTopic::setPoseCovariance(boost::array<double, 36>& cov){
	pose_covariance_ = cov;
}

void GraftOdometryTopic::setTwistCovariance(boost::array<double, 36>& cov){
	twist_covariance_ = cov;
}

nav_msgs::Odometry::ConstPtr GraftOdometryTopic::getMsg(){
	return msg_;
}