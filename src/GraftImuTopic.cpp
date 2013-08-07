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

 #include <graft/GraftImuTopic.h>


GraftImuTopic::GraftImuTopic(){

}

GraftImuTopic::~GraftImuTopic(){

}

void GraftImuTopic::callback(const sensor_msgs::Imu::ConstPtr& msg){
	msg_ = msg;
}

void GraftImuTopic::setName(const std::string& name){
	name_ = name;
}

std::string GraftImuTopic::getName(){
	return name_;
}

graft::GraftSensorResidual::Ptr GraftImuTopic::h(const graft::GraftState& state){
	graft::GraftSensorResidual::Ptr out(new graft::GraftSensorResidual());
	out->header = state.header;
	out->name = name_;
	out->pose = state.pose;
	out->twist = state.twist;
  return out;
}

boost::array<double, 36> largeCovarianceFromSmallCovariance(const boost::array<double, 9>& angular_velocity_covariance){
	boost::array<double, 36> out;
	for(size_t i = 0; i < out.size(); i++){
		out[i] = 0;
	}
	for(size_t i = 0; i < 3; i++){
		out[i+21] = angular_velocity_covariance[i];
	}
	for(size_t i = 3; i < 6; i++){
		out[i+24] = angular_velocity_covariance[i];
	}
	for(size_t i = 6; i < 9; i++){
		out[i+27] = angular_velocity_covariance[i];
	}
	return out;
}

graft::GraftSensorResidual::Ptr GraftImuTopic::z(){
	if(msg_ == NULL){ ///< @TODO If timeout, return NULL
		return graft::GraftSensorResidual::Ptr();
	}
	graft::GraftSensorResidual::Ptr out(new graft::GraftSensorResidual());
	out->header = msg_->header;
	out->name = name_;
	out->pose.orientation = msg_->orientation;
	out->twist.angular = msg_->angular_velocity;
	out->accel = msg_->linear_acceleration;
	if(std::accumulate(orientation_covariance_.begin(),orientation_covariance_.end(),0.0) > 1e-15){ // Override message
		out->pose_covariance = largeCovarianceFromSmallCovariance(orientation_covariance_);
	} else { // Use from message
		out->pose_covariance = largeCovarianceFromSmallCovariance(msg_->orientation_covariance);
	}
	if(std::accumulate(angular_velocity_covariance_.begin(),angular_velocity_covariance_.end(),0.0) > 1e-15){ // Override message
		out->twist_covariance = largeCovarianceFromSmallCovariance(angular_velocity_covariance_);
	} else { // Use from message
		out->twist_covariance = largeCovarianceFromSmallCovariance(msg_->angular_velocity_covariance);
	}
	if(std::accumulate(linear_acceleration_covariance_.begin(),linear_acceleration_covariance_.end(),0.0) > 1e-15){ // Override message
		out->accel_covariance = linear_acceleration_covariance_;
	} else { // Use from message
		out->accel_covariance = msg_->linear_acceleration_covariance;
	}
  return out;
}

void GraftImuTopic::useDeltaOrientation(bool delta_orientation){
	delta_orientation_ = delta_orientation;
}

void GraftImuTopic::setTimeout(double timeout){
	timeout_ = timeout;
}

void GraftImuTopic::setOrientationCovariance(boost::array<double, 9>& cov){
	orientation_covariance_ = cov;
}

void GraftImuTopic::setAngularVelocityCovariance(boost::array<double, 9>& cov){
	angular_velocity_covariance_ = cov;
}

void GraftImuTopic::setLinearAccelerationCovariance(boost::array<double, 9>& cov){
	linear_acceleration_covariance_ = cov;
}

sensor_msgs::Imu::ConstPtr GraftImuTopic::getMsg(){
	return msg_;
}