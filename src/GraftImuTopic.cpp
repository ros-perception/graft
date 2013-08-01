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


GraftImuTopic::GraftImuTopic(std::string& name): name_(name){

}

GraftImuTopic::~GraftImuTopic(){

}

void GraftImuTopic::callback(const sensor_msgs::Imu::ConstPtr& msg){
	msg_ = msg;
}

MatrixXd GraftImuTopic::h(graft::GraftState& state){
  Matrix<double, 1, 1> out;
  out(0) = state.twist.angular.z;
  return out;
}

MatrixXd GraftImuTopic::H(graft::GraftState& state){
  Matrix<double, 1, 2> out;
  out(0) = 0;
  out(1) = 1;
  return out;
}

MatrixXd GraftImuTopic::y(graft::GraftState& predicted){
	MatrixXd meas = z();
	return meas - h(predicted);
}

MatrixXd GraftImuTopic::R(){
  Matrix<double, 1, 1> out;
	out(0) = angular_velocity_covariance_[8]; // Wz
	if(msg_ != NULL && out.diagonal().sum() < 0.001){ // All zero and we shouldn't have negative values, use from message
		out(0) = msg_->angular_velocity_covariance[8]; // Wz
	}
	return out;
}

MatrixXd GraftImuTopic::z(){
  Matrix<double, 1, 1> out;
  if(msg_ != NULL){
    out(0) = msg_->angular_velocity.z;
  }
  return out;
}

void GraftImuTopic::useAbsoluteOrientation(bool absolute_orientation){
	absolute_orientation_ = absolute_orientation;
}

void GraftImuTopic::useDeltaOrientation(bool delta_orientation){
	delta_orientation_ = delta_orientation;
}

void GraftImuTopic::useVelocities(bool use_velocities){
	use_velocities_ = use_velocities;
}

void GraftImuTopic::useAccelerations(bool use_accelerations){
	use_accelerations_ = use_accelerations;
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