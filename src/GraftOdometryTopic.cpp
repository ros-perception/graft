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


GraftOdometryTopic::GraftOdometryTopic(std::string& name): name_(name), absolute_pose_(false), delta_pose_(false),
                                                           use_velocities_(false), timeout_(1.0){

}

GraftOdometryTopic::~GraftOdometryTopic(){

}

void GraftOdometryTopic::callback(const nav_msgs::Odometry::ConstPtr& msg){
	msg_ = msg;
}

MatrixXd GraftOdometryTopic::h(graft::GraftState& state){
  Matrix<double, 2, 1> out;
  out(0) = state.twist.linear.x;
  out(1) = state.twist.angular.z;
  return out;
}

MatrixXd GraftOdometryTopic::H(graft::GraftState& state){
  Matrix<double, 2, 2> out;
  out(0,0) = 1;
  out(1,1) = 1;
  return out;
}

MatrixXd GraftOdometryTopic::y(graft::GraftState& predicted){
	MatrixXd meas = z();
	return meas - h(predicted);
}

MatrixXd GraftOdometryTopic::R(){
	Matrix<double, 2, 2> out;
	out(0,0) = twist_covariance_[0]; // Vx
	out(1,1) = twist_covariance_[35]; // Wz
	if(msg_ != NULL && out.diagonal().sum() < 0.001){ // All zero and we shouldn't have negative values, use from message
		out(0,0) = msg_->twist.covariance[0]; // Vx
	  out(1,1) = msg_->twist.covariance[35]; // Wz
	}
	return out;
}

MatrixXd GraftOdometryTopic::z(){
  Matrix<double, 2, 1> out;
  if(msg_ != NULL){
	  out(0) = msg_->twist.twist.linear.x;
	  out(1) = msg_->twist.twist.angular.z;
  }
  return out;
}

void GraftOdometryTopic::useAbsolutePose(bool absolute_pose){
	absolute_pose_ = absolute_pose;
	if(absolute_pose_ == true){
		ROS_WARN("Absolute pose from odometry not yet supported.");
		absolute_pose_ = false;
	}
}

void GraftOdometryTopic::useDeltaPose(bool delta_pose){
	delta_pose_ = delta_pose;
}

void GraftOdometryTopic::useVelocities(bool use_velocities){
	use_velocities_ = use_velocities;
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