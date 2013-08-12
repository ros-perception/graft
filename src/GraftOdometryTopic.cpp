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

void GraftOdometryTopic::clearMessage(){
	msg_ = nav_msgs::Odometry::ConstPtr();
}

geometry_msgs::Twist::Ptr twistFromPoses(const geometry_msgs::Pose& pose, const geometry_msgs::Pose& last_pose, const double dt){
	geometry_msgs::Twist::Ptr out(new geometry_msgs::Twist());
	if(dt < 1e-10){
		return out;
	}
	tf::Quaternion tfq2, tfq1;
	tf::quaternionMsgToTF(pose.orientation, tfq2);
  tf::quaternionMsgToTF(last_pose.orientation, tfq1);
  tf::Transform tf2(tfq2, tf::Vector3(pose.position.x, pose.position.y, pose.position.z));
  tf::Transform tf1(tfq1, tf::Vector3(last_pose.position.x, last_pose.position.y, last_pose.position.z));

  tf::Transform dtf = tf1.inverse() * tf2;

  out->linear.x = dtf.getOrigin().getX()/dt;
  out->linear.y = dtf.getOrigin().getY()/dt;
  out->linear.y = dtf.getOrigin().getZ()/dt;
  double rX, rY, rZ;
  dtf.getBasis().getEulerYPR(rZ, rY, rX);
  out->angular.x = rX/dt;
  out->angular.y = rY/dt;
	out->angular.z = rZ/dt;

	return out;
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
	if(msg_ == NULL || ros::Time::now() - timeout_ > msg_->header.stamp){
		ROS_WARN_THROTTLE(5.0, "Odometry timeout");
		return graft::GraftSensorResidual::Ptr();
	}
	graft::GraftSensorResidual::Ptr out(new graft::GraftSensorResidual());
	out->header = msg_->header;
	out->name = name_;

	if(delta_pose_){
		if(last_msg_ == NULL){
			last_msg_ = msg_;
			return graft::GraftSensorResidual::Ptr();
		}
		double dt = (msg_->header.stamp - last_msg_->header.stamp).toSec();
		out->twist = *twistFromPoses(msg_->pose.pose, last_msg_->pose.pose, dt);
		last_msg_ = msg_;
		if(std::accumulate(twist_covariance_.begin(),twist_covariance_.end(),0.0) > 1e-15){ // Override message
			out->twist_covariance = twist_covariance_;
		} else { // Use from message
			// Not sure what to do about covariance here...... robot_pose_ekf was strange
			out->twist_covariance = msg_->pose.covariance;
		}
	} else {
		out->twist = msg_->twist.twist;
		if(std::accumulate(twist_covariance_.begin(),twist_covariance_.end(),0.0) > 1e-15){ // Override message
			out->twist_covariance = twist_covariance_;
		} else { // Use from message
			out->twist_covariance = msg_->twist.covariance;
		}
	}

	if(absolute_pose_ && !delta_pose_){ // Delta Pose and Absolute Pose are incompatible
		out->pose = msg_->pose.pose;
		if(std::accumulate(pose_covariance_.begin(),pose_covariance_.end(),0.0) > 1e-15){ // Override message
			out->pose_covariance = pose_covariance_;
		} else { // Use from message
			out->pose_covariance = msg_->pose.covariance;
		}
	}
  return out;
}

void GraftOdometryTopic::useDeltaPose(bool delta_pose){
	delta_pose_ = delta_pose;
}

void GraftOdometryTopic::setTimeout(double timeout){
	if(timeout < 1e-10){
		timeout_ = ros::Duration(1e10); // No timeout enforced
		return;
	}
	timeout_ = ros::Duration(timeout);
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