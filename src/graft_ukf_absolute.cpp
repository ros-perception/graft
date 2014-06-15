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

#include <iostream>
#include <Eigen/Dense>
#include <ros/ros.h>
#include <graft/GraftParameterManager.h>
#include <graft/GraftSensor.h>
#include <graft/GraftOdometryTopic.h>
#include <graft/GraftImuTopic.h>
#include <graft/GraftUKFAbsolute.h>
#include <graft/GraftState.h>
#include <tf/transform_broadcaster.h>

GraftUKFAbsolute ukfv;

ros::Publisher state_pub;
ros::Publisher odom_pub;

nav_msgs::Odometry odom_;

// tf
bool publish_tf_;
boost::shared_ptr<tf::TransformBroadcaster> broadcaster_;

std::string parent_frame_id_;
std::string child_frame_id_;

void publishTF(const nav_msgs::Odometry& msg){
  geometry_msgs::TransformStamped tf;
  tf.header.stamp = msg.header.stamp;
  tf.header.frame_id = msg.header.frame_id;
  tf.child_frame_id = msg.child_frame_id;
  
  tf.transform.translation.x = msg.pose.pose.position.x;
  tf.transform.translation.y = msg.pose.pose.position.y;
  tf.transform.translation.z = msg.pose.pose.position.z;
  tf.transform.rotation = msg.pose.pose.orientation;
  
  broadcaster_->sendTransform(tf);
}

void timer_callback(const ros::TimerEvent& event){
	double dt = ukfv.predictAndUpdate();

	graft::GraftState state = *ukfv.getMessageFromState();
	state.header.stamp = ros::Time::now();
	state_pub.publish(state);

	// Update Odometry
	odom_.header.stamp = ros::Time::now();
	odom_.header.frame_id = parent_frame_id_;
	odom_.child_frame_id = child_frame_id_;
	odom_.pose.pose = state.pose;
	odom_.twist.twist = state.twist;
	odom_pub.publish(odom_);
	if(publish_tf_){
	  publishTF(odom_);
	}
}

int main(int argc, char **argv)
{
	ros::init(argc, argv, "graft_ukf_velocity");
	ros::NodeHandle n;
	ros::NodeHandle pnh("~");
	state_pub = pnh.advertise<graft::GraftState>("state", 5);
	odom_pub = n.advertise<nav_msgs::Odometry>("odom_combined", 5);

	// Load parameters
	std::vector<boost::shared_ptr<GraftSensor> > topics;
	std::vector<ros::Subscriber> subs;
	GraftParameterManager manager(n, pnh);
	manager.loadParameters(topics, subs);

	publish_tf_ = manager.getPublishTF();

	parent_frame_id_ = manager.getParentFrameID();
	child_frame_id_ = manager.getChildFrameID();

	// Set up the E
	std::vector<double> initial_covariance = manager.getInitialCovariance();
	std::vector<double> Q = manager.getProcessNoise();
	ukfv.setAlpha(manager.getAlpha());
	ukfv.setKappa(manager.getKappa());
	ukfv.setBeta(manager.getBeta());
	ukfv.setInitialCovariance(initial_covariance);
	ukfv.setProcessNoise(Q);
	ukfv.setTopics(topics);

	odom_.pose.pose.position.x = 0.0;
	odom_.pose.pose.position.y = 0.0;
	odom_.pose.pose.position.z = 0.0;

	odom_.pose.pose.orientation.w = 1.0;
	odom_.pose.pose.orientation.x = 0.0;
	odom_.pose.pose.orientation.y = 0.0;
	odom_.pose.pose.orientation.z = 0.0;

	odom_.twist.twist.linear.x = 0.0;
	odom_.twist.twist.linear.y = 0.0;
	odom_.twist.twist.linear.z = 0.0;
	odom_.twist.twist.angular.x = 0.0;
	odom_.twist.twist.angular.y = 0.0;
	odom_.twist.twist.angular.z = 0.0;

	// Tf Broadcaster
    broadcaster_.reset(new tf::TransformBroadcaster());

	// Start loop
	ros::Timer timer = n.createTimer(ros::Duration(1.0/manager.getUpdateRate()), timer_callback);

	// Spin
	ros::spin();
}
