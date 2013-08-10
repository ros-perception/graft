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

#ifndef GRAFT_ODOMETRY_TOPIC_H
#define GRAFT_ODOMETRY_TOPIC_H

#include <graft/GraftSensor.h>
#include <ros/ros.h>
#include <Eigen/Dense>
#include <nav_msgs/Odometry.h>
#include <tf/transform_datatypes.h>
#include <numeric>

using namespace Eigen;

class GraftOdometryTopic: public GraftSensor {
  public:
  	GraftOdometryTopic();

  	~GraftOdometryTopic();

  	void callback(const nav_msgs::Odometry::ConstPtr& msg);

    virtual graft::GraftSensorResidual::Ptr h(const graft::GraftState& state);

    virtual graft::GraftSensorResidual::Ptr z();

    virtual void setName(const std::string& name);

    virtual std::string getName();

    virtual void clearMessage();

    //virtual MatrixXd H(graft::GraftState& state);

    //virtual MatrixXd y(graft::GraftState& predicted);

    //virtual MatrixXd R();


    //void useAbsolutePose(bool absolute_pose);

    void useDeltaPose(bool delta_pose);

    //void useVelocities(bool use_velocities);

    void setTimeout(double timeout);

    void setPoseCovariance(boost::array<double, 36>& cov);

    void setTwistCovariance(boost::array<double, 36>& cov);
    
  private:

  	nav_msgs::Odometry::ConstPtr getMsg();

  	ros::Subscriber sub_;
  	nav_msgs::Odometry::ConstPtr msg_;
    nav_msgs::Odometry::ConstPtr last_msg_; // Used for delta calculations

  	std::string name_;
  	bool absolute_pose_;
  	bool delta_pose_;
  	bool use_velocities_;
  	double timeout_;


  	boost::array<double, 36> pose_covariance_;
  	boost::array<double, 36> twist_covariance_;

};

#endif