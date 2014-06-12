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

#ifndef GRAFT_INCLUDE_GRAFT_GRAFT_PARAMETER_MANAGER_H
#define GRAFT_INCLUDE_GRAFT_GRAFT_PARAMETER_MANAGER_H

#include <ros/ros.h>
#include <graft/GraftSensor.h>

#include <graft/GraftOdometryTopic.h>
#include <graft/GraftImuTopic.h>

class GraftParameterManager
{
public:
  GraftParameterManager(ros::NodeHandle n, ros::NodeHandle pnh);

  ~GraftParameterManager();

  void loadParameters(std::vector<boost::shared_ptr<GraftSensor> >& topics, std::vector<ros::Subscriber>& subs);

  void parseNavMsgsOdometryParameters(ros::NodeHandle& tnh, boost::shared_ptr<GraftOdometryTopic>& odom);

  void parseSensorMsgsIMUParameters(ros::NodeHandle& tnh, boost::shared_ptr<GraftImuTopic>& imu);

  std::string getFilterType();

  bool getPlanarOutput();

  std::string getParentFrameID();

  std::string getChildFrameID();

  double getUpdateRate();

  std::string getUpdateTopic();

  double getdtOveride();

  int getQueueSize();

  bool getIncludePose();

  bool getPublishTF();

  std::vector<double> getInitialCovariance();

  std::vector<double> getProcessNoise();

  double getAlpha();

  double getKappa();

  double getBeta();

private:

  ros::NodeHandle n_;
  ros::NodeHandle pnh_;

  std::string filter_type_;  // EKF or UKF
  bool planar_output_;  // Output in 2D instead of 3D
  std::string parent_frame_id_;
  std::string child_frame_id_;
  double update_rate_;  // How often to update
  std::string update_topic_;  // Update when this topic arrives
  double dt_override_;  // Overrides the dt between updates, ignored if 0
  int queue_size_;
  bool publish_tf_;
  std::vector<double> initial_covariance_;
  std::vector<double> process_noise_;
  double alpha_;
  double kappa_;
  double beta_;

  // Derived parameters for filter behavior
  bool include_pose_;
};

#endif  // GRAFT_INCLUDE_GRAFT_GRAFT_PARAMETER_MANAGER_H
