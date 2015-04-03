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


// Commonly used parameter loading function
template <std::size_t SIZE>
bool load_rosparam_array(ros::NodeHandle& nh,
                         const std::string& param_name,
                         boost::array<double, SIZE>& values)
{
  XmlRpc::XmlRpcValue xml_array;
  if (nh.getParam(param_name, xml_array))
  {
    if (xml_array.size() == SIZE)
    {
      for (size_t i = 0; i < xml_array.size(); i++)
      {
        std::stringstream ss; // Convert the list element into doubles
        ss << xml_array[i];
        ss >> values[i] ? values[i] : 0;
      }
      return true;  // loaded params into array
    }
    else
    {
      ROS_WARN("%s/%s parameter requires %d elements, skipping.",
               nh.getNamespace().c_str(), param_name.c_str(), static_cast<int>(SIZE));
      return false;
    }
  }
  return false;
}


GraftParameterManager::GraftParameterManager(ros::NodeHandle n, ros::NodeHandle pnh):
  n_(n), pnh_(pnh), include_pose_(false)
{
}


GraftParameterManager::~GraftParameterManager()
{
}


void GraftParameterManager::parseNavMsgsOdometryParameters(
    ros::NodeHandle& tnh,
    boost::shared_ptr<GraftOdometryTopic>& odom)
{
  // Check how to use this sensor
  bool absolute_pose, delta_pose, use_velocities;
  double timeout;
  tnh.param<bool>("absolute_pose", absolute_pose, false);
  tnh.param<bool>("delta_pose", delta_pose, false);
  tnh.param<bool>("use_velocities", use_velocities, false);
  tnh.param<double>("timeout", timeout, 1.0);

  // Check for incompatible usage
  if (absolute_pose == true)
  {
    delta_pose = false;  // Should not use for both absolute position and velocity
  }
  if (delta_pose == true)
  {
    use_velocities = false;  // Should not use both estimated and reported velocities
  }

  // Apply to global state
  include_pose_ = include_pose_ || absolute_pose;

  // Apply to sensor
  odom->useAbsolutePose(absolute_pose);
  odom->useDeltaPose(delta_pose);
  odom->useVelocities(use_velocities);
  odom->setTimeout(timeout);

  //ROS_INFO("Abs pose: %d\nDelta pose: %d\nUse Vel: %d\nTimeout: %.3f", absolute_pose, delta_pose, use_velocities, timeout);

  // Set covariances
  boost::array<double, 36> covariance;
  if (load_rosparam_array(tnh, "override_pose_covariance", covariance))
    odom->setPoseCovariance(covariance);
  if (load_rosparam_array(tnh, "override_twist_covariance", covariance))
    odom->setTwistCovariance(covariance);
}

void GraftParameterManager::parseSensorMsgsIMUParameters(
    ros::NodeHandle& tnh,
    boost::shared_ptr<GraftImuTopic>& imu)
{
  // Check how to use this sensor
  bool absolute_orientation, delta_orientation, use_velocities, use_accelerations;
  double timeout;
  tnh.param<bool>("absolute_orientation", absolute_orientation, false);
  tnh.param<bool>("delta_orientation", delta_orientation, false);
  tnh.param<bool>("use_velocities", use_velocities, false);
  tnh.param<bool>("use_accelerations", use_accelerations, false);
  tnh.param<double>("timeout", timeout, 1.0);

  // Check for incompatible usage
  if (absolute_orientation == true){
    delta_orientation = false;  // Should not use for both absolute position and velocity
  }
  if (delta_orientation == true){
    use_velocities = false;  // Should not use both estimated and reported velocities
  }

  // Apply to global state
  include_pose_ = include_pose_ || absolute_orientation;

  // Apply to sensor
  //imu->useAbsoluteOrientation(absolute_orientation);
  imu->useDeltaOrientation(delta_orientation);
  //imu->useVelocities(use_velocities);
  //imu->useAccelerations(use_accelerations);
  imu->setTimeout(timeout);

  ROS_INFO("Abs orientation: %d\nDelta orientation: %d\nUse Vel: %d\nTimeout: %.3f",
    absolute_orientation, delta_orientation, use_velocities, timeout);

  // Set covariances
  boost::array<double, 9> covariance;
  if (load_rosparam_array(tnh, "override_orientation_covariance", covariance))
    imu->setOrientationCovariance(covariance);
  if (load_rosparam_array(tnh, "override_angular_velocity_covariance", covariance))
    imu->setAngularVelocityCovariance(covariance);
  if (load_rosparam_array(tnh, "override_linear_acceleration_covariance", covariance))
    imu->setLinearAccelerationCovariance(covariance);
}

void GraftParameterManager::loadParameters(
    std::vector<boost::shared_ptr<GraftSensor> >& topics,
    std::vector<ros::Subscriber>& subs)
{
  // Filter behavior parameters
  pnh_.param<std::string>("filter_type", filter_type_, "EKF");
  pnh_.param<bool>("planar_output", planar_output_, true);

  pnh_.param<std::string>("parent_frame_id", parent_frame_id_, "odom");
  pnh_.param<std::string>("child_frame_id", child_frame_id_, "base_link");

  pnh_.param<double>("freq", update_rate_, 50.0);
  pnh_.param<double>("update_rate", update_rate_, update_rate_); // Overrides 'freq'
  pnh_.param<double>("dt_override", dt_override_, 0.0);

  pnh_.param<bool>("publish_tf", publish_tf_, false);

  pnh_.param<int>("queue_size", queue_size_, 1);

  pnh_.param<double>("alpha", alpha_, 0.001);
  pnh_.param<double>("kappa", kappa_, 0.0);
  pnh_.param<double>("beta", beta_, 2.0);

  // Initial covariance
  XmlRpc::XmlRpcValue xml_initial_covariance;
  if (pnh_.getParam("initial_covariance", xml_initial_covariance))
  {
    initial_covariance_.resize(xml_initial_covariance.size());
    for (size_t i = 0; i < xml_initial_covariance.size(); i++)
    {
      std::stringstream ss;  // Convert the list element into doubles
      ss << xml_initial_covariance[i];
      ss >> initial_covariance_[i] ? initial_covariance_[i] : 0;
    }
  }

  // Process noise covariance
  XmlRpc::XmlRpcValue xml_process_noise;
  if (pnh_.getParam("process_noise", xml_process_noise))
  {
    process_noise_.resize(xml_process_noise.size());
    for (size_t i = 0; i < xml_process_noise.size(); i++)
    {
      std::stringstream ss;  // Convert the list element into doubles
      ss << xml_process_noise[i];
      ss >> process_noise_[i] ? process_noise_[i] : 0;
    }
  }

  // Read each topic config
  try
  {
    XmlRpc::XmlRpcValue topic_list;
    if (!pnh_.getParam("topics", topic_list))
    {
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
      if (!tnh.getParam("type", type))
      {
        ROS_ERROR("Could not get topic type for %s, skipping.", topic_name.c_str());
        continue;
      }

      ROS_INFO("Type: %s", type.c_str());

      if (type == "nav_msgs/Odometry")
      {
        std::string full_topic;
        if (!tnh.getParam("topic", full_topic))
        {
          ROS_ERROR("Could not get full topic for %s, skipping.", topic_name.c_str());
          continue;
        }

        // Create Odometry object
        boost::shared_ptr<GraftOdometryTopic> odom(new GraftOdometryTopic());
        odom->setName(topic_name);
        topics.push_back(odom);

        // Subscribe to topic
        ros::Subscriber sub = n_.subscribe(full_topic, queue_size_, &GraftOdometryTopic::callback, odom);
        subs.push_back(sub);

        // Parse rest of parameters
        parseNavMsgsOdometryParameters(tnh, odom);
      }
      else if (type == "sensor_msgs/Imu")
      {
        std::string full_topic;
        if (!tnh.getParam("topic", full_topic))
        {
          ROS_ERROR("Could not get full topic for %s, skipping.", topic_name.c_str());
          continue;
        }

        // Create Odometry object
        boost::shared_ptr<GraftImuTopic> imu(new GraftImuTopic());
        imu->setName(topic_name);
        topics.push_back(imu);

        // Subscribe to topic
        ros::Subscriber sub = n_.subscribe(full_topic, queue_size_, &GraftImuTopic::callback, imu);
        subs.push_back(sub);

        // Parse rest of parameters
        parseSensorMsgsIMUParameters(tnh, imu);
      }
      else
      {
        ROS_WARN("Unknown type: %s  Not parsing configuration.", type.c_str());
      }
    }
  }
  catch(...)
  {
    ROS_FATAL("XmlRpc Error parsing parameters.  Cannot start.");
    ros::shutdown();
  }
}


std::string GraftParameterManager::getFilterType()
{
  return filter_type_;
}


bool GraftParameterManager::getPlanarOutput()
{
  return planar_output_;
}


std::string GraftParameterManager::getParentFrameID()
{
  return parent_frame_id_;
}


std::string GraftParameterManager::getChildFrameID()
{
  return child_frame_id_;
}


double GraftParameterManager::getUpdateRate()
{
  return update_rate_;
}


std::string GraftParameterManager::getUpdateTopic()
{
  return update_topic_;
}


double GraftParameterManager::getdtOveride()
{
  return dt_override_;
}


int GraftParameterManager::getQueueSize()
{
  return queue_size_;
}


bool GraftParameterManager::getIncludePose()
{
  return include_pose_;
}


bool GraftParameterManager::getPublishTF()
{
  return publish_tf_;
}


std::vector<double> GraftParameterManager::getInitialCovariance()
{
  return initial_covariance_;
}


std::vector<double> GraftParameterManager::getProcessNoise()
{
  return process_noise_;
}


double GraftParameterManager::getAlpha()
{
  return alpha_;
}


double GraftParameterManager::getKappa()
{
  return kappa_;
}


double GraftParameterManager::getBeta()
{
  return beta_;
}
