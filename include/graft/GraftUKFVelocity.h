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

#ifndef GRAFT_UKFVELOCITY_H
#define GRAFT_UKFVELOCITY_H

#include <Eigen/Dense>
#include <Eigen/Cholesky>

#include <graft/GraftState.h>
#include <nav_msgs/Odometry.h>
#include <geometry_msgs/QuaternionStamped.h>
#include <sensor_msgs/Imu.h>
#include <tf/transform_datatypes.h>
 #include <graft/GraftSensor.h>

#define SIZE 2  // State size: vx, wz

using namespace Eigen;

class GraftUKFVelocity{
  public:
    GraftUKFVelocity();
    ~GraftUKFVelocity();

	MatrixXd f(MatrixXd x, double dt);

	std::vector<MatrixXd > predict_sigma_points(std::vector<MatrixXd >& sigma_points, double dt);

	graft::GraftStatePtr getMessageFromState();

	graft::GraftStatePtr getMessageFromState(Matrix<double, SIZE, 1>& state, Matrix<double, SIZE, SIZE>& covariance);

	double predictAndUpdate();

	void setTopics(std::vector<boost::shared_ptr<GraftSensor> >& topics);

	void setVelocityProcessNoise(boost::array<double, 4>& Q);
    
  private:

    Matrix<double, SIZE, 1> graft_state_;
	Matrix<double, SIZE, 1> graft_control_;
	Matrix<double, SIZE, SIZE> graft_covariance_;

	Matrix<double, SIZE, SIZE> Q_;

    ros::Time last_update_time_;
    ros::Time last_imu_time_;

    double alpha_;
    double beta_;
    double kappa_;

    std::vector<boost::shared_ptr<GraftSensor> > topics_;
};

#endif