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

 #include <graft/GraftEKFVelocity.h>
 #include <ros/console.h>

 GraftEKFVelocity::GraftEKFVelocity(){
	graft_state.setZero();
	graft_control.setZero();
	graft_covariance.setIdentity();
 }

GraftEKFVelocity::~GraftEKFVelocity(){

}

Matrix<double, SIZE, 1> GraftEKFVelocity::f(Matrix<double, SIZE, 1> x, Matrix<double, SIZE, 1> u, double dt){
	Matrix<double, SIZE, 1> out;
	out.setZero();
	out(0) = x(0);
	out(1) = x(1);
	return out;
}

Matrix<double, SIZE, SIZE> GraftEKFVelocity::getF(Matrix<double, SIZE, 1> x, Matrix<double, SIZE, 1> u, double dt){
	Matrix<double, SIZE, SIZE> out;
	out.setZero();
	out(0,0) = 1;
	out(1,1) = 1;
    return out;
}

graft::GraftStatePtr GraftEKFVelocity::getMessageFromState(){
	return GraftEKFVelocity::getMessageFromState(graft_state, graft_covariance);
}

graft::GraftStatePtr GraftEKFVelocity::getMessageFromState(Matrix<double, SIZE, 1>& state, Matrix<double, SIZE, SIZE>& covariance){
	graft::GraftStatePtr msg(new graft::GraftState());
	msg->twist.linear.x = state(0);
	msg->twist.angular.z = state(1);

	for(size_t i = 0; i < SIZE*SIZE; i++){
		msg->covariance[i] = covariance(i);
	}
	return msg;
}

MatrixXd verticalConcatenate(MatrixXd& m, MatrixXd& n){
	MatrixXd out;
	out.resize(m.rows()+n.rows(), m.cols());
	out << m,n;
	return out;
}

MatrixXd assembleInnovationCovariance(std::vector<boost::shared_ptr<GraftSensor> >& topics){
	// Gather components (Need to know total size before assembling output matrix)
	size_t diag_length = 0;
	std::vector<MatrixXd> parts;
	for(size_t i = 0; i < topics.size(); i++){
		parts.push_back(topics[i]->R());
		diag_length += parts[i].rows();
	}

	// Create and fill in output matrix
	MatrixXd out = MatrixXd::Zero(diag_length, diag_length);
	size_t diag_position = 0;
	for(size_t i = 0; i < parts.size(); i++){
		std::cout << "Part: " << parts[i]<< std::endl;
		out.block(diag_position, diag_position, parts[i].rows(), parts[i].cols()) = parts[i];
		diag_position += parts[i].rows();
	}
	return out;
}

double GraftEKFVelocity::predictAndUpdate(){
	if(topics_.size() == 0 || topics_[0] == NULL){
		return 0;
	}
	ros::Time t = ros::Time::now();
	double dt = (t - last_update_time_).toSec();
	if(last_update_time_.toSec() < 0.0001){ // No previous updates
		ROS_WARN("Negative dt - odom");
		last_update_time_ = t;
		return 0.0;
	}
	last_update_time_ = t;

	// Prediction
	Matrix<double, SIZE, 1> state_predicted = f(graft_state, graft_control, dt);
	Matrix<double, SIZE, SIZE> F = getF(graft_state, graft_control, dt);
	Matrix<double, SIZE, SIZE> covariance_predicted = F*graft_covariance*F.transpose() + Q_;

	//std::cout << "Q: " << Q_ << std::endl;

	// Update
	MatrixXd y1 = topics_[0]->y(*getMessageFromState(state_predicted, covariance_predicted));
	MatrixXd y2 = topics_[1]->y(*getMessageFromState(state_predicted, covariance_predicted));
	MatrixXd y = verticalConcatenate(y1, y2);
	//std::cout << "y: " << y << std::endl;


	MatrixXd H1 = topics_[0]->H(*getMessageFromState(state_predicted, covariance_predicted));
	MatrixXd H2 = topics_[1]->H(*getMessageFromState(state_predicted, covariance_predicted));
	MatrixXd H = verticalConcatenate(H1, H2);
	//std::cout << "H: " << H << std::endl;

	Matrix<double, 3, 3> R = assembleInnovationCovariance(topics_);
	//std::cout << "R: " << R << std::endl;


	MatrixXd K = covariance_predicted * H.transpose() * (H*covariance_predicted*H.transpose() + R).partialPivLu().inverse();
	//std::cout << "K:\n" << K << std::endl;
	//std::cout << "----------" << std::endl;

	graft_state = state_predicted + K*y;
	graft_covariance = (MatrixXd::Identity(SIZE,SIZE)-K*H)*covariance_predicted;

	return dt;
}

void GraftEKFVelocity::setTopics(std::vector<boost::shared_ptr<GraftSensor> >& topics){
	topics_ = topics;
}

void GraftEKFVelocity::setVelocityProcessNoise(boost::array<double, 4>& Q){
	Q_.resize(std::sqrt(Q.size()),std::sqrt(Q.size()));
	Q_.setZero();
	for(size_t i = 0; i < Q.size(); i++){
		Q_(i) = Q[i];
	}
}

