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

 #include <graft/GraftUKFVelocity.h>
 #include <ros/console.h>

 GraftUKFVelocity::GraftUKFVelocity(){
	graft_state_.setZero();
	graft_control_.setZero();
	graft_covariance_.setIdentity();
	Q_.setZero();
	alpha_ = 0.001;
	kappa_ = 0.0;
	beta_ = 2.0;
 }

GraftUKFVelocity::~GraftUKFVelocity(){

}

MatrixXd verticalConcatenate(MatrixXd& m, MatrixXd& n){
	MatrixXd out;
	out.resize(m.rows()+n.rows(), m.cols());
	out << m,n;
	return out;
}

MatrixXd matrixSqrt(MatrixXd matrix){ ///< @TODO Make a reference?  MatrixXd vs templated....
	// Use LLT Cholesky decomposiion to create stable Matrix Sqrt
	return Eigen::LLT<Eigen::MatrixXd>(matrix).matrixL();
}

std::vector<MatrixXd > generateSigmaPoints(MatrixXd state, MatrixXd covariance, double lambda){
	std::vector<MatrixXd > out;

	double gamma = std::sqrt((state.rows()+lambda));
	MatrixXd sig_sqrt = gamma*matrixSqrt(covariance);
	//std::cout << (state.rows()+lambda)*(covariance) << std::endl;
	//std::cout << sig_sqrt << std::endl;

	// i = 0, push back state as is
	out.push_back(state);

	// i = 1,...,n
	for(size_t i = 1; i <= state.rows(); i++){
		out.push_back(state + sig_sqrt.col(i-1));
	}

	// i = n + 1,...,2n
	for(size_t i = state.rows() + 1; i <= 2*state.rows(); i++){
		out.push_back(state - sig_sqrt.col(i-(state.rows()+1)));
	}
	
	/*std::cout << "---" << std::endl;
	for(size_t i = 0; i < out.size(); i++){
		std::cout << out[i] << std::endl;
		std::cout << "-" << std::endl;
	}*/

	return out;
}

MatrixXd meanFromSigmaPoints(std::vector<MatrixXd >& sigma_points, double n, double lambda){
	double weight_zero = lambda / (n + lambda);
	MatrixXd out = weight_zero * sigma_points[0];
	double weight_i = 1.0/(2*(n + lambda));
	for(size_t i = 1; i <= 2*n; i++){
		out = out + weight_i * sigma_points[i];
	}
	std::cout << "mean:\n" << out << std::endl;
	return out;
}

///< @TODO Combined covariancesFromSigmaPoints with crossCovariance
MatrixXd covarianceFromSigmaPoints(std::vector<MatrixXd >& sigma_points, MatrixXd& mean, MatrixXd process_noise, double n, double alpha, double beta, double lambda){
	double cov_weight_zero = lambda / (n + lambda) + (1 - alpha*alpha + beta);
	MatrixXd out = cov_weight_zero * (sigma_points[0] - mean) * (sigma_points[0] - mean).transpose();
	double weight_i = 1.0/(2*(n + lambda));
	for(size_t i = 1; i <= 2*n; i++){
		out = out + weight_i  * (sigma_points[i] - mean) * (sigma_points[i] - mean).transpose();
	}
	std::cout << "cov:\n" << out << std::endl;
	std::cout << "covQ:\n" << out + process_noise << std::endl;
	return out+process_noise;
}

MatrixXd crossCovariance(std::vector<MatrixXd >& sigma_points, MatrixXd& mean, std::vector<MatrixXd >& meas_sigma_points, MatrixXd& meas_mean, double alpha, double beta, double lambda){
	double n = sigma_points[0].rows();
	double cov_weight_zero = lambda / (n + lambda) + (1 - alpha*alpha + beta);
	MatrixXd out = cov_weight_zero * (sigma_points[0] - mean) * (meas_sigma_points[0] - meas_mean).transpose();
	double weight_i = 1.0/(2*(n + lambda));
	for(size_t i = 1; i <= 2*n; i++){
		out = out + weight_i  * (sigma_points[i] - mean) * (meas_sigma_points[i] - meas_mean).transpose();
	}
	std::cout << "crosscov:\n" << out << std::endl;
	return out;
}


MatrixXd GraftUKFVelocity::f(MatrixXd x, double dt){
	Matrix<double, SIZE, 1> out;
	out.setZero();
	out(0) = x(0);
	out(1) = x(1);
	return out;
}

VectorXd hFromSensors(std::vector<graft::GraftSensorResidual::ConstPtr>& res){
	VectorXd out;
	std::cout << "HSENSORS" << res.size() << std::endl;
	for(size_t i = 0; i < res.size(); i++){
		std::cout << "name: " << res[i]->name << std::endl;
		// Linear Velocity X
		if(res[i]->twist_covariance[0] > 0){
			std::cout << "vx" << std::endl;
			out.resize(out.size() + 1);
			out[out.size()-1] = res[i]->twist.linear.x;
		}
		// Angular Velocity Z
		if(res[i]->twist_covariance[35] > 0){
			std::cout << "wz" << std::endl;
			out.resize(out.size() + 1);
			out[out.size()-1] = res[i]->twist.angular.z;
		}
	}
	return out;
}

graft::GraftState::ConstPtr stateMsgFromMatrix(const MatrixXd& state){
	graft::GraftState::Ptr out(new graft::GraftState());
	out->twist.linear.x = state(0);
	out->twist.angular.z = state(1);
	return out;
}

/*std::vector<MatrixXd> h(std::vector<MatrixXd >& sigma_points, std::vector<boost::shared_ptr<GraftSensor> >& topics){
	std::vector<MatrixXd > out;
	for(size_t i = 0; i < sigma_points.size(); i++){
		std::vector<graft::GraftSensorResidual::ConstPtr> residuals;
		graft::GraftState::ConstPtr state_msg = stateMsgFromMatrix(sigma_points[i]);
		for(size_t j = 0; j < topics.size(); j++){
			residuals.push_back(topics[j]->h(*state_msg));
		}
		out.push_back(hFromSensors(residuals));
	}
	std::cout << "Zt:"<< std::endl;
	std::cout << "---" << std::endl;
	for(size_t i = 0; i < out.size(); i++){
		std::cout << out[i] << std::endl;
		std::cout << "-" << std::endl;
	}
	return out;
}*/

MatrixXd getMeasurement(std::vector<boost::shared_ptr<GraftSensor> >& topics){
	std::vector<graft::GraftSensorResidual::ConstPtr> residuals;
	for(size_t j = 0; j < topics.size(); j++){
		residuals.push_back(topics[j]->z());
	}
	std::cout << "MEAS:~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~\n" << hFromSensors(residuals) << std::endl;
	return hFromSensors(residuals);
}

std::vector<MatrixXd > GraftUKFVelocity::predict_sigma_points(std::vector<MatrixXd >& sigma_points, double dt){
	std::vector<MatrixXd > out;
	for(size_t i = 0; i < sigma_points.size(); i++){
		out.push_back(f(sigma_points[i], dt));
	}
	return out;
}

graft::GraftStatePtr GraftUKFVelocity::getMessageFromState(){
	return GraftUKFVelocity::getMessageFromState(graft_state_, graft_covariance_);
}

graft::GraftStatePtr GraftUKFVelocity::getMessageFromState(Matrix<double, SIZE, 1>& state, Matrix<double, SIZE, SIZE>& covariance){
	graft::GraftStatePtr msg(new graft::GraftState());
	msg->twist.linear.x = state(0);
	msg->twist.angular.z = state(1);

	for(size_t i = 0; i < SIZE*SIZE; i++){
		msg->covariance[i] = covariance(i);
	}
	return msg;
}

VectorXd addElementToVector(const VectorXd& vec, const double& element){
	VectorXd out(vec.size() + 1);
	out << vec, element;
	return out;
}

MatrixXd addElementToColumnMatrix(const MatrixXd& mat, const double& element){
	MatrixXd out(mat.rows() + 1, 1);
	MatrixXd small(1, 1);
	small(0,0) = element;
	if(mat.rows() == 0){
		return small;
	}
	out << mat, small;
	return out;
}

// Returns measurement vector
VectorXd getMeasurements(const std::vector<boost::shared_ptr<GraftSensor> >& topics, const std::vector<MatrixXd>& predicted_sigma_points, std::vector<MatrixXd>& output_measurement_sigmas, MatrixXd& output_innovation_covariance){
	VectorXd actual_measurement;
	output_measurement_sigmas.clear();
	VectorXd innovation_covariance_diagonal;
	innovation_covariance_diagonal.resize(0);
	// Convert the predicted_sigma_points into messages
	std::vector<graft::GraftState::ConstPtr> predicted_sigma_msgs;
	for(size_t i = 0; i < predicted_sigma_points.size(); i++){
		predicted_sigma_msgs.push_back(stateMsgFromMatrix(predicted_sigma_points[i]));
		output_measurement_sigmas.push_back(MatrixXd());
	}

	
	// For each topic
	for(size_t i = 0; i < topics.size(); i++){
		// Get the measurement msg and covariance
		graft::GraftSensorResidual::ConstPtr meas = topics[i]->z();
		// Get the predicted measurements
		std::vector<graft::GraftSensorResidual::ConstPtr> residuals_msgs;
		for(size_t j = 0; j < predicted_sigma_msgs.size(); j++){
			residuals_msgs.push_back(topics[i]->h(*predicted_sigma_msgs[j]));
		}
		// Assemble outputs for this topic
		ROS_INFO("Starting process for %s", topics[i]->getName().c_str());
		if(meas == NULL){ // Timeout or not received or invalid, skip
			ROS_WARN("measurement %s NULL", topics[i]->getName().c_str());
			continue;
		}
		// Linear Velocity X
		if(meas->twist_covariance[0] > 1e-20){
			std::cout << "vx" << std::endl;
			actual_measurement = addElementToVector(actual_measurement, meas->twist.linear.x);
			innovation_covariance_diagonal = addElementToVector(innovation_covariance_diagonal, meas->twist_covariance[0]);
			std::cout << "vec:\n" << innovation_covariance_diagonal << std::endl;
			for(size_t j = 0; j < residuals_msgs.size(); j++){
				output_measurement_sigmas[j] = addElementToColumnMatrix(output_measurement_sigmas[j], residuals_msgs[j]->twist.linear.x);
			}
		}
		// Angular Velocity Z
		if(meas->twist_covariance[35] > 1e-20){
			std::cout << "wz" << std::endl;
			actual_measurement = addElementToVector(actual_measurement, meas->twist.angular.z);
			innovation_covariance_diagonal = addElementToVector(innovation_covariance_diagonal, meas->twist_covariance[35]);
			std::cout << "vec:\n" << innovation_covariance_diagonal << std::endl;
			for(size_t j = 0; j < residuals_msgs.size(); j++){
				output_measurement_sigmas[j] = addElementToColumnMatrix(output_measurement_sigmas[j], residuals_msgs[j]->twist.angular.z);
			}
		}
	}

	// Convert covariance vector to matrix
	output_innovation_covariance = innovation_covariance_diagonal.asDiagonal();

	std::cout << "MEASU !~!~!~!~!~!~!~!~:\n" << actual_measurement << std::endl;
	std::cout << output_innovation_covariance << std::endl;
	std::cout << "###########################" << std::endl;
	
	return actual_measurement;
}

double GraftUKFVelocity::predictAndUpdate(){
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
	double lambda = alpha_*alpha_*(SIZE + kappa_) - SIZE;
	std::vector<MatrixXd > previous_sigma_points = generateSigmaPoints(graft_state_, graft_covariance_, lambda);
	std::vector<MatrixXd > predicted_sigma_points = predict_sigma_points(previous_sigma_points, 0.0);
	MatrixXd predicted_mean = meanFromSigmaPoints(predicted_sigma_points, graft_state_.rows(), lambda);
	MatrixXd predicted_covariance = covarianceFromSigmaPoints(predicted_sigma_points, predicted_mean, Q_, graft_state_.rows(), alpha_, beta_, lambda);
	std::cout << "---------" << std::endl;

	// Update
	std::vector<MatrixXd> observation_sigma_points = generateSigmaPoints(predicted_mean, predicted_covariance, lambda);
	std::vector<MatrixXd> predicted_observation_sigma_points;
	MatrixXd measurement_noise;
	MatrixXd z = getMeasurements(topics_, observation_sigma_points, predicted_observation_sigma_points, measurement_noise);
	if(z.size() == 0){ // No measurements
		return 0.0;
	}
	MatrixXd predicted_measurement = meanFromSigmaPoints(predicted_observation_sigma_points, graft_state_.rows(), lambda);
	std::cout << "........................" << std::endl;
	MatrixXd predicted_measurement_uncertainty = covarianceFromSigmaPoints(predicted_observation_sigma_points, predicted_measurement, measurement_noise, graft_state_.rows(), alpha_, beta_, lambda);
	std::cout << ";;;;;;;;;;;;;;;;;;;;;;;;" << std::endl;
	MatrixXd cross_covariance = crossCovariance(observation_sigma_points, predicted_mean, predicted_observation_sigma_points, predicted_measurement, alpha_, beta_, lambda);
	MatrixXd K = cross_covariance * predicted_measurement_uncertainty.partialPivLu().inverse();
	std::cout << "K:\n" << K << std::endl;
	graft_state_ = predicted_mean + K*(z - predicted_measurement);
	std::cout << "state:\n" << graft_state_ << std::endl;
	graft_covariance_ = predicted_covariance - K*predicted_measurement_uncertainty*K.transpose();

	std::cout << "cov:\n" << graft_covariance_ << std::endl;


	std::cout << "-------_________------------_________----------" << std::endl;

	return dt;
}

void GraftUKFVelocity::setTopics(std::vector<boost::shared_ptr<GraftSensor> >& topics){
	topics_ = topics;
}

void GraftUKFVelocity::setVelocityProcessNoise(boost::array<double, 4>& Q){
	Q_.resize(std::sqrt(Q.size()),std::sqrt(Q.size()));
	Q_.setZero();
	for(size_t i = 0; i < Q.size(); i++){
		Q_(i) = Q[i];
	}
}

