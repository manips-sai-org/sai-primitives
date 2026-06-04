/**
 * @file ActiveObserver.cpp
 * @author William Chong (wmchong@stanford.edu)
 * @brief Kalman active observer class
 * @version 0.1
 * @date 2026-06-03
 *
 * @copyright Copyright (c) 2026
 *
 */

#include "ActiveObserver.h"

#include <algorithm>
#include <cmath>
#include <stdexcept>
#include <string>

namespace {

double binomialCoefficient(const int n, const int k) {
	if (k < 0 || k > n) {
		return 0.0;
	}

	const int kk = std::min(k, n - k);
	double value = 1.0;
	for (int i = 1; i <= kk; ++i) {
		value *= static_cast<double>(n - kk + i);
		value /= static_cast<double>(i);
	}
	return value;
}

void symmetrize(MatrixXd& matrix) {
	matrix = 0.5 * (matrix + matrix.transpose());
}

}  // namespace

namespace SaiPrimitives {

ActiveObserver::ActiveObserver(
	const MatrixXd& state_transition,
	const MatrixXd& command_matrix,
	const MatrixXd& measurement_matrix,
	const MatrixXd& state_feedback_gain,
	const int active_state_order)
	: _order(active_state_order) {
	setSystemMatrices(
		state_transition,
		command_matrix,
		measurement_matrix,
		state_feedback_gain);
}

void ActiveObserver::setSystemMatrices(
	const MatrixXd& state_transition,
	const MatrixXd& command_matrix,
	const MatrixXd& measurement_matrix,
	const MatrixXd& state_feedback_gain) {

	validateSystemMatrices(
		state_transition,
		command_matrix,
		measurement_matrix,
		state_feedback_gain);

	if (_order < 1) {
		throw std::invalid_argument(
			"ActiveObserver: active_state_order must be positive.");
	}

	_state_transition = state_transition;
	_command_matrix = command_matrix;
	_measurement_matrix = measurement_matrix;
	_state_feedback_gain = state_feedback_gain;

	_state_dim = static_cast<int>(_state_transition.rows());
	_input_dim = static_cast<int>(_command_matrix.cols());
	_measurement_dim = static_cast<int>(_measurement_matrix.rows());

	rebuildAugmentedMatrices();
	resetNoiseCovariances();
	resetEstimates();
}

void ActiveObserver::setActiveStateOrder(const int active_state_order) {
	if (active_state_order < 1) {
		throw std::invalid_argument(
			"ActiveObserver: active_state_order must be positive.");
	}

	_order = active_state_order;
	rebuildAugmentedMatrices();
	resetNoiseCovariances();
	resetEstimates();
}

void ActiveObserver::reInitialize() {
	resetEstimates();
}

void ActiveObserver::setStateEstimate(
	const VectorXd& augmented_state_estimate) {

	validateVectorSize(
		augmented_state_estimate,
		_augmented_dim,
		"augmented_state_estimate");

	_state_estimate = augmented_state_estimate;
	_prior_state_estimate = _state_estimate;
}

void ActiveObserver::setPlantStateEstimate(
	const VectorXd& plant_state_estimate) {

	validateVectorSize(
		plant_state_estimate,
		_state_dim,
		"plant_state_estimate");

	_state_estimate.head(_state_dim) = plant_state_estimate;
	_prior_state_estimate = _state_estimate;
}

void ActiveObserver::setActiveStateEstimate(
	const VectorXd& active_state_estimate) {

	validateVectorSize(
		active_state_estimate,
		_input_dim,
		"active_state_estimate");

	_state_estimate.segment(
		_state_dim + (_order - 1) * _input_dim,
		_input_dim) = active_state_estimate;
	_prior_state_estimate = _state_estimate;
}

void ActiveObserver::setCovariance(const MatrixXd& covariance) {
	validateMatrixSize(covariance, _augmented_dim, _augmented_dim, "covariance");

	_covariance = covariance;
	symmetrize(_covariance);
}

void ActiveObserver::setPreviousReference(const VectorXd& previous_reference) {
	validateVectorSize(previous_reference, _input_dim, "previous_reference");

	_previous_reference = previous_reference;
}

void ActiveObserver::setProcessNoiseCovariance(
	const MatrixXd& process_noise_covariance) {

	validateMatrixSize(
		process_noise_covariance,
		_augmented_dim,
		_augmented_dim,
		"process_noise_covariance");

	_process_noise_covariance = process_noise_covariance;
	symmetrize(_process_noise_covariance);
}

void ActiveObserver::setProcessNoiseCovariances(
	const MatrixXd& state_process_noise_covariance,
	const MatrixXd& active_derivative_process_noise_covariance) {

	validateMatrixSize(
		state_process_noise_covariance,
		_state_dim,
		_state_dim,
		"state_process_noise_covariance");
	validateMatrixSize(
		active_derivative_process_noise_covariance,
		_input_dim,
		_input_dim,
		"active_derivative_process_noise_covariance");

	_process_noise_covariance.setZero();
	_process_noise_covariance.topLeftCorner(_state_dim, _state_dim) =
		state_process_noise_covariance;
	_process_noise_covariance.block(
		_state_dim + (_order - 1) * _input_dim,
		_state_dim + (_order - 1) * _input_dim,
		_input_dim,
		_input_dim) = active_derivative_process_noise_covariance;
	symmetrize(_process_noise_covariance);
}

void ActiveObserver::setMeasurementNoiseCovariance(
	const MatrixXd& measurement_noise_covariance) {

	validateMatrixSize(
		measurement_noise_covariance,
		_measurement_dim,
		_measurement_dim,
		"measurement_noise_covariance");

	_measurement_noise_covariance = measurement_noise_covariance;
	symmetrize(_measurement_noise_covariance);
}

void ActiveObserver::setNoiseCovariances(
	const MatrixXd& state_process_noise_covariance,
	const MatrixXd& active_derivative_process_noise_covariance,
	const MatrixXd& measurement_noise_covariance) {

	setProcessNoiseCovariances(
		state_process_noise_covariance,
		active_derivative_process_noise_covariance);
	setMeasurementNoiseCovariance(measurement_noise_covariance);
}

VectorXd ActiveObserver::update(
	const VectorXd& reference,
	const VectorXd& measurement) {

	return update(_previous_reference, reference, measurement);
}

VectorXd ActiveObserver::update(
	const VectorXd& prediction_reference,
	const VectorXd& control_reference,
	const VectorXd& measurement) {

	predict(prediction_reference);
	correct(measurement);

	_last_control_command = computeControlCommand(control_reference);
	_previous_reference = control_reference;

	return _last_control_command;
}

void ActiveObserver::predict(const VectorXd& reference) {
	validateVectorSize(reference, _input_dim, "reference");

	_prior_state_estimate =
		_closed_loop_transition * _state_estimate +
		_reference_matrix * reference;
	_state_estimate = _prior_state_estimate;

	_covariance =
		_open_loop_transition * _covariance *
			_open_loop_transition.transpose() +
		_process_noise_covariance;
	symmetrize(_covariance);
}

void ActiveObserver::correct(const VectorXd& measurement) {
	validateVectorSize(measurement, _measurement_dim, "measurement");

	_last_innovation =
		measurement - _augmented_measurement_matrix * _state_estimate;
	_innovation_covariance =
		_augmented_measurement_matrix * _covariance *
			_augmented_measurement_matrix.transpose() +
		_measurement_noise_covariance;
	symmetrize(_innovation_covariance);

	const MatrixXd covariance_measurement_transpose =
		_covariance * _augmented_measurement_matrix.transpose();
	_kalman_gain =
		solveInnovationSystem(
			_innovation_covariance,
			covariance_measurement_transpose.transpose())
			.transpose();

	_state_estimate += _kalman_gain * _last_innovation;

	const MatrixXd identity =
		MatrixXd::Identity(_augmented_dim, _augmented_dim);
	const MatrixXd correction =
		identity - _kalman_gain * _augmented_measurement_matrix;
	_covariance =
		correction * _covariance * correction.transpose() +
		_kalman_gain * _measurement_noise_covariance * _kalman_gain.transpose();
	symmetrize(_covariance);
}

VectorXd ActiveObserver::computeControlCommand(
	const VectorXd& reference) const {

	validateVectorSize(reference, _input_dim, "reference");

	return reference - _augmented_feedback_gain * _state_estimate;
}

VectorXd ActiveObserver::getPlantStateEstimate() const {
	return _state_estimate.head(_state_dim);
}

VectorXd ActiveObserver::getActiveStateEstimate() const {
	return _state_estimate.segment(
		_state_dim + (_order - 1) * _input_dim,
		_input_dim);
}

VectorXd ActiveObserver::getActiveStateHistory() const {
	return _state_estimate.tail(_active_dim);
}

double ActiveObserver::activeDerivativeVarianceScale(
	const int active_state_order,
	const double relative_stability_factor) {

	if (active_state_order < 1) {
		throw std::invalid_argument(
			"ActiveObserver: active_state_order must be positive.");
	}
	if (relative_stability_factor <= 0.0 ||
		relative_stability_factor > 1.0) {
		throw std::invalid_argument(
			"ActiveObserver: relative_stability_factor must be in (0, 1].");
	}

	const int evolution_order = active_state_order - 1;
	double scale = 0.0;
	for (int j = 0; j <= evolution_order; ++j) {
		const double coefficient =
			binomialCoefficient(evolution_order, j);
		scale += coefficient * coefficient;
	}

	return scale * std::pow(
		relative_stability_factor,
		static_cast<double>(active_state_order));
}

void ActiveObserver::validateSystemMatrices(
	const MatrixXd& state_transition,
	const MatrixXd& command_matrix,
	const MatrixXd& measurement_matrix,
	const MatrixXd& state_feedback_gain) const {

	if (state_transition.rows() == 0 || state_transition.cols() == 0 ||
		command_matrix.rows() == 0 || command_matrix.cols() == 0 ||
		measurement_matrix.rows() == 0 || measurement_matrix.cols() == 0 ||
		state_feedback_gain.rows() == 0 || state_feedback_gain.cols() == 0) {
		throw std::invalid_argument(
			"ActiveObserver: system matrices must be non-empty.");
	}

	if (state_transition.rows() != state_transition.cols()) {
		throw std::invalid_argument(
			"ActiveObserver: state_transition must be square.");
	}
	if (command_matrix.rows() != state_transition.rows()) {
		throw std::invalid_argument(
			"ActiveObserver: command_matrix row count must match state dimension.");
	}
	if (measurement_matrix.cols() != state_transition.rows()) {
		throw std::invalid_argument(
			"ActiveObserver: measurement_matrix column count must match state dimension.");
	}
	if (state_feedback_gain.rows() != command_matrix.cols() ||
		state_feedback_gain.cols() != state_transition.rows()) {
		throw std::invalid_argument(
			"ActiveObserver: state_feedback_gain must be input_dim x state_dim.");
	}
}

void ActiveObserver::validateVectorSize(
	const VectorXd& vector,
	const int expected_size,
	const char* name) const {

	if (vector.size() != expected_size) {
		throw std::invalid_argument(
			std::string("ActiveObserver: invalid size for ") + name + ".");
	}
}

void ActiveObserver::validateMatrixSize(
	const MatrixXd& matrix,
	const int expected_rows,
	const int expected_cols,
	const char* name) const {

	if (matrix.rows() != expected_rows || matrix.cols() != expected_cols) {
		throw std::invalid_argument(
			std::string("ActiveObserver: invalid size for ") + name + ".");
	}
}

void ActiveObserver::rebuildAugmentedMatrices() {
	_active_dim = _order * _input_dim;
	_augmented_dim = _state_dim + _active_dim;

	_active_transition =
		MatrixXd::Zero(_active_dim, _active_dim);
	const MatrixXd input_identity =
		MatrixXd::Identity(_input_dim, _input_dim);

	for (int i = 0; i < _order - 1; ++i) {
		_active_transition.block(
			i * _input_dim,
			(i + 1) * _input_dim,
			_input_dim,
			_input_dim) = input_identity;
	}

	for (int col = 0; col < _order; ++col) {
		const int coefficient_index = _order - col;
		const double sign =
			(coefficient_index % 2 == 0) ? -1.0 : 1.0;
		const double coefficient =
			sign * binomialCoefficient(_order, coefficient_index);
		_active_transition.block(
			(_order - 1) * _input_dim,
			col * _input_dim,
			_input_dim,
			_input_dim) = coefficient * input_identity;
	}

	_open_loop_transition =
		MatrixXd::Zero(_augmented_dim, _augmented_dim);
	_open_loop_transition.topLeftCorner(_state_dim, _state_dim) =
		_state_transition;
	_open_loop_transition.block(
		0,
		_state_dim + (_order - 1) * _input_dim,
		_state_dim,
		_input_dim) = _command_matrix;
	_open_loop_transition.bottomRightCorner(_active_dim, _active_dim) =
		_active_transition;

	_closed_loop_transition =
		MatrixXd::Zero(_augmented_dim, _augmented_dim);
	_closed_loop_transition.topLeftCorner(_state_dim, _state_dim) =
		_state_transition - _command_matrix * _state_feedback_gain;
	_closed_loop_transition.bottomRightCorner(_active_dim, _active_dim) =
		_active_transition;

	_reference_matrix =
		MatrixXd::Zero(_augmented_dim, _input_dim);
	_reference_matrix.topRows(_state_dim) = _command_matrix;

	_augmented_measurement_matrix =
		MatrixXd::Zero(_measurement_dim, _augmented_dim);
	_augmented_measurement_matrix.leftCols(_state_dim) =
		_measurement_matrix;

	_augmented_feedback_gain =
		MatrixXd::Zero(_input_dim, _augmented_dim);
	_augmented_feedback_gain.leftCols(_state_dim) = _state_feedback_gain;
	_augmented_feedback_gain.block(
		0,
		_state_dim + (_order - 1) * _input_dim,
		_input_dim,
		_input_dim) = input_identity;
}

void ActiveObserver::resetNoiseCovariances() {
	const MatrixXd state_process_noise_covariance =
		kDefaultStateProcessNoise *
		MatrixXd::Identity(_state_dim, _state_dim);
	const MatrixXd active_derivative_process_noise_covariance =
		kDefaultActiveProcessNoise *
		MatrixXd::Identity(_input_dim, _input_dim);

	_process_noise_covariance =
		MatrixXd::Zero(_augmented_dim, _augmented_dim);
	setProcessNoiseCovariances(
		state_process_noise_covariance,
		active_derivative_process_noise_covariance);

	_measurement_noise_covariance =
		kDefaultMeasurementNoise *
		MatrixXd::Identity(_measurement_dim, _measurement_dim);
}

void ActiveObserver::resetEstimates() {
	_state_estimate = VectorXd::Zero(_augmented_dim);
	_prior_state_estimate = VectorXd::Zero(_augmented_dim);
	_last_innovation = VectorXd::Zero(_measurement_dim);
	_last_control_command = VectorXd::Zero(_input_dim);
	_previous_reference = VectorXd::Zero(_input_dim);
	_kalman_gain = MatrixXd::Zero(_augmented_dim, _measurement_dim);
	_innovation_covariance =
		MatrixXd::Zero(_measurement_dim, _measurement_dim);
	_covariance = _process_noise_covariance;
}

MatrixXd ActiveObserver::solveInnovationSystem(
	const MatrixXd& innovation_covariance,
	const MatrixXd& rhs) const {

	LDLT<MatrixXd> ldlt(innovation_covariance);
	if (ldlt.info() == Success) {
		return ldlt.solve(rhs);
	}

	return innovation_covariance.completeOrthogonalDecomposition().solve(rhs);
}

}  // namespace SaiPrimitives
