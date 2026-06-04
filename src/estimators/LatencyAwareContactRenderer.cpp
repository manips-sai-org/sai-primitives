/**
 * @file LatencyAwareContactRenderer.cpp
 * @brief Latency-aware predictive haptic contact rendering from a point cloud.
 */

#include "LatencyAwareContactRenderer.h"

#include <algorithm>
#include <cmath>
#include <stdexcept>

namespace {

constexpr double kEpsilon = 1e-12;

}  // namespace

namespace SaiPrimitives {

void LatencyAwareContactRenderer::setPointCloud(
	const std::vector<Eigen::Vector3d>& points,
	const std::vector<Eigen::Vector3d>& normals) {

	if (points.empty()) {
		throw std::invalid_argument(
			"LatencyAwareContactRenderer: point cloud must be non-empty.");
	}
	if (points.size() != normals.size()) {
		throw std::invalid_argument(
			"LatencyAwareContactRenderer: points and normals must have the same size.");
	}

	_points = points;
	_normals.resize(normals.size());
	for (std::size_t i = 0; i < normals.size(); ++i) {
		_normals[i] = normalizedNormal(normals[i]);
	}

	_learned_biases.assign(_points.size(), 0.0);
	_prediction_history.clear();
}

LatencyAwareContactRenderer::ForceResult
LatencyAwareContactRenderer::computeLatencyAwareHapticForce(
	const Query& query) {

	ensurePointCloudIsSet();

	const double latency = std::max(0.0, query.latency);
	const int path_samples = std::max(1, query.path_samples);
	_last_latency = latency;

	ForceResult result;
	result.phi_tau = std::numeric_limits<double>::infinity();

	for (int i = 0; i < path_samples; ++i) {
		const double sample_ratio =
			(path_samples == 1) ?
				0.0 :
				static_cast<double>(i) /
					static_cast<double>(path_samples - 1);
		const double s_i = latency * sample_ratio;
		const Eigen::Vector3d x_i =
			query.robot_position + s_i * query.robot_velocity;
		const double phi_i = evaluatePhiHat(x_i);

		if (phi_i < result.phi_tau) {
			result.phi_tau = phi_i;
			result.s_star = s_i;
			result.x_star = x_i;
		}
	}

	result.normal = evaluateGradientPhiHat(result.x_star);
	const Eigen::Vector3d approach_direction =
		normalizedOrFallback(
			query.approach_direction,
			normalizedOrFallback(query.robot_velocity, -result.normal));

	appendPredictionRecord(
		PredictionRecord{
			query.time,
			result.x_star,
			result.s_star,
			result.phi_tau,
			result.normal,
			approach_direction});

	if (result.phi_tau >= 0.0) {
		result.force.setZero();
		result.in_contact = false;
		result.penetration = 0.0;
		result.eta = 1.0;
		return result;
	}

	result.in_contact = true;
	result.penetration = -result.phi_tau;
	if (query.use_time_to_contact_scaling && latency > kEpsilon) {
		result.eta = std::clamp(
			1.0 - result.s_star / latency,
			0.0,
			1.0);
	} else {
		result.eta = 1.0;
	}

	result.force =
		result.eta * query.stiffness * result.penetration * result.normal -
		query.damping * query.haptic_velocity;

	const double max_force = std::max(0.0, query.max_force);
	const double force_norm = result.force.norm();
	if (max_force > 0.0 && force_norm > max_force) {
		result.force *= max_force / force_norm;
	}

	return result;
}

bool LatencyAwareContactRenderer::updateFromRobotContact(
	const RobotContact& contact) {

	ensurePointCloudIsSet();
	if (_prediction_history.empty()) {
		return false;
	}

	const double latency =
		std::isfinite(contact.latency) ?
			std::max(0.0, contact.latency) :
			_last_latency;
	const double target_prediction_time =
		contact.contact_time - latency;
	const PredictionRecord* prediction =
		findPredictionClosestTo(target_prediction_time);
	if (prediction == nullptr) {
		return false;
	}

	const Eigen::Vector3d approach_direction =
		normalizedOrFallback(
			contact.approach_direction,
			normalizedOrFallback(
				prediction->approach_direction,
				normalizedOrFallback(contact.contact_normal, prediction->normal)));
	const double d_error =
		approach_direction.dot(contact.contact_point - prediction->x_star);

	const double radius = 3.0 * _learning_sigma;
	const std::vector<std::size_t> neighbors =
		neighborIndicesWithinRadius(contact.contact_point, radius);
	if (neighbors.empty()) {
		return false;
	}

	const double two_sigma_squared =
		2.0 * _learning_sigma * _learning_sigma;
	for (const std::size_t point_index : neighbors) {
		const double distance_squared =
			(_points[point_index] - contact.contact_point).squaredNorm();
		const double weight =
			std::exp(-distance_squared / two_sigma_squared);

		_learned_biases[point_index] =
			std::clamp(
				_learned_biases[point_index] -
					_learning_rate * weight * d_error,
				_min_bias,
				_max_bias);
	}

	return true;
}

double LatencyAwareContactRenderer::evaluatePhi0(
	const Eigen::Vector3d& x) const {

	ensurePointCloudIsSet();

	const std::size_t nearest_index = nearestPointIndex(x);
	return _normals[nearest_index].dot(x - _points[nearest_index]);
}

double LatencyAwareContactRenderer::evaluateLearnedBias(
	const Eigen::Vector3d& x) const {

	ensurePointCloudIsSet();

	const double radius = 3.0 * _learning_sigma;
	const std::vector<std::size_t> neighbors =
		neighborIndicesWithinRadius(x, radius);
	if (neighbors.empty()) {
		return _learned_biases[nearestPointIndex(x)];
	}

	const double two_sigma_squared =
		2.0 * _learning_sigma * _learning_sigma;
	double weighted_bias = 0.0;
	double weight_sum = 0.0;
	for (const std::size_t point_index : neighbors) {
		const double distance_squared =
			(_points[point_index] - x).squaredNorm();
		const double weight =
			std::exp(-distance_squared / two_sigma_squared);
		weighted_bias += weight * _learned_biases[point_index];
		weight_sum += weight;
	}

	if (weight_sum <= kEpsilon) {
		return _learned_biases[nearestPointIndex(x)];
	}
	return weighted_bias / weight_sum;
}

double LatencyAwareContactRenderer::evaluatePhiHat(
	const Eigen::Vector3d& x) const {

	return evaluatePhi0(x) -
		   evaluateLearnedBias(x) -
		   _uncertainty_margin;
}

Eigen::Vector3d LatencyAwareContactRenderer::evaluateGradientPhiHat(
	const Eigen::Vector3d& x) const {

	ensurePointCloudIsSet();
	return _normals[nearestPointIndex(x)];
}

void LatencyAwareContactRenderer::setUncertaintyMargin(
	const double uncertainty_margin) {

	if (uncertainty_margin < 0.0) {
		throw std::invalid_argument(
			"LatencyAwareContactRenderer: uncertainty margin must be non-negative.");
	}
	_uncertainty_margin = uncertainty_margin;
}

void LatencyAwareContactRenderer::setLearningRate(
	const double learning_rate) {

	if (learning_rate < 0.0) {
		throw std::invalid_argument(
			"LatencyAwareContactRenderer: learning rate must be non-negative.");
	}
	_learning_rate = learning_rate;
}

void LatencyAwareContactRenderer::setLearningSigma(
	const double learning_sigma) {

	if (learning_sigma <= 0.0) {
		throw std::invalid_argument(
			"LatencyAwareContactRenderer: learning sigma must be positive.");
	}
	_learning_sigma = learning_sigma;
}

void LatencyAwareContactRenderer::setBiasClamp(
	const double min_bias,
	const double max_bias) {

	if (min_bias > max_bias) {
		throw std::invalid_argument(
			"LatencyAwareContactRenderer: min_bias must be <= max_bias.");
	}
	_min_bias = min_bias;
	_max_bias = max_bias;
	for (double& bias : _learned_biases) {
		bias = std::clamp(bias, _min_bias, _max_bias);
	}
}

void LatencyAwareContactRenderer::setMaxHistoryDuration(
	const double max_history_duration) {

	if (max_history_duration < 0.0) {
		throw std::invalid_argument(
			"LatencyAwareContactRenderer: max history duration must be non-negative.");
	}
	_max_history_duration = max_history_duration;
}

void LatencyAwareContactRenderer::clearLearnedBias() {
	std::fill(_learned_biases.begin(), _learned_biases.end(), 0.0);
}

void LatencyAwareContactRenderer::clearPredictionHistory() {
	_prediction_history.clear();
}

void LatencyAwareContactRenderer::ensurePointCloudIsSet() const {
	if (_points.empty()) {
		throw std::runtime_error(
			"LatencyAwareContactRenderer: point cloud has not been set.");
	}
}

std::size_t LatencyAwareContactRenderer::nearestPointIndex(
	const Eigen::Vector3d& x) const {

	std::size_t nearest_index = 0;
	double nearest_distance_squared =
		(_points[0] - x).squaredNorm();

	for (std::size_t i = 1; i < _points.size(); ++i) {
		const double distance_squared =
			(_points[i] - x).squaredNorm();
		if (distance_squared < nearest_distance_squared) {
			nearest_distance_squared = distance_squared;
			nearest_index = i;
		}
	}

	return nearest_index;
}

std::vector<std::size_t>
LatencyAwareContactRenderer::neighborIndicesWithinRadius(
	const Eigen::Vector3d& x,
	const double radius) const {

	const double radius_squared = radius * radius;
	std::vector<std::size_t> neighbors;
	for (std::size_t i = 0; i < _points.size(); ++i) {
		if ((_points[i] - x).squaredNorm() <= radius_squared) {
			neighbors.push_back(i);
		}
	}
	return neighbors;
}

const LatencyAwareContactRenderer::PredictionRecord*
LatencyAwareContactRenderer::findPredictionClosestTo(
	const double target_time) const {

	if (_prediction_history.empty()) {
		return nullptr;
	}

	const PredictionRecord* best_record = &_prediction_history.front();
	double best_time_error =
		std::abs(best_record->time - target_time);
	for (const PredictionRecord& record : _prediction_history) {
		const double time_error =
			std::abs(record.time - target_time);
		if (time_error < best_time_error) {
			best_time_error = time_error;
			best_record = &record;
		}
	}

	return best_record;
}

void LatencyAwareContactRenderer::appendPredictionRecord(
	const PredictionRecord& record) {

	_prediction_history.push_back(record);
	if (_max_history_duration <= 0.0) {
		return;
	}

	while (!_prediction_history.empty() &&
		   record.time - _prediction_history.front().time >
			   _max_history_duration) {
		_prediction_history.pop_front();
	}
}

Eigen::Vector3d LatencyAwareContactRenderer::normalizedOrFallback(
	const Eigen::Vector3d& vector,
	const Eigen::Vector3d& fallback) const {

	if (vector.norm() > kEpsilon) {
		return vector.normalized();
	}
	if (fallback.norm() > kEpsilon) {
		return fallback.normalized();
	}
	return Eigen::Vector3d::UnitZ();
}

Eigen::Vector3d LatencyAwareContactRenderer::normalizedNormal(
	const Eigen::Vector3d& normal) const {

	if (normal.norm() > kEpsilon) {
		return normal.normalized();
	}
	return Eigen::Vector3d::UnitZ();
}

}  // namespace SaiPrimitives
