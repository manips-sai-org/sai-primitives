/**
 * @file LatencyAwareContactRenderer.h
 * @brief Latency-aware predictive haptic contact rendering from a point cloud.
 */

#ifndef SAI_PRIMITIVES_LATENCY_AWARE_CONTACT_RENDERER_H_
#define SAI_PRIMITIVES_LATENCY_AWARE_CONTACT_RENDERER_H_

#include <Eigen/Dense>

#include <cstddef>
#include <deque>
#include <limits>
#include <vector>

namespace SaiPrimitives {

/**
 * @brief Predictive haptic contact renderer using a point-cloud signed distance.
 *
 * The rendered signed distance is
 *
 *     phi_hat(x) = phi0(x) - b_learned(x) - b_uncertainty
 *
 * with phi0 approximated from the nearest point-cloud sample and its normal.
 * Brute-force nearest-neighbor search is used deliberately here; the search
 * helpers are isolated so a KD-tree can replace them later.
 */
class LatencyAwareContactRenderer {
public:
	struct Query {
		double time = 0.0;
		Eigen::Vector3d robot_position = Eigen::Vector3d::Zero();
		Eigen::Vector3d robot_velocity = Eigen::Vector3d::Zero();
		Eigen::Vector3d haptic_velocity = Eigen::Vector3d::Zero();
		Eigen::Vector3d approach_direction = Eigen::Vector3d::Zero();
		double latency = 0.0;
		int path_samples = 8;
		double stiffness = 1000.0;
		double damping = 8.0;
		double max_force = 30.0;
		bool use_time_to_contact_scaling = true;
	};

	struct ForceResult {
		Eigen::Vector3d force = Eigen::Vector3d::Zero();
		bool in_contact = false;
		double penetration = 0.0;
		double eta = 1.0;
		double phi_tau = std::numeric_limits<double>::infinity();
		double s_star = 0.0;
		Eigen::Vector3d x_star = Eigen::Vector3d::Zero();
		Eigen::Vector3d normal = Eigen::Vector3d::UnitZ();
	};

	struct RobotContact {
		double contact_time = 0.0;
		Eigen::Vector3d contact_point = Eigen::Vector3d::Zero();
		Eigen::Vector3d contact_normal = Eigen::Vector3d::UnitZ();
		Eigen::Vector3d approach_direction = Eigen::Vector3d::Zero();
		double latency = std::numeric_limits<double>::quiet_NaN();
	};

	struct PredictionRecord {
		double time = 0.0;
		Eigen::Vector3d x_star = Eigen::Vector3d::Zero();
		double s_star = 0.0;
		double phi_tau = std::numeric_limits<double>::infinity();
		Eigen::Vector3d normal = Eigen::Vector3d::UnitZ();
		Eigen::Vector3d approach_direction = Eigen::Vector3d::Zero();
	};

	LatencyAwareContactRenderer() = default;
	~LatencyAwareContactRenderer() = default;

	void setPointCloud(
		const std::vector<Eigen::Vector3d>& points,
		const std::vector<Eigen::Vector3d>& normals);

	ForceResult computeLatencyAwareHapticForce(const Query& query);
	bool updateFromRobotContact(const RobotContact& contact);

	double evaluatePhi0(const Eigen::Vector3d& x) const;
	double evaluateLearnedBias(const Eigen::Vector3d& x) const;
	double evaluatePhiHat(const Eigen::Vector3d& x) const;
	Eigen::Vector3d evaluateGradientPhiHat(const Eigen::Vector3d& x) const;

	void setUncertaintyMargin(const double uncertainty_margin);
	void setLearningRate(const double learning_rate);
	void setLearningSigma(const double learning_sigma);
	void setBiasClamp(const double min_bias, const double max_bias);
	void setMaxHistoryDuration(const double max_history_duration);

	void clearLearnedBias();
	void clearPredictionHistory();

	const std::vector<Eigen::Vector3d>& getPoints() const { return _points; }
	const std::vector<Eigen::Vector3d>& getNormals() const { return _normals; }
	const std::vector<double>& getLearnedBiases() const {
		return _learned_biases;
	}
	const std::deque<PredictionRecord>& getPredictionHistory() const {
		return _prediction_history;
	}

private:
	void ensurePointCloudIsSet() const;
	std::size_t nearestPointIndex(const Eigen::Vector3d& x) const;
	std::vector<std::size_t> neighborIndicesWithinRadius(
		const Eigen::Vector3d& x,
		const double radius) const;
	const PredictionRecord* findPredictionClosestTo(
		const double target_time) const;
	void appendPredictionRecord(const PredictionRecord& record);
	Eigen::Vector3d normalizedOrFallback(
		const Eigen::Vector3d& vector,
		const Eigen::Vector3d& fallback) const;
	Eigen::Vector3d normalizedNormal(
		const Eigen::Vector3d& normal) const;

	std::vector<Eigen::Vector3d> _points;
	std::vector<Eigen::Vector3d> _normals;
	std::vector<double> _learned_biases;
	std::deque<PredictionRecord> _prediction_history;

	double _uncertainty_margin = 0.002;
	double _learning_rate = 0.25;
	double _learning_sigma = 0.02;
	double _min_bias = -0.05;
	double _max_bias = 0.05;
	double _max_history_duration = 5.0;
	double _last_latency = 0.0;
};

}  // namespace SaiPrimitives

#endif	// SAI_PRIMITIVES_LATENCY_AWARE_CONTACT_RENDERER_H_
