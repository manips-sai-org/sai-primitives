/**
 * @file ActiveObserver.h
 * @author William Chong (wmchong@stanford.edu)
 * @brief Kalman active observer class
 * @version 0.1
 * @date 2026-06-03
 *
 * @copyright Copyright (c) 2026
 *
 */

#ifndef SAI_PRIMITIVES_ACTIVE_OBSERVER_H_
#define SAI_PRIMITIVES_ACTIVE_OBSERVER_H_

#include <Eigen/Dense>

using namespace Eigen;

namespace SaiPrimitives {

/**
 * @brief Discrete-time Kalman Active Observer (AOB).
 *
 * Implements the AOB method from Cortesao, "On Kalman Active Observers",
 * J. Intell. Robot. Syst. 48, 131-155 (2007). The nominal plant is
 *
 *     x_k = A x_{k-1} + B u_{k-1}
 *     y_k = C x_k
 *
 * and the observer augments it with an active input disturbance p. The control
 * command is
 *
 *     u_k = r_k - L x_hat_k - p_hat_k
 *
 * where p_hat_k is the newest active state. For order N > 1, the active state
 * follows the autoregressive AOB-N evolution described in the paper.
 */
class ActiveObserver {
public:
	static constexpr int kDefaultOrder = 1;
	static constexpr double kDefaultStateProcessNoise = 1e-9;
	static constexpr double kDefaultActiveProcessNoise = 1e-6;
	static constexpr double kDefaultMeasurementNoise = 1e-6;

	ActiveObserver(
		const MatrixXd& state_transition,
		const MatrixXd& command_matrix,
		const MatrixXd& measurement_matrix,
		const MatrixXd& state_feedback_gain,
		const int active_state_order = kDefaultOrder);
	~ActiveObserver() = default;

	ActiveObserver() = delete;

	void setSystemMatrices(
		const MatrixXd& state_transition,
		const MatrixXd& command_matrix,
		const MatrixXd& measurement_matrix,
		const MatrixXd& state_feedback_gain);

	void setActiveStateOrder(const int active_state_order);

	void reInitialize();
	void setStateEstimate(const VectorXd& augmented_state_estimate);
	void setPlantStateEstimate(const VectorXd& plant_state_estimate);
	void setActiveStateEstimate(const VectorXd& active_state_estimate);
	void setCovariance(const MatrixXd& covariance);
	void setPreviousReference(const VectorXd& previous_reference);

	void setProcessNoiseCovariance(const MatrixXd& process_noise_covariance);
	void setProcessNoiseCovariances(
		const MatrixXd& state_process_noise_covariance,
		const MatrixXd& active_derivative_process_noise_covariance);
	void setMeasurementNoiseCovariance(
		const MatrixXd& measurement_noise_covariance);
	void setNoiseCovariances(
		const MatrixXd& state_process_noise_covariance,
		const MatrixXd& active_derivative_process_noise_covariance,
		const MatrixXd& measurement_noise_covariance);

	/**
	 * @brief Runs one AOB step and returns the compensated command.
	 *
	 * Prediction uses the internally stored previous reference. The command is
	 * computed with the supplied current reference and then stored as the previous
	 * reference for the next call.
	 */
	VectorXd update(
		const VectorXd& reference,
		const VectorXd& measurement);

	/**
	 * @brief Runs one AOB step with explicit prediction and control references.
	 */
	VectorXd update(
		const VectorXd& prediction_reference,
		const VectorXd& control_reference,
		const VectorXd& measurement);

	void predict(const VectorXd& reference);
	void correct(const VectorXd& measurement);
	VectorXd computeControlCommand(const VectorXd& reference) const;

	int getOrder() const { return _order; }
	int getPlantStateDim() const { return _state_dim; }
	int getInputDim() const { return _input_dim; }
	int getMeasurementDim() const { return _measurement_dim; }
	int getActiveStateDim() const { return _active_dim; }
	int getAugmentedStateDim() const { return _augmented_dim; }

	const MatrixXd& getStateTransitionMatrix() const {
		return _state_transition;
	}
	const MatrixXd& getCommandMatrix() const { return _command_matrix; }
	const MatrixXd& getMeasurementMatrix() const { return _measurement_matrix; }
	const MatrixXd& getStateFeedbackGain() const {
		return _state_feedback_gain;
	}
	const MatrixXd& getOpenLoopTransitionMatrix() const {
		return _open_loop_transition;
	}
	const MatrixXd& getClosedLoopTransitionMatrix() const {
		return _closed_loop_transition;
	}
	const MatrixXd& getActiveTransitionMatrix() const {
		return _active_transition;
	}
	const MatrixXd& getAugmentedMeasurementMatrix() const {
		return _augmented_measurement_matrix;
	}
	const MatrixXd& getAugmentedFeedbackGain() const {
		return _augmented_feedback_gain;
	}
	const MatrixXd& getProcessNoiseCovariance() const {
		return _process_noise_covariance;
	}
	const MatrixXd& getMeasurementNoiseCovariance() const {
		return _measurement_noise_covariance;
	}
	const MatrixXd& getCovariance() const { return _covariance; }
	const MatrixXd& getKalmanGain() const { return _kalman_gain; }
	const MatrixXd& getInnovationCovariance() const {
		return _innovation_covariance;
	}
	const VectorXd& getStateEstimate() const { return _state_estimate; }
	const VectorXd& getPriorStateEstimate() const {
		return _prior_state_estimate;
	}
	const VectorXd& getLastInnovation() const { return _last_innovation; }
	const VectorXd& getLastControlCommand() const {
		return _last_control_command;
	}
	const VectorXd& getPreviousReference() const { return _previous_reference; }

	VectorXd getPlantStateEstimate() const;
	VectorXd getActiveStateEstimate() const;
	VectorXd getActiveStateHistory() const;

	/**
	 * @brief Scale from base active-state process variance to the AOB-N Q block.
	 *
	 * The AOB-N process covariance injects the variance of the (N-1)th evolution
	 * of the active state into the newest active-state block.
	 */
	static double activeDerivativeVarianceScale(
		const int active_state_order,
		const double relative_stability_factor = 1.0);

private:
	void validateSystemMatrices(
		const MatrixXd& state_transition,
		const MatrixXd& command_matrix,
		const MatrixXd& measurement_matrix,
		const MatrixXd& state_feedback_gain) const;
	void validateVectorSize(
		const VectorXd& vector,
		const int expected_size,
		const char* name) const;
	void validateMatrixSize(
		const MatrixXd& matrix,
		const int expected_rows,
		const int expected_cols,
		const char* name) const;

	void rebuildAugmentedMatrices();
	void resetNoiseCovariances();
	void resetEstimates();
	MatrixXd solveInnovationSystem(
		const MatrixXd& innovation_covariance,
		const MatrixXd& rhs) const;

	MatrixXd _state_transition;
	MatrixXd _command_matrix;
	MatrixXd _measurement_matrix;
	MatrixXd _state_feedback_gain;

	int _order;
	int _state_dim;
	int _input_dim;
	int _measurement_dim;
	int _active_dim;
	int _augmented_dim;

	MatrixXd _active_transition;
	MatrixXd _open_loop_transition;
	MatrixXd _closed_loop_transition;
	MatrixXd _reference_matrix;
	MatrixXd _augmented_measurement_matrix;
	MatrixXd _augmented_feedback_gain;

	MatrixXd _process_noise_covariance;
	MatrixXd _measurement_noise_covariance;
	MatrixXd _covariance;
	MatrixXd _kalman_gain;
	MatrixXd _innovation_covariance;

	VectorXd _state_estimate;
	VectorXd _prior_state_estimate;
	VectorXd _last_innovation;
	VectorXd _last_control_command;
	VectorXd _previous_reference;
};

}  // namespace SaiPrimitives

#endif	// SAI_PRIMITIVES_ACTIVE_OBSERVER_H_
