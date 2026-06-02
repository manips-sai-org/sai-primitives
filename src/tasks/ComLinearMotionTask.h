/*
 * ComLinearMotionTask.h
 *
 *      Linear-only center-of-mass motion/force task.
 */

#ifndef SAI_PRIMITIVES_COMLINEARMOTIONTASK_TASK_H_
#define SAI_PRIMITIVES_COMLINEARMOTIONTASK_TASK_H_

#include <helper_modules/OTG_6dof_cartesian.h>
#include <helper_modules/POPCExplicitForceControl.h>
#include <helper_modules/SaiPrimitivesCommonDefinitions.h>

#include <Eigen/Dense>
#include <iostream>
#include <memory>
#include <string>

#include "SaiModel.h"
#include "TemplateTask.h"

using namespace Eigen;
using namespace std;

namespace SaiPrimitives {

class ComLinearMotionTask : public TemplateTask {

struct DefaultParameters {
	static constexpr DynamicDecouplingType dynamic_decoupling_type =
		DynamicDecouplingType::BOUNDED_INERTIA_ESTIMATES;
	static constexpr double bie_threshold = 0.15;
	static constexpr double s_abs_tol = 1e-4;
};

public:
	ComLinearMotionTask(
		std::shared_ptr<SaiModel::SaiModel>& robot, const string& link_name = "",
		const Affine3d& compliant_frame = Affine3d::Identity(),
		const std::string& task_name = "com_linear_motion_task",
		const bool is_force_motion_parametrization_in_compliant_frame = false,
		const double loop_timestep = 0.001);

	ComLinearMotionTask(
		std::shared_ptr<SaiModel::SaiModel>& robot, const string& link_name,
		std::vector<Vector3d> controlled_directions_translation,
		const Affine3d& compliant_frame = Affine3d::Identity(),
		const std::string& task_name = "com_linear_motion_task",
		const bool is_force_motion_parametrization_in_compliant_frame = false,
		const double loop_timestep = 0.001);

	const Vector3d& getCurrentPosition() const { return _current_position; }
	const Vector3d& getCurrentLinearVelocity() const {
		return _current_linear_velocity;
	}
	const Vector3d& getSensedForce() const { return _sensed_force; }

	MatrixXd getTaskNullspace() const override { return _N; }
	MatrixXd getPreviousTasksNullspace() const override { return _N_prec; }
	MatrixXd getTaskAndPreviousNullspace() const override {
		return _N * _N_prec;
	}

	void setGoalPosition(const Vector3d& goal_position) {
		_goal_position = goal_position;
	}
	const Vector3d& getGoalPosition() const { return _goal_position; }

	void setGoalLinearVelocity(const Vector3d& goal_linvel) {
		_goal_linear_velocity = goal_linvel;
	}
	const Vector3d& getGoalLinearVelocity() const {
		return _goal_linear_velocity;
	}

	void setGoalLinearAcceleration(const Vector3d& goal_linaccel) {
		_goal_linear_acceleration = goal_linaccel;
	}
	const Vector3d& getGoalLinearAcceleration() const {
		return _goal_linear_acceleration;
	}

	const VectorXd& getUnitMassForce() const { return _unit_mass_force; }

	Vector3d getPositionError() const;
	Vector3d getLinearVelocityError() const;

	void setPosControlGains(const PIDGains& gains) {
		setPosControlGains(gains.kp, gains.kv, gains.ki);
	}
	void setPosControlGains(double kp_pos, double kv_pos, double ki_pos = 0);
	void setPosControlGains(const Vector3d& kp_pos, const Vector3d& kv_pos,
							const Vector3d& ki_pos = Vector3d::Zero());
	void setPosControlGains(const VectorXd& kp_pos, const VectorXd& kv_pos,
							const VectorXd& ki_pos);
	void setPosControlGains(const VectorXd& kp_pos, const VectorXd& kv_pos) {
		setPosControlGains(kp_pos, kv_pos, VectorXd::Zero(kp_pos.size()));
	}
	vector<PIDGains> getPosControlGains() const;

	void setForceControlGains(const PIDGains& gains) {
		setForceControlGains(gains.kp, gains.kv, gains.ki);
	}
	void setForceControlGains(double kp_force, double kv_force,
							  double ki_force) {
		_kp_force = kp_force * Matrix3d::Identity();
		_kv_force = kv_force * Matrix3d::Identity();
		_ki_force = ki_force * Matrix3d::Identity();
	}
	vector<PIDGains> getForceControlGains() const {
		return vector<PIDGains>(
			1, PIDGains(_kp_force(0, 0), _kv_force(0, 0), _ki_force(0, 0)));
	}

	void setGoalForce(const Vector3d& goal_force) { _goal_force = goal_force; }
	Vector3d getGoalForce() const;

	void enableInternalOtgAccelerationLimited(
		const double max_linear_velocity,
		const double max_linear_acceleration);
	void enableInternalOtgJerkLimited(const double max_linear_velocity,
									  const double max_linear_acceleration,
									  const double max_linear_jerk);
	void disableInternalOtg() { _use_internal_otg_flag = false; }
	bool getInternalOtgEnabled() const { return _use_internal_otg_flag; }
	const OTG_6dof_cartesian& getInternalOtg() const { return *_otg; }

	/**
	 * @brief Enables Ruckig tracking mode for the internal OTG.
	 *
	 * In tracking mode, goal position, velocity and acceleration are passed to
	 * Ruckig Trackig to generate the desired motion target.
	 */
	void enableInternalOtgTrackingMode(
		const double reactiveness = 1.0,
		const size_t look_ahead_cycles = 1,
		const size_t max_iterations = 8,
		const TrackigMode mode = TrackigMode::Optimized) {
		_otg->enableTrackingMode(reactiveness, look_ahead_cycles,
								 max_iterations, mode);
	}

	/// @brief Disables Ruckig tracking mode for the internal OTG.
	void disableInternalOtgTrackingMode() { _otg->disableTrackingMode(); }

	/// @brief Getter for Ruckig tracking mode on the internal OTG.
	bool getInternalOtgTrackingModeEnabled() const {
		return _otg->getTrackingModeEnabled();
	}

	/**
	 * @brief Sets target linear velocity and acceleration limits for internal OTG
	 * tracking mode.
	 */
	void setInternalOtgTrackingTargetLimits(
		const double max_linear_velocity,
		const double max_linear_acceleration) {
		_otg->setTrackingTargetLimits(
			max_linear_velocity * Vector3d::Ones(),
			max_linear_acceleration * Vector3d::Ones(),
			_otg->getMaxAngularVelocity(),
			_otg->getMaxAngularAcceleration());
	}

	/// @brief Uses the regular internal OTG limits for tracking targets.
	void disableInternalOtgTrackingTargetLimits() {
		_otg->disableTrackingTargetLimits();
	}

	bool getInternalOtgTrackingTargetVelocityLimitsEnabled() const {
		return _otg->getTrackingTargetVelocityLimitsEnabled();
	}

	bool getInternalOtgTrackingTargetAccelerationLimitsEnabled() const {
		return _otg->getTrackingTargetAccelerationLimitsEnabled();
	}

	void enableVelocitySaturation(const double linear_vel_sat = 0.3);
	void disableVelocitySaturation() { _use_velocity_saturation_flag = false; }
	bool getVelocitySaturationEnabled() const {
		return _use_velocity_saturation_flag;
	}
	double getLinearSaturationVelocity() const {
		return _linear_saturation_velocity;
	}

	void updateTaskModel(const MatrixXd& N_prec) override;
	VectorXd computeTorques() override;
	VectorXd computeTorques(const Eigen::VectorXd& tau_prec) override;
	void reInitializeTask() override;

	bool goalPositionReached(const double tolerance,
							 const bool verbose = false);

	void setForceSensorFrame(const string link_name,
							 const Affine3d transformation_in_link);
	void updateSensedForce(const Vector3d sensed_force_sensor_frame);

	bool parametrizeForceMotionSpaces(
		const int force_space_dimension,
		const Vector3d& force_or_motion_single_axis = Vector3d::Zero());

	int getForceSpaceDimension() const { return _force_space_dimension; }
	Vector3d getForceMotionSingleAxis() const { return _force_or_motion_axis; }

	Matrix3d sigmaForce() const;
	Matrix3d sigmaPosition() const;

	void setClosedLoopForceControl(
		const bool closed_loop_force_control = true) {
		_closed_loop_force_control = closed_loop_force_control;
		resetIntegratorsLinear();
	}

	void enablePassivity() { _POPC_force->enable(); }
	void disablePassivity() { _POPC_force->disable(); }

	void resetIntegrators();
	void resetIntegratorsLinear();

	Matrix3d posSelectionProjector() const { return _partial_task_projection; }

	VectorXd getUnitControlForces() { return _unit_mass_force; }

	void setDynamicDecouplingType(const DynamicDecouplingType type) {
		_dynamic_decoupling_type = type;
	}
	void setBieThreshold(const double val) { _bie_threshold = val; }
	void setSingularityThreshold(const double val) { _s_abs_tol = val; }

private:
	void initialSetup();

	Vector3d _goal_position;
	Vector3d _goal_linear_velocity;
	Vector3d _goal_linear_acceleration;

	Matrix3d _kp_pos;
	Matrix3d _kv_pos;
	Matrix3d _ki_pos;

	Matrix3d _kp_force;
	Matrix3d _kv_force;
	Matrix3d _ki_force;

	Vector3d _goal_force;

	bool _use_velocity_saturation_flag;
	double _linear_saturation_velocity;

	bool _use_internal_otg_flag;
	std::unique_ptr<OTG_6dof_cartesian> _otg;

	Eigen::VectorXd _task_force;
	Eigen::MatrixXd _N_prec;

	string _link_name;
	Affine3d _compliant_frame;
	bool _is_force_motion_parametrization_in_compliant_frame;

	Vector3d _current_position;
	Vector3d _current_linear_velocity;
	Vector3d _integrated_position_error;

	Affine3d _T_control_to_sensor;
	Vector3d _sensed_force;
	Vector3d _integrated_force_error;

	int _force_space_dimension;
	Vector3d _force_or_motion_axis;

	bool _closed_loop_force_control;
	double _k_ff;

	std::unique_ptr<POPCExplicitForceControl> _POPC_force;

	Vector3d _linear_motion_control;
	Vector3d _linear_force_control;

	bool _are_pos_gains_isotropic;
	DynamicDecouplingType _dynamic_decoupling_type;

	MatrixXd _jacobian;
	MatrixXd _projected_jacobian;
	MatrixXd _Lambda_modified;
	MatrixXd _N;

	MatrixXd _current_task_range;
	int _pos_range;

	Matrix3d _partial_task_projection;

	VectorXd _unit_mass_force;

	double _s_abs_tol;
	double _bie_threshold;
};

} /* namespace SaiPrimitives */

#endif /* SAI_PRIMITIVES_COMLINEARMOTIONTASK_TASK_H_ */
