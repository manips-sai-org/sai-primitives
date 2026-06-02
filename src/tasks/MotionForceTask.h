/*
 * MotionForceTask.h
 *
 *      This class creates a 6Dof position + orientation hybrid controller for a
 * robotic manipulator using operational space formulation and an underlying PID
 * compensator. If used for hybrid position force control, assumes a force
 * sensor is attached to the same link as the control frame and the force sensed
 * values are given in sensor frame. Besides, the force sensed and moment sensed
 * are assumed to be the force and moment that the robot applies to the
 * environment. It requires a robot model parsed from a urdf file to a SaiModel
 * object, as well as the definition of a control frame as a link at which the
 * frame is attached, and an affine transform that determines the position and
 * orientation of the control frame in this link
 *
 *      Author: Mikael Jorda
 */

#ifndef SAI_PRIMITIVES_MOTIONFORCETASK_TASK_H_
#define SAI_PRIMITIVES_MOTIONFORCETASK_TASK_H_

#include <helper_modules/OTG_6dof_cartesian.h>
#include <helper_modules/POPCExplicitForceControl.h>
#include <helper_modules/SaiPrimitivesCommonDefinitions.h>

#include <Eigen/Dense>
#include <memory>
#include <string>

#include "SaiModel.h"
#include "SingularityHandler.h"
#include "TemplateTask.h"

using namespace Eigen;
using namespace std;

namespace SaiPrimitives {

class MotionForceTask : public TemplateTask {
public:
	struct DefaultParameters {
		static constexpr DynamicDecouplingType dynamic_decoupling_type =
			DynamicDecouplingType::BOUNDED_INERTIA_ESTIMATES;
		static constexpr double bie_threshold = 0.1;
		static constexpr double kp_pos = 100.0;
		static constexpr double kv_pos = 20.0;
		static constexpr double ki_pos = 0.0;
		static constexpr double kp_ori = 200.0;
		static constexpr double kv_ori = 28.3;
		static constexpr double ki_ori = 0.0;
		static constexpr double kp_force = 0.7;
		static constexpr double kv_force = 10.0;
		static constexpr double ki_force = 1.3;
		static constexpr double kp_moment = 0.7;
		static constexpr double kv_moment = 10.0;
		static constexpr double ki_moment = 1.3;
		static constexpr double kff_force = 0.95;
		static constexpr double kff_moment = 0.95;
		static constexpr double max_force_control_feedback_output = 20.0;
		static constexpr double max_moment_control_feedback_output = 10.0;
		static constexpr bool closed_loop_force_control = false;
		static constexpr bool closed_loop_moment_control = false;
		static constexpr int force_space_dimension = 0;
		static constexpr int moment_space_dimension = 0;
		static constexpr bool use_velocity_saturation = false;
		static constexpr double linear_saturation_velocity = 0.3;
		static constexpr double angular_saturation_velocity = M_PI / 3;
		static constexpr bool use_internal_otg = true;
		static constexpr double otg_max_linear_velocity = 0.3;
		static constexpr double otg_max_linear_acceleration = 2.0;
		static constexpr double otg_max_angular_velocity = M_PI / 3;
		static constexpr double otg_max_angular_acceleration = 2.0 * M_PI;
		static constexpr bool internal_otg_jerk_limited = false;
		static constexpr double otg_max_linear_jerk = 10.0;
		static constexpr double otg_max_angular_jerk = 10.0 * M_PI;
		static constexpr double singularity_pos_exit_tol = 2e-2;
		static constexpr double singularity_ori_exit_tol = 10 * M_PI / 180;
		static constexpr double singularity_linear_vel_exit_tol = 1e-2;
		static constexpr double singularity_angular_vel_exit_tol = 1e-2;
		static constexpr double singularity_exit_velocity_scaling = 1.0;
		static constexpr double singularity_ramp_linear_acceleration = 0.2;
		static constexpr double singularity_ramp_angular_acceleration = M_PI / 3;
		static constexpr double singularity_bound = 5e-2;
	};

	//------------------------------------------------
	// Constructor
	//------------------------------------------------

	/**
	 * @brief Construct a new Motion Force Task
	 *
	 * @param robot A pointer to a SaiModel object for the robot that is to be
	 * controlled
	 * @param link_name The name of the link in the robot at which to attach the
	 * compliant frame
	 * @param compliant_frame Compliant frame with respect to link frame
	 * @param task_name Name of the task
	 * @param is_force_motion_parametrization_in_compliant_frame Whether the
	 * force and motion space (and potential non isotropic gains) are defined
	 * with respect to the compliant frame or the world frame
	 * @param loop_timestep Time taken by a control loop. Used in trajectory
	 * generation and integral control.
	 */
	MotionForceTask(
		std::shared_ptr<SaiModel::SaiModel>& robot, const string& link_name,
		const Affine3d& compliant_frame = Affine3d::Identity(),
		const std::string& task_name = "motion_force_task",
		const bool is_force_motion_parametrization_in_compliant_frame = false,
		const double loop_timestep = 0.001);

	MotionForceTask(
		std::shared_ptr<SaiModel::SaiModel>& robot, const string& link_name,
		std::vector<Vector3d> controlled_directions_translation,
		std::vector<Vector3d> controlled_directions_rotation,
		const Affine3d& compliant_frame = Affine3d::Identity(),
		const std::string& task_name = "partial_motion_force_task",
		const bool is_force_motion_parametrization_in_compliant_frame = false,
		const double loop_timestep = 0.001);

	//------------------------------------------------
	// Getters Setters
	//------------------------------------------------

	/**
	 * @brief Get the Current Position
	 *
	 * @return const Vector3d& current position of the control point
	 */
	const Vector3d& getCurrentPosition() const { return _current_position; }
	/**
	 * @brief Get the Current Velocity
	 *
	 * @return const Vector3d& current velocity of the control point
	 */
	const Vector3d& getCurrentLinearVelocity() const {
		return _current_linear_velocity;
	}

	/**
	 * @brief Get the Current Orientation
	 *
	 * @return const Matrix3d& current orientation of the control frame
	 */
	const Matrix3d& getCurrentOrientation() const {
		return _current_orientation;
	}

	/**
	 * @brief Get the Current Angular Velocity
	 *
	 * @return const Vector3d& current angular velocity of the control frame
	 */
	const Vector3d& getCurrentAngularVelocity() const {
		return _current_angular_velocity;
	}

	/**
	 * @brief Get the current pose
	 * 
	 * @return Affine3d pose of control frame on robot
	 */
	Affine3d getCurrentPose() const {
		Affine3d pose;
		pose.linear() = _current_orientation;
		pose.translation() = _current_position;
		return pose;
	}

	/**
	 * @brief Get the Sensed Force used for control (resolved at the origin of
	 * the compliant frame), in robot world frame
	 *
	 * @return const Vector3d& sensed force used for control
	 */
	const Vector3d& getSensedForceControlWorldFrame() const {
		return _sensed_force_control_world_frame;
	}

	/**
	 * @brief Get the Sensed Moment used for control, in robot world frame
	 *
	 * @return const Vector3d& sensed moment used for control
	 */
	const Vector3d& getSensedMomentControlWorldFrame() const {
		return _sensed_moment_control_world_frame;
	}

	/**
	 * @brief Get the Sensed Force used for control as given directly by the
	 * sensor, in sensor frame
	 *
	 * @return const Vector3d& sensed force from sensor
	 */
	const Vector3d& getSensedForceSensor() const {
		return _sensed_force_sensor_frame;
	}

	/**
	 * @brief Get the Sensed Moment used for control as given directly by the
	 * sensor, in sensor frame
	 *
	 * @return const Vector3d& sensed moment from sensor
	 */
	const Vector3d& getSensedMomentSensor() const {
		return _sensed_moment_sensor_frame;
	}

	/**
	 * @brief Get the nullspace of this task associated with the constrained,
	 * reduced jacobian
	 *
	 * @return const MatrixXd& Nullspace matrix
	 */
	MatrixXd getTaskNullspace() const override { return _N; }

	/**
	 * @brief Get the Nullspace projector of the previous tasks
	 *
	 * @return Eigen::MatrixXd
	 */
	MatrixXd getPreviousTasksNullspace() const override { return _N_prec; }

	/**
	 * @brief Get the nullspace of this and the previous tasks. Concretely, it
	 * is the task nullspace multiplied by the nullspace of the previous tasks
	 *
	 */
	MatrixXd getTaskAndPreviousNullspace() const override {
		return _N * _N_prec;
	}

	void setGoalPosition(const Vector3d& goal_position) {
		_goal_position = goal_position;
	}
	const Vector3d& getGoalPosition() const { return _goal_position; }

	void setGoalOrientation(const Matrix3d& goal_orientation) {
		_goal_orientation = goal_orientation;
	}
	Matrix3d getGoalOrientation() const { return _goal_orientation; }

	void setGoalLinearVelocity(const Vector3d& goal_linvel) {
		_goal_linear_velocity = goal_linvel;
	}
	const Vector3d& getGoalLinearVelocity() const {
		return _goal_linear_velocity;
	}

	void setGoalAngularVelocity(const Vector3d& goal_angvel) {
		_goal_angular_velocity = goal_angvel;
	}
	const Vector3d& getGoalAngularVelocity() const {
		return _goal_angular_velocity;
	}

	void setGoalLinearAcceleration(const Vector3d& goal_linaccel) {
		_goal_linear_acceleration = goal_linaccel;
	}
	const Vector3d& getGoalLinearAcceleration() const {
		return _goal_linear_acceleration;
	}

	void setGoalAngularAcceleration(const Vector3d& goal_angaccel) {
		_goal_angular_acceleration = goal_angaccel;
	}
	const Vector3d& getGoalAngularAcceleration() const {
		return _goal_angular_acceleration;
	}

	const Vector3d& getDesiredPosition() const { return _desired_position; }
	const Matrix3d& getDesiredOrientation() const {
		return _desired_orientation;
	}
	const Vector3d& getDesiredLinearVelocity() const {
		return _desired_linear_velocity;
	}
	const Vector3d& getDesiredAngularVelocity() const {
		return _desired_angular_velocity;
	}
	const Vector3d& getDesiredLinearAcceleration() const {
		return _desired_linear_acceleration;
	}
	const Vector3d& getDesiredAngularAcceleration() const {
		return _desired_angular_acceleration;
	}

	const VectorXd& getUnitMassForce() const { return _unit_mass_force; }

	Vector3d getPositionError() const;
	Vector3d getOrientationError() const;

	// Gains for motion controller
	void setPosControlGains(const PIDGains& gains) {
		setPosControlGains(gains.kp, gains.kv, gains.ki);
	}
	void setPosControlGains(double kp_pos, double kv_pos, double ki_pos = 0);
	void setPosControlGains(const VectorXd& kp_pos, const VectorXd& kv_pos,
							const VectorXd& ki_pos);
	void setPosControlGains(const VectorXd& kp_pos, const VectorXd& kv_pos) {
		setPosControlGains(kp_pos, kv_pos, VectorXd::Zero(kp_pos.size()));
	}
	vector<PIDGains> getPosControlGains() const;

	void setPosControlGainsUnsafe(const VectorXd& kp_pos,
								  const VectorXd& kv_pos,
								  const VectorXd& ki_pos);

	void setOriControlGains(const PIDGains& gains) {
		setOriControlGains(gains.kp, gains.kv, gains.ki);
	}
	void setOriControlGains(double kp_ori, double kv_ori, double ki_ori = 0);
	void setOriControlGains(const VectorXd& kp_ori, const VectorXd& kv_ori,
							const VectorXd& ki_ori);
	void setOriControlGains(const VectorXd& kp_ori, const VectorXd& kv_ori) {
		setOriControlGains(kp_ori, kv_ori, VectorXd::Zero(kp_ori.size()));
	}
	vector<PIDGains> getOriControlGains() const;

	void setOriControlGainsUnsafe(const VectorXd& kp_ori,
								  const VectorXd& kv_ori,
								  const VectorXd& ki_ori);

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

	void setMomentControlGains(const PIDGains& gains) {
		setMomentControlGains(gains.kp, gains.kv, gains.ki);
	}
	void setMomentControlGains(double kp_moment, double kv_moment,
							   double ki_moment) {
		_kp_moment = kp_moment * Matrix3d::Identity();
		_kv_moment = kv_moment * Matrix3d::Identity();
		_ki_moment = ki_moment * Matrix3d::Identity();
	}
	vector<PIDGains> getMomentControlGains() const {
		return vector<PIDGains>(
			1, PIDGains(_kp_moment(0, 0), _kv_moment(0, 0), _ki_moment(0, 0)));
	}

	void setFeedforwardForceGain(const double kff_force) {
		_kff_force = kff_force;
	}
	double getFeedforwardForceGain() const { return _kff_force; }

	void setFeedforwardmomentGain(const double kff_moment) {
		_kff_moment = kff_moment;
	}
	double getFeedforwardmomentGain() const { return _kff_moment; }

	void setMaxForceControlFeedbackOutput(
		const double max_force_control_feedback_output) {
		_max_force_control_feedback_output = max_force_control_feedback_output;
	}
	double getMaxForceControlFeedbackOutput() const {
		return _max_force_control_feedback_output;
	}

	void setMaxMomentControlFeedbackOutput(
		const double max_moment_control_feedback_output) {
		_max_moment_control_feedback_output =
			max_moment_control_feedback_output;
	}
	double getMaxMomentControlFeedbackOutput() const {
		return _max_moment_control_feedback_output;
	}

	/**
	 * @brief Set the Goal Force in robot world frame
	 *
	 * @param goal_force
	 */
	void setGoalForce(const Vector3d& goal_force) { _goal_force = goal_force; }

	/**
	 * @brief Get the goal Force in robot world frame
	 *
	 * @return const Vector3d& goal force in robot world frame
	 */
	Vector3d getGoalForce() const;

	/**
	 * @brief Set the goal Moment in robot world frame
	 *
	 * @param goal_moment
	 */
	void setGoalMoment(const Vector3d& goal_moment) {
		_goal_moment = goal_moment;
	}

	/**
	 * @brief Get the goal Moment in robot world frame
	 *
	 * @return const Vector3d& goal moment in robot world frame
	 */
	Vector3d getGoalMoment() const;

	// internal otg functions
	/**
	 * @brief 	Enables the internal otg for position and orientation with
	 * acceleration limited trajectory, given the input numbers. By default,
	 * this one is enabled with linear velocity and acceleration limits of 0.3
	 * and 1.0 respectively, and angular velocity and acceleration limits of
	 * pi/3 and pi respectively
	 *
	 * @param max_linear_velelocity
	 * @param max_linear_acceleration
	 * @param max_angular_velocity
	 * @param max_angular_acceleration
	 */
	void enableInternalOtgAccelerationLimited(
		const double max_linear_velelocity,
		const double max_linear_acceleration, const double max_angular_velocity,
		const double max_angular_acceleration);

	/**
	 * @brief 	Enables the internal otg for position and orientation with jerk
	 * limited trajectory, given the input numbers
	 *
	 * @param max_linear_velelocity
	 * @param max_linear_acceleration
	 * @param max_linear_jerk
	 * @param max_angular_velocity
	 * @param max_angular_acceleration
	 * @param max_angular_jerk
	 */
	void enableInternalOtgJerkLimited(const double max_linear_velelocity,
									  const double max_linear_acceleration,
									  const double max_linear_jerk,
									  const double max_angular_velocity,
									  const double max_angular_acceleration,
									  const double max_angular_jerk);

	void disableInternalOtg() { _use_internal_otg_flag = false; }

	bool getInternalOtgEnabled() const { return _use_internal_otg_flag; }

	const OTG_6dof_cartesian& getInternalOtg() const { return *_otg; }

	/**
	 * @brief Enables Ruckig tracking mode for the internal OTG.
	 *
	 * In tracking mode, goal pose, velocity and acceleration are passed to
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
	 * @brief Sets target linear/angular velocity and acceleration limits for
	 * internal OTG tracking mode.
	 */
	void setInternalOtgTrackingTargetLimits(
		const double max_linear_velocity,
		const double max_linear_acceleration,
		const double max_angular_velocity,
		const double max_angular_acceleration) {
		_otg->setTrackingTargetLimits(
			max_linear_velocity, max_linear_acceleration,
			max_angular_velocity, max_angular_acceleration);
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

	// Velocity saturation flag and saturation values
	void enableVelocitySaturation(const double linear_vel_sat = 0.3,
								  const double angular_vel_sat = M_PI / 3);
	void disableVelocitySaturation() { _use_velocity_saturation_flag = false; }

	bool getVelocitySaturationEnabled() const {
		return _use_velocity_saturation_flag;
	}
	double getLinearSaturationVelocity() const {
		return _linear_saturation_velocity;
	}
	double getAngularSaturationVelocity() const {
		return _angular_saturation_velocity;
	}

	// Force-related functions
	void enableForceDampingDecoupling() {
		_singularity_handler->enableForceDampingDecoupling();
	}
	void disableForceDampingDecoupling() {
		_singularity_handler->disableForceDampingDecoupling();
	}

	// Integrator-related functions
    void enableZeroForceCrossing() { _zero_force_crossing_flag = true; }
    void enableZeroMomentCrossing() { _zero_moment_crossing_flag = true; }
    void disableZeroForceCrossing() { _zero_force_crossing_flag = false; }
    void disableZeroMomentCrossing() { _zero_moment_crossing_flag = false; }

	void enableZeroPositionCrossing() { _zero_position_crossing_flag = true; }
    void enableZeroOrientationCrossing() { _zero_orientation_crossing_flag = true; }
    void disableZeroPositionCrossing() { _zero_position_crossing_flag = false; }
    void disableZeroOrientationCrossing() { _zero_orientation_crossing_flag = false; }

	//------------------------------------------------
	// Methods
	//------------------------------------------------

	// -------- core methods --------

	/**
	 * @brief      update the task model (jacobians, task inertia and nullspace
	 *             matrices)
	 * @details    This function updates the jacobian, projected jacobian, task
	 *             inertia matrix (Lambda), dynamically consistent inverse of
	 *             the Jacobian (Jbar) and nullspace matrix of the task N. This
	 *             function uses the robot model and assumes it has been
	 *             updated. There is no use to calling it if the robot
	 *             kinematics or dynamics have not been updated since the last
	 *             call. This function takes the N_prec matrix as a parameter
	 *             which is the product of the nullspace matrices of the higher
	 *             priority tasks. The N matrix will be the matrix to use as
	 *             N_prec for the subsequent tasks. In order to get the
	 *             nullspace matrix of this task alone, one needs to compute _N
	 * * _N_prec.inverse().
	 *
	 * @param      N_prec  The nullspace matrix of all the higher priority
	 *                     tasks. If this is the highest priority task, use
	 *                     identity of size n*n where n in the number of DoF of
	 *                     the robot.
	 */
	void updateTaskModel(const MatrixXd& N_prec) override;

	/**
	 * @brief      Computes the torques associated with this task.
	 * @details    Computes the torques taking into account the last model
	 *             update and updated values for the robot joint
	 *             positions/velocities
	 *
	 * @return Eigen::VectorXd the joint task torques
	 *
	 */
	VectorXd computeTorques() override;

	/**
	 * @brief Computes the joint torques associated with this control task, and
	 * feedforward compensates the disturbances due to the previous tasks.
	 *
	 * @param tau_prec the control torques from the frevious tasks in the
	 * hierarchy
	 * @return Eigen::VectorXd the joint task torques
	 */
	VectorXd computeTorques(const Eigen::VectorXd& tau_prec) override;

	/**
	 * @brief      reinitializes the desired and goal states to the current
	 * robot configuration as well as the integrator terms
	 */
	void reInitializeTask() override;

	/**
	 * @brief      Checks if the goal position is reached op to a certain
	 * tolerance
	 *
	 * @param[in]  tolerance  The tolerance
	 * @param[in]  verbose    display info or not
	 *
	 * @return     true of the position error is smaller than the tolerance
	 */
	bool goalPositionReached(const double tolerance,
							 const bool verbose = false);

	/**
	 * @brief      Checks if the goal orientation has reched the goal up to a
	 * tolerance
	 *
	 * @param[in]  tolerance  The tolerance
	 * @param[in]  verbose    display info or not
	 *
	 * @return     true if the norm of the orientation error is smaller than the
	 * tolerance
	 */
	bool goalOrientationReached(const double tolerance,
								const bool verbose = false);

	bool goalPoseReached(const double pos_tol, 
						 const double ori_tol, 
						 const bool verbose = false) {
		return goalOrientationReached(pos_tol, verbose) && goalOrientationReached(ori_tol, verbose);
	}

	// -------- force control related methods --------

	/**
	 * @brief      Sets the force sensor frame.
	 *
	 * @param[in]  link_name               The link name on which the sensor is
	 * attached
	 * @param[in]  transformation_in_link  The transformation in link of the
	 * sensor
	 */
	void setForceSensorFrame(const string link_name,
							 const Affine3d transformation_in_link);

	/**
	 * @brief      Updates the velues of the sensed force and sensed moment from
	 *             sensor values
	 * @details    Assumes that the sensor is attached to the same link as the
	 *             control frame and that the setSensorFrame finction has been
	 * called. The force and moment values given to this function are assumed to
	 * be in the force sensor frame (values taken directly from the force
	 * sensor) These values are supposed to be the forces that the sensor
	 * applies to the environment (so the opposite of what the sensor feels)
	 *
	 * @param      sensed_force_sensor_frame   The sensed force as the force
	 *                                         that the sensor applies to the
	 *                                         environment in sensor frame
	 * @param      sensed_moment_sensor_frame  The sensed moment as the moment
	 *                                         that the sensor applies to the
	 *                                         environment in sensor frame
	 */
	void updateSensedForceAndMoment(const Vector3d sensed_force_sensor_frame,
									const Vector3d sensed_moment_sensor_frame);

	/**
	 * @brief Parametrizes the force space and motion space for translational
	 * control The first argument is the dimension of the force space (between
	 * 0 and 3, 0 meaning the whole translation is controlled in motion and 3
	 * meaning the whole translation is controlled in force) and the second
	 * argument is the axis defining the space of dimension one (unused if the
	 * first parameter is 0 or 3, and representing the direction of the force
	 * space is the first parameter is 1, or the direction of the motion space
	 * if the first parameter is 2).
	 * Returns true if the parametrization was changed (dimension or axis) and
	 * false otherwise (in which case, the goal position and integrators are
	 * not reset)
	 *
	 * @param force_space_dimension  between 0 and 3
	 * @param force_or_motion_single_axis non singular axis (unused if
	 * force_space_dimension is 0 or 3)
	 * @return true if the parametrization was changed (dimension or axis) and
	 * false otherwise
	 */
	bool parametrizeForceMotionSpaces(
		const int force_space_dimension,
		const Vector3d& force_or_motion_single_axis = Vector3d::Zero());

	int getForceSpaceDimension() const { return _force_space_dimension; }
	Vector3d getForceMotionSingleAxis() const { return _force_or_motion_axis; }

	/**
	 * @brief Parametrizes the moment space and rotational motion space.
	 * The first argument is the dimension of the moment space (between
	 * 0 and 3, 0 meaning the whole rotation is controlled in motion and 3
	 * meaning the whole rotation is controlled in moments) and the second
	 * argument is the axis defining the space of dimension one (unused if the
	 * first parameter is 0 or 3, and representing the direction of the moment
	 * space is the first parameter is 1, or the direction of the rotational
	 * motion space if the first parameter is 2)
	 * Returns true if the parametrization was changed (dimension or axis) and
	 * false otherwise (in which case, the goal orientation and integrators are
	 * not reset)
	 *
	 * @param moment_space_dimension betawwn 0 and 3
	 * @param moment_or_rot_motion_single_axis non singular axis (unused if
	 * moment_space_dimension is 0 or 3)
	 * @return true if the parametrization was changed (dimension or axis) and
	 * false otherwise
	 */
	bool parametrizeMomentRotMotionSpaces(
		const int moment_space_dimension,
		const Vector3d& moment_or_rot_motion_single_axis = Vector3d::Zero());

	int getMomentSpaceDimension() const { return _moment_space_dimension; }
	Vector3d getMomentRotMotionSingleAxis() const {
		return _moment_or_rotmotion_axis;
	}

	Matrix3d sigmaForce() const;
	Matrix3d sigmaPosition() const;
	Matrix3d sigmaMoment() const;
	Matrix3d sigmaOrientation() const;

	/**
	 * @brief      Changes the behavior to closed loop force/moment control for
	 * the force controlled directions in the linear/angular parts of the
	 *             controller
	 */
	void setClosedLoopForceControl(const bool closed_loop_force_control = true);
	void setClosedLoopMomentControl(
		const bool closed_loop_moment_control = true);

	/**
	 * @brief      Enables or disables the passivity based stability for the
	 * closed loop force control (enabled by default)
	 */
	void enablePassivity() { _POPC_force->enable(); }
	void disablePassivity() { _POPC_force->disable(); }

	// ------- helper methods -------

	/**
	 * @brief      Resets all the integrated errors used in I terms
	 */
	void resetIntegrators();

	/**
	 * @brief      Resets the integrated errors used in I terms for linear part
	 *             of task (position_integrated_error and
	 *             force_integrated_error)
	 */
	void resetIntegratorsLinear();

	/**
	 * @brief      Resets the integrated errors used in I terms for angular part
	 *             of task (orientation_integrated_error and
	 *             moment_integrated_error)
	 */
	void resetIntegratorsAngular();

	Matrix3d posSelectionProjector() const {
		return _partial_task_projection.block<3, 3>(0, 0);
	}

	Matrix3d oriSelectionProjector() const {
		return _partial_task_projection.block<3, 3>(3, 3);
	}

	// -------- singularity handling methods --------

	/**
	 * @brief 	Set the Dynamic Decoupling Type. See the definition of the
	 * DynamicDecouplingType enum for more details
	 *
	 * @param type Dynamic decoupling type
	 */
	void setDynamicDecouplingType(const DynamicDecouplingType type) {
		_singularity_handler->setDynamicDecouplingType(type);
	}

	/**
	 * @brief Set the threshold for the bounded inertia estimate
	 *
	 * @param threshold threshold value
	 */
	void setBoundedInertiaEstimateThreshold(const double bie_threshold) {
		_singularity_handler->setBoundedInertiaEstimateThreshold(bie_threshold);
	}

	void setBoundedInertiaEstimateThreshold(const double bie_threshold, const double sjs_threshold) {
		_singularity_handler->setBoundedInertiaEstimateThreshold(bie_threshold, sjs_threshold);
	}

	/**
	 * @brief Get the threshold for the bounded inertia estimate
	 *
	 * @return double threshold value
	 */
	std::pair<double, double> getBoundedInertiaEstimateThreshold() {
		return _singularity_handler->getBoundedInertiaEstimateThreshold();
	}

	/**
	 * @brief Enforces type 1 handling behavior if set to true, otherwise handle
	 * type 1 or type 2 as usual
	 *
	 * @param flag true to enforce type 1 handling behavior
	 */
	void handleAllSingularitiesAsTypeOne(const bool flag) {
		_singularity_handler->handleAllSingularitiesAsTypeOne(flag);
	}

	/**
	 * @brief Enables singularity handling
	 *
	 */
	void enableSingularityHandling() {
		_handle_singularity = true;
		_singularity_handler->enableSingularityHandling();
	}

	/**
	 * @brief Disables singularity handling
	 *
	 */
	void disableSingularityHandling() {
		_handle_singularity = false;
		_singularity_handler->disableSingularityHandling();
	}

	/**
	 * @brief Set the singularity bound based on the eigenvalue of \Lambda^{-1}
	 *
	 * @param s_min lower bound
	 * @param s_max upper bound
	 */
	void setSingularityHandlingBound(const double s_max) {
		_singularity_handler->setSingularityHandlingBound(s_max);
	}

	/**
	 * @brief Set the gains for the singular joint space task for the singularity
	 * strategy
	 *
	 * @param kv_type_1 velocity damping gain for type 1 strategy
	 * @param kv_type_2 velocity damping gain for type 2 strategy
	 */
	void setSingularityHandlingGains(const double& kv_type_1,
									 const double& kv_type_2) {
		_singularity_handler->setSingularityHandlingGains(kv_type_1, kv_type_2);
	}

	/**
	 * @brief Return whether exiting a singularity region
	 * 
	 */
	bool isExitingSingularity() {
		return _singularity_handler->isExitingSingularity();
	}

	void setSingularityExitInterpolatorTol(const double pos_tol, const double ori_tol) {
		_singularity_pos_exit_tol = pos_tol;
		_singularity_ori_exit_tol = ori_tol;
	}

	void setSingularityVelExitInterpolatorTol(const double linear_vel_tol, const double angular_vel_tol) {
		_singularity_linear_vel_exit_tol = linear_vel_tol;
		_singularity_angular_vel_exit_tol = angular_vel_tol;
	}

	void setSingularityExitVelocity(const double linear_vel, const double angular_vel) {
		_singularity_exit_linear_vel = linear_vel;
		_singularity_exit_angular_vel = angular_vel;
	}

	void enableSingularityExitInterpolationVelocityCheck() {
		_singularity_exit_vel_check = true;
	}

	void disableSingularityExitInterpolationVelocityCheck() {
		_singularity_exit_vel_check = false;
	}

	// experimental access
	SingularityHandler& getSingularityHandler() {
		return *_singularity_handler;
	}

	void removeFloatingBaseDependency() {
		_remove_floating_base_dependency = true;
	}

	// // -------- override step computation ----------
	// void enableManualStepPositionError() {
	// 	_use_user_step_position_flag = true;
	// }
	// void disableManualStepPositionError() {
	// 	_use_user_step_position_flag = false;
	// }
	// void setStepPositionError(const Vector3d& user_step_position_error) {
	// 	_user_step_position_error = user_step_position_error;
	// }

	// void enableManualStepOrientationError() {
	// 	_use_user_step_orientation_flag = true;
	// }
	// void disableManualStepOrientationError() {
	// 	_use_user_step_orientation_flag = false;
	// }
	// void setStepOrientationError(const Vector3d& user_step_orientation_error) {
	// 	_user_step_orientation_error = user_step_orientation_error;
	// }

private:
	/**
	 * @brief Initial setup of the task, called in the constructor to avoid
	 * duplicated code
	 *
	 */
	void initialSetup();

	// the goal state is the state the controller tries to reach. If OTG is on,
	// the actual desired state at each timestep will be interpolated between
	// the initial state and the goal state, while the goal state might not
	// change. It defaults to the configuration when the task is created
	// expressed in world frame
	Vector3d _goal_position;
	Matrix3d _goal_orientation;
	Vector3d _goal_linear_velocity;
	Vector3d _goal_angular_velocity;
	Vector3d _goal_linear_acceleration;
	Vector3d _goal_angular_acceleration;

	// the desired state is the state used in the control law. It is the output
	// of the OTG if enabled, and the same as the goal state otherwise
	// expressed in world frame
	Vector3d _desired_position;
	Matrix3d _desired_orientation;
	Vector3d _desired_linear_velocity;
	Vector3d _desired_angular_velocity;
	Vector3d _desired_linear_acceleration;
	Vector3d _desired_angular_acceleration;

	// gains for motion controller
	// defaults to isptropic 50 for p gains, 14 for d gains and 0 for i gains
	Matrix3d _kp_pos, _kp_ori;
	Matrix3d _kv_pos, _kv_ori;
	Matrix3d _ki_pos, _ki_ori;

	// gains for the closed loop force controller
	// by default, the force controller is open loop
	// to set the behavior to closed loop controller, use the functions
	// setClosedLoopForceControl and setClosedLoopMomentControl. the closed loop
	// force controller is a PI controller with feedforward force and velocity
	// based damping. gains default to isotropic 1 for p gains, 0.7 for i gains
	// and 10 for d gains
	Matrix3d _kp_force, _kp_moment;
	Matrix3d _kv_force, _kv_moment;
	Matrix3d _ki_force, _ki_moment;

	// goal force and moment for the force part of the controller
	// defaults to Zero
	Vector3d _goal_force;
	Vector3d _goal_moment;

	// velocity saturation is off by default
	bool _use_velocity_saturation_flag;
	double _linear_saturation_velocity;
	double _angular_saturation_velocity;

	// internal otg using ruckig, on by default with acceleration limited
	// trajectory
	bool _use_internal_otg_flag;
	std::unique_ptr<OTG_6dof_cartesian> _otg;

	Eigen::VectorXd _task_force;
	Eigen::MatrixXd _N_prec;

	// internal variables, not to be touched by the user
	string _link_name;
	Affine3d _compliant_frame;	// in link_frame
	bool _is_force_motion_parametrization_in_compliant_frame;

	// motion quantities
	Vector3d _current_position;		// robot world frame
	Matrix3d _current_orientation;	// robot world frame

	Vector3d _current_linear_velocity;	 // robot world frame
	Vector3d _current_angular_velocity;	 // robot world frame

	Vector3d _orientation_error;			 // robot world frame
	Vector3d _integrated_orientation_error;	 // robot world frame
	Vector3d _integrated_position_error;	 // robot world frame

	// force quantities
	Affine3d _T_control_to_sensor;

	Vector3d _sensed_force_control_world_frame;
	Vector3d _sensed_moment_control_world_frame;

	Vector3d _sensed_force_sensor_frame;
	Vector3d _sensed_moment_sensor_frame;

	Vector3d _integrated_force_error;	// robot world frame
	Vector3d _integrated_moment_error;	// robot world frame

	int _force_space_dimension, _moment_space_dimension;
	Vector3d _force_or_motion_axis, _moment_or_rotmotion_axis;

	bool _closed_loop_force_control;
	bool _closed_loop_moment_control;
	double _kff_force;
	double _kff_moment;
	double _max_force_control_feedback_output;
	double _max_moment_control_feedback_output;

	// POPC for closed loop force control
	std::unique_ptr<POPCExplicitForceControl> _POPC_force;

	// linear control inputs
	Vector3d _linear_motion_control;
	Vector3d _linear_force_control;

	// control parameters
	bool _are_pos_gains_isotropic;	// defaults to true
	bool _are_ori_gains_isotropic;	// defaults to true

	// dynamic decoupling type, defaults to BOUNDED_INERTIA_ESTIMATES
	DynamicDecouplingType _dynamic_decoupling_type;

	// model quantities
	MatrixXd _jacobian;
	MatrixXd _projected_jacobian;
	MatrixXd _Lambda, _Lambda_modified;
	MatrixXd _Jbar;
	MatrixXd _N;

	MatrixXd _current_task_range;
	int _pos_range, _ori_range;

	Matrix<double, 6, 6> _partial_task_projection;

	VectorXd _unit_mass_force;

	// singularity handler
	std::vector<int> _joint_dependency;
	std::unique_ptr<SingularityHandler> _singularity_handler;
	bool _handle_singularity;
	bool _handle_singularity_exit;
	bool _singularity_exit_vel_check;
	bool _prev_velocity_saturation;
	bool _prev_is_in_singularity;
	bool _is_in_singularity;
	double _prev_linear_saturation_velocity;
	double _prev_angular_saturation_velocity;
	double _goal_interpolation_linear_velocity;
	double _goal_interpolation_angular_velocity;
	double _singularity_pos_exit_tol;
	double _singularity_ori_exit_tol;
	double _singularity_linear_vel_exit_tol;
	double _singularity_angular_vel_exit_tol;
	double _singularity_ramp_linear_acceleration;
	double _singularity_ramp_angular_acceleration;
	double _singularity_exit_linear_vel;
	double _singularity_exit_angular_vel;

	// integrator zero crossing reset flags
	Vector3d _prev_force_error;
	Vector3d _prev_moment_error;
	bool _zero_force_crossing_flag;
    bool _zero_moment_crossing_flag;
	bool _zero_position_crossing_flag;
    bool _zero_orientation_crossing_flag;

	// interpolation parameters
	double _default_linear_saturation_velocity;
	double _default_angular_saturation_velocity;

	// floating base
	bool _remove_floating_base_dependency;

};

} /* namespace SaiPrimitives */

#endif /* SAI_PRIMITIVES_MOTIONFORCETASK_TASK_H_ */
