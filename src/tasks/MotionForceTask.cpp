/*
 * MotionForceTask.cpp
 *
 *      Author: Mikael Jorda
 */

#include "MotionForceTask.h"

#include <stdexcept>

using namespace std;
using namespace Eigen;

namespace Sai2Primitives {

namespace {
const double MAX_FEEDBACK_FORCE_FORCE_CONTROLLER = 20.0;
const double MAX_FEEDBACK_MOMENT_FORCE_CONTROLLER = 10.0;
}  // namespace

MotionForceTask::MotionForceTask(
	std::shared_ptr<Sai2Model::Sai2Model>& robot, const string& link_name,
	const Affine3d& compliant_frame, const std::string& task_name,
	const bool is_force_motion_parametrization_in_compliant_frame,
	const double loop_timestep)
	: TemplateTask(robot, task_name, TaskType::MOTION_FORCE_TASK,
				   loop_timestep) {
	_link_name = link_name;
	_compliant_frame = compliant_frame;
	_is_force_motion_parametrization_in_compliant_frame =
		is_force_motion_parametrization_in_compliant_frame;

	_partial_task_projection = Matrix<double, 6, 6>::Identity(6, 6);

	initialSetup();
}

MotionForceTask::MotionForceTask(
	std::shared_ptr<Sai2Model::Sai2Model>& robot, const string& link_name,
	std::vector<Vector3d> controlled_directions_translation,
	std::vector<Vector3d> controlled_directions_rotation,
	const Affine3d& compliant_frame, const std::string& task_name,
	const bool is_force_motion_parametrization_in_compliant_frame,
	const double loop_timestep)
	: TemplateTask(robot, task_name, TaskType::MOTION_FORCE_TASK,
				   loop_timestep) {
	_link_name = link_name;
	_compliant_frame = compliant_frame;
	_is_force_motion_parametrization_in_compliant_frame =
		is_force_motion_parametrization_in_compliant_frame;

	if (controlled_directions_translation.empty() &&
		controlled_directions_rotation.empty()) {
		throw invalid_argument(
			"controlled_directions_translation and "
			"controlled_directions_rotation cannot both be empty in "
			"MotionForceTask::MotionForceTask\n");
	}

	MatrixXd controlled_translation_range_basis = MatrixXd::Zero(3, 1);
	MatrixXd controlled_rotation_range_basis = MatrixXd::Zero(3, 1);

	if (controlled_directions_translation.size() != 0) {
		MatrixXd controlled_translation_vectors =
			MatrixXd::Zero(3, controlled_directions_translation.size());
		for (int i = 0; i < controlled_directions_translation.size(); i++) {
			controlled_translation_vectors.col(i) =
				controlled_directions_translation[i];
		}
		controlled_translation_range_basis =
			Sai2Model::matrixRangeBasis(controlled_translation_vectors);
	}

	if (controlled_directions_rotation.size() != 0) {
		MatrixXd controlled_rotation_vectors =
			MatrixXd::Zero(3, controlled_directions_rotation.size());
		for (int i = 0; i < controlled_directions_rotation.size(); i++) {
			controlled_rotation_vectors.col(i) =
				controlled_directions_rotation[i];
		}

		controlled_rotation_range_basis =
			Sai2Model::matrixRangeBasis(controlled_rotation_vectors);
	}

	_partial_task_projection.setZero();
	_partial_task_projection.block<3, 3>(0, 0) =
		controlled_translation_range_basis *
		controlled_translation_range_basis.transpose();
	_partial_task_projection.block<3, 3>(3, 3) =
		controlled_rotation_range_basis *
		controlled_rotation_range_basis.transpose();

	initialSetup();
}

void MotionForceTask::initialSetup() {

	int dof = getConstRobotModel()->dof();
	_T_control_to_sensor = Affine3d::Identity();
	_force_sensor = std::make_shared<ForceSensor>(getConstRobotModel());

	// POPC force
	_POPC_force.reset(new POPCExplicitForceControl(getLoopTimestep()));

	// motion
	_current_position = getConstRobotModel()->positionInWorld(
		_link_name, _compliant_frame.translation());
	_current_orientation = getConstRobotModel()->rotationInWorld(
		_link_name, _compliant_frame.rotation());

	// default values for gains and velocity saturation
	setPosControlGains(50.0, 14.0, 0.0);
	setOriControlGains(50.0, 14.0, 0.0);
	setForceControlGains(0.7, 10.0, 1.3);
	setMomentControlGains(0.7, 10.0, 1.3);

	disableVelocitySaturation();
	_linear_saturation_velocity = 0;
	_angular_saturation_velocity = 0;

	_k_ff = 0.95;

	_force_space_dimension = 0;
	_moment_space_dimension = 0;
	setClosedLoopForceControl(false);
	setClosedLoopMomentControl(false);

	// initialize matrices sizes
	_jacobian.setZero(6, dof);
	_projected_jacobian.setZero(6, dof);
	_Lambda.setZero(6, 6);
	_Lambda_modified.setZero(6, 6);
	_Jbar.setZero(dof, 6);
	_N.setZero(dof, dof);
	_N_prec = MatrixXd::Identity(dof, dof);

	MatrixXd range_pos =
		Sai2Model::matrixRangeBasis(_partial_task_projection.block<3, 3>(0, 0));
	MatrixXd range_ori =
		Sai2Model::matrixRangeBasis(_partial_task_projection.block<3, 3>(3, 3));

	_pos_range = range_pos.norm() == 0 ? 0 : range_pos.cols();
	_ori_range = range_ori.norm() == 0 ? 0 : range_ori.cols();

	if (_pos_range + _ori_range == 0)  // should not happen
	{
		throw invalid_argument(
			"controlled_directions_translation and "
			"controlled_directions_rotation cannot both be empty in "
			"MotionForceTask::MotionForceTask\n");
	}

	_current_task_range.setZero(6, _pos_range + _ori_range);
	if (_pos_range > 0) {
		_current_task_range.block(0, 0, 3, _pos_range) = range_pos;
	}
	if (_ori_range > 0) {
		_current_task_range.block(3, _pos_range, 3, _ori_range) = range_ori;
	}

	// trajectory generation
	_otg = make_unique<OTG_6dof_cartesian>(
		_current_position, _current_orientation, getLoopTimestep());
	enableInternalOtgAccelerationLimited(0.3, 1.0, M_PI / 3, M_PI);

	// singularity handling
	MatrixXd J_posture = getConstRobotModel()->linkDependency(_link_name);
	_singularity_handler = std::make_unique<SingularityHandler>(getConstRobotModel(), _pos_range + _ori_range, J_posture);
	setSingularityBounds(5e-3, 5e-2);  
	setDynamicDecouplingType(BOUNDED_INERTIA_ESTIMATES);

	reInitializeTask();	
}

void MotionForceTask::reInitializeTask() {
	int dof = getConstRobotModel()->dof();

	// motion
	_current_position = getConstRobotModel()->positionInWorld(
		_link_name, _compliant_frame.translation());
	_goal_position = _current_position;
	_current_orientation = getConstRobotModel()->rotationInWorld(
		_link_name, _compliant_frame.rotation());
	_goal_orientation = _current_orientation;

	_current_linear_velocity.setZero();
	_goal_linear_velocity.setZero();
	_current_angular_velocity.setZero();
	_goal_angular_velocity.setZero();
	_goal_linear_acceleration.setZero();
	_goal_angular_acceleration.setZero();

	_orientation_error.setZero();
	_integrated_position_error.setZero();
	_integrated_orientation_error.setZero();

	_goal_force.setZero();
	_sensed_force.setZero();
	_goal_moment.setZero();
	_sensed_moment.setZero();

	resetIntegrators();

	_task_force.setZero(6);
	_unit_mass_force.setZero(6);

	_otg->reInitialize(_current_position, _current_orientation);
}

void MotionForceTask::updateTaskModel(const MatrixXd& N_prec) {
	const int robot_dof = getConstRobotModel()->dof();
	if (N_prec.rows() != N_prec.cols()) {
		throw invalid_argument(
			"N_prec matrix not square in MotionForceTask::updateTaskModel\n");
	}
	if (N_prec.rows() != robot_dof) {
		throw invalid_argument(
			"N_prec matrix size not consistent with robot dof in "
			"MotionForceTask::updateTaskModel\n");
	}

	_N_prec = N_prec;

	_jacobian = _partial_task_projection *
				getConstRobotModel()->JWorldFrame(
					_link_name, _compliant_frame.translation());
	_projected_jacobian = _jacobian * _N_prec;

	_singularity_handler->updateTaskModel(_projected_jacobian, _N_prec);
	_N = _singularity_handler->getNullspace();  // N_posture * N_ns or N_ns 

}

VectorXd MotionForceTask::computeTorques() {
	VectorXd task_joint_torques = VectorXd::Zero(getConstRobotModel()->dof());
	_jacobian = _partial_task_projection *
				getConstRobotModel()->JWorldFrame(
					_link_name, _compliant_frame.translation());
	_projected_jacobian = _jacobian * _N_prec;

	// update controller state
	_current_position = getConstRobotModel()->positionInWorld(
		_link_name, _compliant_frame.translation());
	_current_orientation = getConstRobotModel()->rotationInWorld(
		_link_name, _compliant_frame.rotation());

	_orientation_error =
		Sai2Model::orientationError(_goal_orientation, _current_orientation);
	_current_linear_velocity =
		_projected_jacobian.block(0, 0, 3, getConstRobotModel()->dof()) *
		getConstRobotModel()->dq();
	_current_angular_velocity =
		_projected_jacobian.block(3, 0, 3, getConstRobotModel()->dof()) *
		getConstRobotModel()->dq();

	if (_pos_range + _ori_range == 0) {
		// there is no controllable degree of freedom for the task, just return
		// zero torques. should maybe print a warning here
		return task_joint_torques;
	}

	Matrix3d sigma_force = sigmaForce();
	Matrix3d sigma_moment = sigmaMoment();
	Matrix3d sigma_position = sigmaPosition();
	Matrix3d sigma_orientation = sigmaOrientation();

	Vector3d goal_force = getGoalForce();
	Vector3d goal_moment = getGoalMoment();

	Vector3d force_feedback_related_force = Vector3d::Zero();
	Vector3d position_related_force = Vector3d::Zero();
	Vector3d moment_feedback_related_force = Vector3d::Zero();
	Vector3d orientation_related_force = Vector3d::Zero();

	Matrix3d kp_pos =
		_current_orientation * _kp_pos * _current_orientation.transpose();
	Matrix3d kv_pos =
		_current_orientation * _kv_pos * _current_orientation.transpose();
	Matrix3d ki_pos =
		_current_orientation * _ki_pos * _current_orientation.transpose();

	// force related terms
	if (_closed_loop_force_control) {
		// update the integrated error
		_integrated_force_error +=
			sigma_force * (_sensed_force - goal_force) * getLoopTimestep();

		// compute the feedback term and saturate it
		Vector3d force_feedback_term =
			sigma_force * (-_kp_force * (_sensed_force - goal_force) -
						   _ki_force * _integrated_force_error);
		if (force_feedback_term.norm() > MAX_FEEDBACK_FORCE_FORCE_CONTROLLER) {
			force_feedback_term *= MAX_FEEDBACK_FORCE_FORCE_CONTROLLER /
								   force_feedback_term.norm();
		}

		// compute the final contribution
		force_feedback_related_force =
			_POPC_force->computePassivitySaturatedForce(
				sigma_force * goal_force, sigma_force * _sensed_force,
				sigma_force * force_feedback_term,
				sigma_force * _current_linear_velocity, _kv_force, _k_ff);
	} else	// open loop force control
	{
		force_feedback_related_force =
			sigma_force * (-_kv_force * _current_linear_velocity);
	}

	// moment related terms
	if (_closed_loop_moment_control) {
		// update the integrated error
		_integrated_moment_error += sigma_moment *
									(_sensed_moment - goal_moment) *
									getLoopTimestep();

		// compute the feedback term
		Vector3d moment_feedback_term =
			sigma_moment * (-_kp_moment * (_sensed_moment - goal_moment) -
							_ki_moment * _integrated_moment_error);

		// saturate the feedback term
		if (moment_feedback_term.norm() >
			MAX_FEEDBACK_MOMENT_FORCE_CONTROLLER) {
			moment_feedback_term *= MAX_FEEDBACK_MOMENT_FORCE_CONTROLLER /
									moment_feedback_term.norm();
		}

		// compute the final contribution
		moment_feedback_related_force =
			sigma_moment *
			(moment_feedback_term - _kv_moment * _current_angular_velocity);
	} else	// open loop moment control
	{
		moment_feedback_related_force =
			sigma_moment * (-_kv_moment * _current_angular_velocity);
	}

	// motion related terms
	// compute next state from trajectory generation
	Vector3d tmp_desired_position = _goal_position;
	Matrix3d tmp_desired_orientation = _goal_orientation;
	Vector3d tmp_desired_linear_velocity = _goal_linear_velocity;
	Vector3d tmp_desired_angular_velocity = _goal_angular_velocity;
	Vector3d tmp_desired_acceleration = _goal_linear_acceleration;
	Vector3d tmp_desired_angular_acceleration = _goal_angular_acceleration;

	if (_use_internal_otg_flag) {
		_otg->setGoalPositionAndLinearVelocity(_goal_position,
											   _goal_linear_velocity);
		_otg->setGoalOrientationAndAngularVelocity(_goal_orientation,
												   _goal_angular_velocity);
		_otg->update();

		tmp_desired_position = _otg->getNextPosition();
		tmp_desired_linear_velocity = _otg->getNextLinearVelocity();
		tmp_desired_acceleration = _otg->getNextLinearAcceleration();
		tmp_desired_orientation = _otg->getNextOrientation();
		tmp_desired_angular_velocity = _otg->getNextAngularVelocity();
		tmp_desired_angular_acceleration = _otg->getNextAngularAcceleration();
	}

	// linear motion
	// update integrated error for I term
	_integrated_position_error += sigma_position *
								  (_current_position - tmp_desired_position) *
								  getLoopTimestep();

	// final contribution
	if (_use_velocity_saturation_flag) {
		tmp_desired_linear_velocity =
			-_kp_pos * _kv_pos.inverse() * sigma_position *
				(_current_position - tmp_desired_position) -
			_ki_pos * _kv_pos.inverse() * _integrated_position_error;
		if (tmp_desired_linear_velocity.norm() > _linear_saturation_velocity) {
			tmp_desired_linear_velocity *=
				_linear_saturation_velocity / tmp_desired_linear_velocity.norm();
		}
		position_related_force =
			sigma_position *
			(tmp_desired_acceleration -
			 _kv_pos * (_current_linear_velocity - tmp_desired_linear_velocity));
	} else {
		position_related_force =
			sigma_position *
			(tmp_desired_acceleration -
			 _kp_pos * (_current_position - tmp_desired_position) -
			 _kv_pos * (_current_linear_velocity - tmp_desired_linear_velocity) -
			 _ki_pos * _integrated_position_error);
	}

	// angular motion
	// orientation error
	Vector3d step_orientation_error =
		sigma_orientation * Sai2Model::orientationError(tmp_desired_orientation,
														_current_orientation);

	// update integrated error for I term
	_integrated_orientation_error += step_orientation_error * getLoopTimestep();

	// final contribution
	if (_use_velocity_saturation_flag) {
		tmp_desired_angular_velocity =
			-_kp_ori * _kv_ori.inverse() * step_orientation_error -
			_ki_ori * _kv_ori.inverse() * _integrated_orientation_error;
		if (tmp_desired_angular_velocity.norm() >
			_angular_saturation_velocity) {
			tmp_desired_angular_velocity *= _angular_saturation_velocity /
											tmp_desired_angular_velocity.norm();
		}
		orientation_related_force =
			sigma_orientation * (tmp_desired_angular_acceleration -
								 _kv_ori * (_current_angular_velocity -
											tmp_desired_angular_velocity));
	} else {
		orientation_related_force =
			sigma_orientation * (tmp_desired_angular_acceleration -
								 _kp_ori * step_orientation_error -
								 _kv_ori * (_current_angular_velocity -
											tmp_desired_angular_velocity) -
								 _ki_ori * _integrated_orientation_error);
	}

	// compute task force
	VectorXd force_moment_contribution(6), position_orientation_contribution(6);
	force_moment_contribution.head(3) = force_feedback_related_force;
	force_moment_contribution.tail(3) = moment_feedback_related_force;

	position_orientation_contribution.head(3) = position_related_force;
	position_orientation_contribution.tail(3) = orientation_related_force;

	_unit_mass_force = position_orientation_contribution;

	VectorXd feedforward_force_moment = VectorXd::Zero(6);
	feedforward_force_moment.head(3) = sigma_force * goal_force;
	feedforward_force_moment.tail(3) = sigma_moment * goal_moment;

	if (_closed_loop_force_control) {
		feedforward_force_moment *= _k_ff;
	}

	_linear_force_control =
		force_feedback_related_force + feedforward_force_moment.head(3);
	_linear_motion_control = position_related_force;
	_impedance_force = force_moment_contribution + feedforward_force_moment;

	// compute torque through singularity handler 
	task_joint_torques = _singularity_handler->computeTorques(_unit_mass_force, _impedance_force);

	return task_joint_torques;
}

void MotionForceTask::enableInternalOtgAccelerationLimited(
	const double max_linear_velelocity, const double max_linear_acceleration,
	const double max_angular_velocity, const double max_angular_acceleration) {
	_otg->setMaxLinearVelocity(max_linear_velelocity);
	_otg->setMaxLinearAcceleration(max_linear_acceleration);
	_otg->setMaxAngularVelocity(max_angular_velocity);
	_otg->setMaxAngularAcceleration(max_angular_acceleration);
	_otg->disableJerkLimits();
	if (!_use_internal_otg_flag) {
		_otg->reInitialize(_current_position, _current_orientation);
	}
	_use_internal_otg_flag = true;
}

void MotionForceTask::enableInternalOtgJerkLimited(
	const double max_linear_velelocity, const double max_linear_acceleration,
	const double max_linear_jerk, const double max_angular_velocity,
	const double max_angular_acceleration, const double max_angular_jerk) {
	_otg->setMaxLinearVelocity(max_linear_velelocity);
	_otg->setMaxLinearAcceleration(max_linear_acceleration);
	_otg->setMaxAngularVelocity(max_angular_velocity);
	_otg->setMaxAngularAcceleration(max_angular_acceleration);
	_otg->setMaxJerk(max_linear_jerk, max_angular_jerk);
	if (!_use_internal_otg_flag) {
		_otg->reInitialize(_current_position, _current_orientation);
	}
	_use_internal_otg_flag = true;
}

Vector3d MotionForceTask::getPositionError() const {
	return sigmaPosition() * (_goal_position - _current_position);
}

Vector3d MotionForceTask::getOrientationError() const {
	return sigmaOrientation() * _orientation_error;
}

Vector3d MotionForceTask::getLinearVelocityError() const {
	return sigmaPosition() * (_goal_linear_velocity - _current_linear_velocity);
}

Vector3d MotionForceTask::getAngularVelocityError() const {
	return sigmaOrientation() * (_goal_angular_velocity - _current_angular_velocity);
}

bool MotionForceTask::goalPositionReached(const double tolerance,
										  const bool verbose) {
	double position_error =
		(_goal_position - _current_position).transpose() * sigmaPosition() *
		(_goal_position - _current_position);
	position_error = sqrt(position_error);
	bool goal_reached = position_error < tolerance;
	if (verbose) {
		cout << "position error in MotionForceTask : " << position_error
			 << endl;
		cout << "Tolerance : " << tolerance << endl;
		cout << "Goal reached : " << goal_reached << endl << endl;
	}

	return goal_reached;
}

bool MotionForceTask::goalOrientationReached(const double tolerance,
											 const bool verbose) {
	double orientation_error = _orientation_error.transpose() *
							   sigmaOrientation() * _orientation_error;
	orientation_error = sqrt(orientation_error);
	bool goal_reached = orientation_error < tolerance;
	if (verbose) {
		cout << "orientation error in MotionForceTask : " << orientation_error
			 << endl;
		cout << "Tolerance : " << tolerance << endl;
		cout << "Goal reached : " << goal_reached << endl << endl;
	}

	return goal_reached;
}

void MotionForceTask::setPosControlGains(double kp_pos, double kv_pos,
										 double ki_pos) {
	if (kp_pos < 0 || kv_pos < 0 || ki_pos < 0) {
		throw invalid_argument(
			"all gains should be positive or zero in "
			"MotionForceTask::setPosControlGains\n");
	}
	if (kv_pos < 1e-2 && _use_velocity_saturation_flag) {
		throw invalid_argument(
			"cannot have kv_pos = 0 if using velocity saturation in "
			"MotionForceTask::setPosControlGains\n");
	}
	_are_pos_gains_isotropic = true;
	_kp_pos = kp_pos * Matrix3d::Identity();
	_kv_pos = kv_pos * Matrix3d::Identity();
	_ki_pos = ki_pos * Matrix3d::Identity();
}

void MotionForceTask::setPosControlGains(const Vector3d& kp_pos,
										 const Vector3d& kv_pos,
										 const Vector3d& ki_pos) {
	if (kp_pos.minCoeff() < 0 || kv_pos.minCoeff() < 0 ||
		ki_pos.minCoeff() < 0) {
		throw invalid_argument(
			"all gains should be positive or zero in "
			"MotionForceTask::setPosControlGains\n");
	}
	if (kv_pos.minCoeff() < 1e-2 && _use_velocity_saturation_flag) {
		throw invalid_argument(
			"cannot have kv_pos = 0 if using velocity saturation in "
			"MotionForceTask::setPosControlGains\n");
	}
	_are_pos_gains_isotropic = false;
	_kp_pos = kp_pos.asDiagonal();
	_kv_pos = kv_pos.asDiagonal();
	_ki_pos = ki_pos.asDiagonal();
}

void MotionForceTask::setPosControlGains(const VectorXd& kp_pos,
										 const VectorXd& kv_pos,
										 const VectorXd& ki_pos) {
	if (kp_pos.size() == 1 && kv_pos.size() == 1 && ki_pos.size() == 1) {
		setPosControlGains(kp_pos(0), kv_pos(0), ki_pos(0));
		return;
	}
	if (kp_pos.size() == 3 && kv_pos.size() == 3 && ki_pos.size() == 3) {
		setPosControlGains(kp_pos, kv_pos, ki_pos);
		return;
	}
	throw invalid_argument(
		"kp_pos, kv_pos and ki_pos should be of size 1 or 3 in "
		"MotionForceTask::setPosControlGains\n");
}

vector<PIDGains> MotionForceTask::getPosControlGains() const {
	if (_are_pos_gains_isotropic) {
		return vector<PIDGains>(
			1, PIDGains(_kp_pos(0, 0), _kv_pos(0, 0), _ki_pos(0, 0)));
	}
	Vector3d aniso_kp_robot_base = _kp_pos.diagonal();
	Vector3d aniso_kv_robot_base = _kv_pos.diagonal();
	Vector3d aniso_ki_robot_base = _ki_pos.diagonal();
	return vector<PIDGains>{
		PIDGains(aniso_kp_robot_base(0), aniso_kv_robot_base(0),
				 aniso_ki_robot_base(0)),
		PIDGains(aniso_kp_robot_base(1), aniso_kv_robot_base(1),
				 aniso_ki_robot_base(1)),
		PIDGains(aniso_kp_robot_base(2), aniso_kv_robot_base(2),
				 aniso_ki_robot_base(2))};
}

void MotionForceTask::setOriControlGains(double kp_ori, double kv_ori,
										 double ki_ori) {
	if (kp_ori < 0 || kv_ori < 0 || ki_ori < 0) {
		throw invalid_argument(
			"all gains should be positive or zero in "
			"MotionForceTask::setOriControlGains\n");
	}
	if (kv_ori < 1e-2 && _use_velocity_saturation_flag) {
		throw invalid_argument(
			"cannot have kv_ori = 0 if using velocity saturation in "
			"MotionForceTask::setOriControlGains\n");
	}
	_are_ori_gains_isotropic = true;
	_kp_ori = kp_ori * Matrix3d::Identity();
	_kv_ori = kv_ori * Matrix3d::Identity();
	_ki_ori = ki_ori * Matrix3d::Identity();
}

void MotionForceTask::setOriControlGains(const Vector3d& kp_ori,
										 const Vector3d& kv_ori,
										 const Vector3d& ki_ori) {
	if (kp_ori.minCoeff() < 0 || kv_ori.minCoeff() < 0 ||
		ki_ori.minCoeff() < 0) {
		throw invalid_argument(
			"all gains should be positive or zero in "
			"MotionForceTask::setOriControlGains\n");
	}
	if (kv_ori.minCoeff() < 1e-2 && _use_velocity_saturation_flag) {
		throw invalid_argument(
			"cannot have kv_ori = 0 if using velocity saturation in "
			"MotionForceTask::setOriControlGains\n");
	}
	_are_ori_gains_isotropic = false;
	_kp_ori = kp_ori.asDiagonal();
	_kv_ori = kv_ori.asDiagonal();
	_ki_ori = ki_ori.asDiagonal();
}

void MotionForceTask::setOriControlGains(const VectorXd& kp_ori,
										 const VectorXd& kv_ori,
										 const VectorXd& ki_ori) {
	if (kp_ori.size() == 1 && kv_ori.size() == 1 && ki_ori.size() == 1) {
		setOriControlGains(kp_ori(0), kv_ori(0), ki_ori(0));
		return;
	}
	if (kp_ori.size() == 3 && kv_ori.size() == 3 && ki_ori.size() == 3) {
		setOriControlGains(kp_ori, kv_ori, ki_ori);
		return;
	}
	throw invalid_argument(
		"kp_ori, kv_ori and ki_ori should be of size 1 or 3 in "
		"MotionForceTask::setOriControlGains\n");
}

vector<PIDGains> MotionForceTask::getOriControlGains() const {
	if (_are_ori_gains_isotropic) {
		return vector<PIDGains>(
			1, PIDGains(_kp_ori(0, 0), _kv_ori(0, 0), _ki_ori(0, 0)));
	}
	Vector3d aniso_kp_robot_base = _kp_ori.diagonal();
	Vector3d aniso_kv_robot_base = _kv_ori.diagonal();
	Vector3d aniso_ki_robot_base = _ki_ori.diagonal();
	return vector<PIDGains>{
		PIDGains(aniso_kp_robot_base(0), aniso_kv_robot_base(0),
				 aniso_ki_robot_base(0)),
		PIDGains(aniso_kp_robot_base(1), aniso_kv_robot_base(1),
				 aniso_ki_robot_base(1)),
		PIDGains(aniso_kp_robot_base(2), aniso_kv_robot_base(2),
				 aniso_ki_robot_base(2))};
}

Vector3d MotionForceTask::getGoalForce() const {
	Matrix3d rotation = _is_force_motion_parametrization_in_compliant_frame
							? getConstRobotModel()->rotationInWorld(
								  _link_name, _compliant_frame.rotation())
							: Matrix3d::Identity();
	return rotation * _goal_force;
}

Vector3d MotionForceTask::getGoalMoment() const {
	Matrix3d rotation = _is_force_motion_parametrization_in_compliant_frame
							? getConstRobotModel()->rotationInWorld(
								  _link_name, _compliant_frame.rotation())
							: Matrix3d::Identity();
	return rotation * _goal_moment;
}

void MotionForceTask::enableVelocitySaturation(const double linear_vel_sat,
											   const double angular_vel_sat) {
	if (linear_vel_sat <= 0 || angular_vel_sat <= 0) {
		throw invalid_argument(
			"Velocity saturation values should be strictly positive or zero in "
			"MotionForceTask::enableVelocitySaturation\n");
	}
	if (_kv_pos.determinant() < 1e-3) {
		throw invalid_argument(
			"Cannot enable velocity saturation if kv_pos is singular in "
			"MotionForceTask::enableVelocitySaturation\n");
	}
	if (_kv_ori.determinant() < 1e-3) {
		throw invalid_argument(
			"Cannot enable velocity saturation if kv_ori is singular in "
			"MotionForceTask::enableVelocitySaturation\n");
	}
	_use_velocity_saturation_flag = true;
	_linear_saturation_velocity = linear_vel_sat;
	_angular_saturation_velocity = angular_vel_sat;
}

void MotionForceTask::setForceSensorFrame(
	const string link_name, const Affine3d transformation_in_link) {
	if (link_name != _link_name) {
		throw invalid_argument(
			"The link to which is attached the sensor should be the same as "
			"the link to which is attached the control frame in "
			"MotionForceTask::setForceSensorFrame\n");
	}
	_T_control_to_sensor = _compliant_frame.inverse() * transformation_in_link;
	_force_sensor->setForceSensorFrame(link_name, _compliant_frame);
}

void MotionForceTask::updateSensedForceAndMoment(
	const Vector3d sensed_force_sensor_frame,
	const Vector3d sensed_moment_sensor_frame) {
	// find the transform from base frame to control frame
	Affine3d T_world_link = getConstRobotModel()->transformInWorld(_link_name);
	Affine3d T_world_compliant_frame = T_world_link * _compliant_frame;

	// compensate force sensor 
	// _force_sensor->getCalibratedForceMoment(sensed_force_sensor_frame, sensed_moment_sensor_frame)

	// find the resolved sensed force and moment in control frame
	_sensed_force = _T_control_to_sensor.rotation() * sensed_force_sensor_frame;
	_sensed_moment =
		_T_control_to_sensor.translation().cross(_sensed_force) +
		_T_control_to_sensor.rotation() * sensed_moment_sensor_frame;

	// rotate the quantities in base frame
	_sensed_force = T_world_compliant_frame.rotation() * _sensed_force;
	_sensed_moment = T_world_compliant_frame.rotation() * _sensed_moment;
}

bool MotionForceTask::parametrizeForceMotionSpaces(
	const int force_space_dimension,
	const Vector3d& force_or_motion_single_axis) {
	if (force_space_dimension < 0 || force_space_dimension > 3) {
		throw invalid_argument(
			"Force space dimension should be between 0 and 3 in "
			"MotionForceTask::parametrizeForceMotionSpaces\n");
	}
	bool reset = force_space_dimension != _force_space_dimension;
	_force_space_dimension = force_space_dimension;
	if (force_space_dimension == 1 || force_space_dimension == 2) {
		if (force_or_motion_single_axis.norm() < 1e-2) {
			throw invalid_argument(
				"Force or motion axis should be a non singular vector in "
				"MotionForceTask::parametrizeForceMotionSpaces\n");
		}
		reset = reset || !force_or_motion_single_axis.normalized().isApprox(
							 _force_or_motion_axis);
		_force_or_motion_axis = force_or_motion_single_axis.normalized();
	}
	if (reset) {
		_goal_position = _current_position;
		_goal_linear_velocity.setZero();
		_goal_linear_acceleration.setZero();
		_otg->reInitializeLinear(_current_position);
		resetIntegratorsLinear();
	}
	return reset;
}

bool MotionForceTask::parametrizeMomentRotMotionSpaces(
	const int moment_space_dimension,
	const Vector3d& moment_or_rot_motion_single_axis) {
	if (moment_space_dimension < 0 || moment_space_dimension > 3) {
		throw invalid_argument(
			"Moment space dimension should be between 0 and 3 in "
			"MotionForceTask::parametrizeMomentRotMotionSpaces\n");
	}
	bool reset = moment_space_dimension != _moment_space_dimension;
	_moment_space_dimension = moment_space_dimension;
	if (moment_space_dimension == 1 || moment_space_dimension == 2) {
		if (moment_or_rot_motion_single_axis.norm() < 1e-2) {
			throw invalid_argument(
				"Moment or rot motion axis should be a non singular vector in "
				"MotionForceTask::parametrizeMomentRotMotionSpaces\n");
		}
		reset =
			reset || !moment_or_rot_motion_single_axis.normalized().isApprox(
						 _moment_or_rotmotion_axis);
		_moment_or_rotmotion_axis =
			moment_or_rot_motion_single_axis.normalized();
	}
	if (reset) {
		_goal_orientation = _current_orientation;
		_goal_angular_velocity.setZero();
		_goal_angular_acceleration.setZero();
		_otg->reInitializeAngular(_current_orientation);
		resetIntegratorsAngular();
	}
	return reset;
}

Matrix3d MotionForceTask::sigmaForce() const {
	Matrix3d rotation = _is_force_motion_parametrization_in_compliant_frame
							? getConstRobotModel()->rotationInWorld(
								  _link_name, _compliant_frame.rotation())
							: Matrix3d::Identity();
	switch (_force_space_dimension) {
		case 0:
			return Matrix3d::Zero();
			break;
		case 1:
			return posSelectionProjector() * rotation * _force_or_motion_axis *
				   _force_or_motion_axis.transpose() * rotation.transpose() *
				   posSelectionProjector().transpose();
			break;
		case 2:
			return posSelectionProjector() *
				   (Matrix3d::Identity() -
					rotation * _force_or_motion_axis *
						_force_or_motion_axis.transpose() *
						rotation.transpose()) *
				   posSelectionProjector().transpose();
			break;
		case 3:
			return posSelectionProjector();
			break;

		default:
			// should never happen
			throw invalid_argument(
				"Force space dimension should be between 0 and 3 in "
				"MotionForceTask::sigmaForce\n");
			break;
	}
}

Matrix3d MotionForceTask::sigmaPosition() const {
	return posSelectionProjector() * (Matrix3d::Identity() - sigmaForce()) *
		   posSelectionProjector().transpose();
}

Matrix3d MotionForceTask::sigmaMoment() const {
	Matrix3d rotation = _is_force_motion_parametrization_in_compliant_frame
							? getConstRobotModel()->rotationInWorld(
								  _link_name, _compliant_frame.rotation())
							: Matrix3d::Identity();
	switch (_moment_space_dimension) {
		case 0:
			return Matrix3d::Zero();
			break;
		case 1:
			return oriSelectionProjector() * rotation *
				   _moment_or_rotmotion_axis *
				   _moment_or_rotmotion_axis.transpose() *
				   rotation.transpose() * oriSelectionProjector().transpose();
			break;
		case 2:
			return oriSelectionProjector() *
				   (Matrix3d::Identity() -
					rotation * _moment_or_rotmotion_axis *
						_moment_or_rotmotion_axis.transpose() *
						rotation.transpose()) *
				   oriSelectionProjector().transpose();
			break;
		case 3:
			return oriSelectionProjector();
			break;

		default:
			// should never happen
			throw invalid_argument(
				"Moment space dimension should be between 0 and 3 in "
				"MotionForceTask::sigmaMoment\n");
			break;
	}
}

Matrix3d MotionForceTask::sigmaOrientation() const {
	return oriSelectionProjector() * (Matrix3d::Identity() - sigmaMoment()) *
		   oriSelectionProjector().transpose();
}

void MotionForceTask::resetIntegrators() {
	resetIntegratorsLinear();
	resetIntegratorsAngular();
}

void MotionForceTask::resetIntegratorsLinear() {
	_integrated_position_error.setZero();
	_integrated_force_error.setZero();
}

void MotionForceTask::resetIntegratorsAngular() {
	_integrated_orientation_error.setZero();
	_integrated_moment_error.setZero();
}

} /* namespace Sai2Primitives */
