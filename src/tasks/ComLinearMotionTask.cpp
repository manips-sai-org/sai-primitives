/*
 * ComLinearMotionTask.cpp
 */

#include "ComLinearMotionTask.h"

#include <stdexcept>

using namespace std;
using namespace Eigen;

namespace SaiPrimitives {

namespace {
const double MAX_FEEDBACK_FORCE_FORCE_CONTROLLER = 20.0;
}  // namespace

ComLinearMotionTask::ComLinearMotionTask(
	std::shared_ptr<SaiModel::SaiModel>& robot, const string& link_name,
	const Affine3d& compliant_frame, const std::string& task_name,
	const bool is_force_motion_parametrization_in_compliant_frame,
	const double loop_timestep)
	: TemplateTask(robot, task_name, TaskType::MOTION_FORCE_TASK,
				   loop_timestep) {
	_link_name = link_name;
	_compliant_frame = compliant_frame;
	_is_force_motion_parametrization_in_compliant_frame =
		is_force_motion_parametrization_in_compliant_frame;
	_partial_task_projection = Matrix3d::Identity();

	initialSetup();
}

ComLinearMotionTask::ComLinearMotionTask(
	std::shared_ptr<SaiModel::SaiModel>& robot, const string& link_name,
	std::vector<Vector3d> controlled_directions_translation,
	const Affine3d& compliant_frame, const std::string& task_name,
	const bool is_force_motion_parametrization_in_compliant_frame,
	const double loop_timestep)
	: TemplateTask(robot, task_name, TaskType::MOTION_FORCE_TASK,
				   loop_timestep) {
	_link_name = link_name;
	_compliant_frame = compliant_frame;
	_is_force_motion_parametrization_in_compliant_frame =
		is_force_motion_parametrization_in_compliant_frame;

	if (controlled_directions_translation.empty()) {
		throw invalid_argument(
			"controlled_directions_translation cannot be empty in "
			"ComLinearMotionTask::ComLinearMotionTask\n");
	}

	MatrixXd controlled_translation_vectors =
		MatrixXd::Zero(3, controlled_directions_translation.size());
	for (int i = 0; i < controlled_directions_translation.size(); i++) {
		controlled_translation_vectors.col(i) =
			controlled_directions_translation[i];
	}
	MatrixXd controlled_translation_range_basis =
		SaiModel::matrixRangeBasis(controlled_translation_vectors);
	_partial_task_projection =
		controlled_translation_range_basis *
		controlled_translation_range_basis.transpose();

	initialSetup();
}

void ComLinearMotionTask::initialSetup() {
	int dof = getConstRobotModel()->dof();
	_T_control_to_sensor = Affine3d::Identity();

	_POPC_force.reset(new POPCExplicitForceControl(getLoopTimestep()));

	_current_position = getConstRobotModel()->comPosition();

	setPosControlGains(50.0, 14.0, 0.0);
	setForceControlGains(0.7, 10.0, 1.3);

	disableVelocitySaturation();
	_linear_saturation_velocity = 0;

	_k_ff = 0.95;

	_force_space_dimension = 0;
	setClosedLoopForceControl(false);

	_jacobian.setZero(3, dof);
	_projected_jacobian.setZero(3, dof);
	_Lambda_modified.setZero(3, 3);
	_N.setZero(dof, dof);
	_N_prec = MatrixXd::Identity(dof, dof);

	MatrixXd range_pos =
		SaiModel::matrixRangeBasis(_partial_task_projection);
	_pos_range = range_pos.norm() == 0 ? 0 : range_pos.cols();
	if (_pos_range == 0) {
		throw invalid_argument(
			"controlled_directions_translation cannot be empty in "
			"ComLinearMotionTask::ComLinearMotionTask\n");
	}

	_current_task_range.setZero(3, _pos_range);
	_current_task_range = range_pos;

	_otg = make_unique<OTG_6dof_cartesian>(
		_current_position, Matrix3d::Identity(), getLoopTimestep());
	enableInternalOtgAccelerationLimited(0.3, 1.0);

	setDynamicDecouplingType(DefaultParameters::dynamic_decoupling_type);
	setBieThreshold(DefaultParameters::bie_threshold);
	setSingularityThreshold(DefaultParameters::s_abs_tol);

	reInitializeTask();
}

void ComLinearMotionTask::reInitializeTask() {
	_current_position = getConstRobotModel()->comPosition();
	_goal_position = _current_position;

	_current_linear_velocity.setZero();
	_goal_linear_velocity.setZero();
	_goal_linear_acceleration.setZero();

	_goal_force.setZero();
	_sensed_force.setZero();

	resetIntegrators();

	_task_force.setZero(3);
	_unit_mass_force.setZero(3);

	_otg->reInitializeLinear(_current_position);
}

void ComLinearMotionTask::updateTaskModel(const MatrixXd& N_prec) {
	const int robot_dof = getConstRobotModel()->dof();
	if (N_prec.rows() != N_prec.cols()) {
		throw invalid_argument(
			"N_prec matrix not square in "
			"ComLinearMotionTask::updateTaskModel\n");
	}
	if (N_prec.rows() != robot_dof) {
		throw invalid_argument(
			"N_prec matrix size not consistent with robot dof in "
			"ComLinearMotionTask::updateTaskModel\n");
	}

	_N_prec = N_prec;

	_jacobian = _partial_task_projection * getConstRobotModel()->comJacobian();
	_projected_jacobian = _jacobian * _N_prec;
	_current_task_range =
		SaiModel::matrixRangeBasis(_projected_jacobian, _s_abs_tol);
	_N = getConstRobotModel()->nullspaceMatrix(
		_current_task_range.transpose() * _projected_jacobian);
}

VectorXd ComLinearMotionTask::computeTorques(
	const Eigen::VectorXd& tau_prec) {
	VectorXd task_torques = computeTorques();
	VectorXd disturbance_compensation =
		(_current_task_range.transpose() * _projected_jacobian).transpose() *
		_Lambda_modified * _current_task_range.transpose() * _jacobian *
		getConstRobotModel()->MInv() * tau_prec;
	return task_torques - disturbance_compensation;
}

VectorXd ComLinearMotionTask::computeTorques() {
	VectorXd task_joint_torques = VectorXd::Zero(getConstRobotModel()->dof());

	_jacobian = _partial_task_projection * getConstRobotModel()->comJacobian();
	_projected_jacobian = _jacobian * _N_prec;

	_current_position = getConstRobotModel()->comPosition();
	_current_linear_velocity = _projected_jacobian * getConstRobotModel()->dq();

	if (_current_task_range.cols() == 0) {
		return task_joint_torques;
	}

	Matrix3d sigma_force = sigmaForce();
	Matrix3d sigma_position = sigmaPosition();

	Vector3d goal_force = getGoalForce();
	Vector3d force_feedback_related_force = Vector3d::Zero();
	Vector3d position_related_force = Vector3d::Zero();

	if (_closed_loop_force_control) {
		_integrated_force_error +=
			sigma_force * (_sensed_force - goal_force) * getLoopTimestep();

		Vector3d force_feedback_term =
			sigma_force * (-_kp_force * (_sensed_force - goal_force) -
						   _ki_force * _integrated_force_error);
		if (force_feedback_term.norm() > MAX_FEEDBACK_FORCE_FORCE_CONTROLLER) {
			force_feedback_term *= MAX_FEEDBACK_FORCE_FORCE_CONTROLLER /
								   force_feedback_term.norm();
		}

		force_feedback_related_force =
			_POPC_force->computePassivitySaturatedForce(
				sigma_force * goal_force, sigma_force * _sensed_force,
				sigma_force * force_feedback_term,
				sigma_force * _current_linear_velocity, _kv_force, _k_ff);
	} else {
		force_feedback_related_force =
			sigma_force * (-_kv_force * _current_linear_velocity);
	}

	Vector3d tmp_desired_position = _goal_position;
	Vector3d tmp_desired_linear_velocity = _goal_linear_velocity;
	Vector3d tmp_desired_acceleration = _goal_linear_acceleration;

	if (_use_internal_otg_flag) {
		_otg->setGoalPositionAndLinearVelocity(_goal_position,
											   _goal_linear_velocity);
		_otg->update();

		tmp_desired_position = _otg->getNextPosition();
		tmp_desired_linear_velocity = _otg->getNextLinearVelocity();
		tmp_desired_acceleration = _otg->getNextLinearAcceleration();
	}

	_integrated_position_error += sigma_position *
								  (_current_position - tmp_desired_position) *
								  getLoopTimestep();

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

	Vector3d feedforward_force = sigma_force * goal_force;
	if (_closed_loop_force_control) {
		feedforward_force *= _k_ff;
	}

	_linear_force_control = force_feedback_related_force + feedforward_force;
	_linear_motion_control = position_related_force;
	_unit_mass_force = position_related_force;

	switch (_dynamic_decoupling_type) {
		case FULL_DYNAMIC_DECOUPLING: {
			_Lambda_modified = getConstRobotModel()->taskInertiaMatrix(
				_current_task_range.transpose() * _projected_jacobian);
			break;
		}

		case IMPEDANCE: {
			_Lambda_modified = MatrixXd::Identity(_projected_jacobian.rows(),
												  _projected_jacobian.rows());
			_current_task_range = MatrixXd::Identity(_projected_jacobian.rows(),
													 _projected_jacobian.rows());
			break;
		}

		case BOUNDED_INERTIA_ESTIMATES: {
			MatrixXd M_BIE = getConstRobotModel()->M();
			for (int i = 0; i < getConstRobotModel()->dof(); i++) {
				if (M_BIE(i, i) < _bie_threshold) {
					M_BIE(i, i) = _bie_threshold;
				}
			}
			MatrixXd M_inv_BIE = M_BIE.llt().solve(MatrixXd::Identity(
				getConstRobotModel()->dof(), getConstRobotModel()->dof()));
			MatrixXd Lambda_inv =
				_current_task_range.transpose() * _projected_jacobian *
				M_inv_BIE *
				(_current_task_range.transpose() * _projected_jacobian)
					.transpose();
			_Lambda_modified = Lambda_inv.llt().solve(
				MatrixXd::Identity(Lambda_inv.rows(), Lambda_inv.rows()));
			break;
		}

		default: {
			_Lambda_modified = getConstRobotModel()->taskInertiaMatrix(
				_current_task_range.transpose() * _projected_jacobian);
			break;
		}
	}

	task_joint_torques =
		(_current_task_range.transpose() * _projected_jacobian).transpose() *
		(_Lambda_modified * _current_task_range.transpose() * _unit_mass_force +
		 _current_task_range.transpose() *
			 (force_feedback_related_force + feedforward_force));

	return task_joint_torques;
}

void ComLinearMotionTask::enableInternalOtgAccelerationLimited(
	const double max_linear_velocity, const double max_linear_acceleration) {
	_otg->setMaxLinearVelocity(max_linear_velocity);
	_otg->setMaxLinearAcceleration(max_linear_acceleration);
	_otg->disableJerkLimits();
	if (!_use_internal_otg_flag) {
		_otg->reInitializeLinear(_current_position);
	}
	_use_internal_otg_flag = true;
}

void ComLinearMotionTask::enableInternalOtgJerkLimited(
	const double max_linear_velocity, const double max_linear_acceleration,
	const double max_linear_jerk) {
	_otg->setMaxLinearVelocity(max_linear_velocity);
	_otg->setMaxLinearAcceleration(max_linear_acceleration);
	_otg->setMaxJerk(max_linear_jerk * Vector3d::Ones(),
					 _otg->getMaxAngularJerk());
	if (!_use_internal_otg_flag) {
		_otg->reInitializeLinear(_current_position);
	}
	_use_internal_otg_flag = true;
}

Vector3d ComLinearMotionTask::getPositionError() const {
	return sigmaPosition() * (_goal_position - _current_position);
}

Vector3d ComLinearMotionTask::getLinearVelocityError() const {
	return sigmaPosition() * (_goal_linear_velocity - _current_linear_velocity);
}

bool ComLinearMotionTask::goalPositionReached(const double tolerance,
											  const bool verbose) {
	double position_error =
		(_goal_position - _current_position).transpose() * sigmaPosition() *
		(_goal_position - _current_position);
	position_error = sqrt(position_error);
	bool goal_reached = position_error < tolerance;
	if (verbose) {
		cout << "position error in ComLinearMotionTask : " << position_error
			 << endl;
		cout << "Tolerance : " << tolerance << endl;
		cout << "Goal reached : " << goal_reached << endl << endl;
	}

	return goal_reached;
}

void ComLinearMotionTask::setPosControlGains(double kp_pos, double kv_pos,
											 double ki_pos) {
	if (kp_pos < 0 || kv_pos < 0 || ki_pos < 0) {
		throw invalid_argument(
			"all gains should be positive or zero in "
			"ComLinearMotionTask::setPosControlGains\n");
	}
	if (kv_pos < 1e-2 && _use_velocity_saturation_flag) {
		throw invalid_argument(
			"cannot have kv_pos = 0 if using velocity saturation in "
			"ComLinearMotionTask::setPosControlGains\n");
	}
	_are_pos_gains_isotropic = true;
	_kp_pos = kp_pos * Matrix3d::Identity();
	_kv_pos = kv_pos * Matrix3d::Identity();
	_ki_pos = ki_pos * Matrix3d::Identity();
}

void ComLinearMotionTask::setPosControlGains(const Vector3d& kp_pos,
											 const Vector3d& kv_pos,
											 const Vector3d& ki_pos) {
	if (kp_pos.minCoeff() < 0 || kv_pos.minCoeff() < 0 ||
		ki_pos.minCoeff() < 0) {
		throw invalid_argument(
			"all gains should be positive or zero in "
			"ComLinearMotionTask::setPosControlGains\n");
	}
	if (kv_pos.minCoeff() < 1e-2 && _use_velocity_saturation_flag) {
		throw invalid_argument(
			"cannot have kv_pos = 0 if using velocity saturation in "
			"ComLinearMotionTask::setPosControlGains\n");
	}
	_are_pos_gains_isotropic = false;
	_kp_pos = kp_pos.asDiagonal();
	_kv_pos = kv_pos.asDiagonal();
	_ki_pos = ki_pos.asDiagonal();
}

void ComLinearMotionTask::setPosControlGains(const VectorXd& kp_pos,
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
		"ComLinearMotionTask::setPosControlGains\n");
}

vector<PIDGains> ComLinearMotionTask::getPosControlGains() const {
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

Vector3d ComLinearMotionTask::getGoalForce() const {
	Matrix3d rotation = _is_force_motion_parametrization_in_compliant_frame
							? getConstRobotModel()->rotationInWorld(
								  _link_name, _compliant_frame.rotation())
							: Matrix3d::Identity();
	return rotation * _goal_force;
}

void ComLinearMotionTask::enableVelocitySaturation(
	const double linear_vel_sat) {
	if (linear_vel_sat <= 0) {
		throw invalid_argument(
			"Velocity saturation values should be strictly positive or zero in "
			"ComLinearMotionTask::enableVelocitySaturation\n");
	}
	if (_kv_pos.determinant() < 1e-3) {
		throw invalid_argument(
			"Cannot enable velocity saturation if kv_pos is singular in "
			"ComLinearMotionTask::enableVelocitySaturation\n");
	}
	_use_velocity_saturation_flag = true;
	_linear_saturation_velocity = linear_vel_sat;
}

void ComLinearMotionTask::setForceSensorFrame(
	const string link_name, const Affine3d transformation_in_link) {
	if (link_name != _link_name) {
		throw invalid_argument(
			"The link to which is attached the sensor should be the same as "
			"the link to which is attached the control frame in "
			"ComLinearMotionTask::setForceSensorFrame\n");
	}
	_T_control_to_sensor = _compliant_frame.inverse() * transformation_in_link;
}

void ComLinearMotionTask::updateSensedForce(
	const Vector3d sensed_force_sensor_frame) {
	Affine3d T_world_link = getConstRobotModel()->transformInWorld(_link_name);
	Affine3d T_world_compliant_frame = T_world_link * _compliant_frame;

	_sensed_force = _T_control_to_sensor.rotation() * sensed_force_sensor_frame;
	_sensed_force = T_world_compliant_frame.rotation() * _sensed_force;
}

bool ComLinearMotionTask::parametrizeForceMotionSpaces(
	const int force_space_dimension,
	const Vector3d& force_or_motion_single_axis) {
	if (force_space_dimension < 0 || force_space_dimension > 3) {
		throw invalid_argument(
			"Force space dimension should be between 0 and 3 in "
			"ComLinearMotionTask::parametrizeForceMotionSpaces\n");
	}
	bool reset = force_space_dimension != _force_space_dimension;
	_force_space_dimension = force_space_dimension;
	if (force_space_dimension == 1 || force_space_dimension == 2) {
		if (force_or_motion_single_axis.norm() < 1e-2) {
			throw invalid_argument(
				"Force or motion axis should be a non singular vector in "
				"ComLinearMotionTask::parametrizeForceMotionSpaces\n");
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

Matrix3d ComLinearMotionTask::sigmaForce() const {
	Matrix3d rotation = _is_force_motion_parametrization_in_compliant_frame
							? getConstRobotModel()->rotationInWorld(
								  _link_name, _compliant_frame.rotation())
							: Matrix3d::Identity();
	switch (_force_space_dimension) {
		case 0:
			return Matrix3d::Zero();
		case 1:
			return posSelectionProjector() * rotation * _force_or_motion_axis *
				   _force_or_motion_axis.transpose() * rotation.transpose() *
				   posSelectionProjector().transpose();
		case 2:
			return posSelectionProjector() *
				   (Matrix3d::Identity() -
					rotation * _force_or_motion_axis *
						_force_or_motion_axis.transpose() *
						rotation.transpose()) *
				   posSelectionProjector().transpose();
		case 3:
			return posSelectionProjector();
		default:
			throw invalid_argument(
				"Force space dimension should be between 0 and 3 in "
				"ComLinearMotionTask::sigmaForce\n");
	}
}

Matrix3d ComLinearMotionTask::sigmaPosition() const {
	return posSelectionProjector() * (Matrix3d::Identity() - sigmaForce()) *
		   posSelectionProjector().transpose();
}

void ComLinearMotionTask::resetIntegrators() {
	resetIntegratorsLinear();
}

void ComLinearMotionTask::resetIntegratorsLinear() {
	_integrated_position_error.setZero();
	_integrated_force_error.setZero();
}

} /* namespace SaiPrimitives */
