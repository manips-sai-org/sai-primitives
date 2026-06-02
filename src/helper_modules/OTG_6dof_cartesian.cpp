/**
 * OTG_6dof_cartesian.cpp
 *
 *	A wrapper to use the Ruckig OTG library
 *	specifically to work for 6DOF position and orientation
 *
 * Author: Mikael Jorda
 * Created: August 2023
 */

#include "OTG_6dof_cartesian.h"

#include <cmath>
#include <string>

using namespace Eigen;
using namespace ruckig;

namespace SaiPrimitives {

namespace {
bool isValidRotation(const Matrix3d mat) {
	if ((mat.transpose() * mat - Matrix3d::Identity()).norm() > 1e-3) {
		return false;
	}
	if (abs(mat.determinant() - 1) > 1e-3) {
		return false;
	}
	return true;
}

void validateTrackingTargetLimitVector(const Vector3d& limits,
									   const std::string& name) {
	for (int i = 0; i < limits.size(); ++i) {
		if (std::isnan(limits(i)) || limits(i) < 0.0) {
			throw std::invalid_argument(
				name +
				" values should be positive or zero in "
				"OTG_6dof_cartesian::setTrackingTargetLimits\n");
		}
	}
}

Matrix3d skewSymmetric(const Vector3d& v) {
	Matrix3d skew;
	skew << 0.0, -v.z(), v.y(), v.z(), 0.0, -v.x(), -v.y(), v.x(), 0.0;
	return skew;
}

Matrix3d leftJacobianSO3(const Vector3d& phi) {
	const double theta = phi.norm();
	const Matrix3d phi_cross = skewSymmetric(phi);
	const Matrix3d phi_cross_squared = phi_cross * phi_cross;

	if (theta < 1e-6) {
		return Matrix3d::Identity() + 0.5 * phi_cross +
			   (1.0 / 6.0) * phi_cross_squared;
	}

	const double theta_squared = theta * theta;
	return Matrix3d::Identity() +
		   ((1.0 - cos(theta)) / theta_squared) * phi_cross +
		   ((theta - sin(theta)) / (theta_squared * theta)) *
			   phi_cross_squared;
}

Matrix3d leftJacobianSO3Inverse(const Vector3d& phi) {
	const double theta = phi.norm();
	const Matrix3d phi_cross = skewSymmetric(phi);
	const Matrix3d phi_cross_squared = phi_cross * phi_cross;

	if (theta < 1e-6) {
		return Matrix3d::Identity() - 0.5 * phi_cross +
			   (1.0 / 12.0) * phi_cross_squared;
	}

	const double theta_squared = theta * theta;
	return Matrix3d::Identity() - 0.5 * phi_cross +
		   (1.0 / theta_squared -
			(1.0 + cos(theta)) / (2.0 * theta * sin(theta))) *
			   phi_cross_squared;
}

Vector3d leftJacobianDotTimesVelocity(const Vector3d& phi,
									  const Vector3d& phi_dot) {
	if (phi_dot.norm() < 1e-12) {
		return Vector3d::Zero();
	}

	constexpr double dt = 1e-6;
	const Matrix3d jacobian_plus = leftJacobianSO3(phi + dt * phi_dot);
	const Matrix3d jacobian_minus = leftJacobianSO3(phi - dt * phi_dot);
	return ((jacobian_plus - jacobian_minus) / (2.0 * dt)) * phi_dot;
}

Matrix3d rotationVectorToMatrix(const Vector3d& phi) {
	const double angle = phi.norm();
	if (angle < 1e-9) {
		return Matrix3d::Identity();
	}

	return AngleAxisd(angle, phi / angle).toRotationMatrix();
}
}  // namespace

OTG_6dof_cartesian::OTG_6dof_cartesian(const Vector3d& initial_position,
									   const Matrix3d& initial_orientation,
									   const double loop_time) {
	_otg = std::make_shared<Ruckig<6, EigenVector>>(loop_time);
	_trackig = std::make_shared<Trackig<6, EigenVector>>(loop_time);
	_input = InputParameter<6, EigenVector>();
	_output = OutputParameter<6, EigenVector>();
	_input.synchronization = Synchronization::Phase;

	// initialize output position to zero such that the getNextOrientation
	// returns a coherent value
	_output.new_position.setZero();

	_reference_frame = initial_orientation;
	_goal_orientation_in_base_frame.setZero();
	_goal_angular_velocity_in_base_frame.setZero();
	_goal_angular_acceleration_in_base_frame.setZero();
	reInitialize(initial_position, initial_orientation);
}

void OTG_6dof_cartesian::reInitialize(const Vector3d& initial_position,
									  const Matrix3d& initial_orientation) {
	setGoalPosition(initial_position);
	setGoalOrientation(initial_orientation);
	_trackig->reset();
	_otg->reset();

	_input.current_position = _input.target_position;
	_input.current_velocity.setZero();
	_input.current_acceleration.setZero();

	_output.new_position = _input.target_position;
	_output.new_velocity.setZero();
	_output.new_acceleration.setZero();
}

void OTG_6dof_cartesian::reInitializeLinear(const Vector3d& initial_position) {
	setGoalPosition(initial_position);
	_trackig->reset();
	_otg->reset();

	_input.current_position.head<3>() = _input.target_position.head<3>();
	_input.current_velocity.head<3>().setZero();
	_input.current_acceleration.head<3>().setZero();

	_output.new_position.head<3>() = _input.target_position.head<3>();
	_output.new_velocity.head<3>().setZero();
	_output.new_acceleration.head<3>().setZero();
}

void OTG_6dof_cartesian::reInitializeAngular(
	const Matrix3d& initial_orientation) {
	setGoalOrientation(initial_orientation);
	_trackig->reset();
	_otg->reset();

	_input.current_position.tail<3>() = _input.target_position.tail<3>();
	_input.current_velocity.tail<3>().setZero();
	_input.current_acceleration.tail<3>().setZero();

	_output.new_position.tail<3>() = _input.target_position.tail<3>();
	_output.new_velocity.tail<3>().setZero();
	_output.new_acceleration.tail<3>().setZero();
}

void OTG_6dof_cartesian::setMaxLinearVelocity(
	const Vector3d& max_linear_velocity) {
	if (max_linear_velocity.minCoeff() <= 0) {
		throw std::invalid_argument(
			"max velocity set to 0 or negative value in some directions in "
			"OTG_6dof_cartesian::setMaxLinearVelocity\n");
	}
	_input.max_velocity.head<3>() = max_linear_velocity;
}

void OTG_6dof_cartesian::setMaxLinearAcceleration(
	const Vector3d& max_linear_acceleration) {
	if (max_linear_acceleration.minCoeff() <= 0) {
		throw std::invalid_argument(
			"max acceleration set to 0 or negative value in some directions in "
			"OTG_6dof_cartesian::setMaxLinearAcceleration\n");
	}
	_input.max_acceleration.head<3>() = max_linear_acceleration;
}

void OTG_6dof_cartesian::setMaxAngularVelocity(const Vector3d& max_velocity) {
	if (max_velocity.minCoeff() <= 0) {
		throw std::invalid_argument(
			"max velocity set to 0 or negative value in some directions in "
			"OTG_6dof_cartesian::setMaxAngularVelocity\n");
	}

	_input.max_velocity.tail<3>() = max_velocity;
}

void OTG_6dof_cartesian::setMaxAngularAcceleration(
	const Vector3d& max_angular_acceleration) {
	if (max_angular_acceleration.minCoeff() <= 0) {
		throw std::invalid_argument(
			"max acceleration set to 0 or negative value in some directions in "
			"OTG_6dof_cartesian::setMaxAngularAcceleration\n");
	}

	_input.max_acceleration.tail<3>() = max_angular_acceleration;
}

void OTG_6dof_cartesian::setMaxJerk(const Vector3d& max_linear_jerk,
									const Vector3d& max_angular_jerk) {
	if (max_linear_jerk.minCoeff() <= 0 || max_angular_jerk.minCoeff() <= 0) {
		throw std::invalid_argument(
			"max jerk set to 0 or negative value in some directions in "
			"OTG_6dof_cartesian::setMaxJerk\n");
	}

	_input.max_jerk.head<3>() = max_linear_jerk;
	_input.max_jerk.tail<3>() = max_angular_jerk;
}

void OTG_6dof_cartesian::setGoalPositionAndLinearVelocity(
	const Vector3d& goal_position, const Vector3d& goal_linear_velocity) {
	setGoalPositionLinearVelocityAndAcceleration(goal_position,
												 goal_linear_velocity,
												 Vector3d::Zero());
}

void OTG_6dof_cartesian::setGoalPositionLinearVelocityAndAcceleration(
	const Vector3d& goal_position, const Vector3d& goal_linear_velocity,
	const Vector3d& goal_linear_acceleration) {
	if (goal_position.isApprox(_input.target_position.head<3>(), 1e-3) &&
		goal_linear_velocity.isApprox(_input.target_velocity.head<3>(), 1e-3) &&
		goal_linear_acceleration.isApprox(_input.target_acceleration.head<3>(),
										   1e-3) &&
		!_tracking_mode_enabled) {
		return;
	}
	_goal_reached = false;
	_input.target_position.head<3>() = goal_position;
	_input.target_velocity.head<3>() = goal_linear_velocity;
	_input.target_acceleration.head<3>() = goal_linear_acceleration;
}

void OTG_6dof_cartesian::setGoalOrientationAndAngularVelocity(
	const Matrix3d& goal_orientation, const Vector3d& goal_angular_velocity) {
	setGoalOrientationAngularVelocityAndAcceleration(goal_orientation,
													 goal_angular_velocity,
													 Vector3d::Zero());
}

void OTG_6dof_cartesian::setGoalOrientationAngularVelocityAndAcceleration(
	const Matrix3d& goal_orientation, const Vector3d& goal_angular_velocity,
	const Vector3d& goal_angular_acceleration) {
	if (!isValidRotation(goal_orientation)) {
		throw std::invalid_argument(
			"goal orientation is not a valid rotation matrix "
			"OTG_6dof_cartesian::setGoalOrientationAndAngularVelocity\n");
	}

	if (_goal_orientation_in_base_frame.isApprox(goal_orientation, 1e-3) &&
		_goal_angular_velocity_in_base_frame.isApprox(goal_angular_velocity,
													  1e-3) &&
		_goal_angular_acceleration_in_base_frame.isApprox(
			goal_angular_acceleration, 1e-3) &&
		!_tracking_mode_enabled) {
		return;
	}

	_goal_reached = false;
	// the new reference frame is the current orientation
	const Matrix3d previous_reference_frame = _reference_frame;
	const Vector3d previous_rotation_vector = _output.new_position.tail<3>();
	const Vector3d previous_rotation_vector_velocity =
		_output.new_velocity.tail<3>();
	const Vector3d previous_rotation_vector_acceleration =
		_output.new_acceleration.tail<3>();
	const Vector3d previous_local_angular_velocity =
		leftJacobianSO3(previous_rotation_vector) *
		previous_rotation_vector_velocity;
	const Vector3d previous_local_angular_acceleration =
		leftJacobianSO3(previous_rotation_vector) *
			previous_rotation_vector_acceleration +
		leftJacobianDotTimesVelocity(previous_rotation_vector,
									 previous_rotation_vector_velocity);
	Matrix3d new_reference_frame = getNextOrientation();
	_reference_frame = new_reference_frame;
	_goal_orientation_in_base_frame = goal_orientation;
	_goal_angular_velocity_in_base_frame = goal_angular_velocity;
	_goal_angular_acceleration_in_base_frame = goal_angular_acceleration;

	// set the new orientation representation vector in otg to zero and
	// rotate input current velocity and acceleration to the new reference
	// frame
	_output.new_position.tail<3>().setZero();
	_output.new_velocity.tail<3>() =
		_reference_frame.transpose() * previous_reference_frame *
		previous_local_angular_velocity;
	_output.new_acceleration.tail<3>() =
		_reference_frame.transpose() * previous_reference_frame *
		previous_local_angular_acceleration;
	_output.pass_to_input(_input);

	// set the target position and velocity in the new reference frame
	Matrix3d reference_to_goal =
		_reference_frame.transpose() * _goal_orientation_in_base_frame;
	AngleAxisd reference_to_goal_angle_axis = AngleAxisd(reference_to_goal);
	_input.target_position.tail<3>() = reference_to_goal_angle_axis.angle() *
									   reference_to_goal_angle_axis.axis();
	const Vector3d target_local_angular_velocity =
		_reference_frame.transpose() * _goal_angular_velocity_in_base_frame;
	_input.target_velocity.tail<3>() =
		leftJacobianSO3Inverse(_input.target_position.tail<3>()) *
		target_local_angular_velocity;
	const Vector3d target_local_angular_acceleration =
		_reference_frame.transpose() *
		_goal_angular_acceleration_in_base_frame;
	_input.target_acceleration.tail<3>() =
		leftJacobianSO3Inverse(_input.target_position.tail<3>()) *
		(target_local_angular_acceleration -
		 leftJacobianDotTimesVelocity(_input.target_position.tail<3>(),
									  _input.target_velocity.tail<3>()));
}

void OTG_6dof_cartesian::enableTrackingMode(
	const double reactiveness, const size_t look_ahead_cycles,
	const size_t max_iterations, const TrackigMode mode) {
	if (reactiveness < 0.0 || reactiveness > 1.0) {
		throw std::invalid_argument(
			"reactiveness must be in [0, 1] in "
			"OTG_6dof_cartesian::enableTrackingMode\n");
	}
	if (look_ahead_cycles == 0 || max_iterations == 0) {
		throw std::invalid_argument(
			"look_ahead_cycles and max_iterations must be positive in "
			"OTG_6dof_cartesian::enableTrackingMode\n");
	}

	_trackig->reactiveness = reactiveness;
	_trackig->look_ahead_cycles = look_ahead_cycles;
	_trackig->max_iterations = max_iterations;
	_trackig->mode = mode;
	_trackig->reset();
	_tracking_mode_enabled = true;
	_goal_reached = false;
}

void OTG_6dof_cartesian::disableTrackingMode() {
	_tracking_mode_enabled = false;
	_otg->reset();
}

void OTG_6dof_cartesian::setTrackingTargetLimits(
	const Vector3d& max_linear_velocity,
	const Vector3d& max_linear_acceleration,
	const Vector3d& max_angular_velocity,
	const Vector3d& max_angular_acceleration) {
	validateTrackingTargetLimitVector(max_linear_velocity,
									  "max target linear velocity");
	validateTrackingTargetLimitVector(max_linear_acceleration,
									  "max target linear acceleration");
	validateTrackingTargetLimitVector(max_angular_velocity,
									  "max target angular velocity");
	validateTrackingTargetLimitVector(max_angular_acceleration,
									  "max target angular acceleration");

	Vector6d max_velocity;
	max_velocity.head<3>() = max_linear_velocity;
	max_velocity.tail<3>() = max_angular_velocity;
	Vector6d max_acceleration;
	max_acceleration.head<3>() = max_linear_acceleration;
	max_acceleration.tail<3>() = max_angular_acceleration;
	_trackig->setTargetLimits(max_velocity, max_acceleration);
}

void OTG_6dof_cartesian::disableTrackingTargetLimits() {
	_trackig->disableTargetLimits();
}

void OTG_6dof_cartesian::update() {
	if (_goal_reached) {
		return;
	}
	// compute next state and get result value
	OutputParameter<6, EigenVector> previous_output = _output;
	if (_tracking_mode_enabled) {
		TargetState<6, EigenVector> target_state;
		target_state.position = _input.target_position;
		target_state.velocity = _input.target_velocity;
		target_state.acceleration = _input.target_acceleration;
		_result_value = _trackig->update(target_state, _input, _output);
	} else {
		_result_value = _otg->update(_input, _output);
	}

	// if the goal is reached, either return if the current velocity is
	// zero, or set a new goal to the current position with zero velocity
	if (_result_value == Result::Finished) {
		if (_output.new_velocity.norm() < 1e-3 &&
			_input.target_velocity.norm() < 1e-6 &&
			_input.target_acceleration.norm() < 1e-6) {
			_goal_reached = true;
		} else if (_tracking_mode_enabled) {
			_output.pass_to_input(_input);
		} else {
			setGoalPosition(_input.target_position.head<3>());
			setGoalOrientation(_goal_orientation_in_base_frame);
		}
		return;
	}

	// if the goal is not reached, update the current state and return
	if (_result_value == Result::Working) {
		_output.pass_to_input(_input);
		return;
	}

	// if an error occured, print a warning and keep the previous output
	_output = previous_output;
	std::cout << "WARNING: error in computing next state in "
				 "OTG_6dof_cartesian::update. Reinitializing current "
				 "trajectory velocity and acceleration to zero. Error code: "
			  << _result_value << "\n";
	_input.current_velocity.setZero();
	_input.current_acceleration.setZero();
}

Matrix3d OTG_6dof_cartesian::getNextOrientation() const {
	const Matrix3d next_orientation =
		rotationVectorToMatrix(_output.new_position.tail<3>());
	return _reference_frame * next_orientation;
}

Vector3d OTG_6dof_cartesian::getNextAngularVelocity() const {
	return _reference_frame *
		   leftJacobianSO3(_output.new_position.tail<3>()) *
		   _output.new_velocity.tail<3>();
}

Vector3d OTG_6dof_cartesian::getNextAngularAcceleration() const {
	return _reference_frame *
		   (leftJacobianSO3(_output.new_position.tail<3>()) *
				_output.new_acceleration.tail<3>() +
			leftJacobianDotTimesVelocity(_output.new_position.tail<3>(),
										 _output.new_velocity.tail<3>()));
}

} /* namespace SaiPrimitives */
