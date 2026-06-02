/**
 * OTG_joints.cpp
 *
 *	A wrapper to use the Ruckig OTG library
 *
 * Author: Mikael Jorda
 * Created: August 2023
 */

#include "OTG_joints.h"

#include <cmath>
#include <string>

using namespace Eigen;
using namespace ruckig;

namespace SaiPrimitives {

namespace {
void validateTrackingTargetLimitVector(const VectorXd& limits, const int dim,
									   const std::string& name) {
	if (limits.size() != dim) {
		throw std::invalid_argument(
			name + " size does not match the dimension of the OTG_joints object "
				   "in OTG_joints::setTrackingTargetLimits\n");
	}

	for (int i = 0; i < limits.size(); ++i) {
		if (std::isnan(limits(i)) || limits(i) < 0.0) {
			throw std::invalid_argument(
				name +
				" values should be positive or zero in "
				"OTG_joints::setTrackingTargetLimits\n");
		}
	}
}
}  // namespace

OTG_joints::OTG_joints(const VectorXd& initial_position,
					   const double loop_time) {
	_dim = initial_position.size();
	_otg.reset(new Ruckig<DynamicDOFs, EigenVector>(_dim, loop_time));
	_trackig.reset(new Trackig<DynamicDOFs, EigenVector>(_dim, loop_time));
	_input = InputParameter<DynamicDOFs, EigenVector>(_dim);
	_output = OutputParameter<DynamicDOFs, EigenVector>(_dim);
	_input.synchronization = Synchronization::Phase;

	reInitialize(initial_position);
}

void OTG_joints::reInitialize(const VectorXd& initial_position) {
	if (initial_position.size() != _dim) {
		throw std::invalid_argument(
			"initial position size does not match the dimension of the "
			"OTG_joints object in OTG_joints::reInitialize\n");
	}

	setGoalPosition(initial_position);
	_trackig->reset();
	_otg->reset();

	_output.new_position = initial_position;
	_output.new_velocity.setZero();
	_output.new_acceleration.setZero();
	_output.pass_to_input(_input);
}

void OTG_joints::setMaxVelocity(const VectorXd& max_velocity) {
	if (max_velocity.size() != _dim) {
		throw std::invalid_argument(
			"max velocity size does not match the dimension of the OTG_joints "
			"object in OTG_joints::setMaxVelocity\n");
	}
	if (max_velocity.minCoeff() <= 0) {
		throw std::invalid_argument(
			"max velocity cannot be 0 or negative in any directions in "
			"OTG_joints::setMaxVelocity\n");
	}

	_input.max_velocity = max_velocity;
}

void OTG_joints::setMaxAcceleration(const VectorXd& max_acceleration) {
	if (max_acceleration.size() != _dim) {
		throw std::invalid_argument(
			"max acceleration size does not match the dimension of the "
			"OTG_joints object in OTG_joints::setMaxAcceleration\n");
	}
	if (max_acceleration.minCoeff() <= 0) {
		throw std::invalid_argument(
			"max acceleration cannot be 0 or negative in any "
			"directions in OTG_joints::setMaxAcceleration\n");
	}

	_input.max_acceleration = max_acceleration;
}

void OTG_joints::setMaxJerk(const VectorXd& max_jerk) {
	if (max_jerk.size() != _dim) {
		throw std::invalid_argument(
			"max jerk size does not match the dimension of the OTG_joints "
			"object in OTG_joints::setMaxJerk\n");
	}
	if (max_jerk.minCoeff() <= 0) {
		throw std::invalid_argument(
			"max jerk cannot be 0 or negative in any directions in "
			"OTG_joints::setMaxJerk\n");
	}

	_input.max_jerk = max_jerk;
}

void OTG_joints::disableJerkLimits() {
	_input.max_jerk.setConstant(std::numeric_limits<double>::infinity());
	_input.current_acceleration.setZero();
}

bool OTG_joints::getJerkLimitEnabled() const {
	return _input.max_jerk !=
		   VectorXd::Constant(_dim, std::numeric_limits<double>::infinity());
}

void OTG_joints::setGoalPositionAndVelocity(const VectorXd& goal_position,
											const VectorXd& goal_velocity) {
	setGoalPositionVelocityAndAcceleration(goal_position, goal_velocity,
										   VectorXd::Zero(_dim));
}

void OTG_joints::setGoalPositionVelocityAndAcceleration(
	const VectorXd& goal_position, const VectorXd& goal_velocity,
	const VectorXd& goal_acceleration) {
	if (goal_position.size() != _dim || goal_velocity.size() != _dim) {
		throw std::invalid_argument(
			"goal position or velocity size does not match the dimension of "
			"the OTG_joints object in "
			"OTG_joints::setGoalPositionAndVelocity\n");
	}
	if (goal_acceleration.size() != _dim) {
		throw std::invalid_argument(
			"goal acceleration size does not match the dimension of "
			"the OTG_joints object in "
			"OTG_joints::setGoalPositionVelocityAndAcceleration\n");
	}

	_goal_position_eigen = goal_position;
	_goal_velocity_eigen = goal_velocity;
	_goal_acceleration_eigen = goal_acceleration;

	if (goal_position.isApprox(_input.target_position) &&
		goal_velocity.isApprox(_input.target_velocity) &&
		goal_acceleration.isApprox(_input.target_acceleration) &&
		!_tracking_mode_enabled) {
		return;
	}

	_goal_reached = false;
	_input.target_position = goal_position;
	_input.target_velocity = goal_velocity;
	_input.target_acceleration = goal_acceleration;
}

void OTG_joints::enableTrackingMode(const double reactiveness,
									const size_t look_ahead_cycles,
									const size_t max_iterations,
									const TrackigMode mode) {
	if (reactiveness < 0.0 || reactiveness > 1.0) {
		throw std::invalid_argument(
			"reactiveness must be in [0, 1] in "
			"OTG_joints::enableTrackingMode\n");
	}
	if (look_ahead_cycles == 0 || max_iterations == 0) {
		throw std::invalid_argument(
			"look_ahead_cycles and max_iterations must be positive in "
			"OTG_joints::enableTrackingMode\n");
	}

	_trackig->reactiveness = reactiveness;
	_trackig->look_ahead_cycles = look_ahead_cycles;
	_trackig->max_iterations = max_iterations;
	_trackig->mode = mode;
	_trackig->reset();
	_tracking_mode_enabled = true;
	_goal_reached = false;
}

void OTG_joints::disableTrackingMode() {
	_tracking_mode_enabled = false;
	_otg->reset();
}

void OTG_joints::setTrackingTargetLimits(const VectorXd& max_velocity,
										 const VectorXd& max_acceleration) {
	validateTrackingTargetLimitVector(max_velocity, _dim,
									  "max target velocity");
	validateTrackingTargetLimitVector(max_acceleration, _dim,
									  "max target acceleration");
	_trackig->setTargetLimits(max_velocity, max_acceleration);
}

void OTG_joints::disableTrackingTargetLimits() {
	_trackig->disableTargetLimits();
}

void OTG_joints::update() {
	if (_goal_reached) {
		return;
	}
	// compute next state and get result value
	OutputParameter<DynamicDOFs, EigenVector> previous_output = _output;
	if (_tracking_mode_enabled) {
		TargetState<DynamicDOFs, EigenVector> target_state(_dim);
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
			setGoalPosition(_goal_position_eigen);
		}
		return;
	}

	// if still working, update the next input and return
	if (_result_value == Result::Working) {
		_output.pass_to_input(_input);
		return;
	}

	// if an error occurred, print a warning and keep the previous output
	_output = previous_output;
	std::cout << "WARNING: error in computing next state in "
				 "OTG_joints::update. reinitializing current trajectory "
				 "velocity and accelerations to 0. Error code: "
			  << _result_value << "\n";
	_input.current_velocity.setZero();
	_input.current_acceleration.setZero();
}

} /* namespace SaiPrimitives */
