/*
 * OrientationTask.cpp
 *
 *      Author: Mikael Jorda
 */

#include "OrientationTask.h"

#include <stdexcept>


namespace Sai2Primitives
{


OrientationTask::OrientationTask(Sai2Model::Sai2Model* robot, 
		const std::string link_name, 
		const Eigen::Affine3d control_frame,
		const double loop_time) : 
	OrientationTask(robot, link_name, control_frame.translation(), control_frame.linear(), loop_time) {}

OrientationTask::OrientationTask(Sai2Model::Sai2Model* robot, 
		const std::string link_name, 
		const Eigen::Vector3d pos_in_link, 
		const Eigen::Matrix3d rot_in_link,
		const double loop_time)
{
	Eigen::Affine3d control_frame = Eigen::Affine3d::Identity();
	control_frame.linear() = rot_in_link;
	control_frame.translation() = pos_in_link;

	_robot = robot;
	_link_name = link_name;
	_control_frame = control_frame;

	int dof = _robot->dof();
	_robot->rotation(_current_orientation, _link_name);
	_current_angular_velocity.setZero();

	// default values for gains and velocity saturation
	_kp = 50.0;
	_kv = 14.0;
	_ki = 0.0;
	_use_velocity_saturation_flag = false;
	_saturation_velocity = M_PI/3;

	// initialize matrices sizes
	_jacobian.setZero(3,dof);
	_projected_jacobian.setZero(3,dof);
	_Lambda.setZero(3,3);
	_Jbar.setZero(dof,3);
	_N.setZero(dof,dof);
	_N_prec = Eigen::MatrixXd::Identity(dof,dof);

#ifdef USING_OTG 
	_use_interpolation_flag = true;
	_loop_time = loop_time;
	_otg = new OTG_ori(_current_orientation, _loop_time);

	_otg->setMaxVelocity(M_PI/3);
	_otg->setMaxAcceleration(M_PI);
	_otg->setMaxJerk(3*M_PI);
#endif

	reInitializeTask();
}

void OrientationTask::reInitializeTask()
{
	int dof = _robot->dof();

	_robot->rotation(_desired_orientation, _link_name);
	_desired_angular_velocity.setZero();
	_desired_angular_acceleration.setZero();

	_step_desired_orientation = _desired_orientation;
	_step_orientation_error.setZero(3);
	_step_desired_angular_velocity.setZero(3);
	_step_desired_angular_acceleration.setZero(3);

	_task_force.setZero();
	_orientation_error.setZero();
	_integrated_orientation_error.setZero();

	_first_iteration = true;

#ifdef USING_OTG 
	_otg->reInitialize(_current_orientation);
#endif
}


void OrientationTask::updateTaskModel(const Eigen::MatrixXd N_prec)
{
	if(N_prec.rows() != N_prec.cols())
	{
		throw std::invalid_argument("N_prec matrix not square in OrientationTask::updateTaskModel\n");
	}
	if(N_prec.rows() != _robot->dof())
	{
		throw std::invalid_argument("N_prec matrix size not consistent with robot dof in OrientationTask::updateTaskModel\n");
	}

	_N_prec = N_prec;

	_robot->Jw(_jacobian, _link_name);
	_projected_jacobian = _jacobian * _N_prec;
	_robot->operationalSpaceMatrices(_Lambda, _Jbar, _N, _projected_jacobian, _N_prec);

}


void OrientationTask::computeTorques(Eigen::VectorXd& task_joint_torques)
{

	// get time since last call for the I term
	if(_first_iteration)
	{
		_t_prev = std::chrono::high_resolution_clock::now();
		_t_curr = std::chrono::high_resolution_clock::now();
		_first_iteration = false;
	}
	else
	{
		_t_curr = std::chrono::high_resolution_clock::now();
	}
	_t_diff = _t_curr - _t_prev;

	// update constroller state
	_robot->rotation(_current_orientation, _link_name);
	_current_orientation = _current_orientation * _control_frame.linear(); // orientation of compliant frame in robot frame
	Sai2Model::orientationError(_orientation_error, _desired_orientation, _current_orientation);
	_current_angular_velocity = _projected_jacobian * _robot->dq();
	_step_desired_orientation = _desired_orientation;
	_step_orientation_error = _orientation_error;
	_step_desired_angular_velocity = _desired_angular_velocity;
	_step_desired_angular_acceleration = _desired_angular_acceleration;

	// compute next state from trajectory generation
#ifdef USING_OTG
	if(_use_interpolation_flag)
	{
		_otg->setGoalPositionAndVelocity(_desired_orientation, _current_orientation, _desired_angular_velocity);
		_otg->computeNextState(_step_desired_orientation, _step_desired_angular_velocity, _step_desired_angular_acceleration);
		Sai2Model::orientationError(_step_orientation_error, _step_desired_orientation, _current_orientation);
	}
#endif

	// update integrated error for I term
	_integrated_orientation_error += _step_orientation_error * _t_diff.count();

	// compute task force
	if(_use_velocity_saturation_flag)
	{
		_step_desired_angular_velocity = -_kp / _kv * (_step_orientation_error) - _ki/_kv * _integrated_orientation_error;
		if(_step_desired_angular_velocity.norm() > _saturation_velocity)
		{
			_step_desired_angular_velocity *= _saturation_velocity/_step_desired_angular_velocity.norm();
		}
		_task_force = _Lambda * (_step_desired_angular_acceleration -_kv*(_current_angular_velocity - _step_desired_angular_velocity));
	}
	else
	{
		_task_force = _Lambda*(_step_desired_angular_acceleration -_kp * _step_orientation_error - _kv*(_current_angular_velocity - _step_desired_angular_velocity ) - _ki * _integrated_orientation_error);
	}

	// compute task torques
	task_joint_torques = _projected_jacobian.transpose()*_task_force;

	// update previous time
	_t_prev = _t_curr;
}


} /* namespace Sai2Primitives */

