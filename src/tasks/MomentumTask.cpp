/**
 * @file MomentumTask.cpp
 *
 * @author William Chong (wmchong@stanford.edu)
 */

#include "MomentumTask.h"

#include <stdexcept>

using namespace Eigen;
using namespace std;

namespace SaiPrimitives {

MomentumTask::MomentumTask(std::shared_ptr<SaiModel::SaiModel>& robot,
						   const std::string& link_name,
						   const Affine3d& compliant_frame,
						   const std::string& task_name,
						   const double loop_timestep)
	: TemplateTask(robot, task_name, TaskType::MOMENTUM_TASK, loop_timestep),
	  _link_name(link_name),
	  _compliant_frame(compliant_frame) {
	initialSetup();
}

void MomentumTask::initialSetup() {
	const int robot_dof = getConstRobotModel()->dof();

	setGains(DefaultParameters::kp);
	setDynamicDecouplingType(DefaultParameters::dynamic_decoupling_type);
	setBoundedInertiaEstimateThreshold(DefaultParameters::bie_threshold);

	_jacobian.setZero(6, robot_dof);
	_projected_jacobian.setZero(6, robot_dof);
	_N.setZero(robot_dof, robot_dof);
	_N_prec = MatrixXd::Identity(robot_dof, robot_dof);
	_Lambda.setIdentity(6, 6);
	_Lambda_modified.setIdentity(6, 6);
	_Jbar.setZero(robot_dof, 6);
	_current_task_range.setIdentity(6, 6);
	_kinetic_energy_gradient.setZero(robot_dof);

	reInitializeTask();
}

void MomentumTask::reInitializeTask() {
	updateJacobian();
	const VectorXd frame_velocity = _jacobian * getConstRobotModel()->dq();
	_current_momentum = frame_velocity;
	_goal_momentum = _current_momentum;
	_goal_momentum_velocity = VectorXd::Zero(6);
	_desired_momentum_velocity = VectorXd::Zero(6);
	_momentum_error = VectorXd::Zero(6);
	_kinetic_energy_gradient.setZero(getConstRobotModel()->dof());
}

void MomentumTask::setGoalMomentum(const VectorXd& goal_momentum) {
	if (goal_momentum.size() != 6) {
		throw invalid_argument(
			"goal momentum vector size should be 6 in "
			"MomentumTask::setGoalMomentum\n");
	}
	_goal_momentum = goal_momentum;
}

void MomentumTask::setGoalMomentumVelocity(
	const VectorXd& goal_momentum_velocity) {
	if (goal_momentum_velocity.size() != 6) {
		throw invalid_argument(
			"goal momentum velocity vector size should be 6 in "
			"MomentumTask::setGoalMomentumVelocity\n");
	}
	_goal_momentum_velocity = goal_momentum_velocity;
}

void MomentumTask::setGains(const double kp) {
	if (kp < 0.0) {
		throw invalid_argument(
			"gain must be positive or zero in MomentumTask::setGains\n");
	}
	_are_gains_isotropic = true;
	_kp = kp * MatrixXd::Identity(6, 6);
}

void MomentumTask::setGains(const VectorXd& kp) {
	if (kp.size() == 1) {
		setGains(kp(0));
		return;
	}
	if (kp.size() != 6) {
		throw invalid_argument(
			"gain vector size should be 1 or 6 in MomentumTask::setGains\n");
	}
	if (kp.minCoeff() < 0.0) {
		throw invalid_argument(
			"all gains must be positive or zero in MomentumTask::setGains\n");
	}
	_are_gains_isotropic = false;
	_kp = kp.asDiagonal();
}

vector<PIDGains> MomentumTask::getGains() const {
	if (_are_gains_isotropic) {
		return vector<PIDGains>(1, PIDGains(_kp(0, 0), 0.0, 0.0));
	}
	vector<PIDGains> gains = {};
	for (int i = 0; i < 6; ++i) {
		gains.push_back(PIDGains(_kp(i, i), 0.0, 0.0));
	}
	return gains;
}

void MomentumTask::updateJacobian() {
	_jacobian = getConstRobotModel()->JWorldFrame(
		_link_name, _compliant_frame.translation());
	_projected_jacobian = _jacobian * _N_prec;
}

void MomentumTask::updateTaskModel(const MatrixXd& N_prec) {
	const auto robot = getConstRobotModel();
	const int robot_dof = robot->dof();
	if (N_prec.rows() != N_prec.cols()) {
		throw invalid_argument(
			"N_prec matrix not square in MomentumTask::updateTaskModel\n");
	}
	if (N_prec.rows() != robot_dof) {
		throw invalid_argument(
			"N_prec matrix size not consistent with robot dof in "
			"MomentumTask::updateTaskModel\n");
	}

	_N_prec = N_prec;
	updateJacobian();

	_current_task_range = SaiModel::matrixRangeBasis(
		_projected_jacobian, DefaultParameters::task_range_basis_tol);
	if (_current_task_range.norm() == 0.0) {
		_N = MatrixXd::Identity(robot_dof, robot_dof);
		return;
	}

	SaiModel::OpSpaceMatrices op_space_matrices =
		robot->operationalSpaceMatrices(
			_current_task_range.transpose() * _projected_jacobian);
	_Lambda = op_space_matrices.Lambda;
	_Jbar = op_space_matrices.Jbar;
	_N = op_space_matrices.N;

	updateDynamicDecoupling();
}

void MomentumTask::updateDynamicDecoupling() {
	const auto robot = getConstRobotModel();
	const MatrixXd task_jacobian =
		_current_task_range.transpose() * _projected_jacobian;

	switch (_dynamic_decoupling_type) {
		case FULL_DYNAMIC_DECOUPLING: {
			_Lambda_modified = _Lambda;
			break;
		}

		case BOUNDED_INERTIA_ESTIMATES: {
			MatrixXd M_BIE = robot->M();
			for (int i = 0; i < robot->dof(); i++) {
				if (M_BIE(i, i) < _bie_threshold) {
					M_BIE(i, i) = _bie_threshold;
				}
			}
			MatrixXd M_inv_BIE =
				M_BIE.llt().solve(MatrixXd::Identity(robot->dof(), robot->dof()));
			MatrixXd Lambda_inv =
				task_jacobian * M_inv_BIE * task_jacobian.transpose();
			_Lambda_modified =
				Lambda_inv.llt().solve(MatrixXd::Identity(Lambda_inv.rows(),
														  Lambda_inv.rows()));
			break;
		}

		case IMPEDANCE: {
			_Lambda_modified = MatrixXd::Identity(task_jacobian.rows(),
												 task_jacobian.rows());
			break;
		}

		default: {
			throw invalid_argument(
				"Dynamic decoupling type not recognized in "
				"MomentumTask::updateDynamicDecoupling\n");
		}
	}
}

VectorXd MomentumTask::computeKineticEnergyGradient() const {
	const auto robot = getConstRobotModel();
	const auto dMdq = robot->getMassMatrixDerivative();
	const VectorXd& dq = robot->dq();
	VectorXd dTdq = VectorXd::Zero(robot->dof());
	for (int i = 0; i < robot->dof(); ++i) {
		dTdq(i) = 0.5 * dq.transpose() * dMdq[i] * dq;
	}
	return dTdq;
}

VectorXd MomentumTask::computeTorques(const Eigen::VectorXd& tau_prec) {
	const auto robot = getConstRobotModel();
	if (tau_prec.size() != robot->dof()) {
		throw invalid_argument(
			"tau_prec vector size not consistent with robot dof in "
			"MomentumTask::computeTorques\n");
	}

	VectorXd task_torques = computeTorques();
	if (_current_task_range.norm() == 0.0) {
		return task_torques;
	}

	VectorXd disturbance_compensation =
		_projected_jacobian.transpose() * _current_task_range * _Lambda *
		_current_task_range.transpose() * _jacobian * robot->MInv() *
		tau_prec;
	return task_torques - disturbance_compensation;
}

VectorXd MomentumTask::computeTorques() {
	const auto robot = getConstRobotModel();
	const int robot_dof = robot->dof();
	updateJacobian();

	if (_current_task_range.norm() == 0.0) {
		return VectorXd::Zero(robot_dof);
	}

	const VectorXd task_velocity =
		_current_task_range.transpose() * _projected_jacobian * robot->dq();
	_current_momentum =
		_current_task_range * _Lambda_modified * task_velocity;
	_momentum_error = _current_momentum - _goal_momentum;
	_desired_momentum_velocity =
		_goal_momentum_velocity - _kp * _momentum_error;
	_kinetic_energy_gradient = computeKineticEnergyGradient();

	VectorXd task_kinetic_energy_gradient =
		_Jbar.transpose() * _kinetic_energy_gradient;
	VectorXd task_momentum_velocity =
		_current_task_range.transpose() * _desired_momentum_velocity;

	VectorXd task_joint_torques =
		_projected_jacobian.transpose() * _current_task_range *
		(-task_kinetic_energy_gradient + task_momentum_velocity);

	return task_joint_torques;
}

} /* namespace SaiPrimitives */
