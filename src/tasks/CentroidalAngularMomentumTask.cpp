/**
 * @file CentroidalAngularMomentumTask.cpp
 *
 * @author William Chong (wmchong@stanford.edu)
 */

#include "CentroidalAngularMomentumTask.h"

#include <stdexcept>

using namespace Eigen;
using namespace std;

namespace SaiPrimitives {

CentroidalAngularMomentumTask::CentroidalAngularMomentumTask(
	std::shared_ptr<SaiModel::SaiModel>& robot, const std::string& task_name,
	const double loop_timestep)
	: TemplateTask(robot, task_name,
				   TaskType::CENTROIDAL_ANGULAR_MOMENTUM_TASK,
				   loop_timestep) {
	initialSetup();
}

void CentroidalAngularMomentumTask::initialSetup() {
	const int robot_dof = getConstRobotModel()->dof();

	setGains(DefaultParameters::kp);
	setDynamicDecouplingType(DefaultParameters::dynamic_decoupling_type);
	setBoundedInertiaEstimateThreshold(DefaultParameters::bie_threshold);

	_jacobian.setZero(3, robot_dof);
	_projected_jacobian.setZero(3, robot_dof);
	_N.setZero(robot_dof, robot_dof);
	_N_prec = MatrixXd::Identity(robot_dof, robot_dof);
	_Lambda.setIdentity(3, 3);
	_Lambda_modified.setIdentity(3, 3);
	_Jbar.setZero(robot_dof, 3);
	_current_task_range.setIdentity(3, 3);
	_kinetic_energy_gradient.setZero(robot_dof);

	reInitializeTask();
}

void CentroidalAngularMomentumTask::reInitializeTask() {
	updateJacobian();
	_current_momentum = _jacobian * getConstRobotModel()->dq();
	_goal_momentum = _current_momentum;
	_goal_momentum_velocity.setZero();
	_desired_momentum_velocity.setZero();
	_momentum_error.setZero();
	_kinetic_energy_gradient.setZero(getConstRobotModel()->dof());
}

void CentroidalAngularMomentumTask::setGoalMomentum(
	const Vector3d& goal_momentum) {
	_goal_momentum = goal_momentum;
}

void CentroidalAngularMomentumTask::setGoalMomentumVelocity(
	const Vector3d& goal_momentum_velocity) {
	_goal_momentum_velocity = goal_momentum_velocity;
}

void CentroidalAngularMomentumTask::setGains(const double kp) {
	if (kp < 0.0) {
		throw invalid_argument(
			"gain must be positive or zero in "
			"CentroidalAngularMomentumTask::setGains\n");
	}
	_are_gains_isotropic = true;
	_kp = kp * Matrix3d::Identity();
}

void CentroidalAngularMomentumTask::setGains(const Vector3d& kp) {
	if (kp.minCoeff() < 0.0) {
		throw invalid_argument(
			"all gains must be positive or zero in "
			"CentroidalAngularMomentumTask::setGains\n");
	}
	_are_gains_isotropic = false;
	_kp = kp.asDiagonal();
}

vector<PIDGains> CentroidalAngularMomentumTask::getGains() const {
	if (_are_gains_isotropic) {
		return vector<PIDGains>(1, PIDGains(_kp(0, 0), 0.0, 0.0));
	}
	return vector<PIDGains>{
		PIDGains(_kp(0, 0), 0.0, 0.0),
		PIDGains(_kp(1, 1), 0.0, 0.0),
		PIDGains(_kp(2, 2), 0.0, 0.0)};
}

void CentroidalAngularMomentumTask::updateJacobian() {
	_jacobian = getConstRobotModel()->getAngularCentroidalJacobianAnalytic();
	_projected_jacobian = _jacobian * _N_prec;
}

void CentroidalAngularMomentumTask::updateTaskModel(const MatrixXd& N_prec) {
	const auto robot = getConstRobotModel();
	const int robot_dof = robot->dof();
	if (N_prec.rows() != N_prec.cols()) {
		throw invalid_argument(
			"N_prec matrix not square in "
			"CentroidalAngularMomentumTask::updateTaskModel\n");
	}
	if (N_prec.rows() != robot_dof) {
		throw invalid_argument(
			"N_prec matrix size not consistent with robot dof in "
			"CentroidalAngularMomentumTask::updateTaskModel\n");
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

void CentroidalAngularMomentumTask::updateDynamicDecoupling() {
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
				"CentroidalAngularMomentumTask::updateDynamicDecoupling\n");
		}
	}
}

VectorXd CentroidalAngularMomentumTask::computeKineticEnergyGradient() const {
	const auto robot = getConstRobotModel();
	const auto dMdq = robot->getMassMatrixDerivative();
	const VectorXd& dq = robot->dq();
	VectorXd dTdq = VectorXd::Zero(robot->dof());
	for (int i = 0; i < robot->dof(); ++i) {
		dTdq(i) = 0.5 * dq.transpose() * dMdq[i] * dq;
	}
	return dTdq;
}

VectorXd CentroidalAngularMomentumTask::computeTorques(
	const Eigen::VectorXd& tau_prec) {
	const auto robot = getConstRobotModel();
	if (tau_prec.size() != robot->dof()) {
		throw invalid_argument(
			"tau_prec vector size not consistent with robot dof in "
			"CentroidalAngularMomentumTask::computeTorques\n");
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

VectorXd CentroidalAngularMomentumTask::computeTorques() {
	const auto robot = getConstRobotModel();
	const int robot_dof = robot->dof();
	updateJacobian();

	if (_current_task_range.norm() == 0.0) {
		return VectorXd::Zero(robot_dof);
	}

	_current_momentum = _Lambda_modified * _projected_jacobian * robot->dq();
	_momentum_error = _current_momentum - _goal_momentum;
	_desired_momentum_velocity =
		_goal_momentum_velocity - _kp * _momentum_error;
	_kinetic_energy_gradient = computeKineticEnergyGradient();

	VectorXd task_kinetic_energy_gradient =
		_Jbar.transpose() * _kinetic_energy_gradient;

	VectorXd task_joint_torques =
		_projected_jacobian.transpose() * _current_task_range *
		(-task_kinetic_energy_gradient + _desired_momentum_velocity);

	return task_joint_torques;
}

} /* namespace SaiPrimitives */
