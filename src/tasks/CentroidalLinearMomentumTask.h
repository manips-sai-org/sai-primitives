/**
 * @file CentroidalLinearMomentumTask.h
 *
 *      This class creates a centroidal linear momentum tracking task using the
 * linear centroidal momentum matrix returned by SaiModel.
 *
 * @author William Chong (wmchong@stanford.edu)
 */

#ifndef SAI_PRIMITIVES_CENTROIDAL_LINEAR_MOMENTUM_TASK_H_
#define SAI_PRIMITIVES_CENTROIDAL_LINEAR_MOMENTUM_TASK_H_

#include <Eigen/Dense>
#include <memory>
#include <string>
#include <vector>

#include "SaiModel.h"
#include "TemplateTask.h"
#include "helper_modules/SaiPrimitivesCommonDefinitions.h"

using namespace Eigen;
using namespace std;

namespace SaiPrimitives {

class CentroidalLinearMomentumTask : public TemplateTask {
public:
	struct DefaultParameters {
		static constexpr double kp = 50.0;
		static constexpr DynamicDecouplingType dynamic_decoupling_type =
			DynamicDecouplingType::FULL_DYNAMIC_DECOUPLING;
		static constexpr double bie_threshold = 0.1;
		static constexpr double task_range_basis_tol = 1e-2;
	};

	CentroidalLinearMomentumTask(
		std::shared_ptr<SaiModel::SaiModel>& robot,
		const std::string& task_name = "centroidal_linear_momentum_task",
		const double loop_timestep = 0.001);

	void updateTaskModel(const MatrixXd& N_prec) override;

	VectorXd computeTorques() override;
	VectorXd computeTorques(const Eigen::VectorXd& tau_prec) override;

	void reInitializeTask() override;

	MatrixXd getTaskNullspace() const override { return _N; }
	MatrixXd getPreviousTasksNullspace() const override { return _N_prec; }
	MatrixXd getTaskAndPreviousNullspace() const override {
		return _N * _N_prec;
	}

	const Vector3d& getCurrentMomentum() const { return _current_momentum; }

	void setGoalMomentum(const Vector3d& goal_momentum);
	const Vector3d& getGoalMomentum() const { return _goal_momentum; }

	void setGoalMomentumVelocity(const Vector3d& goal_momentum_velocity);
	const Vector3d& getGoalMomentumVelocity() const {
		return _goal_momentum_velocity;
	}

	const Vector3d& getMomentumError() const { return _momentum_error; }

	void setGains(const double kp);
	void setGains(const Vector3d& kp);
	vector<PIDGains> getGains() const;

	void setDynamicDecouplingType(const DynamicDecouplingType& type) {
		_dynamic_decoupling_type = type;
	}

	void setBoundedInertiaEstimateThreshold(const double threshold) {
		_bie_threshold = threshold < 0.0 ? 0.0 : threshold;
	}

	double getBoundedInertiaEstimateThreshold() const {
		return _bie_threshold;
	}

	const MatrixXd& getJacobian() const { return _jacobian; }

private:
	void initialSetup();
	void updateJacobian();
	void updateDynamicDecoupling();

	Vector3d _goal_momentum;
	Vector3d _goal_momentum_velocity;
	Vector3d _current_momentum;
	Vector3d _momentum_error;

	bool _are_gains_isotropic;
	Matrix3d _kp;

	MatrixXd _jacobian;
	MatrixXd _projected_jacobian;
	MatrixXd _N;
	MatrixXd _N_prec;
	MatrixXd _Lambda;
	MatrixXd _Lambda_modified;
	MatrixXd _Jbar;
	MatrixXd _current_task_range;

	DynamicDecouplingType _dynamic_decoupling_type;
	double _bie_threshold;
};

} /* namespace SaiPrimitives */

#endif /* SAI_PRIMITIVES_CENTROIDAL_LINEAR_MOMENTUM_TASK_H_ */
