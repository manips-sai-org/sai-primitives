/**
 * @file MomentumTask.h
 *
 *      This class creates a link momentum tracking task using the operational
 * space inertia and the world-frame Jacobian of a control frame attached to a
 * robot link.
 *
 * @author William Chong (wmchong@stanford.edu)
 */

#ifndef SAI_PRIMITIVES_MOMENTUM_TASK_H_
#define SAI_PRIMITIVES_MOMENTUM_TASK_H_

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

class MomentumTask : public TemplateTask {
public:
	struct DefaultParameters {
		static constexpr double kp = 50.0;
		static constexpr DynamicDecouplingType dynamic_decoupling_type =
			DynamicDecouplingType::BOUNDED_INERTIA_ESTIMATES;
		static constexpr double bie_threshold = 0.1;
		static constexpr double task_range_basis_tol = 1e-2;
	};

	MomentumTask(
		std::shared_ptr<SaiModel::SaiModel>& robot,
		const std::string& link_name,
		const Affine3d& compliant_frame = Affine3d::Identity(),
		const std::string& task_name = "momentum_task",
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

	const std::string& getLinkName() const { return _link_name; }
	const Affine3d& getCompliantFrame() const { return _compliant_frame; }

	const VectorXd& getCurrentMomentum() const { return _current_momentum; }

	void setGoalMomentum(const VectorXd& goal_momentum);
	const VectorXd& getGoalMomentum() const { return _goal_momentum; }

	void setGoalMomentumVelocity(const VectorXd& goal_momentum_velocity);
	const VectorXd& getGoalMomentumVelocity() const {
		return _goal_momentum_velocity;
	}

	const VectorXd& getDesiredMomentumVelocity() const {
		return _desired_momentum_velocity;
	}

	const VectorXd& getMomentumError() const { return _momentum_error; }

	void setGains(const double kp);
	void setGains(const VectorXd& kp);
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
	const VectorXd& getKineticEnergyGradient() const {
		return _kinetic_energy_gradient;
	}

	VectorXd computeKineticEnergyGradient() const;

private:
	void initialSetup();
	void updateJacobian();
	void updateDynamicDecoupling();

	std::string _link_name;
	Affine3d _compliant_frame;

	VectorXd _goal_momentum;
	VectorXd _goal_momentum_velocity;
	VectorXd _desired_momentum_velocity;
	VectorXd _current_momentum;
	VectorXd _momentum_error;

	bool _are_gains_isotropic;
	MatrixXd _kp;

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

	VectorXd _kinetic_energy_gradient;
};

} /* namespace SaiPrimitives */

#endif /* SAI_PRIMITIVES_MOMENTUM_TASK_H_ */
