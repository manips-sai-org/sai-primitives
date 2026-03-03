/**
 * @file SingularityHandler.h
 * @author William Chong (wmchong@stanford.edu)
 * @brief Singularity handling class  
 * @version 0.1
 * @date 2026-02-10
 * 
 * @copyright Copyright (c) 2026
 * 
 */

#ifndef SAI_PRIMITIVES_SINGULARITY_HANDLER_
#define SAI_PRIMITIVES_SINGULARITY_HANDLER_

#include <helper_modules/SaiPrimitivesCommonDefinitions.h>
#include "SaiModel.h"

#include <nlopt.hpp>
#include <Eigen/Dense>
#include <queue>
#include <memory>
#include <algorithm>
#include <numeric>
#include <iostream>

using namespace Eigen;
namespace SaiPrimitives {

enum SingularityType {
    NO_SINGULARITY = 0,
    TYPE_1_SINGULARITY,  
    TYPE_2_SINGULARITY
};

struct Singularity {
    SingularityType type;
    VectorXd u;
    VectorXd v;
    double sigma;
    double lambda;
    VectorXd dsdq;
    VectorXd u_toward_singularity;
    bool is_degenerate;

    Singularity(
        const VectorXd& u,
        const VectorXd& v,
        const double sigma,
        const double lambda,
        const VectorXd& dsdq,
        const VectorXd& u_toward_singularity,
        const SingularityType type,
        const bool is_degenerate = false) : 
        u(u), v(v), sigma(sigma), lambda(lambda), dsdq(dsdq), 
        u_toward_singularity(u_toward_singularity), type(type), is_degenerate(is_degenerate) {}

    Singularity() : type(NO_SINGULARITY) {}
};

struct NloptData {
    std::vector<VectorXd> basis;
    MatrixXd singular_task_range;
    MatrixXd projected_jacobian;
    Vector3d starting_position;
    Matrix3d starting_orientation;
    VectorXd starting_q;
    bool flag_zero_value;
    double perturb_step_size;
    bool flag_type_1_search;

    NloptData(const double perturb_step_size) : perturb_step_size(perturb_step_size) {}

    void setData(const std::vector<VectorXd>& basis_,
                 const MatrixXd& singular_task_range_,
                 const MatrixXd& projected_jacobian_,
                 const Vector3d& starting_position_,
                 const Matrix3d starting_orientation_,
                 const VectorXd& starting_q_,
                 const bool& flag_zero_value_,
                 const bool& flag_type_1_search_) {
        basis = basis_;
        singular_task_range = singular_task_range_;
        projected_jacobian = projected_jacobian_;
        starting_position = starting_position_;
        starting_orientation = starting_orientation_;
        starting_q = starting_q_;
        flag_zero_value = flag_zero_value_;
        flag_type_1_search = flag_type_1_search_;
    }
};

class SingularityHandler {
public:

    struct DefaultParameters {

        static constexpr double kv_type_1 = 15;
        static constexpr double kv_type_2 = 15;  
        static constexpr double s_abs_tol = 1e-6;
        static constexpr double type_1_tol = 5e-2;
        static constexpr double type_1_max_vel_away_from_singularity = M_PI;  // type 1 retract
        static constexpr double type_1_max_vel_towards_singularity = M_PI;  // type 1 approach
        static constexpr double type_1_step_size_classification_towards_singularity = 1 * M_PI / 180;  // to determine motion direction for towards/away from type 1 singularity 
        static constexpr double type_2_max_vel = M_PI;
        static constexpr double type_2_min_force = 1e-6; 
        static constexpr double max_force_norm = 1;  
        static constexpr double joint_limit_buffer = 5 * M_PI / 180;
        static constexpr double bie_threshold = 0.15;
        static constexpr double singular_bie_threshold = 0.15;
        static constexpr double xtol_rel = 1e-3;
        static constexpr double ftol_rel = 1e-3;
        static constexpr double xtol_abs = 1e-3;
        static constexpr double max_time = 0.1;  // ms
        static constexpr int type_1_search_max_iter = 50;
        static constexpr double degenerate_singular_value_spacing = 5e-2;
        static constexpr double type_1_search_tol = 1e-6;  // singular value tolerance for {u, v} disassociation
        static constexpr double type_1_step_size_for_line_search = 10 * M_PI / 180;  // to determine motion direction for towards/away from type 1 singularity 
        static constexpr double type_1_vel_ramp_factor = 2;
        static constexpr double type_2_vel_ramp_factor = 2;
        static constexpr double type_1_alignment_factor = 5;  // 5
    };

    SingularityHandler(
        std::shared_ptr<SaiModel::SaiModel> robot,
        const std::string link_name,
        const Affine3d compliant_frame,
        const int task_rank,
        const std::vector<int> joint_dependency,
        const double dt = 0.001,
        const bool verbose = false);

    void updateTaskModel(MatrixXd& projected_jacobian, const MatrixXd& N_prec);
    VectorXd computeTorques(const VectorXd& unit_mass_force, const VectorXd& force_related_terms);
    MatrixXd getNullspace() { return _N; };

    void setSingularityHandlingBound(const double s_max) {
        _s_max = s_max;
    }

    void setSingularityHandlingGains(const double kv_type_1, const double kv_type_2) {
        _kv_type_1 = kv_type_1;
        _kv_type_2 = kv_type_2;
    }

    void handleAllSingularitiesAsTypeOne(const bool flag) {
        _enforce_type_1_strategy = flag;
    }

    void enableSingularityHandling() {
        _enforce_handling_strategy = true;
    }

    void disableSingularityHandling() {
        _enforce_handling_strategy = false;
    }

    void setTypeOneParameters(
        const double max_vel_towards_singularity,
        const double max_vel_away_from_singularity,
        const double type_1_step_size_classification_towards_singularity) {

        _type_1_max_vel_towards_singularity = max_vel_towards_singularity;
        _type_1_max_vel_away_from_singularity = max_vel_away_from_singularity;
        _type_1_step_size_classification_towards_singularity = type_1_step_size_classification_towards_singularity;
    }

   void enableForceDampingDecoupling() {
        _enable_force_damping_decoupling = true;
    }

    void disableForceDampingDecoupling() {
        _enable_force_damping_decoupling = false;
    }

    void setDynamicDecouplingType(const DynamicDecouplingType type) {
        _dynamic_decoupling_type = type;
    }

    void setBoundedInertiaEstimateThreshold(
        const double threshold) {

		if (threshold < 0){
			_bie_threshold = 0;
		}
		_bie_threshold = threshold;
	}

	void setBoundedInertiaEstimateThreshold(
        const double threshold,
        const double singular_bie_threshold) {

		if (threshold < 0){
			_bie_threshold = 0;
		}
        if (singular_bie_threshold < 0) {
            _singular_bie_threshold = 0;
        }
		_bie_threshold = threshold;
        _singular_bie_threshold = singular_bie_threshold;
	}

    void setType1Tol(const double tol) {
        _type_1_tol = tol;
    }

    void setType1Velocity(const double vel_toward, const double vel_away) {
        _type_1_max_vel_towards_singularity = vel_toward;
        _type_1_max_vel_away_from_singularity = vel_away;
    }

    void setType2Velocity(const double velocity) {
        _type_2_max_vel = velocity;
    }

    void setType2RampFactor(const double val) {
        _type_2_vel_ramp_factor = val;
    }

    void setType1RampFactor(const double val) {
        _type_1_vel_ramp_factor = val;
    }

    void setType1AlignmentFactor(const double val) {
        _type_1_alignment_factor = val;
    }

    // getters
	std::pair<double, double> getBoundedInertiaEstimateThreshold() {
		return std::make_pair(_bie_threshold, _singular_bie_threshold);
	}

    // non-singular containers
    MatrixXd getNonSingularJacobian() {
        return _projected_jacobian_ns;
    }

    MatrixXd getNonSingularLambda() {
        return _Lambda_ns_modified;
    }

    MatrixXd getNonSingularTaskRange() {
        return _task_range_ns;
    }

    // singular joint space containers
    MatrixXd getSingularJointSpaceJacobian() {
        return _posture_projected_jacobian;
    }

    MatrixXd getSingularJointSpaceLambda() {
        return _Lambda_sjs;
    }

    // singular containers
    MatrixXd getSingularJacobian() {
        return _projected_jacobian_s;
    }

    MatrixXd getSingularTaskRange() {
        return _task_range_s;
    }

    MatrixXd getSingularJointTaskRange() {
        return _joint_task_range_s;
    }

    MatrixXd getSingularLambda() {
        return _Lambda_s_modified;
    }

    // values 
    VectorXd getSingularValues() {
        return _svd_s;
    }

    VectorXd getSingularEigenValues() {
        return _eig_values;
    }

    VectorXd getBlendingVector() {
        return _alpha_vec;
    }

    // torques
    VectorXd getNonSingularTaskTorques() {
        return _non_singular_task_torques;
    }

    VectorXd getSingularTaskTorques() {
        return _singular_task_torques;
    }
    
    VectorXd getJointSingularityHandlingTorques() {
        return _joint_strategy_torques;
    }

    // flags 
    bool isFullySingularTask() {
        return _fully_singular_task;
    }

    bool isExitingSingularity() {
        return _handle_singularity_exit;
    }

    bool getSingularityStatus() {
        return _is_in_singularity;
    }

    int getNumSingularities() {
        return _num_singularities;
    }

    bool getSingularityTransitionStatus() {
        return _singularity_exit_transition;
    }

    // experimental
    VectorXd getUnmodifiedSingularTaskTorques() {
        return _unmodified_singular_task_torques;
    }

    VectorXi getClassification() {
        VectorXi classification = VectorXi::Zero(_task_rank);
        int cnt = 0;
        for (auto singularity : _active_singularities) {
            classification(cnt) = singularity.type;
            cnt++;
        }
        return classification;
    }

    std::vector<Singularity> getActiveSingularities() {
        return _active_singularities;
    }

    double getType2Alignment() {
        return _force_dotted_singular_direction;
    }

    void enableJointStrategy() {
        _enable_joint_strategy = true;
    }

    void disableJointStrategy() {
        _enable_joint_strategy = false;
    }

private:

    void classifySingularity(
        const MatrixXd& projected_jacobian,
        const MatrixXd& singular_task_range, 
        const MatrixXd& singular_joint_task_range);

    bool classifySingularityType(
        const std::vector<MatrixXd>& kinematic_hessian,
        const VectorXd& u,
        const VectorXd& v);

    VectorXd getSecondOrderExpansion(
        const std::vector<MatrixXd>& kinematic_hessian,
        const VectorXd& dq);                            

    MatrixXd getProjectedHessian(
        const std::vector<MatrixXd>& kinematic_hessian, //  dof x (6 x dof)
        const VectorXd& direction);

    VectorXd getInitialVector(
        const std::vector<MatrixXd>& kinematic_hessian,
        const MatrixXd& U,
        const MatrixXd& V); 
                                                               
    static double objective(const std::vector<double> &x, std::vector<double> &grad, void* f_data);
    static double equality(const std::vector<double> &x, std::vector<double> &grad, void* f_data);


    std::pair<VectorXd, VectorXd> getTowardSingularityDirection(
        const VectorXd& curr_q,
        const Vector3d& curr_pos,
        const Matrix3d& curr_ori,
        const VectorXd& u,
        const VectorXd& dsdq,
        const double step_size);

    // singularity setup
    std::shared_ptr<SaiModel::SaiModel> _robot;
    JacobiSVD<MatrixXd> _svd_solver;
    SelfAdjointEigenSolver<MatrixXd> _eig_solver;
    DynamicDecouplingType _dynamic_decoupling_type;
	double _bie_threshold;
    double _singular_bie_threshold;
    std::string _link_name;
    Affine3d _compliant_frame;
    int _task_rank;
    int _dof;
    VectorXd _joint_midrange, _q_upper, _q_lower, _tau_upper, _tau_lower, _dq_max;
    bool _enforce_type_1_strategy;
    bool _enforce_handling_strategy;
    double _dt;
    bool _verbose;

    // singularity information
    std::vector<SingularityType> _singularity_types;

    // type 1 specifications
    VectorXd _q_prior, _dq_prior;
    double _kv_type_1;
    double _type_1_search_tol;
    double _type_1_tol;
    double _type_1_max_vel_towards_singularity;
    double _type_1_max_vel_away_from_singularity;
    double _type_1_step_size_classification_towards_singularity;
    double _type_1_step_size_for_line_search;
    int _type_1_search_max_iter;

    // type 2 specifications
    double _kv_type_2;
    double _type_2_max_vel;
    double _type_2_min_force;

    // model quantities 
    MatrixXd _eig_vectors;
    VectorXd _eig_values;
    MatrixXd _svd_U, _svd_V;
    VectorXd _svd_s;
    double _s_abs_tol;  
    double _s_min, _s_max;
    MatrixXd _N;
    MatrixXd _N_prec;
    MatrixXd _task_range_ns, _task_range_s, _joint_task_range_s;
    MatrixXd _projected_jacobian_ns, _projected_jacobian_s, _projected_jacobian;
    MatrixXd _Lambda_ns, _Jbar_ns, _N_ns;
    MatrixXd _Lambda_s;
    MatrixXd _Lambda_ns_modified, _Lambda_s_modified;
    MatrixXd _Lambda_joint_s, _Lambda_joint_s_modified;
    VectorXd _eig_s_singular;
    VectorXd _svd_s_singular;
    double _degenerate_singular_value_spacing;

    // joint task quantities 
    MatrixXd _posture_projected_jacobian, _M_partial;
    MatrixXd _M_inv_BIE_SINGULARITY;
    
    VectorXd _non_singular_task_torques;
    VectorXd _singular_task_torques;
    VectorXd _joint_strategy_torques;
    VectorXd _unmodified_singular_task_torques;
    bool _enable_force_damping_decoupling;

    bool _is_in_singularity;
    bool _fully_singular_task;
    bool _handle_singularity_exit;  // handle exit when leaving singularity handling

    // derivatives
    std::vector<int> _joint_dependency;
    std::vector<MatrixXd> _dJdq;

    VectorXd _alpha_vec;

    // degenerate singularity gracking
    std::vector<std::vector<int>> _degenerate_indices;
    std::vector<Singularity> _active_singularities;
    
    std::vector<VectorXd> _degenerate_singular_values;
    std::vector<VectorXd> _degenerate_eigen_values;
    std::vector<MatrixXd> _degenerate_singular_task_range;
    std::vector<MatrixXd> _degenerate_singular_joint_task_range;
    std::vector<MatrixXd> _type_1_degenerate_singular_task_range;
    std::vector<MatrixXd> _type_1_degenerate_singular_joint_task_range;
    std::vector<MatrixXd> _type_2_degenerate_singular_task_range;
    std::vector<MatrixXd> _type_2_degenerate_singular_joint_task_range;

    bool _type_1_retracting;

    // multi-singularity handling containers 
    MatrixXd _Lambda_sjs;
    MatrixXd _N_sjs_init;
    int _num_singularities;
    int _prev_num_singularities;
    bool _singularity_exit_transition;
    bool _singularity_enter_transition;
    double _max_force_norm;

    // nelder-mead parameters
    double _nm_tol;
    int _nm_max_iter;
    double _nm_step_size;

    // nlopt 
    std::map<int, std::unique_ptr<nlopt::opt>> _nl_opt;
    std::unique_ptr<NloptData> _nl_opt_data;

    double _type_2_vel_ramp_factor;
    double _type_1_vel_ramp_factor;
    double _type_1_alignment_factor;

    // experimental
    double _force_dotted_singular_direction;
    bool _enable_joint_strategy;

};

}  // namespace

#endif