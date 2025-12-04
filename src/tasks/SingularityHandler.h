/*
 * SingularityHandling.h
 *
 *      This class creates a singularity classifying and handling class for 
 * type 1 and type 2 singularities. The singularity strategy linearly blends 
 * the torques from the singular task directions and the torques from the 
 * singularity torque strategy, and adds this to the torques from the 
 * non-singular task directions.
 *
 *      Author: William Chong 
 */

#ifndef SAI2_PRIMITIVES_SINGULARITY_HANDLER_
#define SAI2_PRIMITIVES_SINGULARITY_HANDLER_

#include <helper_modules/Sai2PrimitivesCommonDefinitions.h>
#include "Sai2Model.h"
#include <Eigen/Dense>
#include <queue>
#include <memory>

using namespace Eigen;
namespace Sai2Primitives {

enum SingularityType {
    NO_SINGULARITY = 0,
    TYPE_1_SINGULARITY,  
    TYPE_2_SINGULARITY
};

const std::vector<std::string> singularity_labels {"No Singularity", "Type 1 Singularity", "Type 2 Singularity"};

class SingularityHandler {
public:

    struct DefaultParameters {
        static constexpr double kp_type_1 = 100;
        static constexpr double kv_type_1 = 20;
        static constexpr double kp_type_2 = 100;
        static constexpr double kv_type_2 = 20;
        static constexpr double s_abs_tol = 1e-3;  
        static constexpr double type_1_tol = 0.5;   
        static constexpr double perturb_step_size = 5e0;
        static constexpr double type_2_angle_threshold = 15 * M_PI / 180;
        static constexpr double type_2_force_threshold = 0.01;
        static constexpr double type_2_max_vel = M_PI / 2;
        static constexpr double buffer_size = 200;  // singularity history 
        static constexpr double type_1_buffer_size = 1;
        static constexpr double type_1_max_vel_away_from_singularity = M_PI / 3;  // type 1 retract
        static constexpr double type_1_max_vel_towards_singularity = M_PI / 6;  
        static constexpr double type_1_step_size_control_towards_singularity = 1e-1;
        static constexpr double type_1_step_size_classification_towards_singularity = 1e-3;  // to determine motion direction for towards/away from type 1 singularity 
        static constexpr double max_force_norm = 1;  // admittance force -> velocity scaling
        static constexpr double joint_limit_buffer = 5 * M_PI / 180;
        static constexpr double bie_threshold = 0.5;
        static constexpr double singular_bie_threshold = 0.5;
    };

    /**
     * @brief Construct a new Singularity Handler task
     * 
     * @param robot robot model from motion force task
     * @param link_name control link of motion force task
     * @param compliant_frame compliant frame of motion force task 
     * @param task_rank rank of the motion force task after partial task projection
     * @param joint_dependency joint indices that the task uses
     * @param dt control timestep
     * @param verbose set to true to print singularity status every timestep 
     */
    SingularityHandler(std::shared_ptr<Sai2Model::Sai2Model> robot,
                       const std::string link_name,
                       const Affine3d compliant_frame,
                       const int task_rank,
                       const std::vector<int> joint_dependency,
                       const double dt = 0.001,
                       const bool verbose = false);

    /**
     * @brief Updates the model quantities for the singularity handling task, and performs singularity classification
     * 
     * @param projected_jacobian Projected jacobian from motion force task
     * @param N_prec Nullspace of preceding tasks from motion force task
     */
    void updateTaskModel(MatrixXd& projected_jacobian, const MatrixXd& N_prec);

    /**
     * @brief Computes the torques from the singularity handling. If the projected jacobian isn't classified singular, then
     * the torque is computed as usual.
     * If the projected jacobian is classified singular, then the torque is computed as
     * \tau = \tau_{ns} + (1 - \_alpha) * \tau_{joint strategy} + \alpha * \tau_{s} where \alpha is the linear blending ratio, 
     * \tau_{ns} is the torque computed from the non-singular terms, \tau_{s} is the torque computed from the singular 
     * terms, and \tau_{joint strategy} is the torque computed from the singularity strategy.
     * 
     * @param unit_mass_force Desired unit mass forces from motion force task
     * @param force_related_terms Desired forces from motion force task
     * @return VectorXd Torque vector 
     */
    VectorXd computeTorques(const VectorXd& unit_mass_force, const VectorXd& force_related_terms);

    /**
     * @brief Get the nullspace 
     * 
     * @return MatrixXd nullspace 
     */
    MatrixXd getNullspace() { return _N; };

    /**
     * @brief Set the singularity bounds for torque blending based on the inverse of the condition number
     * The linear blending coefficient \alpha is computed as \alpha = (s - _s_min) / (_s_max - _s_min),
     * and is clamped between 0 and 1.
     * 
     * @param s_min lower bound
     * @param s_max upper bound 
     */
    void setSingularityHandlingBounds(const double s_min, const double s_max) {
        _s_min = s_min;
        _s_max = s_max;
    }

    /**
     * @brief Set the gains for the partial joint task for the singularity strategy
     * 
     * @param kp_type_1 position gain for type 1 strategy
     * @param kv_type_1 velocity gain for type 1 strategy
     * @param kp_type_2 position gain for type 2 strategy
     * @param kv_type_2 velocity gain for type 2 strategy
     */
    void setSingularityHandlingGains(const double kp_type_1, 
                                     const double kv_type_1, 
                                     const double kp_type_2, 
                                     const double kv_type_2) {
        _kp_type_1 = kp_type_1;
        _kv_type_1 = kv_type_1;
        _kp_type_2 = kp_type_2;
        _kv_type_2 = kv_type_2;
    }

    /**
     * @brief Enforces type 1 handling behavior if set to true, otherwise handle 
     *  type 1 or type 2 as usual
     * 
     * @param flag  true to enforce type 1 handling behavior 
     */
    void handleAllSingularitiesAsTypeOne(const bool flag) {
        _enforce_type_1_strategy = flag;
    }

    /**
     * @brief Set the desired type 1 posture 
     * 
     * @param q_des desired posture 
     */
    void setTypeOnePosture(const VectorXd& q_des) {
        _q_prior = q_des;
    }

    void enableSingularityHandling() {
        _enforce_handling_strategy = true;
    }

    void disableSingularityHandling() {
        _enforce_handling_strategy = false;
    }

    void setTypeOneParameters(const double max_vel_towards_singularity,
                            const double max_vel_away_from_singularity,
                            const double type_1_step_size_control_towards_singularity,
                            const double type_1_step_size_classification_towards_singularity) {
        _type_1_max_vel_towards_singularity = max_vel_towards_singularity;
        _type_1_max_vel_away_from_singularity = max_vel_away_from_singularity;
        _type_1_step_size_control_towards_singularity = type_1_step_size_control_towards_singularity;
        _type_1_step_size_classification_towards_singularity = type_1_step_size_classification_towards_singularity;
    }

    void setTypeTwoDirection(const VectorXd& type_2_direction) {
        _type_2_direction = type_2_direction;
    }

    void enableForceDecoupling() {
        _enable_force_decoupling = true;
    }

    void disableForceDecoupling() {
        _enable_force_decoupling = false;
    }

    void setDynamicDecouplingType(const DynamicDecouplingType& type) {
        _dynamic_decoupling_type = type;
    }

	void setBoundedInertiaEstimateThreshold(const double threshold,
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

    /*
        Getters
    */
	double getBoundedInertiaEstimateThreshold() {
		return _bie_threshold;
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

    // singular containers
    MatrixXd getSingularJacobian() {
        return _projected_jacobian_s;
    }

    MatrixXd getSingularTaskRange() {
        return _task_range_s;
    }

    MatrixXd getSingularLambda() {
        return _Lambda_s_modified;
    }

    // values 
    VectorXd getSingularValues() {
        return _svd_s;
    }

    MatrixXd getBlendingMatrix() {
        return _alpha_blending_matrix;
    }

    VectorXd getBlendingVector() {
        return _alpha_vec;
    }

    // torques
    VectorXd getNonSingularTaskTorques() {
        return _non_singular_task_torques;
    }

    VectorXd getSingularTaskTorques() {
        return _task_torques_with_singularity;
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

    // experimental baseline values 
    VectorXd getNonHandlingTorques() {
        return _task_torques_with_singularity;
    }

private:

    /**
     * @brief Classifies the singularity based on a joint perturbation in the singular joint space 
     * 
     * @param singular_task_range Singular task range corresponding to the columns of U from SVD
     * @param singular_joint_task_range Singular task range corresponding to the columns of V from SVD
     */
    void classifySingularity(const MatrixXd& projected_jacobian,
                             const MatrixXd& singular_task_range, 
                             const MatrixXd& singular_joint_task_range);

    // singularity setup
    std::shared_ptr<Sai2Model::Sai2Model> _robot;
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
    double _perturb_step_size;
    std::deque<SingularityType> _singularity_history;
    int _type_1_counter, _type_2_counter;
    int _buffer_size;

    // type 1 specifications
    VectorXd _q_prior, _dq_prior;
    double _kp_type_1, _kv_type_1;
    double _type_1_tol;
    double _type_1_max_vel_towards_singularity;
    double _type_1_max_vel_away_from_singularity;
    double _type_1_step_size_control_towards_singularity;
    double _type_1_step_size_classification_towards_singularity;
    int _type_1_buffer_size;

    // type 2 specifications
    double _type_2_angle_threshold;
    double _kp_type_2, _kv_type_2;
    VectorXd _type_2_max_vel_vector;
    VectorXd _type_2_direction;
    double _type_2_force_threshold;

    // model quantities 
    MatrixXd _svd_U, _svd_V;
    VectorXd _svd_s;
    double _s_abs_tol;  
    double _s_min, _s_max;
    MatrixXd _N;
    MatrixXd _task_range_ns, _task_range_s, _joint_task_range_s;
    MatrixXd _projected_jacobian_ns, _projected_jacobian_s;
    MatrixXd _Lambda_ns, _Jbar_ns, _N_ns;
    MatrixXd _Lambda_s;
    MatrixXd _Lambda_ns_modified, _Lambda_s_modified;
    MatrixXd _Lambda_joint_s, _Lambda_joint_s_modified;
    VectorXd _svd_s_singular;
    double _alpha;

    // joint task quantities 
    MatrixXd _posture_projected_jacobian, _M_partial;
    MatrixXd _M_inv_BIE_SINGULARITY;
    
    VectorXd _non_singular_task_torques;
    VectorXd _singular_task_torques;
    VectorXd _joint_strategy_torques;
    VectorXd _task_torques_with_singularity;
    bool _enable_force_decoupling;

    bool _is_in_singularity;
    bool _fully_singular_task;
    bool _handle_singularity_exit;  // handle exit when leaving singularity handling

    // pino model for higher-order Jacobian derivatives 
    std::vector<int> _joint_dependency;
    std::vector<VectorXd> _dsdq_vec;
    std::deque<bool> _motion_towards_singularity_history;
    int _motion_towards_singularity_buffer_size;
    std::vector<MatrixXd> _dJdq;

    MatrixXd _alpha_blending_matrix;
    VectorXd _condition_ratio_vec;
    VectorXd _alpha_vec;
    std::deque<VectorXd> _alpha_history;

    // degenerate singularity gracking
    bool _is_degenerate_singularity;
    VectorXd _prev_singular_vector;
    bool _type_1_retracting;

    // multi-singularity handling containers 
    MatrixXd _N_sjs_init;
    int _num_singularities;
    int _prev_num_singularities;
    bool _singularity_exit_transition;
    double _max_force_norm;

};

}  // namespace

#endif