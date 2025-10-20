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
    /**
     * @brief Construct a new Singularity Handler task
     * 
     * @param robot robot model from motion force task
     * @param link_name control link of motion force task
     * @param compliant_frame compliant frame of motion force task 
     * @param task_rank rank of the motion force task after partial task projection
     * @param verbose set to true to print singularity status every timestep 
     */
    SingularityHandler(std::shared_ptr<Sai2Model::Sai2Model> robot,
                       const std::string& link_name,
                       const Affine3d& compliant_frame,
                       const int& task_rank,
                       const std::vector<int> joint_dependency,
                       const double& dt,
                       const bool& verbose = false);

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
     * @brief Set the dynamic decoupling type 
     * 
     * @param type DynamicDecoupling type 
     */
    void setDynamicDecouplingType(const DynamicDecouplingType& type) {
        _dynamic_decoupling_type = type;
    }

	void setBoundedInertiaEstimateThreshold(const double& threshold,
                                            const double& singularity_threshold) {
		if(threshold < 0){
			_bie_threshold = 0;
		}
		_bie_threshold = threshold;
        _singularity_bie_threshold = threshold;
	}

	double getBoundedInertiaEstimateThreshold() {
		return _bie_threshold;
	}

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
    void setSingularityHandlingBounds(const double& s_min, const double& s_max) {
        _s_min = s_min;
        _s_max = s_max;
    }

    /**
     * @brief Set the gains for the partial joint task for the singularity strategy
     * 
     * @param kp_type_1 position gain for type 1 strategy
     * @param kv_type_1 velocity damping gain for type 1 strategy
     * @param kv_type_2 velocity damping gain for type 2 strategy
     */
    void setSingularityHandlingGains(const double& kp_type_1, const double& kv_type_1, const double& kp_type_2, const double& kv_type_2) {
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
    void handleAllSingularitiesAsType1(const bool flag) {
        _enforce_type_1_strategy = flag;
    }

    /**
     * @brief Set the desired type 1 posture 
     * 
     * @param q_des desired posture 
     */
    void setType1Posture(const VectorXd& q_des) {
        _q_prior = q_des;
    }

    /**
     * @brief Enables singularity handling
     * 
     */
    void enableSingularityHandling() {
        _enforce_handling_strategy = true;
    }

    /**
     * @brief Disables singularity handling 
     * 
     */
    void disableSingularityHandling() {
        _enforce_handling_strategy = false;
    }

    /**
     * @brief Set the singularity handling parameters for classification
     * 
     * @param s_abs_tol if all singular values are below this value, then the task is
    *                      fully singular 
     * @param type_1_tol tolerance to classify type 1 singularity
     * @param type_2_torque_ratio torque ratio of max torques to move joints for type 2 singularity
     * @param type_2_angle_threshold reverses the torque direction if joint approaches within the 
     *                                  angle threshold for type 2 singularity
     * @param perturb_step_size step size to take for singularity classification
     * @param buffer_size buffer size to store singularity classification history 
     */
    void setSingularityHandlingParams(const double& s_abs_tol,
                                      const double& type_1_tol,
                                      const double& type_2_torque_ratio,
                                      const double& type_2_angle_threshold,
                                      const double& perturb_step_size,
                                      const int& buffer_size) {
        _s_abs_tol = s_abs_tol;
        _type_1_tol = type_1_tol; 
        _type_2_torque_ratio = type_2_torque_ratio;
        _type_2_angle_threshold = type_2_angle_threshold;
        _perturb_step_size = perturb_step_size;
        _buffer_size = buffer_size;
    }

    void setType2Direction(const VectorXd& type_2_direction) {
        _type_2_direction = type_2_direction;
    }

    void enableForceDecoupling(const bool flag) {
        _enable_force_decoupling = flag;
    }

    /**
     * @brief Getters 
     * 
     */
    MatrixXd getNonSingularJacobian() {
        return _projected_jacobian_ns;
    }

    VectorXd getImpedanceForceTorques() {
        return _impedance_force_torques;
    }

    MatrixXd getNonSingularLambda() {
        return _Lambda_ns_modified;
    }

    MatrixXd getNonSingularTaskRange() {
        return _task_range_ns;
    }

    VectorXd getJointSingularityHandlingTorques() {
        return _joint_strategy_torques;
    }

    double getBlendingCoefficient() {
        return _alpha;
    }

    MatrixXd getSingularJacobian() {
        return _projected_jacobian_s;
    }

    MatrixXd getSingularTaskRange() {
        return _task_range_s;
    }

    MatrixXd getSingularLambda() {
        return _Lambda_s_modified;
    }

    VectorXd getSingularValues() {
        return _svd_s;
    }

    VectorXd getSingularTaskTorques() {
        return _task_torques_with_singularity;
    }

    bool isFullySingularTask() {
        return _fully_singular_task;
    }

    bool isExitingSingularity() {
        return _handle_singularity_exit;
    }

    bool getSingularityStatus() {
        return _is_in_singularity;
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
    double _singularity_bie_threshold;
    std::string _link_name;
    Affine3d _compliant_frame;
    int _task_rank;
    int _dof;
    VectorXd _joint_midrange, _q_upper, _q_lower, _tau_upper, _tau_lower, _dq_max;
    bool _enforce_type_1_strategy;
    bool _enforce_handling_strategy;
    double _dt;
    bool _verbose;
    int _n_floating;

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

    // type 2 specifications
    double _type_2_torque_ratio;  // use X% of the max joint torque 
    double _type_2_angle_threshold;
    double _kp_type_2, _kv_type_2;
    VectorXd _type_2_max_vel_vector;
    VectorXd _type_2_torque_vector;
    VectorXd _type_2_direction;
    double _type_2_force_threshold;
    // std::unique_ptr<Sai2Common::ButterworthLowPass> _low_pass_filter;  // LPF for desired velocity 

    // model quantities 
    MatrixXd _svd_U, _svd_V;
    VectorXd _svd_s;
    double _s_abs_tol;  
    double _s_min, _s_max;
    double _alpha;
    MatrixXd _N;
    MatrixXd _N_Vs;
    MatrixXd _task_range_ns, _task_range_s, _joint_task_range_s;
    MatrixXd _projected_jacobian_ns, _projected_jacobian_s;
    MatrixXd _Lambda_ns, _Jbar_ns, _N_ns;
    MatrixXd _Lambda_s;
    MatrixXd _Lambda_ns_modified, _Lambda_s_modified;
    MatrixXd _Lambda_joint_s, _Lambda_joint_s_modified;
    VectorXd _svd_s_singular;

    // joint task quantities 
    MatrixXd _posture_projected_jacobian, _M_partial;
    
    VectorXd _singular_task_torques;
    VectorXd _joint_strategy_torques;
    VectorXd _impedance_force_torques;
    VectorXd _task_torques_with_singularity;
    bool _enable_force_decoupling;

    bool _is_in_singularity;
    bool _fully_singular_task;
    bool _handle_singularity_exit;  // handle exit when leaving singularity handling

    // pino model for higher-order Jacobian derivatives 
    std::vector<int> _joint_dependency;
    std::deque<double> _alpha_history;  // singular value ratio history   
    VectorXd _q_target;  // posture target for type 1 and type 2 singularities 
    std::vector<VectorXd> _dsdq_vec;
    std::deque<bool> _motion_towards_singularity_history;
    int _motion_towards_singularity_buffer_size;
    std::vector<MatrixXd> _dJdq;

    MatrixXd _alpha_blending_matrix;

    // degenerate singularity gracking
    bool _is_degenerate_singularity;
    VectorXd _prev_singular_vector;
    bool _type_1_retracting;

};

}  // namespace

#endif