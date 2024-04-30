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

#include "Sai2Model.h"
#include <Eigen/Dense>
#include <queue>
#include <memory>

using namespace Eigen;
namespace Sai2Primitives {

// For logging purposes only
enum SingularityType {
    NO_SINGULARITY = 0,
    TYPE_1_SINGULARITY,  
    TYPE_2_SINGULARITY
};

const std::vector<std::string> singularity_labels {"No Singularity", "Type 1 Singularity", "Type 2 Singularity"};      

enum DynamicDecouplingType {
    FULL_DYNAMIC_DECOUPLING = 0, // use the real Lambda matrix
    IMPEDANCE,					 // use Identity for the mass matrix
    BOUNDED_INERTIA_ESTIMATES,	 // use a Lambda computed from a saturated
                                 // joint space mass matrix
};            

class SingularityHandler {
public:
    /**
     * @brief Construct a new Singularity Handler task
     * 
     * @param robot robot model from motion force task
     * @param link_name control link of motion force task
     * @param compliant_frame compliant frame of motion force task 
     * @param task_rank rank of the motion force task after partial task projection
     * @param s_abs_tol tolerance to declare task completely singular if all singular values from the projected
     * jacobian are under this value
     * @param type_1_tol tolerance to classify type 1 singularities 
     * @param type_2_torque_ratio percentage of the max joint torque to use for the type 2 singularity strategy
     * @param perturb_step_size the joint angle increment to classify singularity 
     * @param buffer_size size of buffer to compare which singularity type has more counts 
     * @param verbose set to true to print singularity status every timestep 
     */
    SingularityHandler(std::shared_ptr<Sai2Model::Sai2Model> robot,
                       const std::string& link_name,
                       const Affine3d& compliant_frame,
                       const int& task_rank,
                       const double& s_abs_tol = 1e-3,
                       const double& type_1_tol = 0.5,
                       const double& type_2_torque_ratio = 1e-2,
                       const double& perturb_step_size = 5,
                       const int& buffer_size = 200,
                       const bool& verbose = false);

    /**
     * @brief Updates the model quantities for the singularity handling task, and performs singularity classification
     * 
     * @param projected_jacobian Projected jacobian from motion force task
     * @param N_prec Nullspace of preceding tasks from motion force task
     */
    void updateTaskModel(const MatrixXd& projected_jacobian, const MatrixXd& N_prec);

    /**
     * @brief Classifies the singularity based on a joint perturbation in the singular joint space 
     * 
     * @param singular_task_range Singular task range corresponding to the columns of U from SVD
     * @param singular_joint_task_range Singular task range corresponding to the columns of V from SVD
     */
    void classifySingularity(const MatrixXd& singular_task_range, 
                             const MatrixXd& singular_joint_task_range);

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
    void setSingularityBounds(const double& s_min, const double& s_max) {
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
    void setGains(const double& kp_type_1, const double& kv_type_1, const double& kv_type_2) {
        _kp_type_1 = kp_type_1;
        _kv_type_1 = kv_type_1;
        _kv_type_2 = kv_type_2;
    }

    /**
     * @brief Enforces type 1 handling behavior if set to true, otherwise handle 
     * type 1 or type 2 as usual
     * 
     * @param flag  true to enforce type 1 handling behavior 
     */
    void setType1Behavior(const bool flag) {
        _enforce_type_1_strategy = true;
    }

private:
    // singularity setup
    std::shared_ptr<Sai2Model::Sai2Model> _robot;
    DynamicDecouplingType _dynamic_decoupling_type;
    std::string _link_name;
    Affine3d _compliant_frame;
    int _task_rank;
    VectorXd _joint_midrange, _q_upper, _q_lower, _tau_upper, _tau_lower;
    bool _enforce_type_1_strategy;
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

    // type 2 specifications
    double _type_2_torque_ratio;  // use X% of the max joint torque 
    double _kv_type_2;
    VectorXd _type_2_torque_vector;
    VectorXd _type_2_direction;

    // model quantities 
    MatrixXd _svd_U, _svd_V;
    VectorXd _svd_s;
    double _s_abs_tol;  
    double _s_min, _s_max;
    double _alpha;
    MatrixXd _N;
    MatrixXd _task_range_ns, _task_range_s, _joint_task_range_s;
    MatrixXd _projected_jacobian_ns, _projected_jacobian_s;
    MatrixXd _Lambda_ns, _Jbar_ns, _N_ns;
    MatrixXd _Lambda_s;
    MatrixXd _Lambda_ns_modified, _Lambda_s_modified;
    MatrixXd _Lambda_joint_s, _Lambda_joint_s_modified;

    // joint task quantities 
    MatrixXd _posture_projected_jacobian, _M_partial;
    
    VectorXd _singular_task_torques;
    VectorXd _joint_strategy_torques;
};

}  // namespace

#endif