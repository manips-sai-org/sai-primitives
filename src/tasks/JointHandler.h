/*
 * JointHandler.h
 *
 *      This class creates a joint limit handling class for joint position,
 * velocity, and torque limits.
 *
 *      Author: William Chong 
 */

#ifndef SAI2_PRIMITIVES_JOINT_HANDLER_
#define SAI2_PRIMITIVES_JOINT_HANDLER_

#include <helper_modules/Sai2PrimitivesCommonDefinitions.h>
#include "Sai2Model.h"
#include <Eigen/Dense>

using namespace Eigen;
namespace Sai2Primitives {

struct DefaultParameters {
    static constexpr double pos_zone_1 = 12 * M_PI / 180;  // outer zone
    static constexpr double pos_zone_2 = 8 * M_PI / 180;  // inner zone
    static constexpr double t_delta_pos = 0.15;
    static constexpr double t_delta_vel = 0.15;
    static constexpr double kv_damping = 15;
    static constexpr double eta = 0.05;
};

enum JointState {
    SAFE = 0,
    MIN_VEL,
    MAX_VEL,
    MIN_SOFT_POS,  
    MIN_HARD_POS,  
    MAX_SOFT_POS,   
    MAX_HARD_POS   
};

class JointHandler {
public:

    JointHandler(
        std::shared_ptr<Sai2Model::Sai2Model> robot,
        const std::vector<int>& joint_selection_to_skip = {},
        const bool verbose = false);

    void enableJointLimits() {
        _enable_limit_flag = true; 
    }

    void disableJointLimits() {
        _enable_limit_flag = false;
    }

    void enableVelLimits() {
        _enable_vel_limits = true;
    }

    void disableVelLimits() {
        _enable_vel_limits = false;
    }

    void setJointSelection(const std::vector<int>& joint_selection_to_skip) {
        _joint_selection_to_skip = joint_selection_to_skip;
    }

    void updateTaskModel(const MatrixXd& N_prec);
    VectorXd computeTorques(const VectorXd& torques);

    // model parameters
    MatrixXd getTaskJacobian() {
        return _projected_jacobian;
    }

    MatrixXd getTaskAndPreviousNullspace() {
        return _Nc * _N_prec;
    }

    MatrixXd getNullspaceMatrix() {
        return _Nc;
    }

    // settings
    void setMaxJointLimit(const VectorXd& q_max) {
        _q_max = q_max;
    }
     
    void setMinJointLimit(const VectorXd& q_min) {
        _q_min = q_min;
    }

    VectorXd getMaxJointLimit() {
        return _q_max;
    }
     
    VectorXd getMinJointLimit() {
        return _q_min;
    }

    VectorXd getPosZone1Threshold() {
        return _pos_zone_1_threshold;
    }

    VectorXd getPosZone2Threshold() {
        return _pos_zone_2_threshold;
    }

    void setPosZone1Threshold(const VectorXd& zone_1_threshold) {
        _pos_zone_1_threshold = zone_1_threshold;
    }

    void setPosZone1ThresholdIndex(const double threshold, const int index) {
        _pos_zone_1_threshold(index) = threshold;
    }

    void setPosZone2Threshold(const VectorXd& zone_2_threshold) {
        _pos_zone_2_threshold = zone_2_threshold;
        _rho_0 = zone_2_threshold;
    }

    void setPosZone2ThresholdIndex(const double threshold, const int index) {
        _pos_zone_2_threshold(index) = threshold;
    }

    void setTorqueThreshold(const double tau) {
        _unit_tau_thresh = tau;
    }

    void setEta(const double eta) {
        _eta = eta * VectorXd::Ones(_dof);
    }

    VectorXd getApfTorques() {
        return _apf_torques;
    }

    void setPosTimeBuffer(const double time) {
        _t_delta_pos = time;
    }

    void setVelTimeBuffer(const double time) {
        _t_delta_vel = time;
    }

    std::vector<JointState> getJointState() {
        return _joint_state;
    }

    void setDamping(const double kv) {
        _kv_pos_limit = kv * VectorXd::Ones(_dof);
    }

    void enableVariableVelocityZone(const bool flag) {
        _variable_vel_zone = flag;
    }

    void setApfThreshFlag(const bool flag) {
        _use_apf_thresh_flag = flag;
    }

private:

    std::shared_ptr<Sai2Model::Sai2Model> _robot;
    std::vector<int> _joint_selection_to_skip;
    bool _verbose;
    bool _enable_limit_flag;

    std::vector<JointState> _joint_state;

    double _t_delta_pos;
    double _t_delta_vel;

    double _rho_min_tol;
    double _unit_tau_thresh;

    bool _use_apf_thresh_flag;

    bool _is_floating;
    int _dof;
    int _num_con;
    VectorXd _q_min;
    VectorXd _q_max;
    VectorXd _dq_abs_max;
    VectorXd _tau_abs_max;
    VectorXd _pos_zone_1_threshold;
    VectorXd _pos_zone_2_threshold;
    VectorXd _kv_pos_limit;

    bool _enable_vel_limits;
    VectorXd _rho;
    VectorXd _rho_0;
    VectorXd _eta;
    VectorXd _apf_torques;
    VectorXd _damping_torques;
    VectorXd _max_vel;

    // task parameters
    MatrixXd _N_prec;
    MatrixXd _Jc;
    MatrixXd _Jbar_c;
    MatrixXd _Nc;
    MatrixXd _Lambda_c;
    MatrixXd _projected_jacobian;
    MatrixXd _current_task_range;
    
    // collision handling
    VectorXd _entry_velocity;
    VectorXd _exit_velocity;
    double _dq_exit_tol;
    bool _variable_vel_zone;
    bool _apf_thresh;

};

}  // namespace

#endif 