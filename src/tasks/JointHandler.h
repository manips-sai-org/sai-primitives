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

enum JointState {
    SAFE = 0,
    MIN_SOFT_VEL,  // 1
    MIN_HARD_VEL,  // 2
    MAX_SOFT_VEL,  // 3
    MAX_HARD_VEL,  // 4
    MIN_SOFT_POS,  // 5
    MIN_HARD_POS,  // 6
    MAX_SOFT_POS,  // 7 
    MAX_HARD_POS   // 8
};

class JointHandler {
public:

    JointHandler(std::shared_ptr<Sai2Model::Sai2Model> robot,
                 const bool& verbose = true,
                 const bool& truncation_flag = false,
                 const bool& is_floating = false,
                 const double& pos_zone_1 = 9,
                 const double& pos_zone_2 = 6,
                 const double& vel_zone_1 = 40,
                 const double& vel_zone_2 = 30,
                 const double& tau_thresh = 1,
                 const double& tau_vel_thresh = 1,
                 const double& t_delta = 0.1,
                 const double& kv = 20,
                 const double& gamma = 1e0,
                 const std::vector<int>& joint_selection = {});

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

    void setJointSelection(const std::vector<int>& joint_selection) {
        _joint_selection = joint_selection;
    }

    MatrixXd getBlendingMatrix() {
        return _blending_matrix;
    }

    VectorXd getNonTaskSafetyTorques() {
        return _non_task_safety_torques;
    }

    MatrixXd getTaskJacobian() {
        return _projected_jacobian;
    }

    MatrixXd getTaskAndPreviousNullspace() {
        return _Nc * _N_prec;
    }

    MatrixXd getNullspaceMatrix() {
        return _Nc;
    }

    void updateTaskModel(const MatrixXd& N_prec);

    VectorXd computeTorques(const VectorXd& torques,
                            const bool constraint_only = false);

    /*
        Threshold and parameter setting 
    */

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

    void setPosZone2Threshold(const VectorXd& zone_2_threshold) {
        _pos_zone_2_threshold = zone_2_threshold;
    }

    void setVelZone1Threshold(const VectorXd& zone_1_threshold) {
        _vel_zone_1_threshold = zone_1_threshold;
    }

    void setVelZone2Threshold(const VectorXd& zone_2_threshold) {
        _vel_zone_2_threshold = zone_2_threshold;
    }

    void setDampingCoeff(const double& kv) {
        _kv_pos_limit = kv * VectorXd::Ones(_dof);
    }

    void setTorqueThreshold(const double& tau) {
        _tau_thresh = tau;
    }

    VectorXd computePositionIntegration(const VectorXd& q, 
                                        const VectorXd& dq, 
                                        const double& t_delta) {
        VectorXd result(q.size());
        for (int i = 0; i < q.size(); ++i) {
            result[i] = q[i] + (dq[i] / _kv_pos_limit[i]) * (1 - std::exp(-_kv_pos_limit[i] * t_delta));
        }
        return result;
    }

    std::pair<VectorXi, VectorXi> getJointState() {
        return std::make_pair(_joint_state, _joint_vel_state);
    }

private:

    std::shared_ptr<Sai2Model::Sai2Model> _robot;
    std::vector<int> _joint_selection;
    bool _verbose;
    bool _enable_limit_flag;
    bool _is_floating;
    int _dof;
    int _num_con;
    VectorXd _q_min;
    VectorXd _q_max;
    VectorXd _dq_abs_max;
    VectorXd _tau_abs_max;
    VectorXd _pos_zone_1_threshold;
    VectorXd _pos_zone_2_threshold;
    VectorXd _var_pos_zone_1_threshold;
    VectorXd _vel_zone_1_threshold;
    VectorXd _vel_zone_2_threshold;
    VectorXi _joint_state;
    VectorXi _joint_vel_state;
    VectorXi _joint_entry_state;
    VectorXi _task_direction_wrt_constraint;
    VectorXd _vel_gamma;
    VectorXd _kv_pos_limit;
    bool _truncation_flag;
    double _tau_thresh;
    double _tau_vel_thresh;
    double _t_delta;
    VectorXd _pos_entry_velocities;
    VectorXd _vel_entry_velocities;

    bool _enable_vel_limits;
    VectorXd _rho;
    VectorXd _rho_0;
    VectorXd _eta;

    // verbose output 
    std::vector<std::string> _constraint_description;

    // task parameters
    // std::vector<double> _blending_coefficients;
    // std::vector<double> _vel_blending_coefficients;
    VectorXd _blending_coefficients;
    VectorXd _vel_blending_coefficients;
    MatrixXd _blending_matrix;
    VectorXd _non_task_safety_torques;
    MatrixXd _N_prec;
    MatrixXd _Jc;
    MatrixXd _Jbar_c;
    MatrixXd _Nc;
    MatrixXd _Lambda_c;
    MatrixXd _projected_jacobian;
    MatrixXd _current_task_range;

};

}  // namespace

#endif 