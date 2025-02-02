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
                 const bool& is_floating = true,
                 const double& pos_zone_1 = 2,
                 const double& pos_zone_2 = 1,
                 const double& vel_zone_1 = 0.01,
                 const double& vel_zone_2 = 0.005,
                 const double& kv = 0,
                 const double& gamma = 1e-4,
                 const std::vector<int>& joint_selection = {});

    void enableJointLimits() {
        _enable_limit_flag = true; 
    }

    void disableJointLimits() {
        _enable_limit_flag = false;
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

    VectorXd computeTorques(const VectorXd& torques);

    /*
        Threshold and parameter setting 
    */
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

private:

    std::shared_ptr<Sai2Model::Sai2Model> _robot;
    std::vector<int> _joint_selection;
    bool _verbose;
    bool _enable_limit_flag;
    bool _is_floating;
    int _dof;
    VectorXd _q_min;
    VectorXd _q_max;
    VectorXd _dq_abs_max;
    VectorXd _tau_abs_max;
    VectorXd _pos_zone_1_threshold;
    VectorXd _pos_zone_2_threshold;
    VectorXd _vel_zone_1_threshold;
    VectorXd _vel_zone_2_threshold;
    VectorXi _joint_state;
    VectorXd _vel_gamma;
    VectorXd _kv_pos_limit;
    bool _truncation_flag;

    // verbose output 
    std::vector<std::string> _constraint_description;

    // task parameters
    std::vector<double> _blending_coefficients;
    MatrixXd _blending_matrix;
    VectorXd _non_task_safety_torques;
    MatrixXd _N_prec;
    MatrixXd _Jc;
    MatrixXd _Nc;
    MatrixXd _Lambda_c;
    MatrixXd _projected_jacobian;
    MatrixXd _current_task_range;

};

}  // namespace

#endif 