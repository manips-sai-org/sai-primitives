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
                 const double& pos_zone_1 = 3,
                 const double& pos_zone_2 = 1,
                 const double& vel_zone_1 = 0.1,
                 const double& vel_zone_2 = 0.05,
                 const double& kv = 20,
                 const double& gamma = 1e-4);

    VectorXd computeTorques(const VectorXd& torques);

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
    bool _verbose;
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

    // verbose output 
    std::vector<std::string> _constraint_description;

};

}  // namespace

#endif 