/*
 * JointHandler.cpp
 *
 *      Author: William Chong 
 */

#include "JointHandler.h"

namespace Sai2Primitives {

int getSign(double value) {
    if (value > 0) {
        return 1;
    } else if (value < 0) {
        return -1;
    } else {
        return 0;
    }
}

JointHandler::JointHandler(std::shared_ptr<Sai2Model::Sai2Model> robot,
                           const bool& verbose,
                           const double& pos_zone_1,
                           const double& pos_zone_2,
                           const double& vel_zone_1,
                           const double& vel_zone_2,
                           const double& kv,
                           const double& gamma) : _robot(robot), _verbose(verbose) {

    // initialize variables
    _dof = _robot->dof();
    _q_min = VectorXd::Zero(_dof);
    _q_max = VectorXd::Zero(_dof);
    _dq_abs_max = VectorXd::Zero(_dof);
    _tau_abs_max = VectorXd::Zero(_dof);
    _joint_state = VectorXi::Zero(_dof);
    for (int i = 0; i < _dof; ++i) {
        _joint_state(i) = SAFE;
    }
    _kv_pos_limit = kv * VectorXd::Ones(_dof);

    // default zone 1 and zone 2 threshold at 6 and 9 degrees
    // default zone 1 and zone threshold at 0.6 and 0.8 rad/s
    _pos_zone_1_threshold = (pos_zone_1 * M_PI / 180) * VectorXd::Ones(_dof);
    _pos_zone_2_threshold = (pos_zone_2 * M_PI / 180) * VectorXd::Ones(_dof);
    _vel_zone_1_threshold = vel_zone_1 * VectorXd::Ones(_dof);
    _vel_zone_2_threshold = vel_zone_2 * VectorXd::Ones(_dof);

    // get joint limits from robot 
    auto joint_limits = _robot->jointLimits();
    for (int i = 0; i < joint_limits.size(); ++i) {
        _q_min(i) = joint_limits[i].position_lower;
        _q_max(i) = joint_limits[i].position_upper;
        _dq_abs_max(i) = joint_limits[i].velocity;
        _tau_abs_max(i) = joint_limits[i].effort;
    }

    _vel_gamma = gamma * VectorXd::Ones(_dof);

    if (_verbose) {
        std::cout << "Joint handler parameters:\n";
        std::cout << "q min: \n" << _q_min.transpose() << "\n";
        std::cout << "q max: \n" << _q_max.transpose() << "\n";
        std::cout << "dq max: \n" << _dq_abs_max.transpose() << "\n";
        std::cout << "tau max: \n" << _tau_abs_max.transpose() << "\n";
    }

    _constraint_description = {"Safe", 
                               "Minimum Soft Velocity",
                               "Minimum Hard Velocity",
                               "Maximum Soft Velocity",
                               "Maximum Hard Velocity",
                               "Minimum Soft Position",
                               "Minimum Hard Position",
                               "Maximum Soft Position",
                               "Maximum Hard Position"};

}

VectorXd JointHandler::computeTorques(const VectorXd& torques) {
    
    VectorXd q = _robot->q();
    VectorXd dq = _robot->dq();

    // check each joint for state (position is priority over velocity)
    _joint_state.setZero();
    std::vector<double> alpha = {};

    for (int i = 0; i < _dof; ++i) {

        // // velocity zone check
        // if (std::abs(dq(i)) > _dq_abs_max(i) - _vel_zone_2_threshold(i)) {
        //     double dq_zone_lower = _dq_abs_max(i) - _vel_zone_2_threshold(i);
        //     double dq_zone_upper = _dq_abs_max(i);
        //     alpha.push_back(std::clamp((std::abs(dq(i)) - dq_zone_lower) / (dq_zone_upper - dq_zone_lower), 0.0, 1.0));
        //     if (getSign(dq(i)) > 0) {
        //         _joint_state(i) = MAX_HARD_VEL;
        //     } else {
        //         _joint_state(i) = MIN_HARD_VEL;
        //     }
        // } else if (std::abs(dq(i)) > _dq_abs_max(i) - _vel_zone_1_threshold(i)) {
        //     double dq_zone_lower = _dq_abs_max(i) - _vel_zone_1_threshold(i);
        //     double dq_zone_upper = _dq_abs_max(i) - _vel_zone_2_threshold(i);
        //     alpha.push_back(std::clamp((std::abs(dq(i)) - dq_zone_lower) / (dq_zone_upper - dq_zone_lower), 0.0, 1.0));
        //     if (getSign(dq(i)) > 0) {
        //         _joint_state(i) = MAX_SOFT_VEL;
        //     } else {
        //         _joint_state(i) = MIN_SOFT_VEL;
        //     }
        // }
            
        // position zone check
        if (getSign(q(i)) > 0) {
            if (q(i) > _q_max(i) - _pos_zone_2_threshold(i)) {
                _joint_state(i) = MAX_HARD_POS;
                double q_zone_lower = _q_max(i) - _pos_zone_2_threshold(i);
                double q_zone_upper = _q_max(i);
                alpha.push_back(std::clamp((q(i) - q_zone_lower) / (q_zone_upper - q_zone_lower), 0.0, 1.0));
            } else if (q(i) > _q_max(i) - _pos_zone_1_threshold(i)) {
                _joint_state(i) = MAX_SOFT_POS;
                double q_zone_lower = _q_max(i) - _pos_zone_1_threshold(i);
                double q_zone_upper = _q_max(i) - _pos_zone_2_threshold(i);
                alpha.push_back(std::clamp((q(i) - q_zone_lower) / (q_zone_upper - q_zone_lower), 0.0, 1.0));
            }
        } else {
            if (q(i) < _q_min(i) + _pos_zone_2_threshold(i)) {
                _joint_state(i) = MIN_HARD_POS;
                double q_zone_lower = _q_min(i) + _pos_zone_2_threshold(i);
                double q_zone_upper = _q_min(i);
                alpha.push_back(std::clamp((q(i) - q_zone_lower) / (q_zone_upper - q_zone_lower), 0.0, 1.0));
            } else if (q(i) < _q_min(i) + _pos_zone_1_threshold(i)) {
                _joint_state(i) = MIN_SOFT_POS;
                double q_zone_lower = _q_min(i) + _pos_zone_1_threshold(i);
                double q_zone_upper = _q_min(i) + _pos_zone_2_threshold(i);
                alpha.push_back(std::clamp((q(i) - q_zone_lower) / (q_zone_upper - q_zone_lower), 0.0, 1.0));
            }
        }
    }
    
    // form constraint-space op-space terms
    int num_con = 0;
    for (int i = 0; i < _dof; ++i) {
        if (_joint_state(i) != SAFE) {
            num_con++;
        }
    }

    if (num_con == 0) {
        return torques;

    } else {

        // verbose
        if (_verbose) {
            std::cout << "---\n";
            for (int i = 0; i < _dof; ++i) {
                int con_des = _joint_state[i];
                // std::cout << "Joint " << i << ": " << _constraint_description[con_des] << "\n";
                if (con_des != SAFE) {
                    std::cout << "Joint " << i << " State: " << _constraint_description[con_des] << "\n";
                    // if (con_des == MIN_SOFT_POS || con_des == MIN_HARD_POS) {
                    //     std::cout << "q: " << q(i) << ", q_min: " << _q_min(i) << "\n";
                    // } else if (con_des == MAX_SOFT_POS || con_des == MAX_HARD_POS) {
                    //     std::cout << "q: " << q(i) << ", q_max: " << _q_max(i) << "\n";
                    // }
                }
            }

            // alphas
            std::cout << "alpha\n";
            for (auto val : alpha) {
                std::cout << val << "\n";
            }
        }

        // Vector to store the indices of non-zero elements
        std::vector<int> non_zero_indices;

        // Iterate over the non-zero elements and store their indices
        for (int i = 0; i < _joint_state.size(); ++i) {
            if (_joint_state[i] != SAFE) {
                non_zero_indices.push_back(i);
            }
        }
        MatrixXd Jc = MatrixXd::Zero(num_con, _dof);
        for (int i = 0; i < num_con; ++i) {
            Jc(i, non_zero_indices[i]) = 1;
        }

        Sai2Model::OpSpaceMatrices con_matrices = _robot->operationalSpaceMatrices(Jc);
        MatrixXd Lambda_c = con_matrices.Lambda;
        MatrixXd Jbar_c = con_matrices.Jbar;
        MatrixXd N_c = con_matrices.N;

        VectorXd safety_torques = VectorXd::Zero(num_con);
        VectorXd unit_mass_torques = VectorXd::Zero(num_con);  // stores the unit-mass torques from constraint strategy
        VectorXd original_torques = torques;

        // joint handling for constrained joints 
        int cnt = 0;
        for (int i = 0; i < _dof; ++i) {
            if (_joint_state(i) == MIN_SOFT_VEL) {
                safety_torques(cnt) = (1 - alpha[cnt]) * torques(i);
                original_torques(i) = 0;
                cnt++;

            } else if (_joint_state(i) == MIN_HARD_VEL) {
                safety_torques(cnt) = _vel_gamma(i) * std::pow(alpha[cnt], 4) * _tau_abs_max(i);
                original_torques(i) = 0;
                cnt++;

            } else if (_joint_state(i) == MAX_SOFT_VEL) {
                safety_torques(cnt) = (1 - alpha[cnt]) * torques(i);
                original_torques(i) = 0;
                cnt++;

            } else if (_joint_state(i) == MAX_HARD_VEL) {
                safety_torques(cnt) = - _vel_gamma(i) * std::pow(alpha[cnt], 4) * _tau_abs_max(i);
                original_torques(i) = 0;
                cnt++;

            } else if (_joint_state(i) == MIN_SOFT_POS) {
                safety_torques(cnt) = torques(i);
                unit_mass_torques(cnt) = - std::pow(alpha[cnt], 2) * _kv_pos_limit(i) * dq(i);
                original_torques(i) = 0;
                cnt++;

            } else if (_joint_state(i) == MIN_HARD_POS) {
                safety_torques(cnt) = std::pow(1 - alpha[cnt], 2) * torques(i) + \
                                        std::pow(alpha[cnt], 2) * _tau_abs_max(i);
                unit_mass_torques(cnt) = - _kv_pos_limit(i) * dq(i);
                original_torques(i) = 0;
                cnt++;

            } else if (_joint_state(i) == MAX_SOFT_POS) {
                safety_torques(cnt) = torques(i);
                unit_mass_torques(cnt) = - std::pow(alpha[cnt], 2) * _kv_pos_limit(i) * dq(i);
                original_torques(i) = 0;
                cnt++;

            } else if (_joint_state(i) == MAX_HARD_POS) {
                safety_torques(cnt) = std::pow(1 - alpha[cnt], 2) * torques(i) + \
                                        (- 1.0) * std::pow(alpha[cnt], 2) * _tau_abs_max(i);
                unit_mass_torques(cnt) = - _kv_pos_limit(i) * dq(i);
                original_torques(i) = 0;
                cnt++;

            } else {
                // do nothing (safe)
            }
        }
        
        // compute constrained torques
        // std::cout << "unit mass torques: \n" << Jc.transpose() * Lambda_c * unit_mass_torques << "\n";
        // VectorXd total_torques = \
        //     Jc.transpose() * (safety_torques + Lambda_c * unit_mass_torques) + N_c.transpose() * torques;
        VectorXd total_torques = \
            Jc.transpose() * (safety_torques + Lambda_c * unit_mass_torques) + original_torques;

        // torque saturation    
        for (int i = 0; i < _dof; ++i) {
            if (std::abs(total_torques(i)) > _tau_abs_max(i)) {
                total_torques(i) = getSign(total_torques(i)) * _tau_abs_max(i);
            }
        }

        return total_torques;
    }

}

}  // namespace 