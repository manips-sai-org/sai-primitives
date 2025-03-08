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
                           const bool& truncation_flag,
                           const bool& is_floating,
                           const double& pos_zone_1,
                           const double& pos_zone_2,
                           const double& vel_zone_1,
                           const double& vel_zone_2,
                           const double& tau_thresh,
                           const double& tau_vel_thresh,
                           const double& t_delta,
                           const double& kv,
                           const double& gamma,
                           const std::vector<int>& joint_selection) : 
                           _robot(robot), 
                           _verbose(verbose), 
                           _is_floating(is_floating), 
                           _truncation_flag(truncation_flag),
                           _joint_selection(joint_selection),
                           _tau_thresh(tau_thresh),
                           _tau_vel_thresh(tau_vel_thresh),
                           _t_delta(t_delta) {

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

    // thresholds 
    _pos_zone_1_threshold = (pos_zone_1 * M_PI / 180) * VectorXd::Ones(_dof);
    _pos_zone_2_threshold = (pos_zone_2 * M_PI / 180) * VectorXd::Ones(_dof);
    _vel_zone_1_threshold = (vel_zone_1 * M_PI / 180) * VectorXd::Ones(_dof);
    _vel_zone_2_threshold = (vel_zone_2 * M_PI / 180) * VectorXd::Ones(_dof);
    _var_pos_zone_1_threshold = _pos_zone_1_threshold;

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

    _enable_limit_flag = true;
    _pos_entry_velocities = VectorXd::Zero(robot->dof());
    _vel_entry_velocities = VectorXd::Zero(robot->dof());
    _task_direction_wrt_constraint.setZero();
}

/**
 * @brief Update task model. Compute the constrained joint jacobian, blending coefficients 
 * 
 */
void JointHandler::updateTaskModel(const MatrixXd& N_prec) {
    
    _N_prec = N_prec;
    // if (_truncation_flag) {
    //     _blending_matrix = MatrixXd::Identity(_dof, _dof);
    // } else {
    //     _blending_matrix = MatrixXd::Zero(_dof, _dof);  // add the nullspace 
    // }

    // get kinematics 
    VectorXd q = _robot->q();
    VectorXd dq = _robot->dq();

    // project forward velocity to shift the first boundary layer if needed
    VectorXd q_proj = computePositionIntegration(q, dq, _t_delta);

    // check each joint for state (position is priority over velocity)
    // _joint_state.setZero();
    std::vector<double> alpha = {};
    int offset = 0;
    if (_is_floating) {
        offset = 6;
    }

    for (int i = 0 + offset; i < _dof; ++i) {

        if (_joint_selection.size() != 0) {
            if (std::find(_joint_selection.begin(), _joint_selection.end(), i) == _joint_selection.end()) {
                continue;
            }
        }

        /*
            Velocity limits check 
        */

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
        // } else {
        //     _joint_state(i) = SAFE;
        // }
            
        /*
            Position limits check 
        */

        // adjust variable position zone 1 threshold based on velocity 
        if (_joint_state(i) == SAFE) {
            // _var_pos_zone_1_threshold = _pos_zone_1_threshold;  // reset zone 1 threshold if safe 
            if (q_proj(i) > _q_max(i) - _pos_zone_2_threshold(i)) {
                _var_pos_zone_1_threshold(i) = std::max(_q_max(i) - q(i), _pos_zone_1_threshold(i));
                std::cout << "var threshold: " << _var_pos_zone_1_threshold(i) * 180 / M_PI << "\n";
            } else if (q_proj(i) < _q_min(i) + _pos_zone_2_threshold(i)) {
                _var_pos_zone_1_threshold(i) = std::max(q(i) - _q_min(i), _pos_zone_1_threshold(i));
                std::cout << "var threshold: " << _var_pos_zone_1_threshold(i) * 180 / M_PI << "\n";
            }
        }

        // position zone checks 
        if (q(i) >= _q_max(i) - _pos_zone_2_threshold(i)) {
            // apf 
            _joint_state(i) = MAX_HARD_POS;
            double q_zone_lower = _q_max(i) - _pos_zone_2_threshold(i);
            double q_zone_upper = _q_max(i);
            alpha.push_back(std::clamp((q(i) - q_zone_lower) / (q_zone_upper - q_zone_lower), 0.0, 1.0));
        } else if (q(i) >= _q_max(i) - _var_pos_zone_1_threshold(i)) {
            // velocity damping 
            _joint_state(i) = MAX_SOFT_POS;
            double q_zone_lower = _q_max(i) - _var_pos_zone_1_threshold(i);
            double q_zone_upper = _q_max(i) - _pos_zone_2_threshold(i);
            if (_joint_state(i) != SAFE) {
                alpha.back() = std::clamp((q(i) - q_zone_lower) / (q_zone_upper - q_zone_lower), 0.0, 1.0);
            } else {
                alpha.push_back(std::clamp((q(i) - q_zone_lower) / (q_zone_upper - q_zone_lower), 0.0, 1.0));
            }
        } else if (q(i) <= _q_min(i) + _pos_zone_2_threshold(i)) {
            // apf
            _joint_state(i) = MIN_HARD_POS;
            double q_zone_lower = _q_min(i) + _pos_zone_2_threshold(i);
            double q_zone_upper = _q_min(i);
            alpha.push_back(std::clamp((q(i) - q_zone_lower) / (q_zone_upper - q_zone_lower), 0.0, 1.0));
        } else if (q(i) <= _q_min(i) + _var_pos_zone_1_threshold(i)) {
            // velocity damping 
            _joint_state(i) = MIN_SOFT_POS;
            double q_zone_lower = _q_min(i) + _var_pos_zone_1_threshold(i);
            double q_zone_upper = _q_min(i) + _pos_zone_2_threshold(i);
            if (_joint_state(i) != SAFE) {
                alpha.back() = std::clamp((q(i) - q_zone_lower) / (q_zone_upper - q_zone_lower), 0.0, 1.0);
            } else {
                alpha.push_back(std::clamp((q(i) - q_zone_lower) / (q_zone_upper - q_zone_lower), 0.0, 1.0));
            }
        } else {
            _joint_state(i) = SAFE;
        }

        // // position zones 
        // if (q(i) > _q_max(i) - _pos_zone_2_threshold(i)) {
        //     // apf 
        //     _joint_state(i) = MAX_HARD_POS;
        //     double q_zone_lower = _q_max(i) - _pos_zone_2_threshold(i);
        //     double q_zone_upper = _q_max(i);
        //     alpha.push_back(std::clamp((q(i) - q_zone_lower) / (q_zone_upper - q_zone_lower), 0.0, 1.0));

        // } else if (q(i) > _q_max(i) - _pos_zone_1_threshold(i)) {
        //     // velocity damping region 
        //     _joint_state(i) = MAX_SOFT_POS;
        //     double q_zone_lower = _q_max(i) - _pos_zone_1_threshold(i);
        //     double q_zone_upper = _q_max(i) - _pos_zone_2_threshold(i);
        //     alpha.push_back(std::clamp((q(i) - q_zone_lower) / (q_zone_upper - q_zone_lower), 0.0, 1.0));

        // } else if (q(i) < _q_min(i) + _pos_zone_2_threshold(i)) {
        //     // apf
        //     _joint_state(i) = MIN_HARD_POS;
        //     double q_zone_lower = _q_min(i) + _pos_zone_2_threshold(i);
        //     double q_zone_upper = _q_min(i);
        //     alpha.push_back(std::clamp((q(i) - q_zone_lower) / (q_zone_upper - q_zone_lower), 0.0, 1.0));

        // } else if (q(i) < _q_min(i) + _pos_zone_1_threshold(i)) {
        //     // velocity damping region
        //     _joint_state(i) = MIN_SOFT_POS;
        //     double q_zone_lower = _q_min(i) + _pos_zone_1_threshold(i);
        //     double q_zone_upper = _q_min(i) + _pos_zone_2_threshold(i);
        //     alpha.push_back(std::clamp((q(i) - q_zone_lower) / (q_zone_upper - q_zone_lower), 0.0, 1.0));

        // } else {
        //     _joint_state(i) = SAFE;
        // }
    }

    // // set alpha to the smallest value 
    // // Find the smallest element
    // double min_val = *std::min_element(alpha.begin(), alpha.end());

    // // Set all elements to the smallest value
    // std::fill(alpha.begin(), alpha.end(), min_val);
    
    // form constraint-space op-space terms
    int num_con = 0;
    for (int i = 0; i < _dof; ++i) {
        if (_joint_state(i) != SAFE) {
            num_con++;
        }
    }

    if (num_con == 0 || !_enable_limit_flag) {
        _blending_coefficients = alpha;
        _non_task_safety_torques = VectorXd::Zero(_dof);
        _Jc = MatrixXd::Zero(1, _dof);
        _Nc = MatrixXd::Identity(_dof, _dof);
        _blending_matrix = MatrixXd::Zero(_dof, _dof);
        return;
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
                std::cout << val << ", ";
            }
        }

        // Vector to store the indices of non-zero elements
        std::vector<int> non_zero_indices;
        _blending_matrix = MatrixXd::Zero(num_con, _dof);

        // Iterate over the non-zero elements and store their indices
        int cnt = 0;
        for (int i = 0; i < _joint_state.size(); ++i) {
            if (_joint_state[i] != SAFE) {
                non_zero_indices.push_back(i);
                if (_joint_state[i] == MAX_HARD_POS || _joint_state[i] == MIN_HARD_POS) {
                    _blending_matrix(cnt, i) = std::pow((1 - alpha[cnt]), 2);
                    cnt++;
                } else if (_joint_state[i] == MAX_HARD_VEL || _joint_state[i] == MIN_HARD_VEL) {
                    _blending_matrix(cnt, i) = 0;
                    cnt++;
                } else if (_joint_state[i] == MAX_SOFT_VEL || _joint_state[i] == MIN_SOFT_VEL) {
                    _blending_matrix(cnt, i) = 1 - alpha[cnt];
                    cnt++;
                } else if (_joint_state[i] == MAX_SOFT_POS || _joint_state[i] == MIN_SOFT_POS) {
                    _blending_matrix(cnt, i) = 1;
                    cnt++;
                }
            }
        }
        _Jc = MatrixXd::Zero(num_con, _dof);
        for (int i = 0; i < num_con; ++i) {
            _Jc(i, non_zero_indices[i]) = 1;
        }
        // _blending_matrix = _blending_matrix * _Jc;
        _projected_jacobian = _Jc * _N_prec;

        // decomposition 
        _current_task_range = Sai2Model::matrixRangeBasis(_projected_jacobian);
        // _current_task_range.setIdentity();

        // if (_is_floating) {
        //     MatrixXd U = MatrixXd::Zero(6, _dof);
        //     U.block(0, 0, 6, 6).setIdentity();
        //     MatrixXd N_float = _robot->nullspaceMatrix(U);
        //     Jc *= N_float;
        // }

        Sai2Model::OpSpaceMatrices con_matrices = _robot->operationalSpaceMatrices(_current_task_range.transpose() * _projected_jacobian);
        // Sai2Model::OpSpaceMatrices con_matrices = _robot->operationalSpaceMatrices(_projected_jacobian);
        _Lambda_c = con_matrices.Lambda;
        _Jbar_c = con_matrices.Jbar;
        _Nc = con_matrices.N;
        _blending_coefficients = alpha;
        _blending_matrix = (_current_task_range.transpose() * _projected_jacobian).transpose() * _current_task_range.transpose() * \
                                _blending_matrix * (MatrixXd::Identity(_dof, _dof) - _Nc.transpose());  // blending the torque into the constraint space from the total torques
        // _blending_matrix = (_projected_jacobian).transpose() * \
                                // _blending_matrix * (MatrixXd::Identity(_dof, _dof) - _Nc.transpose());  // blending the torque into the constraint space from the total torques
    }
}

VectorXd JointHandler::computeTorques(const VectorXd& torques) {
    
    // update robot  
    VectorXd q = _robot->q();
    VectorXd dq = _robot->dq();
    
    // form constraint-space op-space terms
    int num_con = 0;
    for (int i = 0; i < _dof; ++i) {
        if (_joint_state(i) != SAFE) {
            num_con++;
        }
    }

    if (num_con == 0 || !_enable_limit_flag) {
        return torques;
    } else {        

        // compute torques in constraint and dot-product/magnitude check 
        VectorXd projected_torques_in_constraint = (MatrixXd::Identity(_dof, _dof) - _Nc.transpose()) * torques;
        // VectorXd projected_torques_in_constraint = _Jbar_c.transpose() * torques;
        // std::cout << "projected torques in constraint: " << projected_torques_in_constraint.transpose() << "\n";
        VectorXd projected_torques_not_in_constraint = _Nc.transpose() * torques; 

        // containers for apf torques, unit-mass damping torques
        VectorXd con_apf_torques = VectorXd::Zero(num_con);
        VectorXd con_task_torques = VectorXd::Zero(num_con);
        VectorXd con_unit_damping_torques = VectorXd::Zero(num_con);

        // VectorXd element_wise_projected_task_torques = VectorXd::Zero(num_con);
        // VectorXd element_wise_non_task_safety_torques = VectorXd::Zero(num_con);
        // VectorXd safety_torques = VectorXd::Zero(num_con);
        // VectorXd unit_mass_torques = VectorXd::Zero(num_con);  // stores the unit-mass torques from constraint strategy
        // VectorXd truncated_torques = torques;

        // main joint handling 
        int cnt = 0;
        for (int i = 0; i < _dof; ++i) {

            // if (_joint_state[i] != SAFE && _truncation_flag) {
            //     projected_torques_not_in_constraint(i) = 0;
            // }

            if (_joint_state(i) == MIN_SOFT_VEL) {
                // element_wise_projected_task_torques(cnt) = (1 - _blending_coefficients[cnt]) * projected_torques_in_constraint(i);
                // safety_torques(cnt) = (1 - _blending_coefficients[cnt]) * torques(i);
                // truncated_torques(i) = 0;
                // cnt++;

                double dq_des = - _dq_abs_max(i) + _vel_zone_1_threshold(i);
                con_unit_damping_torques(cnt) = - _kv_pos_limit(i) * (dq(i) - dq_des);  // velocity follower
                if (projected_torques_in_constraint(i) > _tau_vel_thresh) {
                    con_task_torques(cnt) = projected_torques_in_constraint(i);
                    con_unit_damping_torques(cnt) = 0;
                }
                cnt++;

            } else if (_joint_state(i) == MIN_HARD_VEL) {
                // element_wise_non_task_safety_torques(cnt) = _vel_gamma(i) * std::pow(_blending_coefficients[cnt], 4) * _tau_abs_max(i);
                // _non_task_safety_torques(i) = _vel_gamma(i) * std::pow(_blending_coefficients[cnt], 4) * _tau_abs_max(i);
                // safety_torques(cnt) = _vel_gamma(i) * std::pow(_blending_coefficients[cnt], 4) * _tau_abs_max(i);
                // element_wise_non_task_safety_torques(cnt) = _vel_gamma(i) * std::pow(_blending_coefficients[cnt], 4) * _tau_abs_max(i);
                // truncated_torques(i) = 0;
                // cnt++;

                double dq_des = - _dq_abs_max(i) + _vel_zone_1_threshold(i);
                con_unit_damping_torques(cnt) = - _kv_pos_limit(i) * (dq(i) - dq_des);  // velocity follower
                if (projected_torques_in_constraint(i) > _tau_vel_thresh) {
                    con_task_torques(cnt) = projected_torques_in_constraint(i);
                    con_unit_damping_torques(cnt) = 0;
                }
                cnt++;

            } else if (_joint_state(i) == MAX_SOFT_VEL) {
                // element_wise_projected_task_torques(cnt) = (1 - _blending_coefficients[cnt]) * projected_torques_in_constraint(i);
                // safety_torques(cnt) = (1 - _blending_coefficients[cnt]) * torques(i);
                // truncated_torques(i) = 0;
                // cnt++;

                double dq_des = _dq_abs_max(i) - _vel_zone_1_threshold(i);
                con_unit_damping_torques(cnt) = - _kv_pos_limit(i) * (dq(i) - dq_des);  // velocity follower
                if (projected_torques_in_constraint(i) <  - _tau_vel_thresh) {
                    con_task_torques(cnt) = projected_torques_in_constraint(i);
                    con_unit_damping_torques(cnt) = 0;
                }
                cnt++;

            } else if (_joint_state(i) == MAX_HARD_VEL) {
                // element_wise_non_task_safety_torques(cnt) = - _vel_gamma(i) * std::pow(_blending_coefficients[cnt], 4) * _tau_abs_max(i);
                // _non_task_safety_torques(i) = - _vel_gamma(i) * std::pow(_blending_coefficients[cnt], 4) * _tau_abs_max(i);
                // safety_torques(cnt) = - _vel_gamma(i) * std::pow(_blending_coefficients[cnt], 4) * _tau_abs_max(i);
                // element_wise_non_task_safety_torques(cnt) = - _vel_gamma(i) * std::pow(_blending_coefficients[cnt], 4) * _tau_abs_max(i);
                // truncated_torques(i) = 0;
                // cnt++;

                double dq_des = _dq_abs_max(i) - _vel_zone_1_threshold(i);
                con_unit_damping_torques(cnt) = - _kv_pos_limit(i) * (dq(i) - dq_des);  // velocity follower
                if (projected_torques_in_constraint(i) <  - _tau_vel_thresh) {
                    con_task_torques(cnt) = projected_torques_in_constraint(i);
                    con_unit_damping_torques(cnt) = 0;
                }
                cnt++;

            } else if (_joint_state(i) == MIN_SOFT_POS) {

                // apply damping 
                // con_unit_damping_torques(cnt) = - _kv_pos_limit(i) * dq(i);

                // dot product check
                if (projected_torques_in_constraint(i) > _tau_thresh) {
                    con_task_torques(cnt) = projected_torques_in_constraint(i);
                    _var_pos_zone_1_threshold(i) = _pos_zone_1_threshold(i);
                    // con_unit_damping_torques(cnt) = - std::pow(_blending_coefficients[cnt], 2) * _kv_pos_limit(i) * dq(i);
                } else {
                    con_unit_damping_torques(cnt) = - _kv_pos_limit(i) * dq(i);
                }
                cnt++;

            } else if (_joint_state(i) == MIN_HARD_POS) {
                // apply damping 
                // con_apf_torques(cnt) = std::pow(_blending_coefficients[cnt], 2) * _tau_abs_max(i);
                // con_unit_damping_torques(cnt) = - std::pow(_blending_coefficients[cnt], 2) * _kv_pos_limit(i) * dq(i);
                // con_unit_damping_torques(cnt) = - _kv_pos_limit(i) * dq(i);

                // dot product check
                if (projected_torques_in_constraint(i) > _tau_thresh) {
                    con_task_torques(cnt) = projected_torques_in_constraint(i);
                    _var_pos_zone_1_threshold(i) = _pos_zone_1_threshold(i);
                    // con_unit_damping_torques(cnt) = - std::pow(_blending_coefficients[cnt], 2) * _kv_pos_limit(i) * dq(i);
                } else {
                    con_unit_damping_torques(cnt) = - _kv_pos_limit(i) * dq(i);
                }
                cnt++;

            } else if (_joint_state(i) == MAX_SOFT_POS) {                
                // apply apf 
                con_apf_torques(cnt) = - std::pow(_blending_coefficients[cnt], 2) * _tau_abs_max(i);
                // con_unit_damping_torques(cnt) = - std::pow(_blending_coefficients[cnt], 2) * _kv_pos_limit(i) * dq(i);
                // con_unit_damping_torques(cnt) = - _kv_pos_limit(i) * dq(i);

                // dot product check
                if (projected_torques_in_constraint(i) < - _tau_thresh) {
                    con_task_torques(cnt) = projected_torques_in_constraint(i);
                    _var_pos_zone_1_threshold(i) = _pos_zone_1_threshold(i);
                    // con_unit_damping_torques(cnt) = - std::pow(_blending_coefficients[cnt], 2) * _kv_pos_limit(i) * dq(i);
                } else {
                    con_unit_damping_torques(cnt) = - _kv_pos_limit(i) * dq(i);
                }
                cnt++;

            } else if (_joint_state(i) == MAX_HARD_POS) {
                // apply apf 
                con_apf_torques(cnt) = - std::pow(_blending_coefficients[cnt], 2) * _tau_abs_max(i);
                // con_unit_damping_torques(cnt) = - std::pow(_blending_coefficients[cnt], 2) * _kv_pos_limit(i) * dq(i);
                con_unit_damping_torques(cnt) = - _kv_pos_limit(i) * dq(i);

                // dot product check
                if (projected_torques_in_constraint(i) < - _tau_thresh) {
                    con_task_torques(cnt) = projected_torques_in_constraint(i);
                    _var_pos_zone_1_threshold(i) = _pos_zone_1_threshold(i);
                    // con_unit_damping_torques(cnt) = - std::pow(_blending_coefficients[cnt], 2) * _kv_pos_limit(i) * dq(i);
                } else {
                    con_unit_damping_torques(cnt) = - _kv_pos_limit(i) * dq(i);
                }
                cnt++;

            } else {
                // do nothing (safe)
            }
        }
        
        // compute constrained torques (unit mass damping + task torques + apf torques)
        VectorXd total_torques = VectorXd::Zero(_dof);
        total_torques += 1 * (_current_task_range.transpose() * _projected_jacobian).transpose() * \
                                _Lambda_c * _current_task_range.transpose() * con_unit_damping_torques;
        total_torques += 1 * (_current_task_range.transpose() * _projected_jacobian).transpose() * \
                                _current_task_range.transpose() * (1 * con_task_torques + 1 * con_apf_torques);
        // total_torques += 1 * (_projected_jacobian).transpose() * _Lambda_c * con_unit_damping_torques;
        // total_torques += _projected_jacobian.transpose() * (con_task_torques + con_apf_torques);
        total_torques += 1 * projected_torques_not_in_constraint;

        // _non_task_safety_torques = (_current_task_range.transpose() * _projected_jacobian).transpose() * _current_task_range.transpose() * element_wise_non_task_safety_torques;
        // _non_task_safety_torques += (_current_task_range.transpose() * _projected_jacobian).transpose() * _Lambda_c * _current_task_range.transpose() * unit_mass_torques * 0;
        
        // torque saturation    
        for (int i = 0; i < _dof; ++i) {
            if (std::abs(total_torques(i)) > _tau_abs_max(i)) {
                total_torques(i) = getSign(total_torques(i)) * _tau_abs_max(i);
            }
        }

        // total torques debug
        // std::cout << "total torques: " << total_torques.transpose() << "\n";
        // std::cout << "con apf torques: \n" << con_apf_torques.transpose() << "\n";

        return total_torques;
    }

}

}  // namespace 