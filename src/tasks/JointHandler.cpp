/*
 * JointHandler.cpp
 *
 *      Author: William Chong 
 */

#include "JointHandler.h"

namespace Sai2Primitives {

inline int getSign(double value) {
    if (value > 0) {
        return 1;
    } else if (value < 0) {
        return -1;
    } else {
        return 0;
    }
}

// sigmoid function for velocity saturation
// alpha from 0 -> 1 as approaching constraint 
inline double getMaxVelFunction(double alpha, double entry_vel, double exit_vel) {
    // std::cout << "alpha: " << alpha << "\n";
    if (getSign(entry_vel) != getSign(exit_vel)) {
        exit_vel *= -1;
    }
    return exit_vel + (entry_vel - exit_vel) * ((1 - cos(M_PI * (1 - alpha))) / 2);
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
    _joint_vel_state = VectorXi::Zero(_dof);
    for (int i = 0; i < _dof; ++i) {
        _joint_state(i) = SAFE;
        _joint_vel_state(i) = SAFE;
    }
    _kv_pos_limit = kv * VectorXd::Ones(_dof);

    // thresholds 
    // _pos_zone_1_threshold = (pos_zone_1 * M_PI / 180) * VectorXd::Ones(_dof);
    _pos_zone_1_threshold = (pos_zone_2 * M_PI / 180) * VectorXd::Ones(_dof);  // set at zone 2 threshold by default
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

    // blending coefficients (one for each joint (pos/vel))
    _blending_coefficients = VectorXd::Zero(robot->dof());
    _vel_blending_coefficients = VectorXd::Zero(robot->dof());
    _non_task_safety_torques = VectorXd::Zero(_dof);
    _blending_matrix = MatrixXd::Zero(_dof, _dof);
    _num_con = 0;

    _enable_vel_limits = false;

    // setup firas function
    _rho = VectorXd::Zero(robot->dof());
    _rho_0 = _pos_zone_2_threshold;
    _eta = 0.1 * VectorXd::Ones(robot->dof());
    _apf_torques = VectorXd::Zero(robot->dof());
    _joint_distances = std::numeric_limits<double>::infinity() * VectorXd::Ones(robot->dof());

    // setup moving outer zone boundary layer 
    _t_collision = 0.3;  // time until collision check 
    // _t_collision = 0.5;
    _entry_velocity = VectorXd::Zero(robot->dof());
    _exit_velocity = VectorXd::Zero(robot->dof());
    _dq_exit_tol = 0.01;
    // _exit_velocity = 0.1 * VectorXd::Ones(robot->dof());

    _max_vel = - 1 * VectorXd::Ones(_dof);
    _variable_vel_zone = false;
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
    // VectorXd q_proj = computePositionIntegration(q, dq, _t_delta);

    // check each joint for state (position is priority over velocity)
    // _joint_state.setZero();
    _joint_vel_state.setZero();
    _blending_coefficients.setZero();
    _vel_blending_coefficients.setZero();
    _rho.setZero();
    _joint_distances = std::numeric_limits<double>::infinity() * VectorXd::Ones(_robot->dof());
    _rho_0 = _pos_zone_2_threshold;
    // _rho_0.setZero();
    // std::vector<double> alpha = {};
    // std::vector<double> vel_alpha = {};
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

        // // adjust variable position zone 1 threshold based on velocity 
        // if (_joint_state(i) != MIN_HARD_POS && \
        //     _joint_state(i) != MAX_HARD_POS && \
        //     _joint_state(i) != MIN_SOFT_POS && \
        //     _joint_state(i) != MAX_SOFT_POS) {
        //     _var_pos_zone_1_threshold(i) = _pos_zone_1_threshold(i);  // reset zone 1 threshold if safe 
        //     if (q_proj(i) > _q_max(i) - _pos_zone_2_threshold(i)) {
        //         _var_pos_zone_1_threshold(i) = std::max(_q_max(i) - q(i), _pos_zone_1_threshold(i));
        //         std::cout << "var threshold: " << _var_pos_zone_1_threshold(i) * 180 / M_PI << "\n";
        //     } else if (q_proj(i) < _q_min(i) + _pos_zone_2_threshold(i)) {
        //         _var_pos_zone_1_threshold(i) = std::max(q(i) - _q_min(i), _pos_zone_1_threshold(i));
        //         std::cout << "var threshold: " << _var_pos_zone_1_threshold(i) * 180 / M_PI << "\n";
        //     }
        // }

        /*
        ********************************
            Velocity limits check 
        ********************************
        */
        if (_enable_vel_limits) {
            if (std::abs(dq(i)) > _dq_abs_max(i) - _vel_zone_2_threshold(i)) {
                double dq_zone_lower = _dq_abs_max(i) - _vel_zone_2_threshold(i);
                double dq_zone_upper = _dq_abs_max(i);
                _vel_blending_coefficients(i) = std::clamp((std::abs(dq(i)) - dq_zone_lower) / (dq_zone_upper - dq_zone_lower), 0.0, 1.0);
                // vel_alpha.push_back(std::clamp((std::abs(dq(i)) - dq_zone_lower) / (dq_zone_upper - dq_zone_lower), 0.0, 1.0));
                if (getSign(dq(i)) > 0) {
                    _joint_vel_state(i) = MAX_HARD_VEL;
                } else {
                    _joint_vel_state(i) = MIN_HARD_VEL;
                }
            } else if (std::abs(dq(i)) > _dq_abs_max(i) - _vel_zone_1_threshold(i)) {
                double dq_zone_lower = _dq_abs_max(i) - _vel_zone_1_threshold(i);
                double dq_zone_upper = _dq_abs_max(i) - _vel_zone_2_threshold(i);
                _vel_blending_coefficients(i) = std::clamp((std::abs(dq(i)) - dq_zone_lower) / (dq_zone_upper - dq_zone_lower), 0.0, 1.0);
                // vel_alpha.push_back(std::clamp((std::abs(dq(i)) - dq_zone_lower) / (dq_zone_upper - dq_zone_lower), 0.0, 1.0));
                if (getSign(dq(i)) > 0) {
                    _joint_vel_state(i) = MAX_SOFT_VEL;
                } else {
                    _joint_vel_state(i) = MIN_SOFT_VEL;
                }
            } else {
                _joint_vel_state(i) = SAFE;
            }
        }

        /*
        ********************************
            Position limits check 
        ********************************
        */

        // check collision with the zone 1 threshold 
        double q_future = q(i) + dq(i) * _t_collision;

        // check for zone 2 threshold IF robot is SAFE 
        if (_joint_state(i) == SAFE && _variable_vel_zone) {
            if (q_future > _q_max(i) - _pos_zone_2_threshold(i) && q(i) < _q_max(i) - _pos_zone_2_threshold(i)) {
                // trigger velocity saturation
                std::cout << "JOINT HANDLER SAFE CHECK: MAX SOFT\n";
                _joint_state(i) = MAX_SOFT_POS;
                setPosZone1ThresholdIndex(_q_max(i) - q(i), i);
                _blending_coefficients(i) = 0;
                _entry_velocity(i) = dq(i);
            } else if (q_future < _q_min(i) + _pos_zone_2_threshold(i) && q(i) > _q_min(i) + _pos_zone_2_threshold(i)) {
                std::cout << "JOINT HANDLER SAFE CHECK: MIN SOFT\n";
                // trigger velocity saturation region 
                _joint_state(i) = MIN_SOFT_POS;
                setPosZone1ThresholdIndex(q(i) - _q_min(i), i);
                _blending_coefficients(i) = 0;
                _entry_velocity(i) = dq(i);
            } else {                
                setPosZone1ThresholdIndex(_pos_zone_2_threshold(i), i);  // reset to the zone 1 threshold by default
                _entry_velocity(i) = dq(i);
            }
        } else {            
            // position zone checks 
            if (q(i) > _q_max(i) - _pos_zone_2_threshold(i)) {
                // apf 
                _joint_state(i) = MAX_HARD_POS;
                double q_zone_lower = _q_max(i) - _pos_zone_2_threshold(i);
                double q_zone_upper = _q_max(i);
                // alpha.push_back(std::clamp((q(i) - q_zone_lower) / (q_zone_upper - q_zone_lower), 0.0, 1.0));
                // _blending_coefficients(i) = std::clamp(std::abs((q(i) - q_zone_lower) / (q_zone_upper - q_zone_lower)), 0.0, 1.0);

                _rho(i) = std::clamp(_q_max(i) - _robot->q()(i), 1e-6, _pos_zone_2_threshold(i));
                _joint_distances(i) = _rho(i);

            // } else if (q(i) >= _q_max(i) - _var_pos_zone_1_threshold(i)) {
            } else if (q(i) > _q_max(i) - _pos_zone_1_threshold(i)) {
            // } else if (q(i) > _q_max(i) - _pos_zone_1_threshold(i)) {

                if (_joint_state(i) == MAX_HARD_POS) {
                    _rho(i) = _pos_zone_2_threshold(i);
                    _joint_distances(i) = _rho(i);
                } else {

                    // velocity damping 
                    _joint_state(i) = MAX_SOFT_POS;
                    // double q_zone_lower = _q_max(i) - _var_pos_zone_1_threshold(i);
                    double q_zone_lower = _q_max(i) - _pos_zone_1_threshold(i);
                    double q_zone_upper = _q_max(i) - _pos_zone_2_threshold(i);
                    // alpha.push_back(std::clamp((q(i) - q_zone_lower) / (q_zone_upper - q_zone_lower), 0.0, 1.0));
                    _blending_coefficients(i) = std::clamp(std::abs((q(i) - q_zone_lower) / (q_zone_upper - q_zone_lower)), 0.0, 1.0);

                    _joint_distances(i) = std::abs(q(i) - q_zone_lower);
                }

            } else if (q(i) < _q_min(i) + _pos_zone_2_threshold(i)) {
                // apf
                _joint_state(i) = MIN_HARD_POS;
                double q_zone_lower = _q_min(i) + _pos_zone_2_threshold(i);
                double q_zone_upper = _q_min(i);
                // alpha.push_back(std::clamp((q(i) - q_zone_lower) / (q_zone_upper - q_zone_lower), 0.0, 1.0));
                // _blending_coefficients(i) = std::clamp(std::abs((q(i) - q_zone_lower) / (q_zone_upper - q_zone_lower)), 0.0, 1.0);

                _rho(i) = std::clamp(_robot->q()(i) - _q_min(i), 1e-6, _pos_zone_2_threshold(i));
                _joint_distances(i) = _rho(i);

            // } else if (q(i) <= _q_min(i) + _var_pos_zone_1_threshold(i)) {
            } else if (q(i) < _q_min(i) + _pos_zone_1_threshold(i)) {

                if (_joint_state(i) == MIN_HARD_POS) {
                    _rho(i) = _pos_zone_2_threshold(i);
                    _joint_distances(i) = _rho(i);
                } else {

                // } else if (q(i) < _q_min(i) + _pos_zone_1_threshold(i)) {
                    // velocity damping 
                    // std::cout << "Task Min Soft Passthrough\n";
                    // std:: cout << q(i) << "\n";
                    // std::cout << _q_min(i) + _pos_zone_1_threshold(i) << "\n";
                    // std::cout << _q_max(i) - _pos_zone_1_threshold(i) << "\n";
                    _joint_state(i) = MIN_SOFT_POS;
                    // double q_zone_lower = _q_min(i) + _var_pos_zone_1_threshold(i);
                    double q_zone_lower = _q_min(i) + _pos_zone_1_threshold(i);
                    double q_zone_upper = _q_min(i) + _pos_zone_2_threshold(i);
                    // alpha.push_back(std::clamp((q(i) - q_zone_lower) / (q_zone_upper - q_zone_lower), 0.0, 1.0));
                    _blending_coefficients(i) = std::clamp(std::abs((q(i) - q_zone_lower) / (q_zone_upper - q_zone_lower)), 0.0, 1.0);

                    _joint_distances(i) = std::abs(q(i) - q_zone_lower);

                }

            } else {
                // safe if outside at all 
                _joint_state(i) = SAFE;
            }

        }
    

    }

    // // set alpha to the smallest value 
    // // Find the smallest element
    // double min_val = *std::min_element(alpha.begin(), alpha.end());

    // // Set all elements to the smallest value
    // std::fill(alpha.begin(), alpha.end(), min_val);
    
    // form constraint-space op-space terms
    _num_con = 0;
    for (int i = 0; i < _dof; ++i) {
        if (_joint_state(i) != SAFE || _joint_vel_state(i) != SAFE) {
            _num_con++;
        }
    }

    if (_num_con == 0 || !_enable_limit_flag) {
        _num_con = 0;
        // _blending_coefficients = alpha;
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
                    std::cout << "Joint name: " << _robot->jointName(i) << "\n";
                    std::cout << "Current joint angle: " << q(i) << "\n";
                    // std::cout << "rho " << _rho(i) << "\n";
                    // std::cout << "rho_0" << _rho_0(i) << "\n";
                    // if (con_des == MIN_SOFT_POS || con_des == MIN_HARD_POS) {
                    //     std::cout << "q: " << q(i) << ", q_min: " << _q_min(i) << "\n";
                    // } else if (con_des == MAX_SOFT_POS || con_des == MAX_HARD_POS) {
                    //     std::cout << "q: " << q(i) << ", q_max: " << _q_max(i) << "\n";
                    // }
                }
                con_des = _joint_vel_state[i];
                if (con_des != SAFE) {
                    // std::cout << "Joint " << i << " Vel State: " << _constraint_description[con_des] << "\n";
                }
            }

            // alphas
            // std::cout << "alpha: \n" << _blending_coefficients.transpose() << "\n";
            // std::cout << "vel alpha: \n" << _vel_blending_coefficients.transpose() << "\n";
            // for (auto val : alpha) {
                // std::cout << val << ", ";
            // }
        }

        // Vector to store the indices of non-zero elements
        std::vector<int> non_zero_indices;
        _blending_matrix = MatrixXd::Zero(_num_con, _dof);

        // Iterate over the non-zero elements and store their indices
        int cnt = 0;
        for (int i = 0; i < _dof; ++i) {
            if (_joint_state[i] != SAFE || _joint_vel_state[i] != SAFE) {
                non_zero_indices.push_back(i);
                // if (_joint_state[i] == MAX_HARD_POS || _joint_state[i] == MIN_HARD_POS) {
                //     // _blending_matrix(cnt, i) = std::pow((1 - alpha[cnt]), 2);
                //     cnt++;
                // } else if (_joint_vel_state[i] == MAX_HARD_VEL || _joint_vel_state[i] == MIN_HARD_VEL) {
                //     // _blending_matrix(cnt, i) = 0;
                //     cnt++;
                // } else if (_joint_vel_state[i] == MAX_SOFT_VEL || _joint_vel_state[i] == MIN_SOFT_VEL) {
                //     // _blending_matrix(cnt, i) = 1 - alpha[cnt];
                //     cnt++;
                // } else if (_joint_state[i] == MAX_SOFT_POS || _joint_state[i] == MIN_SOFT_POS) {
                //     // _blending_matrix(cnt, i) = 1;
                //     cnt++;
                // }
            }
        }
        _Jc = MatrixXd::Zero(_num_con, _dof);
        for (int i = 0; i < _num_con; ++i) {
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
        // _blending_coefficients = alpha;
        // _vel_blending_coefficients = vel_alpha;
        _blending_matrix = (_current_task_range.transpose() * _projected_jacobian).transpose() * _current_task_range.transpose() * \
                                _blending_matrix * (MatrixXd::Identity(_dof, _dof) - _Nc.transpose());  // blending the torque into the constraint space from the total torques
        // _blending_matrix = (_projected_jacobian).transpose() * \
                                // _blending_matrix * (MatrixXd::Identity(_dof, _dof) - _Nc.transpose());  // blending the torque into the constraint space from the total torques
    }
}

VectorXd JointHandler::computeTorques(const VectorXd& torques,
                                      const bool constraint_only,
                                      const bool baseline,
                                      const bool no_exit,
                                      const bool joint_task) {
    
    // update robot  
    VectorXd q = _robot->q();
    VectorXd dq = _robot->dq();

    // torque saturation
    VectorXd saturated_torques = torques;
    for (int i = 0; i < _dof; ++i) {
        if (std::abs(saturated_torques(i)) > _tau_abs_max(i)) {
            saturated_torques(i) = getSign(saturated_torques(i)) * _tau_abs_max(i);
        }
    }

    if (_num_con == 0 || !_enable_limit_flag) {
        return saturated_torques;
    } else {        

        // compute torques in constraint and dot-product/magnitude check 
        VectorXd projected_torques_in_constraint = (MatrixXd::Identity(_dof, _dof) - _Nc.transpose()) * saturated_torques;
        VectorXd projected_torques_not_in_constraint = _Nc.transpose() * saturated_torques; 

        // containers for apf torques, unit-mass damping torques
        VectorXd con_apf_torques = VectorXd::Zero(_num_con);
        VectorXd con_task_torques = VectorXd::Zero(_num_con);
        VectorXd con_unit_damping_torques = VectorXd::Zero(_num_con);

        // main joint handling 
        int cnt = 0;

        for (int i = 0; i < _dof; ++i) {

            // if (_joint_state[i] != SAFE && _truncation_flag) {
            //     projected_torques_not_in_constraint(i) = 0;
            // }

            bool constrained_joint = false;

            /*
                Position handling 
            */
            if (_joint_state(i) == MIN_SOFT_POS) {

                /*
                    LEAVE ALONE AND JUST RETURN POSITION WITHIN ZONE
                */

                // con_task_torques(cnt) = projected_torques_in_constraint(i);
                // con_unit_damping_torques(cnt) = 0;
                // constrained_joint = true;

                /*
                    APPLY DAMPING 
                */
                std::cout << "MIN SOFT HANDLING\n";

                // apply damping 
                con_unit_damping_torques(cnt) = - _kv_pos_limit(i) * dq(i);
                constrained_joint = true;

                // check exit
                if (projected_torques_in_constraint(i) > _tau_thresh && !baseline) {
                    std::cout << "MIN SOFT SAFE EXIT\n";
                    con_task_torques(cnt) = projected_torques_in_constraint(i);
                    con_unit_damping_torques(cnt) = 0;
                    constrained_joint = false;
                }

                // // apply velocity saturation based on scaling 
                // double max_vel = getMaxVelFunction(_blending_coefficients(i), std::abs(_entry_velocity(i)), std::abs(_exit_velocity(i)));
                // _max_vel(i) = max_vel;

                // // back out the desired velocity from the force in the constraint direction
                // double forward_vel = dq(i) + (projected_torques_in_constraint(i) / mass_in_constraint) * (1 / _kv_pos_limit(i)); 

                // bool safe_exit = projected_torques_in_constraint(i) > _tau_thresh && std::abs(dq(i)) < _dq_exit_tol;

                // // if (std::abs(dq(i)) > max_vel) {
                // if ((std::abs(forward_vel) > std::abs(max_vel) && getSign(forward_vel) < 0) 
                //     || (!safe_exit && projected_torques_in_constraint(i) > 0)) {
                //     // apply velocity saturation
                //     con_unit_damping_torques(cnt) = - _kv_pos_limit(i) * (dq(i) - (-max_vel));  // saturate to max velocity 
                // } 
                // // else {
                //     // con_unit_damping_torques(cnt) = 0;  // normal control, since velocity is not exceeded 
                //     // con_task_torques(cnt) = projected_torques_in_constraint(i);
                // // }

                // // dot product check to exit 
                // // if (projected_torques_in_constraint(i) > _tau_thresh) {
                // if (safe_exit) {
                //     std::cout << "MIN SOFT SAFE EXIT\n";
                //     con_task_torques(cnt) = projected_torques_in_constraint(i);
                //     // _joint_state(i) = SAFE; 
                //     con_unit_damping_torques(cnt) = 0;
                //     // _var_pos_zone_1_threshold(i) = _pos_zone_1_threshold(i);
                //     // con_unit_damping_torques(cnt) = - std::pow(_blending_coefficients[cnt], 2) * _kv_pos_limit(i) * dq(i);
                // }
                // constrained_joint = true;

            } else if (_joint_state(i) == MIN_HARD_POS) {

                /*  
                    APPLY APF AND NULLSPACE TRUNCATION
                    RETURN SAFE EXIT BACK TO MIN SOFT POS IF TASK WANTS TO
                */
                constrained_joint = true;

                // apply damping 
                // con_apf_torques(cnt) = std::pow(_blending_coefficients[i], 2) * _tau_abs_max(i);
                con_apf_torques(cnt) = _eta(i) * std::abs(((1 / _rho(i)) - (1 / _rho_0(i))) * (1 / std::pow(_rho(i), 2)));

                // con_unit_damping_torques(cnt) = - std::pow(_blending_coefficients[cnt], 2) * _kv_pos_limit(i) * dq(i);
                con_unit_damping_torques(cnt) = - _kv_pos_limit(i) * dq(i);

                // compare the projected torques with the apf torques 
                // if the projected torques are greater than the apf torques, then use the projected torques
                // pass through with safety 

                // dot product check
                // if (projected_torques_in_constraint(i) > _tau_thresh && std::abs(dq(i)) < 0.01) {
                // if (projected_torques_in_constraint(i) > con_apf_torques(cnt)) {
                // if (projected_torques_in_constraint(i) > _tau_thresh && std::abs(dq(i)) < _dq_exit_tol && !baseline) {
                if (projected_torques_in_constraint(i) > _tau_thresh && !baseline) {
                    std::cout << "MIN HARD SAFE EXIT; RETURNING TO MIN SOFT POS USING SAME VELOCITY CURVE AS BEFORE\n";

                    // _joint_state(i) = MIN_SOFT_POS;
                    con_task_torques(cnt) = projected_torques_in_constraint(i);;
                    con_apf_torques(cnt) = 0;
                    // con_unit_damping_torques(cnt) = 0;

                    // con_task_torques(cnt) = projected_torques_in_constraint(i);
                    // _joint_state(i) = SAFE;
                    // con_unit_damping_torques(cnt) = 0;  // DISABLE FOR BASELINE
                    // _var_pos_zone_1_threshold(i) = _pos_zone_1_threshold(i);
                    // con_unit_damping_torques(cnt) = - std::pow(_blending_coefficients[cnt], 2) * _kv_pos_limit(i) * dq(i);
                } 

            } else if (_joint_state(i) == MAX_SOFT_POS) {

                /*
                    LEAVE ALONE AND JUST RETURN POSITION WITHIN ZONE
                */
                // con_task_torques(cnt) = projected_torques_in_constraint(i);
                // con_unit_damping_torques(cnt) = 0;
                // constrained_joint = true;

                std::cout << "MAX SOFT HANDLING\n";

                /*
                    APPLY DAMPING
                */
                // apply damping 
                con_unit_damping_torques(cnt) = - _kv_pos_limit(i) * dq(i);
                constrained_joint = true;

                // check exit
                if (projected_torques_in_constraint(i) < - _tau_thresh && !baseline) {
                    std::cout << "MAX SOFT SAFE EXIT\n";
                    con_task_torques(cnt) = projected_torques_in_constraint(i);
                    con_unit_damping_torques(cnt) = 0;
                    constrained_joint = false;
                }

                // // apply apf 
                // // con_apf_torques(cnt) = - std::pow(_blending_coefficients[cnt], 2) * _tau_abs_max(i);
                // // con_unit_damping_torques(cnt) = - std::pow(_blending_coefficients[cnt], 2) * _kv_pos_limit(i) * dq(i);
                // // con_unit_damping_torques(cnt) = - _kv_pos_limit(i) * dq(i);

                // // get mass 
                // double mass_in_constraint = _robot->M()(i, i);

                // // apply velocity saturation based on scaling 
                // double max_vel = getMaxVelFunction(_blending_coefficients(i), std::abs(_entry_velocity(i)), std::abs(_exit_velocity(i)));
                // _max_vel(i) = max_vel;

                // // back out the desired velocity from the force in the constraint direction
                // double forward_vel = dq(i) + (projected_torques_in_constraint(i) / mass_in_constraint) * (1 / _kv_pos_limit(i)); 

                // bool safe_exit = projected_torques_in_constraint(i) < - _tau_thresh && std::abs(dq(i)) < _dq_exit_tol;

                // // if (std::abs(dq(i)) > max_vel) {
                // if ((std::abs(forward_vel) > std::abs(max_vel) && getSign(forward_vel) > 0) 
                //     || (!safe_exit && projected_torques_in_constraint(i) > 0)) {
                //     // apply velocity saturation
                //     con_unit_damping_torques(cnt) = - _kv_pos_limit(i) * (dq(i) - max_vel);  // saturate to max velocity 
                // } 
                // // else {
                //     // con_unit_damping_torques(cnt) = 0;  // normal control, since velocity is not exceeded 
                //     // con_task_torques(cnt) = projected_torques_in_constraint(i);
                // // }

                // // dot product check
                // // if (projected_torques_in_constraint(i) < - _tau_thresh) {
                // if (safe_exit) {
                //     con_task_torques(cnt) = projected_torques_in_constraint(i);
                //     _joint_state(i) = SAFE;
                //     con_unit_damping_torques(cnt) = 0;
                //     // _var_pos_zone_1_threshold(i) = _pos_zone_1_threshold(i);
                //     // con_unit_damping_torques(cnt) = - std::pow(_blending_coefficients[cnt], 2) * _kv_pos_limit(i) * dq(i);
                // } 
                // constrained_joint = true;

            } else if (_joint_state(i) == MAX_HARD_POS) {
                /*  
                    APPLY APF AND NULLSPACE TRUNCATION
                    RETURN SAFE EXIT BACK TO MAX SOFT POS IF TASK WANTS TO
                */

                // con_apf_torques(cnt) = - std::pow(_blending_coefficients[i], 2) * _tau_abs_max(i);
                con_apf_torques(cnt) = - _eta(i) * std::abs(((1 / _rho(i)) - (1 / _rho_0(i))) * (1 / std::pow(_rho(i), 2)));

                // std::cout << "apf torques: " << con_apf_torques(cnt) << "\n";

                // con_unit_damping_torques(cnt) = - std::pow(_blending_coefficients[cnt], 2) * _kv_pos_limit(i) * dq(i);
                con_unit_damping_torques(cnt) = - _kv_pos_limit(i) * dq(i);

                // dot product check
                // if (projected_torques_in_constraint(i) < - _tau_thresh && std::abs(dq(i)) < 0.01) {
                // if (projected_torques_in_constraint(i) < -_tau_thresh && std::abs(dq(i)) < _dq_exit_tol && !baseline) {
                if (projected_torques_in_constraint(i) < -_tau_thresh && !baseline) {

                    std::cout << "MIN HARD SAFE EXIT; RETURNING TO MIN SOFT POS USING SAME VELOCITY CURVE AS BEFORE\n";
                    // _joint_state(i) = MAX_SOFT_POS;
                    con_task_torques(cnt) = projected_torques_in_constraint(i);;
                    con_apf_torques(cnt) = 0;

                    // std::cout << "Max Hard Passthrough\n";
                    // std::cout << "Projected torque: " << projected_torques_in_constraint(i) << "\n";
                    // std::cout << "APF torque: " << con_apf_torques(cnt) << "\n";
                    // con_task_torques(cnt) = projected_torques_in_constraint(i);
                    // _joint_state(i) = SAFE;
                    // con_unit_damping_torques(cnt) = 0;  // DISABLE FOR BASELINE 
                    // _var_pos_zone_1_threshold(i) = _pos_zone_1_threshold(i);
                    // con_unit_damping_torques(cnt) = - std::pow(_blending_coefficients[cnt], 2) * _kv_pos_limit(i) * dq(i);
                } 

                constrained_joint = true;
            } 

            /*
                Velocity handling (only if position is safe or leaving constraints) 
            */
            if (_joint_state(i) == SAFE && _enable_vel_limits) {
                if (_joint_vel_state(i) == MIN_SOFT_VEL) {

                    // double dq_des = - _dq_abs_max(i) + _vel_zone_1_threshold(i);
                    // con_unit_damping_torques(cnt) = - _kv_pos_limit(i) * (dq(i) - dq_des);  // velocity follower
                    con_task_torques(cnt) = projected_torques_in_constraint(i) * std::pow(1 - _vel_blending_coefficients[i], 1);
                    if (projected_torques_in_constraint(i) > _tau_vel_thresh) {
                        con_task_torques(cnt) = projected_torques_in_constraint(i);
                        // con_unit_damping_torques(cnt) = 0;
                    }
                    constrained_joint = true;

                } else if (_joint_vel_state(i) == MIN_HARD_VEL) {

                    // double dq_des = - _dq_abs_max(i) + _vel_zone_1_threshold(i);
                    // con_unit_damping_torques(cnt) = - _kv_pos_limit(i) * (dq(i) - dq_des);  // velocity follower
                    con_apf_torques(cnt) = std::pow(_vel_blending_coefficients[i], 2) * _vel_gamma(i) * _tau_abs_max(i);
                    if (projected_torques_in_constraint(i) > _tau_vel_thresh) {
                        con_task_torques(cnt) = projected_torques_in_constraint(i);
                        // con_unit_damping_torques(cnt) = 0;
                    }
                    constrained_joint = true;

                } else if (_joint_vel_state(i) == MAX_SOFT_VEL) {

                    // double dq_des = _dq_abs_max(i) - _vel_zone_1_threshold(i);
                    // con_unit_damping_torques(cnt) = - _kv_pos_limit(i) * (dq(i) - dq_des);  // velocity follower
                    con_task_torques(cnt) = projected_torques_in_constraint(i) * std::pow(1 - _vel_blending_coefficients[i], 1);
                    if (projected_torques_in_constraint(i) <  - _tau_vel_thresh) {
                        con_task_torques(cnt) = projected_torques_in_constraint(i);
                        // con_unit_damping_torques(cnt) = 0;
                    }
                    constrained_joint = true;

                } else if (_joint_vel_state(i) == MAX_HARD_VEL) {
                    // double dq_des = _dq_abs_max(i) - _vel_zone_1_threshold(i);
                    // con_unit_damping_torques(cnt) = - _kv_pos_limit(i) * (dq(i) - dq_des);  // velocity follower
                    con_apf_torques(cnt) = - std::pow(_vel_blending_coefficients[i], 2) * _vel_gamma(i) * _tau_abs_max(i);
                    if (projected_torques_in_constraint(i) < - _tau_vel_thresh) {
                        con_task_torques(cnt) = projected_torques_in_constraint(i);
                        // con_unit_damping_torques(cnt) = 0;
                    }
                    constrained_joint = true;

                } 
            } 

            if (constrained_joint) {
                cnt++;
            }
        }

        // std::cout << "APF TORQUES: " << con_apf_torques.transpose() << "\n";

        _apf_torques = 1 * (_current_task_range.transpose() * _projected_jacobian).transpose() * \
                                    _Lambda_c * _current_task_range.transpose() * (1 * con_apf_torques);

        // std::cout << "APF TORQUES TO MOTOR: " << _apf_torques.transpose() << "\n";

        VectorXd total_torques = VectorXd::Zero(_dof);
        if (baseline) {
            // std::cout << "baseline\n";
            total_torques += saturated_torques;
            total_torques += 1 * (_current_task_range.transpose() * _projected_jacobian).transpose() * \
                                    _Lambda_c * _current_task_range.transpose() * (1 * con_apf_torques);
            total_torques += 1 * (_current_task_range.transpose() * _projected_jacobian).transpose() * \
                                    _Lambda_c * _current_task_range.transpose() * con_unit_damping_torques;
            // std::cout << "apf: " << con_apf_torques.transpose() << "\n";

            // if (total_torques(2) > 87) {
            //     std::cout << "saturated\n";
            //     total_torques(2) = 80;
            // } else if (total_torques(2) < -87) {
            //     total_torques(2) = -80;
            //     std::cout << "saturated\n";
            // }
        } else {

            /*
                Compute torques
                If soft region, then pass through
                If hard region, then nullspace truncation and APF torques 
            */
        
            // compute constrained torques (unit mass damping + task torques + apf torques)
            // std::cout << "apf: " << con_apf_torques.transpose() << "\n";
            // VectorXd total_torques = VectorXd::Zero(_dof);
            total_torques += 1 * (_current_task_range.transpose() * _projected_jacobian).transpose() * \
                                    _Lambda_c * _current_task_range.transpose() * con_unit_damping_torques;
            // total_torques += 1 * (_current_task_range.transpose() * _projected_jacobian).transpose() * \
                                    // _current_task_range.transpose() * (1 * con_task_torques + 1 * con_apf_torques);
            total_torques += 1 * (_current_task_range.transpose() * _projected_jacobian).transpose() * \
                                    _Lambda_c * _current_task_range.transpose() * (1 * con_apf_torques);

            // if (!no_exit) {
                total_torques += 1 * (_current_task_range.transpose() * _projected_jacobian).transpose() * \
                                        _current_task_range.transpose() * (1 * con_task_torques);
            // std::cout << "con task torques: \n" << con_task_torques.transpose() << "\n";
            // }
            // total_torques += 1 * (_projected_jacobian).transpose() * _Lambda_c * con_unit_damping_torques;
            // total_torques += _projected_jacobian.transpose() * (con_task_torques + con_apf_torques);

            if (!constraint_only) {
                total_torques += 1 * projected_torques_not_in_constraint;
            } 
        }

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

        // return torques;
        return total_torques;
    }

}

VectorXi JointHandler::getExitState(const VectorXd& torques) {
    VectorXi exit_state = VectorXi::Zero(_dof);

    for (int i = 0; i < _dof; ++i) {
        if (_joint_state(i) == MAX_HARD_POS) {
            if (torques(i) < _tau_thresh) {
                exit_state(i) = 1;
            }
        } else if (_joint_state(i) == MIN_HARD_POS) {
            if (torques(i) > _tau_thresh) {
                exit_state(i) = 1;
            }
        }
    }
    return exit_state;
}


}  // namespace 