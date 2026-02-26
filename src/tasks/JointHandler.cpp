/*
 * JointHandler.cpp
 *
 *      Author: William Chong 
 */

#include "JointHandler.h"

namespace {

    const std::vector<std::string> constraint_description = {
                                    "Safe", 
                                    "Minimum Velocity",
                                    "Maximum Velocity",
                                    "Minimum Soft Position",
                                    "Minimum Hard Position",
                                    "Maximum Soft Position",
                                    "Maximum Hard Position"};

    int getSign(double value) {
        if (value > 0) {
            return 1;
        } else if (value < 0) {
            return -1;
        } else {
            return 0;
        }
    }

    double getApf(const double eta, const double rho, const double rho_0) {
        return eta * std::abs(((1 / rho) - (1 / rho_0)) * (1 / std::pow(rho, 2)));
    }

}

namespace SaiPrimitives {

JointHandler::JointHandler(
    std::shared_ptr<SaiModel::SaiModel> robot,
    const std::vector<int>& joint_selection_to_skip,
    const bool verbose) : 
    _robot(robot), 
    _joint_selection_to_skip(joint_selection_to_skip),
    _verbose(verbose) {

    // initialize variables
    _dof = _robot->dof();
    _q_min = VectorXd::Zero(_dof);
    _q_max = VectorXd::Zero(_dof);
    _dq_abs_max = VectorXd::Zero(_dof);
    _tau_abs_max = VectorXd::Zero(_dof);
    _joint_state = {};
    for (int i = 0; i < _dof; ++i) {
        _joint_state.push_back(SAFE);
    }
    _kv_pos_limit = DefaultParameters::kv_damping * VectorXd::Ones(_dof);

    _t_delta_pos = DefaultParameters::t_delta_pos;
    _t_delta_vel = DefaultParameters::t_delta_vel;

    // thresholds 
    _pos_zone_1_threshold = DefaultParameters::pos_zone_1 * VectorXd::Ones(_dof); 
    _pos_zone_2_threshold = DefaultParameters::pos_zone_2 * VectorXd::Ones(_dof);

    // get joint limits from robot 
    auto joint_limits = _robot->jointLimits();
    for (int i = 0; i < joint_limits.size(); ++i) {
        _q_min(i) = joint_limits[i].position_lower;
        _q_max(i) = joint_limits[i].position_upper;
        _dq_abs_max(i) = joint_limits[i].velocity;
        _tau_abs_max(i) = joint_limits[i].effort;
    }

    if (_verbose) {
        std::cout << "Joint handler parameters:\n";
        std::cout << "q min: \n" << _q_min.transpose() << "\n";
        std::cout << "q max: \n" << _q_max.transpose() << "\n";
        std::cout << "dq max: \n" << _dq_abs_max.transpose() << "\n";
        std::cout << "tau max: \n" << _tau_abs_max.transpose() << "\n";
    }

    _enable_limit_flag = true;
    _enable_vel_limits = false;
    _num_con = 0;

    // setup apf function
    _rho = VectorXd::Zero(robot->dof());
    _rho_0 = _pos_zone_2_threshold;
    _eta = DefaultParameters::eta * VectorXd::Ones(robot->dof());
    _apf_torques = VectorXd::Zero(robot->dof());

    // setup moving outer zone boundary layer 
    _entry_velocity = VectorXd::Zero(robot->dof());
    _exit_velocity = VectorXd::Zero(robot->dof());

    // flags
    _variable_vel_zone = false;
    _use_apf_thresh_flag = true;
}

void JointHandler::updateTaskModel(const MatrixXd& N_prec) {
    
    _N_prec = N_prec;

    // get kinematics 
    VectorXd q = _robot->q();
    VectorXd dq = _robot->dq();

    // reset
    _rho.setZero();
    _rho_0 = _pos_zone_2_threshold;

    for (int i = 0; i < _dof; ++i) {

        if (_joint_selection_to_skip.size() != 0) {
            if (std::find(_joint_selection_to_skip.begin(), _joint_selection_to_skip.end(), i) != _joint_selection_to_skip.end()) {
                continue;
            }
        }

        // check collision with the inner zone
        double q_future = q(i) + dq(i) * _t_delta_pos;

        // check for zone 2 threshold if robot is SAFE 
        if (_joint_state[i] == SAFE && _variable_vel_zone) {
            if (q_future > _q_max(i) - _pos_zone_2_threshold(i) && q(i) < _q_max(i) - _pos_zone_2_threshold(i)) {
                // set outer zone boundary at maximum
                // if (_verbose) {
                    std::cout << "Joint handler: setting outer velocity upper boundary\n";
                // }
                _joint_state[i] = MAX_SOFT_POS;
                setPosZone1ThresholdIndex(_q_max(i) - q(i), i);
                _entry_velocity(i) = dq(i);

            } else if (q_future < _q_min(i) + _pos_zone_2_threshold(i) && q(i) > _q_min(i) + _pos_zone_2_threshold(i)) {
                // set outer zone boundary at the minimum 
                // if (_verbose) {
                    std::cout << "Joint handler: setting outer velocity lower boundary\n";
                // }
                _joint_state[i] = MIN_SOFT_POS;
                setPosZone1ThresholdIndex(q(i) - _q_min(i), i);
                _entry_velocity(i) = dq(i);

            } else {                
                // continually set the outer zone boundary at the inner zone boundary to disable velocity region by default
                setPosZone1ThresholdIndex(_pos_zone_2_threshold(i), i); 
                _entry_velocity(i) = dq(i);
            }
        } else {            
            // normal check without variable velocity zone
            if (q(i) > _q_max(i) - _pos_zone_2_threshold(i)) {
                // upper limit, apf region
                _joint_state[i] = MAX_HARD_POS;
                double q_zone_lower = _q_max(i) - _pos_zone_2_threshold(i);
                double q_zone_upper = _q_max(i);
                _rho(i) = std::clamp(_q_max(i) - _robot->q()(i), _rho_min_tol, _pos_zone_2_threshold(i));

            } else if (q(i) > _q_max(i) - _pos_zone_1_threshold(i)) {
                // upper limit, velocity region
                _joint_state[i] = MAX_SOFT_POS;

            } else if (q(i) < _q_min(i) + _pos_zone_2_threshold(i)) {
                // lower limit, apf region
                _joint_state[i] = MIN_HARD_POS;
                double q_zone_lower = _q_min(i) + _pos_zone_2_threshold(i);
                double q_zone_upper = _q_min(i);
                _rho(i) = std::clamp(_robot->q()(i) - _q_min(i), _rho_min_tol, _pos_zone_2_threshold(i));

            } else if (q(i) < _q_min(i) + _pos_zone_1_threshold(i)) {
                // lower limit, velocity region
                _joint_state[i] = MIN_SOFT_POS;

            } else {
                _joint_state[i] = SAFE;
            }
        }
    }
    
    // form op-space model quantities
    _num_con = 0;
    for (int i = 0; i < _dof; ++i) {
        if (_joint_state[i] != SAFE) {
            _num_con++;
        }
    }

    if (_num_con == 0 || !_enable_limit_flag) {
        _num_con = 0;
        _Jc = MatrixXd::Zero(1, _dof);
        _Nc = N_prec;  // pass through limit handling
        return;
    } else {

        // if (_verbose) {
            std::cout << "------\n";
            for (int i = 0; i < _dof; ++i) {
                if (_joint_state[i] != SAFE) {
                    std::cout << "Joint " << i << " State: " << constraint_description[_joint_state[i]] << "\n";
                    std::cout << "Joint name: " << _robot->jointName(i) << "\n";
                    std::cout << "Current joint angle: " << q(i) << "\n";
                }
            }
        // }

        // store non-zero indices
        std::vector<int> non_zero_indices;

        int cnt = 0;
        for (int i = 0; i < _dof; ++i) {
            if (_joint_state[i] != SAFE) {
                non_zero_indices.push_back(i);
            }
        }
        _Jc = MatrixXd::Zero(_num_con, _dof);
        for (int i = 0; i < _num_con; ++i) {
            _Jc(i, non_zero_indices[i]) = 1;
        }
        _projected_jacobian = _Jc * _N_prec;

        // decomposition 
        _current_task_range = SaiModel::matrixRangeBasis(_projected_jacobian);
        SaiModel::OpSpaceMatrices con_matrices = _robot->operationalSpaceMatrices(_current_task_range.transpose() * _projected_jacobian);
        _Lambda_c = con_matrices.Lambda;
        _Jbar_c = con_matrices.Jbar;
        _Nc = con_matrices.N;
    }
}

VectorXd JointHandler::computeTorques(const VectorXd& torques) {
    
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

        // separate torques in and not in constraint space
        VectorXd projected_torques_in_constraint = (MatrixXd::Identity(_dof, _dof) - _Nc.transpose()) * saturated_torques;
        VectorXd projected_torques_not_in_constraint = _Nc.transpose() * saturated_torques;

        // containers for apf torques, unit-mass damping torques
        VectorXd con_apf_torques = VectorXd::Zero(_num_con);
        VectorXd con_task_torques = VectorXd::Zero(_num_con);
        VectorXd con_unit_damping_torques = VectorXd::Zero(_num_con);

        // position limits
        int cnt = 0;

        for (int i = 0; i < _dof; ++i) {

            bool constrained_joint = false;
            
            // position limits
            if (_joint_state[i] == MIN_SOFT_POS) {
                // apply damping 
                con_unit_damping_torques(cnt) = - _kv_pos_limit(i) * dq(i);
                constrained_joint = true;

                if (!_use_apf_thresh_flag) {
                    if (con_unit_damping_torques(cnt) > _unit_tau_thresh) {
                        con_task_torques(cnt) = projected_torques_in_constraint(i);
                        con_unit_damping_torques(cnt) = 0;
                        constrained_joint = false;
                    }
                }

            } else if (_joint_state[i] == MIN_HARD_POS) {
                // apply apf and damping
                // con_apf_torques(cnt) = _eta(i) * std::abs(((1 / _rho(i)) - (1 / _rho_0(i))) * (1 / std::pow(_rho(i), 2)));
                con_apf_torques(cnt) = getApf(_eta(i), _rho(i), _rho_0(i));
                con_unit_damping_torques(cnt) = - _kv_pos_limit(i) * dq(i);
                constrained_joint = true;

                if (!_use_apf_thresh_flag) {
                    if (con_apf_torques(cnt) > _unit_tau_thresh) {
                        con_task_torques(cnt) = projected_torques_in_constraint(i);
                        con_apf_torques(cnt) = 0;
                        con_unit_damping_torques(cnt) = 0;
                        constrained_joint = false;
                    }
                }

            } else if (_joint_state[i] == MAX_SOFT_POS) {
                // apply damping 
                con_unit_damping_torques(cnt) = - _kv_pos_limit(i) * dq(i);
                constrained_joint = true;

                if (!_use_apf_thresh_flag) {
                    if (con_unit_damping_torques(cnt) < _unit_tau_thresh) {
                        con_task_torques(cnt) = projected_torques_in_constraint(i);
                        con_unit_damping_torques(cnt) = 0;
                        constrained_joint = false;
                    }
                }

            } else if (_joint_state[i] == MAX_HARD_POS) {
                // apply apf and damping
                // con_apf_torques(cnt) = - _eta(i) * std::abs(((1 / _rho(i)) - (1 / _rho_0(i))) * (1 / std::pow(_rho(i), 2)));
                con_apf_torques(cnt) = - getApf(_eta(i), _rho(i), _rho_0(i));
                con_unit_damping_torques(cnt) = - _kv_pos_limit(i) * dq(i);
                constrained_joint = true;

                if (!_use_apf_thresh_flag) {
                    if (con_apf_torques(cnt) < _unit_tau_thresh) {
                        con_task_torques(cnt) = projected_torques_in_constraint(i);
                        con_apf_torques(cnt) = 0;
                        con_unit_damping_torques(cnt) = 0;
                        constrained_joint = false;
                    }
                }
            } 

            if (constrained_joint) {
                cnt++;
            }
        }

        _apf_torques = (_current_task_range.transpose() * _projected_jacobian).transpose() * 
                                    _Lambda_c * _current_task_range.transpose() * con_apf_torques;

        _damping_torques = (_current_task_range.transpose() * _projected_jacobian).transpose() * 
                                    _Lambda_c * _current_task_range.transpose() * con_unit_damping_torques;

        // exit conditions based on the apf threshold
        if (_use_apf_thresh_flag) {
            cnt = 0;  // reset count
            for (int i = 0; i < _dof; ++i) {
                if (_joint_state[i] != SAFE) {
                    if (_joint_state[i] == MIN_SOFT_POS) {
                        if (projected_torques_in_constraint(i) > _damping_torques(cnt) && 
                            projected_torques_in_constraint(i) > 0) {
                            con_task_torques(cnt) = projected_torques_in_constraint(i);
                            con_unit_damping_torques(cnt) = 0;
                        }
                    } else if (_joint_state[i] == MAX_SOFT_POS) {
                        if (projected_torques_in_constraint(i) < _damping_torques(cnt) && 
                            projected_torques_in_constraint(i) < 0) {
                            con_task_torques(cnt) = projected_torques_in_constraint(i);
                            con_unit_damping_torques(cnt) = 0;
                        }
                    } else if (_joint_state[i] == MIN_HARD_POS) {
                        if (projected_torques_in_constraint(i) > _apf_torques(cnt)) {
                            con_task_torques(cnt) = projected_torques_in_constraint(i);
                            con_apf_torques(cnt) = 0;
                        }
                    } else if (_joint_state[i] == MAX_HARD_POS) {
                        if (projected_torques_in_constraint(i) < _apf_torques(cnt)) {
                            con_task_torques(cnt) = projected_torques_in_constraint(i);;
                            con_apf_torques(cnt) = 0;
                        }
                    }
                    cnt++;
                }
            }
        }

        VectorXd pos_handling_torques = VectorXd::Zero(_dof);
        pos_handling_torques += (_current_task_range.transpose() * _projected_jacobian).transpose() * 
                                _Lambda_c * _current_task_range.transpose() * con_unit_damping_torques;
        pos_handling_torques += (_current_task_range.transpose() * _projected_jacobian).transpose() * 
                                _Lambda_c * _current_task_range.transpose() * con_apf_torques;
        pos_handling_torques += (_current_task_range.transpose() * _projected_jacobian).transpose() * 
                                        _current_task_range.transpose() * con_task_torques;

        if (_enable_vel_limits) {

            // joint velocity saturation through torque saturation, within the joint position limit hierarchy
            std::vector<int> non_zero_indices;
            int vel_cnt = 0;  
            VectorXd ddq = _robot->MInv() * torques;
            VectorXd ddq_des = VectorXd::Zero(_dof);
            for (int i = 0; i < _dof; ++i) {
                if (_joint_state[i] == SAFE) {
                    if (dq(i) + ddq(i) * _t_delta_vel > _dq_abs_max(i)) {
                        // saturate acceleration
                        ddq_des(i) = std::max(0.0, (1. / _t_delta_vel) * (_dq_abs_max(i) - dq(i)));
                        vel_cnt++;
                        non_zero_indices.push_back(i);
                    } else if (dq(i) + ddq(i) * _t_delta_vel < - _dq_abs_max(i)) {
                        // saturate acceleration
                        ddq_des(i) = std::min(0.0, (1. / _t_delta_vel) * (dq(i) - _dq_abs_max(i)));
                        vel_cnt++;
                        non_zero_indices.push_back(i);
                    }
                }
            }

            // compute op-space terms
            MatrixXd Jc_vel = MatrixXd::Zero(vel_cnt, _dof);
            for (int i = 0; i < vel_cnt; ++i) {
                Jc_vel(i, non_zero_indices[i]) = 1;
            }
            MatrixXd projected_jacobian = Jc_vel * _Nc * _N_prec;

            // decomposition 
            MatrixXd current_task_range = SaiModel::matrixRangeBasis(projected_jacobian);
            SaiModel::OpSpaceMatrices con_matrices = _robot->operationalSpaceMatrices(current_task_range.transpose() * projected_jacobian);
            MatrixXd Lambda_c = con_matrices.Lambda;
            MatrixXd Jbar_c = con_matrices.Jbar;
            MatrixXd Nc = con_matrices.N;
            _Nc = Nc * _Nc;

            // compute revised torques
            VectorXd total_torques = pos_handling_torques + Nc.transpose() * projected_torques_not_in_constraint + 
                (current_task_range.transpose() * projected_jacobian).transpose() * Lambda_c * current_task_range.transpose() * ddq_des;

            // torque saturation    
            for (int i = 0; i < _dof; ++i) {
                if (std::abs(total_torques(i)) > _tau_abs_max(i)) {
                    total_torques(i) = getSign(total_torques(i)) * _tau_abs_max(i);
                }
            }
            return total_torques;
            
        } else {
            VectorXd total_torques = pos_handling_torques + projected_torques_not_in_constraint;
            // torque saturation    
            for (int i = 0; i < _dof; ++i) {
                if (std::abs(total_torques(i)) > _tau_abs_max(i)) {
                    total_torques(i) = getSign(total_torques(i)) * _tau_abs_max(i);
                }
            }
            return total_torques;
        }
    }

}

}  // namespace 