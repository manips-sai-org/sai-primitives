/**
 * @file SingularityHandler.cpp
 * @author William Chong (wmchong@stnaford.edu)
 * @brief 
 * @version 0.1
 * @date 2024-01-13
 * 
 * @copyright Copyright (c) 2024
 * 
 */

#include "SingularityHandler.h"

namespace Sai2Primitives {

SingularityHandler::SingularityHandler(std::shared_ptr<Sai2Model::Sai2Model> robot,
                                       const int& task_rank,
                                       const MatrixXd& J_posture,
                                       const double& s_abs_tol,
                                       const double& type_1_tol,
                                       const double& type_2_torque_ratio,
                                       const int& buffer_size) : _robot(robot), _task_rank(task_rank),
                                       _J_posture(J_posture), _s_abs_tol(s_abs_tol), _type_1_tol(type_1_tol), 
                                       _type_2_torque_ratio(type_2_torque_ratio), _buffer_size(buffer_size)
{
    // initialize robot specific variables 
    _joint_midrange = VectorXd::Zero(_robot->dof());
    _type_2_torque_vector = VectorXd::Zero(_robot->dof());
    auto joint_limits = _robot->jointLimits();
    for (int i = 0; i < joint_limits.size(); ++i) {
        _type_2_torque_vector(i) = _type_2_torque_ratio * joint_limits[i].effort;
        _joint_midrange(i) = 0.5 * (joint_limits[i].position_lower + joint_limits[i].position_upper);
    }

    // initialize singularity variables 
    _sing_history = std::queue<SingularityType>();
    _sing_direction_buffer = std::queue<Eigen::Matrix<double, 6, 1>>();
    _q_prior = robot->q();
    _dq_prior = robot->dq();
    double kp = 10;
    double kv = 2 * std::sqrt(kp);
    setGains(kp, kv);
    setSingularity(NO_SINGULARITY);

    // initialize singularity history
    // the entire buffer history must be classified a certain singularity type, otherwise it's type 1 by default
    // type 2 singularity is more unstable since it's open-loop torque 
    _sing_history = std::queue<SingularityType>();
    for (int i = 0; i < _buffer_size; ++i) {
        _sing_history.push(NO_SINGULARITY);
    }
}

/*
    Performs the steps:
    - Compute SVD of the projected jacobian (order of decreasing singular values)
    - Compute the non-singular and singular task ranges 
    - Classify singularity 
    - Compute the op-space matrices using the non-singular and singular jacobians 
    - Perform torque blending 
*/
SingularityOpSpaceMatrices SingularityHandler::updateTaskModel(const MatrixXd& projected_jacobian, const MatrixXd& N_prec) {
    
    // task range decomposition
    int dof = _robot->dof();
    Sai2Model::SvdData J_svd = Sai2Model::matrixSvd(projected_jacobian);
    
    // debug
    _s_values = J_svd.s;
    // VectorXd condition_numbers = J_svd.s(0) / J_svd.s.array();
    // std::cout << "absolute singular values: " << J_svd.s.transpose() << "\n";
    // std::cout << "condition numbers: " << condition_numbers.transpose() << "\n";

    // task is completely singular
    if (J_svd.s(0) < _s_abs_tol) {
        // non-singular task
        _task_range_ns = MatrixXd::Zero(6, _task_rank);
        _projected_jacobian_ns = MatrixXd::Zero(_task_rank, dof);
        _Lambda_ns = MatrixXd::Zero(_task_rank, _task_rank);
        _Jbar_ns = MatrixXd::Zero(dof, _task_rank);
        _N_ns = MatrixXd::Zero(dof, dof);

        // singular task 
        _task_range_s = J_svd.U.leftCols(_task_rank);
        _projected_jacobian_s = _task_range_s.transpose() * projected_jacobian;
        Sai2Model::OpSpaceMatrices s_matrices =
            _robot->operationalSpaceMatrices(_projected_jacobian_s);
        _Lambda_s = s_matrices.Lambda;
        _Jbar_s = s_matrices.Jbar;
        _N_s = s_matrices.N;
    } else {
        // includes the partial task projection handling 
        for (int i = 1; i < _task_rank; ++i) {
            double condition_number = J_svd.s(i) / J_svd.s(0);
            _alpha = std::clamp((condition_number - _s_min) / (_s_max - _s_min), 0., 1.);

            if (condition_number < _s_max) {
                // non-singular task
                _task_range_ns = J_svd.U.leftCols(i);
                _projected_jacobian_ns = _task_range_ns.transpose() * projected_jacobian;
                Sai2Model::OpSpaceMatrices ns_matrices =
                    _robot->operationalSpaceMatrices(_projected_jacobian_ns);
                _Lambda_ns = ns_matrices.Lambda;
                _Jbar_ns = ns_matrices.Jbar;
                _N_ns = ns_matrices.N;

                // singular task 
                _task_range_s = J_svd.U.block(0, i, J_svd.U.rows(), _task_rank - i);
                _projected_jacobian_s = _task_range_s.transpose() * projected_jacobian;
                Sai2Model::OpSpaceMatrices s_matrices =
                    _robot->operationalSpaceMatrices(_projected_jacobian_s);
                _Lambda_s = s_matrices.Lambda;
                _Jbar_s = s_matrices.Jbar;
                _N_s = s_matrices.N;
                break;

            } else if (i == _task_rank - 1) {
                // fully non-singular task 
                // non-singular task
                _task_range_ns = J_svd.U.leftCols(_task_rank); 
                _projected_jacobian_ns = _task_range_ns.transpose() * projected_jacobian;
                Sai2Model::OpSpaceMatrices ns_matrices =
                    _robot->operationalSpaceMatrices(_projected_jacobian_ns);
                _Lambda_ns = ns_matrices.Lambda;
                _Jbar_ns = ns_matrices.Jbar;
                _N_ns = ns_matrices.N;

                // singular task 
                _task_range_s = MatrixXd::Zero(6, _task_rank);
                _projected_jacobian_s = _task_range_s.transpose() * projected_jacobian;
                _Lambda_s = MatrixXd::Zero(_task_rank, _task_rank);
                _Jbar_s = MatrixXd::Zero(dof, _task_rank);
                _N_s = MatrixXd::Zero(dof, dof);
            }
        }
    }

    // classify singularities 
    classifySingularity(_task_range_s);

    // task updates 
    if (_sing_type == NO_SINGULARITY) {
        _N = _N_ns;  // _N = N_ns (rank(N_ns) = task_rank)
    } else if (_sing_type != NO_SINGULARITY) {
        // update posture task 
        _posture_projected_jacobian = _J_posture * _N_ns * N_prec;
        _current_task_range = Sai2Model::matrixRangeBasis(_posture_projected_jacobian); 
        Sai2Model::OpSpaceMatrices op_space_matrices =
            _robot->operationalSpaceMatrices(_current_task_range.transpose() * _posture_projected_jacobian);
        _M_partial = op_space_matrices.Lambda;
        _N = op_space_matrices.N * _N_ns;  // _N = N_partial_joint * N_ns is rank of task_rank
    }

    return SingularityOpSpaceMatrices{_projected_jacobian_ns,
                                      _Lambda_ns,
                                      _N_ns,
                                      _task_range_ns,
                                      _projected_jacobian_s,
                                      _Lambda_s,
                                      _N_s,
                                      _task_range_s};
}

void SingularityHandler::classifySingularity(const MatrixXd& singular_task_range) {
    // memory of entering conditions 
    if (_sing_type == NO_SINGULARITY) {
        _q_prior = _robot->q();
        _dq_prior = _robot->dq();
    }

    if (singular_task_range.norm() == 0 || std::min(singular_task_range.rows(), singular_task_range.cols()) == _task_rank) {
        _sing_type = NO_SINGULARITY;
        // clear singularity direction queue 
        while (!_sing_direction_buffer.empty()) {
            _sing_direction_buffer.pop();
        }
        return;
    }

    // fill buffer with the most singular task range (already dis-included zero columns)
    _sing_direction_buffer.push(singular_task_range.rightCols(1));
    VectorXd last_direction = _sing_direction_buffer.back();
    VectorXd current_direction = _sing_direction_buffer.front();

    // check singular direction alignment for type 1 singularity
    // type 1 singularity is preferred due to stable behavior
    // type 2 is classified only if the entire past buffer_size timesteps is type 2 
    if (std::abs(current_direction.dot(last_direction)) > _type_1_tol) {
        _sing_history.push(TYPE_1_SINGULARITY);
        _sing_type = TYPE_1_SINGULARITY;
    } else {
        _sing_history.push(TYPE_2_SINGULARITY);
        if (allElementsSame(_sing_history)) {
            _sing_type = TYPE_2_SINGULARITY;
        } else {
            _sing_type = TYPE_1_SINGULARITY;
        }
    }

}

VectorXd SingularityHandler::computeTorques(const VectorXd& unit_mass_force, const VectorXd& force_related_terms)
{
    VectorXd joint_strategy_torques = VectorXd::Zero(_robot->dof());
    VectorXd unit_torques = VectorXd::Zero(_robot->dof());

    // debug
    if (_sing_type != NO_SINGULARITY) {
        std::cout << "Singularity: " << singularity_labels[_sing_type] << "\n---\n";
    }

    if (_sing_type == TYPE_1_SINGULARITY) {
        unit_torques = - _kp * (_robot->q() - _q_prior) - _kv * _robot->dq();  
        joint_strategy_torques = (_current_task_range.transpose() * _posture_projected_jacobian).transpose() * \
                                            _M_partial * _current_task_range.transpose() * unit_torques;

    } else if (_sing_type == TYPE_2_SINGULARITY) {
        // apply open-loop torque proportional to dot(unit mass force, singular direction)
        _M_partial.setIdentity();  
        double fTd = ((unit_mass_force.normalized()).transpose() * _task_range_s.rightCols(1))(0);
        VectorXd midrange_distance = _joint_midrange - _robot->q();
        VectorXd torque_sign = (midrange_distance.array() > 0).cast<double>() - (midrange_distance.array() < 0).cast<double>();
        VectorXd magnitude_unit_torques = std::abs(fTd) * _type_2_torque_vector;
        unit_torques = torque_sign.array() * magnitude_unit_torques.array();
        joint_strategy_torques = (_current_task_range.transpose() * _posture_projected_jacobian).transpose() * \
                                            _M_partial * _current_task_range.transpose() * unit_torques;
    }

    // Combine non-singular torques and blended singular torques with joint strategy torques 
    VectorXd tau_ns = _projected_jacobian_ns.transpose() * (_Lambda_ns * _task_range_ns.transpose() * unit_mass_force + \
                            _task_range_ns.transpose() * force_related_terms);
    if (_sing_type == NO_SINGULARITY) {
        return tau_ns;
    } else {
        VectorXd singular_task_force = _alpha * _Lambda_s * _task_range_s.transpose() * unit_mass_force + \
                                            _task_range_s.transpose() * force_related_terms;
        VectorXd tau_s = _projected_jacobian_s.transpose() * singular_task_force + \
                            (1 - _alpha) * joint_strategy_torques;
        return tau_ns + tau_s;
    }
                
}

// debug
VectorXd SingularityHandler::getSigmaValues() {
    return _s_values;
}

}  // namespace