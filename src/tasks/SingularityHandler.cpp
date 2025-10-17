/*
 * SingularityHandler.cpp
 *
 *      Author: William Chong 
 */

#include "SingularityHandler.h"

// Default parameters 
namespace {
    // constants 
    double S_ABS_TOL = 1e-3;
    double TYPE_1_TOL = 0.5;  // 5 perturb step size
    // double TYPE_1_TOL = 0.1;  // 5 perturb step size
    double PERTURB_STEP_SIZE = 5e0;
    // double PERTURB_STEP_SIZE = 1e0;
    double TYPE_2_TORQUE_RATIO = 1e-2;
    double TYPE_2_ANGLE_THRESHOLD = 15 * M_PI / 180;
    double TYPE_2_VEL_RATIO = 1e-2;
    double TYPE_2_MAX_VEL = M_PI / 2;
    double BUFFER_SIZE = 200;
    double MOTION_TOWARDS_SINGULARITY_BUFFER_SIZE = 1;
    double KP_TYPE_1 = 100;
    double KV_TYPE_1 = 20;
    double KP_TYPE_2 = 100;
    double KV_TYPE_2 = 20;
    double TYPE_1_STEP_VEL = M_PI / 3;  // type 1 approach
    double DIRECTION_STEP_SIZE = 1e-3;  // to determine motion direction for towards/away from type 1 singularity 
    // double DIRECTION_STEP_SIZE = 5e0;  // to determine motion direction for towards/away from type 1 singularity 
    // double Q_LIMIT_DELTA = 5 * M_PI / 180;
    // double TYPE_1_DIR_TOL = 1e-3;
    double MAX_FORCE_NORM = 1;  // admittance force -> velocity scaling
    double JOINT_LIMIT_BUFFER = 5 * M_PI / 180;
    double DEGENERATE_TOL = 1e-1;  // when two singular values are close enough
    double BIE_THRESHOLD = 0.5;

    // functions
    int sign(double x) {
        return (x > 0) - (x < 0);
    }

    Eigen::VectorXd saturateBox(const Eigen::VectorXd& x,
                                const Eigen::VectorXd& min_vec,
                                const Eigen::VectorXd& max_vec) {
        assert(x.size() == min_vec.size() && x.size() == max_vec.size());
        // clamp elementwise: first ensure ≥ min, then ensure ≤ max
        return x.cwiseMax(min_vec).cwiseMin(max_vec);
    }

    bool majorityElement(const std::deque<bool>& dq) {
        size_t count_true = std::count(dq.begin(), dq.end(), true);
        return count_true * 2 >= dq.size(); // true if majority (or tie) are true
    }

    Eigen::MatrixXd collectColumns(const Eigen::MatrixXd& M, const std::vector<int>& cols) {
        Eigen::MatrixXd result(M.rows(), cols.size());
        for (size_t i = 0; i < cols.size(); ++i) {
            result.col(i) = M.col(cols[i]);
        }
        return result;
    }
}

namespace Sai2Primitives {

SingularityHandler::SingularityHandler(std::shared_ptr<Sai2Model::Sai2Model> robot,
                                       std::shared_ptr<AutoDiffRigidBodyDynamics::Model> ad_robot,
                                       const std::string& link_name,
                                       const Affine3d& compliant_frame,
                                       const int& task_rank,
                                       const std::vector<int> joint_dependency,
                                       const double& dt,
                                       const bool& verbose) : 
                                       _robot(robot),
                                       _ad_robot(ad_robot),
                                       _link_name(link_name),
                                       _compliant_frame(compliant_frame),
                                       _task_rank(task_rank),
                                       _joint_dependency(joint_dependency),
                                       _dt(dt),
                                       _verbose(verbose)
{
    // initialize limits 
    _dof = _robot->dof();
    _q_upper = VectorXd::Zero(_dof);
    _q_lower = VectorXd::Zero(_dof);
    _tau_upper = VectorXd::Zero(_dof);
    _tau_lower = VectorXd::Zero(_dof);
    _joint_midrange = VectorXd::Zero(_dof);
    _type_2_torque_vector = VectorXd::Zero(_dof);
    _dq_max = VectorXd::Zero(_dof);
    auto joint_limits = _robot->jointLimits();
    for (int i = 0; i < joint_limits.size(); ++i) {
        _q_upper(i) = joint_limits[i].position_upper - JOINT_LIMIT_BUFFER;
        _q_lower(i) = joint_limits[i].position_lower + JOINT_LIMIT_BUFFER;
        _dq_max(i) = joint_limits[i].velocity;
        _joint_midrange(i) = 0.5 * (joint_limits[i].position_lower + joint_limits[i].position_upper);
        _type_2_torque_vector(i) = _type_2_torque_ratio * joint_limits[i].effort;
        _tau_upper(i) = joint_limits[i].effort;
        _tau_lower(i) = - joint_limits[i].effort;
    }

    // initialize singularity handling variables 
    _singularity_types.resize(0);
    _q_prior = _joint_midrange;
    _dq_prior = VectorXd::Zero(_dof);
    setSingularityHandlingGains(KP_TYPE_1, KV_TYPE_1, KP_TYPE_2, KV_TYPE_2);
    setDynamicDecouplingType(BOUNDED_INERTIA_ESTIMATES);
	setBoundedInertiaEstimateThreshold(BIE_THRESHOLD, BIE_THRESHOLD);
    _type_1_counter = 0;
    _type_2_counter = 0;
    _type_2_direction = - VectorXd::Ones(_dof);
    _enforce_type_1_strategy = false;
    _enforce_handling_strategy = true;

    // initialize singularity handling classification variables
    _s_abs_tol = S_ABS_TOL;
    _type_1_tol = TYPE_1_TOL; 
    _type_2_torque_ratio = TYPE_2_TORQUE_RATIO;
    // _type_2_vel_ratio = TYPE_2_VEL_RATIO;
    _type_2_max_vel_vector = TYPE_2_MAX_VEL * VectorXd::Ones(_dof);  // max velocity for type 2 velocity strategy 
    _type_2_force_threshold = 0.01;  // force threshold to consider type 2 strategy 
    _type_2_angle_threshold = TYPE_2_ANGLE_THRESHOLD;
    _perturb_step_size = PERTURB_STEP_SIZE;
    _buffer_size = BUFFER_SIZE;
    _motion_towards_singularity_buffer_size = MOTION_TOWARDS_SINGULARITY_BUFFER_SIZE;

    _impedance_force_torques = VectorXd::Zero(_dof);
    _enable_force_decoupling = true;
    _fully_singular_task = false;
    _handle_singularity_exit = false;
    _is_in_singularity = false;

    _alpha_blending_matrix = MatrixXd::Zero(1, 1);
    _prev_singular_vector = VectorXd::Zero(_task_rank);
    _type_1_retracting = false;

}

void SingularityHandler::updateTaskModel(MatrixXd& projected_jacobian, const MatrixXd& N_prec, const bool& is_floating) {
    
    // task range decomposition
    JacobiSVD<MatrixXd> J_svd(projected_jacobian, ComputeThinU | ComputeThinV);
    _svd_U = J_svd.matrixU();
    _svd_s = J_svd.singularValues();
    _svd_V = J_svd.matrixV();

    // compute jacobian derivatives 
    VectorXdual q_dual = _robot->q().cast<dual>();
    Vector3dual pos_in_link_dual = _compliant_frame.translation().cast<dual>();
    _dJdq = jacobianDerivative(_ad_robot, 
                                q_dual, 
                                _link_name, 
                                pos_in_link_dual, 
                                AutoDiffRigidBodyDynamics::BOTH, 
                                _joint_dependency);

    // svd check
    _fully_singular_task = false;

    if (_svd_s(0) < _s_abs_tol) {
        std::cout << "WARNING: Fully singular task\n";

        _fully_singular_task = true;

        // fully singular task
        _alpha = 0;

        // placeholder non-singular terms 
        _task_range_ns = MatrixXd::Zero(_task_rank, 1);
        _projected_jacobian_ns = MatrixXd::Zero(_task_rank, _dof);
        _Lambda_ns = MatrixXd::Zero(_task_rank, _task_rank);

        // singular task 
        _task_range_s = _svd_U.leftCols(_task_rank);
        _joint_task_range_s = _svd_V.leftCols(_task_rank);
        _projected_jacobian_s = _task_range_s.transpose() * projected_jacobian;
        _Lambda_s = (_projected_jacobian_s *
					_robot->MInv() * 
					_projected_jacobian_s.transpose()).completeOrthogonalDecomposition().pseudoInverse();

        _alpha_blending_matrix = MatrixXd::Zero(_task_rank, _task_rank);

        // update flags 
        _is_in_singularity = true;

    } else {
        for (int i = 1; i < _task_rank; ++i) {
            double inv_condition_number = _svd_s(i) / _svd_s(0);

            if (inv_condition_number < _s_max) {

                // task enters singularity blending region
                _alpha = std::clamp((inv_condition_number - _s_min) / (_s_max - _s_min), 0., 1.);

                // non-singular task
                _task_range_ns = _svd_U.leftCols(i);
                _projected_jacobian_ns = _task_range_ns.transpose() * projected_jacobian;
                Sai2Model::OpSpaceMatrices ns_matrices =
                    _robot->operationalSpaceMatrices(_projected_jacobian_ns);
                _Lambda_ns = ns_matrices.Lambda;
                _Jbar_ns = ns_matrices.Jbar;
                _N_ns = ns_matrices.N;

                // singular task: task range only collects columns of U up to size task_rank - non-singular task rank
                _task_range_s = _svd_U.block(0, i, _svd_U.rows(), _task_rank - i);  
                _joint_task_range_s = _svd_V.block(0, i, _svd_V.rows(), _task_rank - i);
                _projected_jacobian_s = _task_range_s.transpose() * projected_jacobian;  
                _Lambda_s = (_projected_jacobian_s * _robot->MInv() * _projected_jacobian_s.transpose()).inverse();

                int n_singularities = _task_range_s.cols();
                _alpha_blending_matrix = MatrixXd::Zero(n_singularities, n_singularities);
                for (int j = 0; j < n_singularities; ++j) {
                    double curr_inv_condition_number = _svd_s(i + j) / _svd_s(0);
                    _alpha_blending_matrix(j, j) = std::clamp((curr_inv_condition_number - _s_min) / (_s_max - _s_min), 0., 1.);
                }

                if (_verbose) {
                    std::cout << "alpha blending matrix: \n" << _alpha_blending_matrix << "\n";
                }

                // update flags 
                _is_in_singularity = true;
                break;

            } else if (i == _task_rank - 1) {
                // fully non-singular task  
                _alpha = 1;
                
                // non-singular task
                _task_range_ns = _svd_U.leftCols(_task_rank); 
                _projected_jacobian_ns = _task_range_ns.transpose() * projected_jacobian;
                Sai2Model::OpSpaceMatrices ns_matrices =
                    _robot->operationalSpaceMatrices(_projected_jacobian_ns);
                _Lambda_ns = ns_matrices.Lambda;
                _Jbar_ns = ns_matrices.Jbar;
                _N_ns = ns_matrices.N;

                // placeholder singular task terms 
                _task_range_s = MatrixXd::Zero(_task_rank, _task_rank);
                _joint_task_range_s = MatrixXd::Zero(_dof, 1);
                _projected_jacobian_s = MatrixXd::Zero(_task_rank, _dof);
                _Lambda_s = MatrixXd::Zero(_task_rank, _task_rank);

                _alpha_blending_matrix = MatrixXd::Zero(_task_rank, _task_rank);

                // update flags 
                _is_in_singularity = false;
            }
        }
    }

    // model updates 
    if (_task_range_s.norm() == 0 || !_enforce_handling_strategy) {
        _N = _N_ns;  
        _Lambda_joint_s = MatrixXd::Zero(1, 1);  // placeholder
        _N_Vs = MatrixXd::Zero(1, 1);
    } else if (_task_range_ns.norm() == 0) {
        _N = N_prec;  // if task is fully singular, then pass through the task 
        _Lambda_joint_s = MatrixXd::Zero(1, 1);  // placeholder
        _N_Vs = MatrixXd::Identity(1, 1);
    } else {
        _posture_projected_jacobian = _joint_task_range_s.transpose() * _N_ns * N_prec;
        Sai2Model::OpSpaceMatrices op_space_matrices =
            _robot->operationalSpaceMatrices(_posture_projected_jacobian);
        _Lambda_joint_s = op_space_matrices.Lambda;
        _N = op_space_matrices.N * _N_ns; 
        _N_Vs = op_space_matrices.N;  // only nullspace of Vs

    }

    switch (_dynamic_decoupling_type) {
        case FULL_DYNAMIC_DECOUPLING: {
            _Lambda_ns_modified = _Lambda_ns;
            _Lambda_s_modified = _Lambda_s;
            _Lambda_joint_s_modified = _Lambda_joint_s;
            break;
        }

        case IMPEDANCE: {
            _Lambda_ns_modified = MatrixXd::Identity(_task_range_ns.cols(), _task_range_ns.cols());
            _Lambda_s_modified = MatrixXd::Identity(_task_range_s.cols(), _task_range_s.cols());
            _Lambda_joint_s_modified = MatrixXd::Identity(_joint_task_range_s.cols(), _joint_task_range_s.cols());
            break;
        }

        case BOUNDED_INERTIA_ESTIMATES: {
            MatrixXd M_BIE = _robot->M();
            MatrixXd M_BIE_SINGULARITY = _robot->M();
            for (int i = 0; i < _robot->dof(); i++) {
                if (M_BIE(i, i) < _bie_threshold) {
                    M_BIE(i, i) = _bie_threshold;
                }
                if (M_BIE_SINGULARITY(i, i) < _singularity_bie_threshold) {
                    M_BIE_SINGULARITY(i, i) = _singularity_bie_threshold;
                }
            }
            MatrixXd M_inv_BIE = M_BIE.inverse();
            MatrixXd M_inv_BIE_SINGULARITY = M_BIE_SINGULARITY.inverse();

            // non-singular lambda
            if (_task_range_ns.norm() != 0) {
                MatrixXd Lambda_inv_BIE =
                    _projected_jacobian_ns *
                    M_inv_BIE * 
                    _projected_jacobian_ns.transpose();
                _Lambda_ns_modified = Lambda_inv_BIE.inverse();
            } else {
                _Lambda_ns_modified = _Lambda_ns;
            }

            // singular lambda
            if (_task_range_s.norm() != 0) {
                MatrixXd Lambda_inv_BIE =
                    _projected_jacobian_s *
                    M_inv_BIE * 
                    _projected_jacobian_s.transpose();
                _Lambda_s_modified = Lambda_inv_BIE.completeOrthogonalDecomposition().pseudoInverse();
                // _Lambda_s_modified = _Lambda_s;
            } else {
                _Lambda_s_modified = _Lambda_s;
            }

            // joint strategy lambda 
            if (_task_range_s.norm() != 0 && _enforce_handling_strategy) {
                MatrixXd Lambda_inv_BIE = 
                    _posture_projected_jacobian * 
                    M_inv_BIE_SINGULARITY * 
                    _posture_projected_jacobian.transpose();
                _Lambda_joint_s_modified = Lambda_inv_BIE.completeOrthogonalDecomposition().pseudoInverse();
                // _Lambda_joint_s_modified = _Lambda_joint_s;
            } else {
                _Lambda_joint_s_modified = _Lambda_joint_s;
            }
            break;
        }

        default: {
            _Lambda_s_modified = _Lambda_s;
            _Lambda_ns_modified = _Lambda_ns;
            _Lambda_joint_s_modified = _Lambda_joint_s;
            break;
        }
	}

    classifySingularity(projected_jacobian, _task_range_s, _joint_task_range_s);
}

void SingularityHandler::classifySingularity(const MatrixXd& projected_jacobian,
                                             const MatrixXd& singular_task_range,
                                             const MatrixXd& singular_joint_task_range) {
    // memory of entering conditions 
    if (_singularity_types.size() == 0 || (_type_2_counter > _type_1_counter) || _enforce_type_1_strategy) {
        _q_prior = _robot->q();
        _dq_prior = _robot->dq();
    } 

    // if singular task range is empty, return no singularities 
    if (singular_task_range.norm() == 0) {
        _singularity_types.resize(0);
        _singularity_history.clear();
        _alpha_history.clear();
        _motion_towards_singularity_history.clear();
        _type_1_counter = 0;
        _type_2_counter = 0;
        return;
    }

    // classify each singular direction based on the 2nd order taylor expansion: smaller = type 2, larger = type 1
    // also log ds/dq for each singularity for gradient descent in posture space    
    VectorXd curr_q = _robot->q();
    Vector3d curr_pos = _robot->position(_link_name, _compliant_frame.translation());
    Matrix3d curr_ori = _robot->rotation(_link_name, _compliant_frame.linear());
    _dsdq_vec = {};
    _singularity_types.resize(singular_task_range.cols());
    for (int i = 0; i < singular_task_range.cols(); ++i) {
        VectorXd dsdq = VectorXd::Zero(_robot->dof());
        for (int j = 0; j < _robot->dof(); ++j) {
            dsdq(j) = singular_task_range.col(i).transpose() * _dJdq[j] * singular_joint_task_range.col(i);
        }
        _dsdq_vec.push_back(dsdq);

        // compute large perturbation, and only retain the 2nd order component
        VectorXd delta_q = _perturb_step_size * singular_joint_task_range.col(i);
        _robot->setQ(curr_q + delta_q);
        _robot->updateKinematics();

        // compute the first order term
        VectorXd first_order_perturb = projected_jacobian * delta_q;

        // compute classification based on motion along singular direction from perturbation 
        Vector3d pos_delta = _robot->position(_link_name, _compliant_frame.translation()) - curr_pos - first_order_perturb.head(3);
        Vector3d ori_delta = Sai2Model::orientationError(_robot->rotation(_link_name, _compliant_frame.linear()), curr_ori) - first_order_perturb.tail(3);
        VectorXd delta_vector = VectorXd::Zero(6);
        delta_vector.head(3) = pos_delta;
        // delta_vector.tail(3) = ori_delta;
        double motion_along_singular_direction = std::abs(delta_vector.dot(singular_task_range.col(i)));
        if (motion_along_singular_direction > _type_1_tol) {
            _singularity_types[i] = TYPE_1_SINGULARITY;
        } else {
            _singularity_types[i] = TYPE_2_SINGULARITY;
        }
    }

    // reset 
    _robot->setQ(curr_q);
    _robot->updateKinematics();

    // add to buffer and counters (preference for handling type 1 over type 2 for multiple, simultaneous singularities)
    auto it = std::find(_singularity_types.begin(), _singularity_types.end(), TYPE_1_SINGULARITY);
    if (it != _singularity_types.end()) {
        _singularity_history.push_back(TYPE_1_SINGULARITY);
        _alpha_history.push_back(_alpha);
        _type_1_counter++;
    } else {
        _singularity_history.push_back(TYPE_2_SINGULARITY);
        _alpha_history.push_back(_alpha);
        _type_2_counter++;
    }

    // pop oldest if greater than buffer size
    if (_singularity_history.size() > _buffer_size) {
        if (_singularity_history.front() == TYPE_1_SINGULARITY) {
            _type_1_counter--;
        } else if (_singularity_history.front() == TYPE_2_SINGULARITY) {
            _type_2_counter--;
        }
        _singularity_history.pop_front();
        _alpha_history.pop_front();
    }

    // initialize previous singular vector if zero
    if (_prev_singular_vector.isZero()) {
        _prev_singular_vector = singular_task_range.col(0);
    }

}

VectorXd SingularityHandler::computeTorques(const VectorXd& unit_mass_force, const VectorXd& force_related_terms) {
    if (_verbose) {
        if (_singularity_types.size() != 0 && _enforce_handling_strategy) {
            for (auto type : _singularity_types) {
                std::cout << "Singularity: " << singularity_labels[type] << " | ";
            }
            std::cout << "\n---\n";
        }
    }

    _impedance_force_torques = _projected_jacobian_ns.transpose() * _task_range_ns.transpose() * force_related_terms;
    _singular_task_torques = VectorXd::Zero(_dof);
    _joint_strategy_torques = VectorXd::Zero(_dof);
    VectorXd tau_ns = VectorXd::Zero(_dof);

    if (_singularity_types.size() == 0 || !_enforce_handling_strategy) {
        if (_enable_force_decoupling) {
            tau_ns = _projected_jacobian_ns.transpose() * (_Lambda_ns_modified * _task_range_ns.transpose() * unit_mass_force + \
                            _Lambda_ns_modified * _task_range_ns.transpose() * force_related_terms);
        } else {
            tau_ns = _projected_jacobian_ns.transpose() * (_Lambda_ns_modified * _task_range_ns.transpose() * unit_mass_force + \
                            _task_range_ns.transpose() * force_related_terms);
        }
        _task_torques_with_singularity = tau_ns;
        return tau_ns;
    } else if (_dynamic_decoupling_type == IMPEDANCE) {
        return _projected_jacobian_ns.transpose() * (_task_range_ns.transpose() * unit_mass_force + \
                    _task_range_ns.transpose() * force_related_terms);
    } else {
        VectorXd tau_ns = VectorXd::Zero(_dof);

        // compute non-singular torques 
        if (_task_range_ns.norm() == 0) {
            _task_torques_with_singularity = tau_ns;
            return tau_ns;  // pass through task if fully singular 
        } else {
            tau_ns = _projected_jacobian_ns.transpose() * (_Lambda_ns_modified * _task_range_ns.transpose() * unit_mass_force + \
                        _task_range_ns.transpose() * force_related_terms);
            _task_torques_with_singularity = tau_ns;
            if (!_enforce_handling_strategy) {
                return tau_ns;
            }
        } 

        /*
            Handle type 1 or type 2 singularity 
            - Type 1: (+/-) ds/dq if approaching or leaving singularity
            - Type 2: orthogonal condition
            Singular joint strategies are velocity-controlled targets to the goal posture
            If multiple singularities:
                - Type 1 handling: find dsdq most closely aligned in motion + force direction 
                    - (creates resulting singular posture motion to drive in that direction)
                - Type 2 handling: normal method 
        */

        VectorXd q_curr = _robot->q();
        VectorXd unit_torques = VectorXd::Zero(_robot->dof());
        Vector3d normalized_linear_force = (unit_mass_force + force_related_terms).head(3).normalized();
        Vector3d normalized_angular_moment = (unit_mass_force + force_related_terms).tail(3).normalized();
        VectorXd normalized_force_moment(6);
        normalized_force_moment << normalized_linear_force, normalized_angular_moment;
        
        _is_degenerate_singularity = false;

        // compute a target desired singular joint space velocity from the singular task force 
        // velocity-impedance scaling 
        // singular joint-space directionality from transfer
        // VectorXd singular_task_force = _Lambda_s_modified * _task_range_s.transpose() * unit_mass_force;
        // VectorXd dq_des = std::clamp(singular_task_force.norm() / max_force, 0.0, 1.0);

        // handle 1 singularity at a time based on counter 
        if (_type_1_counter > _type_2_counter || _enforce_type_1_strategy) {

            VectorXd u_proj = _task_range_s.col(0);

            VectorXd dsdq = VectorXd::Zero(_robot->dof());
            // higher-order singularities, compute dsdq in closest singular direction
            if (_task_range_s.cols() > 1) {
                // find index for type 1 singularity (smallest singular value)
                std::vector<int> type_1_indices;  // collect all type 1 indices
                for (int i = 0; i < _singularity_types.size(); ++i) {
                    if (_singularity_types[i] == TYPE_1_SINGULARITY) {
                        type_1_indices.push_back(i);
                    }
                }

                MatrixXd task_range_s_type_1 = collectColumns(_task_range_s, type_1_indices);

                // if degenerate singular values, then compute projection
                // otherwise, use the smallest singular value 
                // assumes the smallest singular value is degenerate (only smallest singular value is addressed)
                // compute the dsdq most closesly aligned with current task direction for determining approaching/leaving singularity
                if (std::abs(_svd_s(0) - _svd_s(1)) < DEGENERATE_TOL) {
                    _is_degenerate_singularity = true;

                    // VectorXd u_proj = _task_range_s * _task_range_s.transpose() * normalized_force_moment;
                    // VectorXd u_coeff = task_range_s_type_1.transpose() * normalized_force_moment;
                    u_proj = _task_range_s * _task_range_s.transpose() * _prev_singular_vector;  // use previous singular vector
                    VectorXd u_coeff = task_range_s_type_1.transpose() * _prev_singular_vector;
                    VectorXd v_proj = VectorXd::Zero(_robot->dof());
                    for (int i = 0; i < u_coeff.size(); ++i) {
                        v_proj += u_coeff(i) * _joint_task_range_s.col(i);
                    }

                    // re-compute the weighted dsdq 
                    for (int i = 0; i < _robot->dof(); ++i) {
                        dsdq(i) = u_proj.transpose() * _dJdq[i] * v_proj;
                    }

                    // for (int i = 0; i < task_range_s_type_1.cols(); ++i) {
                        // dsdq += u_coeff(i) * _dsdq_vec[i];
                    // }
                } else {
                    dsdq = _dsdq_vec[type_1_indices[0]];
                    u_proj = task_range_s_type_1.col(0);
                }
            } else {
                dsdq = _dsdq_vec[0];
                u_proj = _task_range_s.col(0);
            }

            if (!_is_degenerate_singularity) {
                _prev_singular_vector = u_proj;
            }

            // determine if approaching or leaving type 1 singularity 
            Vector3d curr_pos = _robot->position(_link_name, _compliant_frame.translation());
            Matrix3d curr_ori = _robot->rotation(_link_name, _compliant_frame.linear());
            // VectorXd q_toward_singularity = saturateBox(q_curr - DIRECTION_STEP_SIZE * dsdq, _q_lower, _q_upper);
            VectorXd q_toward_singularity = q_curr - DIRECTION_STEP_SIZE * dsdq;
            _robot->setQ(q_toward_singularity);
            _robot->updateKinematics();

            // compute motion direction approaching singularity
            Vector3d pos_delta = _robot->position(_link_name, _compliant_frame.translation()) - curr_pos;
            Vector3d ori_delta = Sai2Model::orientationError(_robot->rotation(_link_name, _compliant_frame.linear()), curr_ori);
            VectorXd delta_vector = VectorXd::Zero(6);
            delta_vector.head(3) = pos_delta;
            // delta_vector.tail(3) = ori_delta;

            // reset 
            _robot->setQ(q_curr);
            _robot->updateKinematics();

            if (_verbose) {
                std::cout << "delta vector normalized: " << delta_vector.normalized().transpose() << "\n";
                std::cout << "force: " << (unit_mass_force + force_related_terms).normalized().transpose() << "\n";
                std::cout << "singular direction: " << _task_range_s.transpose() << "\n";
            }

            // determine the directionality of u that is aligned with the maximizing direction
            Vector3d u_proj_maximizing = u_proj;
            if (delta_vector.head(3).normalized().transpose() * u_proj_maximizing < 0) {
                u_proj_maximizing *= -1;
            }

            // double motion_toward_singularity = delta_vector.normalized().transpose() * (unit_mass_force + force_related_terms).normalized();
            // double motion_toward_singularity = delta_vector.head(3).normalized().transpose() * normalized_linear_force;
            double motion_toward_singularity = u_proj_maximizing.transpose() * normalized_linear_force;
            _motion_towards_singularity_history.push_back(motion_toward_singularity > 0);
            // pop oldest if greater than buffer size
            if (_motion_towards_singularity_history.size() > _motion_towards_singularity_buffer_size) {
                _motion_towards_singularity_history.pop_front();
            }
            bool is_moving_towards_singularity = majorityElement(_motion_towards_singularity_history);

            if (is_moving_towards_singularity) {
                // command is towards singularity, thus should approach in singular joint space
                _type_1_retracting = false;

                // pure damping (allow admittance from force scaling towards type 1 singularity)
                unit_torques = - _kv_type_1 * _robot->dq();

                if (_verbose) {
                    std::cout << "Type 1: Towards Singularity\n";
                    // std::cout << "dsdq: " << dsdq.normalized().transpose() << "\n";
                    // std::cout << "Delta q: " << delta_q.transpose() << "\n";
                    // std::cout << "q des: " << q_des.transpose() << "\n";
                    // std::cout << "vel scaling: " << vel_scaling << "\n";
                }
            } else {

                _type_1_retracting = true;

                // command to move away from singularity
                // in this case, prefer to move towards the entering conditions (q_prior)

                // velocity-saturated towards holding posture (entering posture) 
                // VectorXd dq_des = vel_scaling * (_kp_type_1 / _kv_type_1) * (_q_prior - q_curr);
                VectorXd dq_des = (_kp_type_1 / _kv_type_1) * (_q_prior - q_curr);
                double vel_scaling = std::clamp((unit_mass_force + force_related_terms).norm() / MAX_FORCE_NORM, 0.0, 1.0);
                // double vel_scaling = std::clamp(1 - _alpha, 0.0, 1.0);
                if (dq_des.norm() > vel_scaling * TYPE_1_STEP_VEL) {
                    dq_des = vel_scaling * TYPE_1_STEP_VEL * dq_des.normalized();
                } 
                unit_torques = - _kv_type_1 * (_robot->dq() - dq_des);

                if (_verbose) {
                    std::cout << "Type 1: Away From Singularity\n";
                    // std::cout << "dsdq: " << dsdq.normalized().transpose() << "\n";
                    // std::cout << "Delta q: " << delta_q.transpose() << "\n";
                    // std::cout << "q des: " << q_des.transpose() << "\n";
                    // std::cout << "vel scaling: " << vel_scaling << "\n";
                }
            }

        } else {

            // type 2 handling 
            if ((unit_mass_force + force_related_terms).norm() < _type_2_force_threshold) {
                // damping only if force is small (deadband)
                unit_torques = - _kv_type_2 * _robot->dq();

            } else {

                // change direction if angle threshold is met 
                for (int i = 0; i < _dof; ++i) {

                    // set direction to the direction of the current joint velocities 
                    _type_2_direction(i) = sign(_robot->dq()(i));

                    if (std::abs(q_curr(i) - _q_upper(i)) < _type_2_angle_threshold) {
                        _type_2_direction(i) = - 1;
                    } else if (std::abs(q_curr(i) - _q_lower(i)) < _type_2_angle_threshold) {
                        _type_2_direction(i) = 1;
                    } 
                }

                VectorXd u_proj = _task_range_s.col(0);
                if (_task_range_s.cols() > 1) {

                    // find index for type 2 singularity (smallest singular value)
                    std::vector<int> type_2_indices;
                    for (int i = 0; i < _singularity_types.size(); ++i) {
                        if (_singularity_types[i] == TYPE_2_SINGULARITY) {
                            type_2_indices.push_back(i);
                        }
                    }

                    MatrixXd task_range_s_type_2 = collectColumns(_task_range_s, type_2_indices);

                    // if degenerate singular values, then compute projection
                    // otherwise, use the smallest singular value 
                    if (std::abs(_svd_s(0) - _svd_s(1)) < DEGENERATE_TOL) {
                        _is_degenerate_singularity = true;
                        // get the singular direction in the singular task range closest to force direction
                        // u_proj = task_range_s_type_2 * task_range_s_type_2.transpose() * normalized_force_moment;
                        u_proj = task_range_s_type_2 * task_range_s_type_2.transpose() * _prev_singular_vector;
                    } else {
                        u_proj = task_range_s_type_2.col(type_2_indices[0]);  // if not degenerate, then default type 2 singularity 
                    }
                } 

                if (!_is_degenerate_singularity) {
                    _prev_singular_vector = u_proj;
                }

                double fTd = normalized_force_moment.dot(u_proj);
                VectorXd scaled_velocity = std::abs(fTd) * _type_2_max_vel_vector;
                VectorXd q_des = saturateBox(q_curr + (_type_2_direction.cwiseProduct(scaled_velocity)) * _dt, _q_lower, _q_upper);
                unit_torques = - _kp_type_2 * (q_curr - q_des) - _kv_type_2 * _robot->dq();

                if (_verbose) {
                    std::cout << "Type 2 torque magnitude: " << unit_torques.norm() << "\n";
                }

            }
        }

        // compute joint torques at the end 
        _joint_strategy_torques = _posture_projected_jacobian.transpose() * \
                                        _Lambda_joint_s_modified * _joint_task_range_s.transpose() * unit_torques;

        if (_verbose) {
            std::cout << "joint strategy torque norm: " << _joint_strategy_torques.norm() << "\n";
        }

        // combine non-singular torques and blended singular torques with joint strategy torques
        // _singular_task_torques = _projected_jacobian_s.transpose() * (_Lambda_s_modified * _task_range_s.transpose() * unit_mass_force + \
        //                                     _task_range_s.transpose() * force_related_terms);

        // further project the singular task within the nullspace of Vs 

        if (_is_in_singularity) {
            if (_type_1_retracting) {
                // _singular_task_torques = _N_Vs.transpose() * _projected_jacobian_s.transpose() * \
                //                             (_Lambda_s_modified * _alpha_blending_matrix * _task_range_s.transpose() * unit_mass_force + \
                //                             _task_range_s.transpose() * force_related_terms);
                _singular_task_torques = _projected_jacobian_s.transpose() * \
                                            (_Lambda_s_modified * _alpha_blending_matrix * _task_range_s.transpose() * unit_mass_force + \
                                            _task_range_s.transpose() * force_related_terms);
                std::cout << "singular task torques norm: " << _singular_task_torques.norm() << "\n";
                // issue is that _N_Vs will always cut task in region
                // task force != joint space dynamics 
                // characterize task conflict
            } else {
                _singular_task_torques = _projected_jacobian_s.transpose() * \
                                            (_Lambda_s_modified * _alpha_blending_matrix * _task_range_s.transpose() * unit_mass_force + \
                                            _task_range_s.transpose() * force_related_terms);
            }
        } else {
            _singular_task_torques.setZero();
        }

        _task_torques_with_singularity = tau_ns + _singular_task_torques;

        _impedance_force_torques += _projected_jacobian_s.transpose() * _task_range_s.transpose() * force_related_terms;

        for (int i = 0; i < _dof; ++i) {
            if (isnan(_singular_task_torques(i))) {
                _singular_task_torques(i) = 0;  
            }             
            // else if (_singular_task_torques(i) > _tau_upper(i)) {
            //     _singular_task_torques(i) = _tau_upper(i);
            // } else if (_singular_task_torques(i) < _tau_lower(i)) {
            //     _singular_task_torques(i) = _tau_lower(i);
            // }
        }

        // return tau_ns + _alpha * _singular_task_torques + (1 - _alpha) * _joint_strategy_torques;
        // return tau_ns + _alpha * _singular_task_torques + 1 * _joint_strategy_torques;
        return tau_ns + _singular_task_torques + _joint_strategy_torques;
    }
}

}  // namespace