/*
 * SingularityHandler.cpp
 *
 *      Author: William Chong 
 */

#include "SingularityHandler.h"

// Default parameters 
namespace {
    // functions
    int sign(double x) {
        return (x > 0) - (x < 0);
    }

    MatrixXd pseudoInverse(const MatrixXd& A,
                           const double tol = 1e-6) {
        Eigen::JacobiSVD<MatrixXd> svd(A,
            Eigen::ComputeThinU | Eigen::ComputeThinV);

        const auto& S = svd.singularValues();
        MatrixXd S_inv = MatrixXd::Zero(A.cols(), A.rows());

        for (int i = 0; i < S.size(); ++i) {
            if (S(i) > tol) {             
                S_inv(i, i) = 1.0 / S(i);
            }
        }

        return svd.matrixV() * S_inv * svd.matrixU().transpose();
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

    std::vector<int> indicesBelow(const Eigen::VectorXd& v, double threshold = 1e-6) {
        std::vector<int> idx;
        idx.reserve(v.size());

        for (int i = 0; i < v.size(); ++i) {
            if (v[i] < threshold) {
                idx.push_back(i);
            }
        }
        return idx;
    }

    bool contains(const std::vector<int>& v, int value) {
        return std::find(v.begin(), v.end(), value) != v.end();
    }
}

namespace Sai2Primitives {

SingularityHandler::SingularityHandler(std::shared_ptr<Sai2Model::Sai2Model> robot,
                                       const std::string link_name,
                                       const Affine3d compliant_frame,
                                       const int task_rank,
                                       const std::vector<int> joint_dependency,
                                       const double dt,
                                       const bool verbose) : 
                                       _robot(robot),
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
    _dq_max = VectorXd::Zero(_dof);
    auto joint_limits = _robot->jointLimits();
    for (int i = 0; i < joint_limits.size(); ++i) {
        _q_upper(i) = joint_limits[i].position_upper - DefaultParameters::joint_limit_buffer;
        _q_lower(i) = joint_limits[i].position_lower + DefaultParameters::joint_limit_buffer;
        _dq_max(i) = joint_limits[i].velocity;
        _joint_midrange(i) = 0.5 * (joint_limits[i].position_lower + joint_limits[i].position_upper);
        _tau_upper(i) = joint_limits[i].effort;
        _tau_lower(i) = - joint_limits[i].effort;
    }

    // initialize singularity handling variables 
    _singularity_types.resize(0);
    _q_prior = _joint_midrange;
    _dq_prior = VectorXd::Zero(_dof);
    setSingularityHandlingGains(DefaultParameters::kp_type_1, 
                                DefaultParameters::kv_type_1, 
                                DefaultParameters::kp_type_2, 
                                DefaultParameters::kv_type_2);
    setDynamicDecouplingType(BOUNDED_INERTIA_ESTIMATES);
	setBoundedInertiaEstimateThreshold(DefaultParameters::bie_threshold, DefaultParameters::singular_bie_threshold);
    _type_1_counter = 0;
    _type_2_counter = 0;
    _type_2_direction = - VectorXd::Ones(_dof);
    _enforce_type_1_strategy = false;
    _enforce_handling_strategy = true;

    // initialize singularity handling classification variables
    _s_abs_tol = DefaultParameters::s_abs_tol;
    _type_1_tol = DefaultParameters::type_1_tol; 
    _perturb_step_size = DefaultParameters::perturb_step_size;

    _type_2_force_threshold = DefaultParameters::type_2_force_threshold;  
    _type_2_angle_threshold = DefaultParameters::type_2_angle_threshold;
    _type_2_max_vel_vector = DefaultParameters::type_2_max_vel * VectorXd::Ones(_dof);   

    _buffer_size = DefaultParameters::buffer_size;

    _type_1_buffer_size = DefaultParameters::type_1_buffer_size;
    _type_1_max_vel_away_from_singularity = DefaultParameters::type_1_max_vel_away_from_singularity;
    _type_1_max_vel_towards_singularity = DefaultParameters::type_1_max_vel_towards_singularity;
    _type_1_step_size_control_towards_singularity = DefaultParameters::type_1_step_size_control_towards_singularity;
    _type_1_step_size_classification_towards_singularity = DefaultParameters::type_1_step_size_classification_towards_singularity;

    _max_force_norm = DefaultParameters::max_force_norm;

    _enable_force_decoupling = true;
    _fully_singular_task = false;
    _handle_singularity_exit = false;
    _is_in_singularity = false;

    _alpha_blending_matrix = MatrixXd::Zero(1, 1);
    _prev_singular_vector = VectorXd::Zero(_task_rank);
    _type_1_retracting = false;

}

void SingularityHandler::updateTaskModel(MatrixXd& projected_jacobian, const MatrixXd& N_prec) {
    
    // task range decomposition
    JacobiSVD<MatrixXd> J_svd(projected_jacobian, ComputeThinU | ComputeThinV);
    _svd_U = J_svd.matrixU();
    _svd_s = J_svd.singularValues();  // descending order 
    _svd_V = J_svd.matrixV();   

    // compute jacobian derivatives 
    _dJdq = _robot->getJacobianDerivative(_link_name, _compliant_frame.translation());

    // svd check
    _fully_singular_task = false;
    if (_svd_s(0) < _s_abs_tol) {
        if (_verbose) {
            std::cout << "WARNING: Fully singular task\n";
        }

        _fully_singular_task = true;

        // placeholder non-singular terms 
        _task_range_ns = MatrixXd::Zero(_task_rank, 1);
        _projected_jacobian_ns = MatrixXd::Zero(_task_rank, _dof);
        _Lambda_ns = MatrixXd::Zero(_task_rank, _task_rank);

        // singular task 
        _task_range_s = _svd_U.leftCols(_task_rank);
        _joint_task_range_s = _svd_V.leftCols(_task_rank);
        _projected_jacobian_s = _task_range_s.transpose() * projected_jacobian;
        _Lambda_s = pseudoInverse(_projected_jacobian_s *
                                  _robot->MInv() * 
                                  _projected_jacobian_s.transpose(), _s_abs_tol);
        _svd_s_singular = _svd_s;

        // sjs info
        _prev_num_singularities = _num_singularities;
        _num_singularities = _task_rank;
        _alpha_blending_matrix = MatrixXd::Zero(_task_rank, _task_rank);
        _alpha_vec = VectorXd::Zero(_task_rank);
        _condition_ratio_vec = VectorXd::Zero(_task_rank);

        // update flags 
        _is_in_singularity = true;

    } else {
        // iterate through only up to the task rank (do not consider the zero singular values from the task projection)
        for (int i = 1; i < _task_rank; ++i) {
            double inv_condition_number = _svd_s(i) / _svd_s(0);

            if (inv_condition_number < _s_max) {

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
                _svd_s_singular = _svd_s.tail(_task_rank - i);

                // sjs info
                _prev_num_singularities = _num_singularities;
                _num_singularities = _task_range_s.cols();
                _alpha_blending_matrix = MatrixXd::Zero(_num_singularities, _num_singularities);
                _alpha_vec = VectorXd::Zero(_num_singularities);
                _condition_ratio_vec = VectorXd::Zero(_num_singularities);
                for (int j = 0; j < _num_singularities; ++j) {
                    double curr_inv_condition_number = _svd_s(i + j) / _svd_s(0);
                    _alpha_blending_matrix(j, j) = std::clamp((curr_inv_condition_number - _s_min) / (_s_max - _s_min), 0., 1.);
                    _alpha_vec(j) = _alpha_blending_matrix(j, j);
                    _condition_ratio_vec(j) = curr_inv_condition_number / _s_min;
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
                _svd_s_singular = VectorXd::Zero(_task_rank);

                // sjs info
                _prev_num_singularities = _num_singularities;
                _num_singularities = 0;
                _alpha_blending_matrix = MatrixXd::Zero(_task_rank, _task_rank);
                _alpha_vec = VectorXd::Zero(_task_rank);
                _condition_ratio_vec = VectorXd::Zero(_task_rank);

                // update flags 
                _is_in_singularity = false;
            }
        }
    }

    // exit transition check
    if (_num_singularities < _prev_num_singularities && _num_singularities > 0) {
        _singularity_exit_transition = true;
    } else {
        _singularity_exit_transition = false;
    }

    // model updates 
    if (!_is_in_singularity || !_enforce_handling_strategy) {
        _N = _N_ns;  
        _Lambda_joint_s = MatrixXd::Zero(1, 1);  // placeholder
    } else if (_fully_singular_task) {
        _N = N_prec;  // if task is fully singular, then pass through the task 
        _Lambda_joint_s = MatrixXd::Zero(1, 1);  // placeholder
    } else {
        _posture_projected_jacobian = _joint_task_range_s.transpose() * _N_ns * N_prec;
        Sai2Model::OpSpaceMatrices op_space_matrices =
            _robot->operationalSpaceMatrices(_posture_projected_jacobian);
        // _Lambda_joint_s = op_space_matrices.Lambda;
        _N = op_space_matrices.N * _N_ns; 
        _N_sjs_init = _N_ns * N_prec;
    }

    // dynamic decoupling 
    switch (_dynamic_decoupling_type) {
        case FULL_DYNAMIC_DECOUPLING: {
            _Lambda_ns_modified = _Lambda_ns;
            _Lambda_s_modified = _Lambda_s;
            // _Lambda_joint_s_modified = _Lambda_joint_s;
            break;
        }

        case IMPEDANCE: {
            _Lambda_ns_modified = MatrixXd::Identity(_task_range_ns.cols(), _task_range_ns.cols());
            _Lambda_s_modified = MatrixXd::Identity(_task_range_s.cols(), _task_range_s.cols());
            // _Lambda_joint_s_modified = MatrixXd::Identity(_joint_task_range_s.cols(), _joint_task_range_s.cols());
            break;
        }

        case BOUNDED_INERTIA_ESTIMATES: {
            MatrixXd M_BIE = _robot->M();
            MatrixXd M_BIE_SINGULARITY = _robot->M();
            for (int i = 0; i < _robot->dof(); i++) {
                if (M_BIE(i, i) < _bie_threshold) {
                    M_BIE(i, i) = _bie_threshold;
                }
                if (M_BIE_SINGULARITY(i, i) < _singular_bie_threshold) {
                    M_BIE_SINGULARITY(i, i) = _singular_bie_threshold;
                }
            }
            MatrixXd M_inv_BIE = M_BIE.inverse();
            _M_inv_BIE_SINGULARITY = M_BIE_SINGULARITY.inverse();

            // non-singular lambda
            if (!_task_range_ns.isZero(1e-8)) {
                MatrixXd Lambda_inv_BIE =
                    _projected_jacobian_ns *
                    M_inv_BIE * 
                    _projected_jacobian_ns.transpose();
                _Lambda_ns_modified = Lambda_inv_BIE.inverse();
            } else {
                _Lambda_ns_modified = _Lambda_ns;
            }

            // singular lambda
            if (!_task_range_s.isZero(1e-8)) {
                MatrixXd Lambda_inv_BIE =
                    _projected_jacobian_s *
                    M_inv_BIE * 
                    _projected_jacobian_s.transpose();
                _Lambda_s_modified = pseudoInverse(Lambda_inv_BIE, _s_abs_tol);
            } else {
                _Lambda_s_modified = _Lambda_s;
            }

            // // joint strategy lambda 
            // if (_task_range_s.norm() != 0 && _enforce_handling_strategy) {
            //     MatrixXd Lambda_inv_BIE = 
            //         _posture_projected_jacobian * 
            //         // M_inv_BIE_SINGULARITY * 
            //         _M_inv_BIE_SINGULARITY * 
            //         _posture_projected_jacobian.transpose();
            //     _Lambda_joint_s_modified = Lambda_inv_BIE.completeOrthogonalDecomposition().pseudoInverse();
            //     // _Lambda_joint_s_modified = _Lambda_joint_s;
            // } else {
            //     _Lambda_joint_s_modified = _Lambda_joint_s;
            // }
            break;
        }

        default: {
            _Lambda_s_modified = _Lambda_s;
            _Lambda_ns_modified = _Lambda_ns;
            // _Lambda_joint_s_modified = _Lambda_joint_s;
            break;
        }
	}

    classifySingularity(projected_jacobian, _task_range_s, _joint_task_range_s);
}

void SingularityHandler::classifySingularity(const MatrixXd& projected_jacobian,
                                             const MatrixXd& singular_task_range,
                                             const MatrixXd& singular_joint_task_range) {
    // memory of entering singularity state
    if (_singularity_types.size() == 0) {
        _q_prior = _robot->q();
        _dq_prior = _robot->dq();
    }

    // if singular task range is empty, return no singularities 
    if (singular_task_range.isZero(1e-8)) {
        _singularity_types.resize(0);
        _singularity_history.clear();
        _motion_towards_singularity_history.clear();
        _type_1_counter = 0;
        _type_2_counter = 0;
        return;
    }

    // classify each singular direction based on the n-th order taylor expansion: smaller = type 2, larger = type 1
    // retain ds/dq for each singularity for gradient descent in posture space for type 1
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

        // compute large perturbation
        VectorXd delta_q = _perturb_step_size * singular_joint_task_range.col(i);
        _robot->setQ(curr_q + delta_q);
        _robot->updateKinematics();

        // compute the first order term
        VectorXd first_order_perturb = projected_jacobian * delta_q;

        // compute classification based on motion along singular direction from perturbation 
        Vector3d pos_delta = _robot->position(_link_name, _compliant_frame.translation()) - curr_pos - first_order_perturb.head(3);
        // Vector3d ori_delta = Sai2Model::orientationError(_robot->rotation(_link_name, _compliant_frame.linear()), curr_ori) - first_order_perturb.tail(3);
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

    // reset robot
    _robot->setQ(curr_q);
    _robot->updateKinematics();

    // add to buffer and counters
    auto it = std::find(_singularity_types.begin(), _singularity_types.end(), TYPE_1_SINGULARITY);
    if (it != _singularity_types.end()) {
        _singularity_history.push_back(TYPE_1_SINGULARITY);
        _alpha_history.push_back(_alpha_vec);
        _type_1_counter++;
    } else {
        _singularity_history.push_back(TYPE_2_SINGULARITY);
        _alpha_history.push_back(_alpha_vec);
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

}

VectorXd SingularityHandler::computeTorques(const VectorXd& unit_mass_force, const VectorXd& force_related_terms) {
    if (_verbose) {
        if (_is_in_singularity && _enforce_handling_strategy) {
            for (auto type : _singularity_types) {
                std::cout << "Singularity: " << singularity_labels[type] << " | ";
            }
            std::cout << "\n---\n";
        }
    }

    // reset containers
    _non_singular_task_torques = VectorXd::Zero(_dof);
    _singular_task_torques = VectorXd::Zero(_dof);
    _joint_strategy_torques = VectorXd::Zero(_dof);

    if (!_is_in_singularity || !_enforce_handling_strategy) {
        if (_enable_force_decoupling) {
            _non_singular_task_torques = _projected_jacobian_ns.transpose() * \
                                            _Lambda_ns_modified * _task_range_ns.transpose() * (unit_mass_force + force_related_terms);
        } else {
            _non_singular_task_torques = _projected_jacobian_ns.transpose() * \
                                            (_Lambda_ns_modified * _task_range_ns.transpose() * unit_mass_force + \
                                             _task_range_ns.transpose() * force_related_terms);
        }
        return _non_singular_task_torques;
    } else {
        // compute non-singular torques 
        if (_fully_singular_task) {
            return _non_singular_task_torques;  // pass through task if fully singular (zero torques)
        } else {
            if (_dynamic_decoupling_type == IMPEDANCE) {
                _non_singular_task_torques = _projected_jacobian_ns.transpose() * \
                                                _task_range_ns.transpose() * (unit_mass_force + force_related_terms);
            } else {
                if (_enable_force_decoupling) {
                    _non_singular_task_torques = _projected_jacobian_ns.transpose() * \
                                                    _Lambda_ns_modified * _task_range_ns.transpose() * (unit_mass_force + force_related_terms); \
                } else {
                    _non_singular_task_torques = _projected_jacobian_ns.transpose() * \
                                                    (_Lambda_ns_modified * _task_range_ns.transpose() * unit_mass_force + \
                                                     _task_range_ns.transpose() * force_related_terms);
                }
                if (!_enforce_handling_strategy) {
                    return _non_singular_task_torques;
                }
            }
        } 

        // singularity handling setup
        VectorXd q_curr = _robot->q();
        VectorXd unit_torques = VectorXd::Zero(_robot->dof());
        VectorXd singular_joint_task_torques = VectorXd::Zero(_robot->dof());

        // Vector3d normalized_linear_force = (unit_mass_force + force_related_terms).head(3).normalized();
        // Vector3d normalized_angular_moment = (unit_mass_force + force_related_terms).tail(3).normalized();
        // VectorXd normalized_force_moment(6);
        // normalized_force_moment << normalized_linear_force, normalized_angular_moment;
        VectorXd normalized_force_moment = (unit_mass_force + force_related_terms).normalized();
        VectorXd singular_task_force = _task_range_s * _task_range_s.transpose() * (unit_mass_force + force_related_terms);

        // containers 
        MatrixXd N_prec = _N_sjs_init;
        
        // check for degenerate singularities
        std::vector<int> degenerate_indices = indicesBelow(_svd_s_singular);
        _is_degenerate_singularity = degenerate_indices.size() > 0;

        // handle each singularity recursively
        // if degenerate, use projection of task force onto singular space to determine sjs strategy
        // forward-compensate for disturbance torques based on hierarchy
        for (int i = 0; i < _singularity_types.size(); ++i) {

            VectorXd u_proj = _task_range_s.col(i);
            VectorXd v_proj = _joint_task_range_s.col(i);
            VectorXd dsdq = _dsdq_vec[i];

            bool degenerate_singularity = contains(degenerate_indices, i);
            if (degenerate_singularity) {

                // collect degenerate task range, and project task force for singular direction
                MatrixXd degenerate_task_range_s = collectColumns(_task_range_s, degenerate_indices);
                u_proj = (degenerate_task_range_s * degenerate_task_range_s.transpose() * (unit_mass_force + force_related_terms)).normalized();
                MatrixXd degenerate_joint_task_range_s = collectColumns(_joint_task_range_s, degenerate_indices);

                if (_singularity_types[i] == TYPE_1_SINGULARITY) {
                    // recompute dsdq for degenerate singularity for type 1 only
                    VectorXd u_coeff = degenerate_task_range_s.transpose() * u_proj;
                    VectorXd v_proj = VectorXd::Zero(_robot->dof());
                    for (int i = 0; i < u_coeff.size(); ++i) {
                        v_proj += u_coeff(i) * degenerate_joint_task_range_s.col(i);  // only columns related to the degenerate indices
                    }

                    // compute the new dsdq based on the new u_proj and v_proj
                    for (int i = 0; i < _robot->dof(); ++i) {
                        dsdq(i) = u_proj.transpose() * _dJdq[i] * v_proj;
                    }
                }
            } 
        
            if (_singularity_types[i] == TYPE_1_SINGULARITY) {

                // determine if approaching or leaving singularity
                Vector3d curr_pos = _robot->position(_link_name, _compliant_frame.translation());
                Matrix3d curr_ori = _robot->rotation(_link_name, _compliant_frame.linear());
                // VectorXd q_toward_singularity = saturateBox(q_curr - DIRECTION_STEP_SIZE * dsdq, _q_lower, _q_upper);
                VectorXd q_toward_singularity = q_curr - _type_1_step_size_classification_towards_singularity * dsdq;
                _robot->setQ(q_toward_singularity);
                _robot->updateKinematics();

                // compute motion direction approaching singularity
                Vector3d pos_delta = _robot->position(_link_name, _compliant_frame.translation()) - curr_pos;
                // Vector3d ori_delta = Sai2Model::orientationError(_robot->rotation(_link_name, _compliant_frame.linear()), curr_ori);
                VectorXd delta_vector = VectorXd::Zero(6);
                delta_vector.head(3) = pos_delta;
                // delta_vector.tail(3) = ori_delta;
                // Vector3d delta_vector = pos_delta;

                // reset 
                _robot->setQ(q_curr);
                _robot->updateKinematics();

                if (_verbose) {
                    std::cout << "delta vector normalized: " << delta_vector.normalized().transpose() << "\n";
                    std::cout << "force: " << (unit_mass_force + force_related_terms).normalized().transpose() << "\n";
                    std::cout << "singular direction: " << _task_range_s.transpose() << "\n";
                }

                // determine the directionality of u that is aligned with the maximizing singular value direction
                VectorXd u_proj_maximizing = u_proj;
                if (delta_vector.transpose() * u_proj_maximizing < 0) {
                    u_proj_maximizing *= -1;
                }

                // double motion_toward_singularity = delta_vector.normalized().transpose() * (unit_mass_force + force_related_terms).normalized();
                // double motion_toward_singularity = delta_vector.head(3).normalized().transpose() * normalized_linear_force;
                // double motion_toward_singularity = u_proj_maximizing.transpose() * normalized_linear_force;
                double motion_toward_singularity = u_proj_maximizing.transpose() * (unit_mass_force + force_related_terms);
                _motion_towards_singularity_history.push_back(motion_toward_singularity > 0);
                // pop oldest if greater than buffer size
                if (_motion_towards_singularity_history.size() > _type_1_buffer_size) {
                    _motion_towards_singularity_history.pop_front();
                }
                bool is_moving_towards_singularity = majorityElement(_motion_towards_singularity_history);

                // compute control for approaching or leaving type 1 singularity
                if (is_moving_towards_singularity) {
                    // command is towards singularity, thus should approach in singular joint space
                    _type_1_retracting = false;

                    if (_alpha_vec(i) != 0) {
                        // pure damping, as the singular task force is non-zero (let this dictate control towards singularity)
                        unit_torques = - _kv_type_1 * _robot->dq();
                    } else {
                        // gradient descent towards singularity
                        // scaled by the minimum of the force magnitude and condition number
                        // reduce velocity to zero as robot gets closer to singularity to avoid oscillations

                        // VectorXd singular_task_force = _task_range_s * _task_range_s.transpose() * (unit_mass_force + force_related_terms);
                        double force_vel_scaling = std::clamp(singular_task_force.norm() / _max_force_norm, 0.0, 1.0);
                        double condition_number_scaling = std::clamp(_condition_ratio_vec(i), 0.0, 1.0);
                        double vel_scaling = std::min(force_vel_scaling, condition_number_scaling);

                        VectorXd q_des = q_curr - _type_1_step_size_control_towards_singularity * dsdq;

                        // saturate within joint limits 
                        q_des = saturateBox(q_des, _q_lower, _q_upper);

                        // compute velocity-saturated control
                        VectorXd dq_des = (_kp_type_1 / _kv_type_1) * (q_des - q_curr);
                        if (dq_des.norm() > vel_scaling * _type_1_max_vel_towards_singularity) {
                            dq_des = vel_scaling * _type_1_max_vel_towards_singularity * dq_des.normalized();
                        } 
                        unit_torques = - _kv_type_1 * (_robot->dq() - dq_des);
                    }

                    if (_verbose) {
                        std::cout << "Type 1: Towards Singularity\n";
                        // std::cout << "dsdq: " << dsdq.normalized().transpose() << "\n";
                        // std::cout << "Delta q: " << delta_q.transpose() << "\n";
                        // std::cout << "q des: " << q_des.transpose() << "\n";
                        // std::cout << "vel scaling: " << vel_scaling << "\n";
                    }
                } else {

                    // command to move away from singularity
                    // in this case, prefer to move towards the entering conditions (q_prior)
                    _type_1_retracting = true;

                    // velocity-saturated towards holding posture (entering posture) 
                    VectorXd dq_des = (_kp_type_1 / _kv_type_1) * (_q_prior - q_curr);
                    double vel_scaling = std::clamp(singular_task_force.norm() / _max_force_norm, 0.0, 1.0);
                    if (dq_des.norm() > vel_scaling * _type_1_max_vel_away_from_singularity) {
                        dq_des = vel_scaling * _type_1_max_vel_away_from_singularity * dq_des.normalized();
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

            } else if (_singularity_types[i] == TYPE_2_SINGULARITY) {
                // type 2 handling 
                if (singular_task_force.norm() < _type_2_force_threshold) {
                    // damping only if force is small (deadband)
                    unit_torques = - _kv_type_2 * _robot->dq();
                } else {
                    // change direction if angle threshold is met 
                    for (int i = 0; i < _dof; ++i) {

                        // set direction to the direction of the current joint velocities 
                        _type_2_direction(i) = sign(_robot->dq()(i));

                        // direction change
                        if (std::abs(q_curr(i) - _q_upper(i)) < _type_2_angle_threshold) {
                            _type_2_direction(i) = - 1;
                        } else if (std::abs(q_curr(i) - _q_lower(i)) < _type_2_angle_threshold) {
                            _type_2_direction(i) = 1;
                        } 
                    }

                    double force_dotted_singular_direction = normalized_force_moment.dot(u_proj);
                    VectorXd scaled_velocity = std::abs(force_dotted_singular_direction) * _type_2_max_vel_vector;
                    VectorXd q_des = saturateBox(q_curr + (_type_2_direction.cwiseProduct(scaled_velocity)) * _dt, _q_lower, _q_upper);
                    VectorXd dq_des = (_kp_type_2 / _kv_type_2) * (q_des - q_curr);
                    unit_torques = - _kv_type_2 * (_robot->dq() - dq_des);

                    if (_verbose) {
                        std::cout << "Type 2 torque magnitude: " << unit_torques.norm() << "\n";
                    }

                }
            }

            // compute torques and forward compensate disturbance torques for multi-singularity hierarchy
            MatrixXd sjs_jacobian = v_proj.transpose() * N_prec;
            Sai2Model::OpSpaceMatrices op_matrices = _robot->operationalSpaceMatrices(sjs_jacobian);
            MatrixXd Lambda_sjs_modified = op_matrices.Lambda;
            if (_dynamic_decoupling_type == BOUNDED_INERTIA_ESTIMATES) { 
                MatrixXd Lambda_inv_BIE = sjs_jacobian * _M_inv_BIE_SINGULARITY * sjs_jacobian.transpose();
                Lambda_sjs_modified = pseudoInverse(Lambda_inv_BIE, _s_abs_tol);
            } 
            singular_joint_task_torques -= (MatrixXd::Identity(_robot->dof(), _robot->dof()) - op_matrices.N).transpose() * singular_joint_task_torques;  // forward compensation 
            singular_joint_task_torques += sjs_jacobian.transpose() * Lambda_sjs_modified * v_proj.transpose() * unit_torques;
            N_prec = op_matrices.N * N_prec;

        }

        _joint_strategy_torques = singular_joint_task_torques;

        if (_is_in_singularity) {
            _singular_task_torques = _projected_jacobian_s.transpose() * \
                                        (_Lambda_s_modified * _alpha_blending_matrix * _task_range_s.transpose() * unit_mass_force + \
                                        _task_range_s.transpose() * force_related_terms);
        }

        {
            // debug for experimental baseline
            _task_torques_with_singularity = _non_singular_task_torques;
            _task_torques_with_singularity += _projected_jacobian_s.transpose() * \
                                                    (_Lambda_s * _task_range_s.transpose() * unit_mass_force + \
                                                    _task_range_s.transpose() * force_related_terms);
        }

        for (int i = 0; i < _dof; ++i) {
            if (isnan(_singular_task_torques(i))) {
                _singular_task_torques(i) = 0;  
            }
        }

        return _non_singular_task_torques + _singular_task_torques + _joint_strategy_torques;
    }
}

}  // namespace