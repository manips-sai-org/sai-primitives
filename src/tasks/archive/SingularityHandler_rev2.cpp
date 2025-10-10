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
    double TYPE_2_MAX_VEL = M_PI / 3;
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
    double MAX_FORCE_NORM = 5;  // admittance force -> velocity scaling
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
    auto dJdq = jacobianDerivative(_ad_robot, 
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

                // update flags 
                _is_in_singularity = false;
            }
        }
    }

    // model updates 
    if (_task_range_s.norm() == 0 || !_enforce_handling_strategy) {
        _N = _N_ns;  
        _Lambda_joint_s = MatrixXd::Zero(1, 1);  // placeholder
    } else if (_task_range_ns.norm() == 0) {
        _N = N_prec;  // if task is fully singular, then pass through the task 
        _Lambda_joint_s = MatrixXd::Zero(1, 1);  // placeholder
    } else {
        _posture_projected_jacobian = _joint_task_range_s.transpose() * _N_ns * N_prec;
        Sai2Model::OpSpaceMatrices op_space_matrices =
            _robot->operationalSpaceMatrices(_posture_projected_jacobian);
        _Lambda_joint_s = op_space_matrices.Lambda;
        _N = op_space_matrices.N * _N_ns; 
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

    classifySingularity(projected_jacobian, _task_range_s, _joint_task_range_s, dJdq);
}

void SingularityHandler::classifySingularity(const MatrixXd& projected_jacobian,
                                             const MatrixXd& singular_task_range,
                                             const MatrixXd& singular_joint_task_range,
                                             const std::vector<MatrixXd>& dJdq) {
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
            dsdq(j) = singular_task_range.col(i).transpose() * dJdq[j] * singular_joint_task_range.col(i);
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

    // // classify each column in the singular task range
    // _singularity_types.resize(singular_task_range.cols());
    // VectorXd curr_q = _robot->q();
    // Vector3d curr_pos = _robot->position(_link_name, _compliant_frame.translation());
    // Matrix3d curr_ori = _robot->rotation(_link_name, _compliant_frame.linear());

    // for (int i = 0; i < singular_task_range.cols(); ++i) {
    //     VectorXd delta_q = _perturb_step_size * singular_joint_task_range.col(i);
    //     _robot->setQ(curr_q + delta_q);
    //     _robot->updateKinematics();

    //     // compute classification based on motion along singular direction from perturbation 
    //     Vector3d pos_delta = _robot->position(_link_name, _compliant_frame.translation()) - curr_pos;
    //     Vector3d ori_delta = Sai2Model::orientationError(_robot->rotation(_link_name, _compliant_frame.linear()), curr_ori);
    //     VectorXd delta_vector(6);
    //     delta_vector.head(3) = pos_delta;
    //     delta_vector.tail(3) = ori_delta;
    //     double motion_along_singular_direction = std::abs(delta_vector.dot(singular_task_range.col(i)));
    //     if (motion_along_singular_direction > _type_1_tol) {
    //         _singularity_types[i] = TYPE_1_SINGULARITY;
    //     } else {
    //         _singularity_types[i] = TYPE_2_SINGULARITY;
    //     }
            
    //     _robot->setQ(curr_q);
    //     _robot->updateKinematics();
    // }

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

    // // type 1 singularity: take N steps of gradient descent/ascent for approaching/leaving singularity
    // // type 2 singularity: velocity control 
    // _q_target = _robot->q();
    // if (_type_1_counter > _type_2_counter || _enforce_type_1_strategy) {
    //     auto alpha_trend = getAvgDiff(_alpha_history);
    //     if (alpha_trend == Trend::Increasing) {
    //         // leaving type 1
    //         _q_target += POSTURE_STEP_SIZE * (1. / 1000) * dsdq; 
    //     } else if (alpha_trend == Trend::Decreasing) {
    //         // approaching type 1
    //         _q_target -= POSTURE_STEP_SIZE * (1. / 1000) * dsdq;
    //     } 
    // } 

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

        // handle 1 singularity at a time based on counter 
        if (_type_1_counter > _type_2_counter || _enforce_type_1_strategy) {

            VectorXd dsdq = VectorXd::Zero(_robot->dof());
            // higher-order singularities, compute dsdq in closest singular direction
            if (_task_range_s.cols() > 1) {

                // find index for type 1 singularity (smallest singular value)
                int idx = 0;
                for (int j = 0; j < _singularity_types.size(); ++j) {
                    if (_singularity_types[j] == TYPE_1_SINGULARITY) {
                        idx = j;
                        break;
                    }
                }

                // if degenerate singular values, then compute projection
                // otherwise, use the smallest singular value 
                if (std::abs(_svd_s(0) - _svd_s(1)) < DEGENERATE_TOL) {
                    VectorXd u_proj = _task_range_s * _task_range_s.transpose() * (unit_mass_force + force_related_terms).normalized();
                    VectorXd u_coeff = _task_range_s.transpose() * (unit_mass_force + force_related_terms).normalized();
                    for (int i = 0; i < _task_range_s.cols(); ++i) {
                        dsdq += u_coeff(i) * _dsdq_vec[i];
                    }
                } else {
                    dsdq = _dsdq_vec[idx];
                }
            } else {
                dsdq = _dsdq_vec[0];
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

            // double motion_toward_singularity = delta_vector.normalized().transpose() * (unit_mass_force + force_related_terms).normalized();
            double motion_toward_singularity = delta_vector.head(3).normalized().transpose() * normalized_linear_force;
            _motion_towards_singularity_history.push_back(motion_toward_singularity > 0);
            // pop oldest if greater than buffer size
            if (_motion_towards_singularity_history.size() > _motion_towards_singularity_buffer_size) {
                _motion_towards_singularity_history.pop_front();
            }
            bool is_moving_towards_singularity = majorityElement(_motion_towards_singularity_history);

            // scale the posture step based on the unit mass force magnitude
            // move faster if high force, move slower if low force (similar to admittance force-velocity control)
            // double min_q_dist = std::min((q_curr - _q_lower).minCoeff(), (_q_upper - q_curr).minCoeff());
            // double vel_scaling = std::clamp(min_q_dist / Q_LIMIT_DELTA, 0.0, 1.0);
            double vel_scaling = std::clamp((unit_mass_force + force_related_terms).norm() / MAX_FORCE_NORM, 0.0, 1.0);
            VectorXd delta_q = vel_scaling * TYPE_1_STEP_VEL * _dt * dsdq.normalized();
            // VectorXd delta_q = POSTURE_STEP_VEL * _dsdq_vec[i].normalized();

            if (is_moving_towards_singularity) {

                // command is towards singularity, thus should approach in singular joint space

                // position option
                VectorXd q_des = saturateBox(q_curr + delta_q, _q_lower, _q_upper);
                // unit_torques = - _kp_type_1 * (q_curr - q_des) - _kv_type_1 * _robot->dq();
                unit_torques = - _kp_type_1 * delta_q - 0 * _kv_type_1 * _robot->dq();

                // velocity option 
                // VectorXd dq_des = - vel_scaling * TYPE_1_STEP_VEL * (_kp_type_1 / _kv_type_1) * dsdq;
                // VectorXd dq_des = vel_scaling * (_kp_type_1 / _kv_type_1) * dsdq;
                // if (dq_des.norm() > TYPE_1_STEP_VEL) {
                    // dq_des = TYPE_1_STEP_VEL * dq_des.normalized();
                // }
                // unit_torques = - _kv_type_2 * (_robot->dq() - dq_des);

                // pure damping
                unit_torques = - _kv_type_1 * _robot->dq();

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
                // take velocity step towards q_prior
                // delta_q = vel_scaling * TYPE_1_STEP_VEL * _dt * (q_curr - _q_prior).normalized();
                delta_q = vel_scaling * (q_curr - _q_prior);

                // command is away from singularity, thus should move away in singular joint space

                // position option
                VectorXd q_des = saturateBox(q_curr + delta_q, _q_lower, _q_upper);

                // // for retracting away from singularity, move towards entering conditions (q prior)
                // VectorXd q_prior_dir = q_curr - _q_prior;
                // for (int j = 0; j < q_prior_dir.size(); ++j) {
                //     if (sign(q_des(j)) != sign(q_prior_dir(j))) {
                //         q_des(j) *= -1;
                //     }
                // }

                // unit_torques = - _kp_type_1 * delta_q - _kv_type_1 * _robot->dq();

                // velocity-saturated towards holding 
                // VectorXd dq_des = vel_scaling * (_kp_type_1 / _kv_type_1) * (_q_prior - q_curr);
                VectorXd dq_des = (_kp_type_1 / _kv_type_1) * (_q_prior - q_curr);
                if (dq_des.norm() > TYPE_1_STEP_VEL) {
                    dq_des = TYPE_1_STEP_VEL * dq_des.normalized();
                }
                unit_torques = - _kv_type_1 * (_robot->dq() - dq_des);

                // velocity option 
                // VectorXd dq_des = vel_scaling * TYPE_1_STEP_VEL * (_kp_type_1 / _kv_type_2) * (_q_prior - q_curr);
                // VectorXd dq_des = vel_scaling * (_kp_type_1 / _kv_type_1) * (_q_prior - q_curr);
                // VectorXd dq_des = vel_scaling * (_kp_type_1 / _kv_type_1) * (_joint_midrange - q_curr);
                // if (dq_des.norm() > TYPE_1_STEP_VEL) {
                    // dq_des = TYPE_1_STEP_VEL * dq_des.normalized();
                // }
                // unit_torques = - _kv_type_2 * (_robot->dq() - dq_des);

                if (_verbose) {
                    std::cout << "Type 1: Away From Singularity\n";
                    // std::cout << "dsdq: " << dsdq.normalized().transpose() << "\n";
                    // std::cout << "Delta q: " << delta_q.transpose() << "\n";
                    // std::cout << "q des: " << q_des.transpose() << "\n";
                    // std::cout << "vel scaling: " << vel_scaling << "\n";
                }
            }

            // default holding 
            // unit_torques = - _kp_type_1 * (q_curr - _q_prior) - _kv_type_1 * _robot->dq();
            // unit_torques = - _kp_type_1 * (q_curr - _q_prior) - _kv_type_1 * _robot->dq();

        } else {

            // type 2 handling 
            if ((unit_mass_force + force_related_terms).norm() < _type_2_force_threshold) {
                // damping only
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

                    // find index for type 1 singularity (smallest singular value)
                    int idx = 0;
                    for (int j = 0; j < _singularity_types.size(); ++j) {
                        if (_singularity_types[j] == TYPE_2_SINGULARITY) {
                            idx = j;
                            break;
                        }
                    }

                    // if degenerate singular values, then compute projection
                    // otherwise, use the smallest singular value 
                    if (std::abs(_svd_s(0) - _svd_s(1)) < DEGENERATE_TOL) {
                        // get the singular direction in the singular task range closest to force direction
                        u_proj = _task_range_s * _task_range_s.transpose() * (unit_mass_force + force_related_terms).normalized();
                    } else {
                        u_proj = _task_range_s.col(idx);
                    }
                }

                double fTd = ((unit_mass_force + force_related_terms).normalized()).dot(u_proj);
                VectorXd magnitude_unit_torques = std::abs(fTd) * _type_2_max_vel_vector;
                VectorXd q_des = saturateBox(q_curr + (_type_2_direction.cwiseProduct(magnitude_unit_torques)) * _dt, _q_lower, _q_upper);
                unit_torques = - _kp_type_2 * (q_curr - q_des) - _kv_type_2 * _robot->dq();
                // unit_torques(i) = _joint_task_range_s.col(i).transpose() * q_force;
                // unit_torques = q_force;

                if (_verbose) {
                    std::cout << "Type 2 torque magnitude: " << unit_torques.norm() << "\n";
                }

            }
        }

        // for (int i = 0; i < _singularity_types.size(); ++i) {
        //     if (_singularity_types[i] == TYPE_1_SINGULARITY || _enforce_type_1_strategy) {

        //         // determine if approaching or leaving type 1 singularity 
        //         Vector3d curr_pos = _robot->position(_link_name, _compliant_frame.translation());
        //         Matrix3d curr_ori = _robot->rotation(_link_name, _compliant_frame.linear());
        //         VectorXd q_toward_singularity = saturateBox(q_curr - DIRECTION_STEP_SIZE * _dsdq_vec[i].normalized(), _q_lower, _q_upper);
        //         _robot->setQ(q_toward_singularity);
        //         _robot->updateKinematics();

        //         // compute motion direction approaching singularity
        //         Vector3d pos_delta = _robot->position(_link_name, _compliant_frame.translation()) - curr_pos;
        //         Vector3d ori_delta = Sai2Model::orientationError(_robot->rotation(_link_name, _compliant_frame.linear()), curr_ori);
        //         VectorXd delta_vector(6);
        //         delta_vector.head(3) = pos_delta;
        //         delta_vector.tail(3) = ori_delta;

        //         double motion_toward_singularity = delta_vector.transpose() * (unit_mass_force + force_related_terms).normalized();

        //         // scale the posture step velocity down as joint approaches limit 
        //         double min_q_dist = std::min((q_curr - _q_lower).minCoeff(), (_q_upper - q_curr).minCoeff());
        //         double vel_scaling = std::clamp(min_q_dist / Q_LIMIT_DELTA, 0.0, 1.0);
        //         VectorXd delta_q = vel_scaling * POSTURE_STEP_VEL * (1. / 1000) * _dsdq_vec[i].normalized();
        //         // VectorXd delta_q = POSTURE_STEP_VEL * _dsdq_vec[i].normalized();

        //         if (motion_toward_singularity > 0) {

        //             // command is towards singularity, thus should approach in singular joint space
        //             VectorXd q_des = saturateBox(q_curr - delta_q, _q_lower, _q_upper);
        //             // VectorXd q_des = q_curr - delta_q;
        //             VectorXd q_force = - _kp_type_1 * (q_curr - q_des) - _kv_type_1 * _robot->dq();
        //             // unit_torques(i) = _joint_task_range_s.col(i).transpose() * q_force;
        //             unit_torques = q_force;

        //             if (_verbose) {
        //                 std::cout << "Type 1: Towards Singularity\n";
        //                 std::cout << "dsdq: " << _dsdq_vec[i].normalized().transpose() << "\n";
        //                 std::cout << "Delta q: " << delta_q.transpose() << "\n";
        //                 std::cout << "q des: " << q_des.transpose() << "\n";
        //             }
        //         } else if (motion_toward_singularity < 0) {

        //             // command is away from singularity, thus should move away in singular joint space
        //             VectorXd q_des = saturateBox(q_curr + delta_q, _q_lower, _q_upper);
        //             // VectorXd q_des = q_curr + delta_q;

        //             // for retracting away from singularity, move towards entering conditions (q prior)
        //             VectorXd q_prior_dir = q_curr - _q_prior;
        //             for (int j = 0; j < q_prior_dir.size(); ++j) {
        //                 if (sign(q_des(j)) != sign(q_prior_dir(j))) {
        //                     q_des(j) *= -1;
        //                 }
        //             }

        //             VectorXd q_force = - _kp_type_1 * (q_curr - q_des) - _kv_type_1 * _robot->dq();
        //             // unit_torques(i) = _joint_task_range_s.col(i).transpose() * q_force;
        //             unit_torques = q_force;

        //             if (_verbose) {
        //                 std::cout << "Type 1: Away From Singularity\n";
        //                 std::cout << "dsdq: " << _dsdq_vec[i].normalized().transpose() << "\n";
        //                 std::cout << "Delta q: " << delta_q.transpose() << "\n";
        //                 std::cout << "q des: " << q_des.transpose() << "\n";
        //             }
        //         } else {
        //             throw runtime_error("");
        //         }

        //         // reset 
        //         _robot->setQ(q_curr);
        //         _robot->updateKinematics();

        //         // if (motion_toward_singularity > _type_1_tol) {
        //             // _singularity_types[i] = TYPE_1_SINGULARITY;
        //         // } else {
        //             // _singularity_types[i] = TYPE_2_SINGULARITY;
        //         // }
        //     } else if (_singularity_types[i] == TYPE_2_SINGULARITY) {

        //         if ((unit_mass_force + force_related_terms).norm() < _type_2_force_threshold) {
        //             // damping only


        //         } else {

        //             // change direction if angle threshold is met 
        //             for (int i = 0; i < _dof; ++i) {

        //                 // set direction to the direction of the current joint velocities 
        //                 _type_2_direction(i) = sign(_robot->dq()(i));

        //                 if (std::abs(q_curr(i) - _q_upper(i)) < _type_2_angle_threshold) {
        //                     _type_2_direction(i) = - 1;
        //                 } else if (std::abs(q_curr(i) - _q_lower(i)) < _type_2_angle_threshold) {
        //                     _type_2_direction(i) = 1;
        //                 } 
        //             }

        //             double fTd = ((unit_mass_force + force_related_terms).normalized()).dot(_task_range_s.col(0));
        //             VectorXd magnitude_unit_torques = std::abs(fTd) * _type_2_max_vel_vector;
        //             VectorXd q_des = saturateBox(q_curr + _type_2_direction * magnitude_unit_torques * (1. / 1000), _q_lower, _q_upper);
        //             VectorXd q_force = - _kp_type_2 * (q_curr - q_des) - _kv_type_2 * _robot->dq();
        //             // unit_torques(i) = _joint_task_range_s.col(i).transpose() * q_force;
        //             unit_torques = q_force;
        //         }
        //     }

        // }

        // compute joint torques at the end 
        _joint_strategy_torques = _posture_projected_jacobian.transpose() * _Lambda_joint_s_modified * _joint_task_range_s.transpose() * unit_torques;

        if (_verbose) {
            std::cout << "joint strategy torque norm: " << _joint_strategy_torques.norm() << "\n";
        }

        // /*
        //     Old method 
        // */
        // if (_type_1_counter > _type_2_counter || _enforce_type_1_strategy) {
        //     // joint holding to entering joint conditions  
        //     VectorXd unit_torques = - _kp_type_1 * (_robot->q() - _q_prior) - _kv_type_1 * _robot->dq();  
        //     _joint_strategy_torques = _posture_projected_jacobian.transpose() * _Lambda_joint_s_modified * _joint_task_range_s.transpose() * unit_torques;
        // } else {
        //     // apply open-loop torque proportional to dot(unit mass force, singular direction)
        //     // zero torque achieved when singular direction is orthogonal to the desired unit mass force direction
        //     // the direction is reversed if the joint is approaching a joint limit 
        //     // for (int i = 0; i < _joint_task_range_s.rows(); ++i) {
        //     //     if (_joint_task_range_s(i, 0) != 0) {
        //     //         if (std::abs(_robot->q()(i) - _q_upper(i)) < _type_2_angle_threshold) {
        //     //             _type_2_direction(i) = - 1;
        //     //         } else if (std::abs(_robot->q()(i) - _q_lower(i)) < _type_2_angle_threshold) {
        //     //             _type_2_direction(i) = 1;
        //     //         } 
        //     //     }
        //     // }

        //     for (int i = 0; i < _dof; ++i) {
        //         if (std::abs(_robot->q()(i) - _q_upper(i)) < _type_2_angle_threshold) {
        //             _type_2_direction(i) = - 1;
        //         } else if (std::abs(_robot->q()(i) - _q_lower(i)) < _type_2_angle_threshold) {
        //             _type_2_direction(i) = 1;
        //         } 
        //     }

        //     // std::cout << "norm: " << (unit_mass_force + force_related_terms).norm() << "\n";

        //     if ((unit_mass_force + force_related_terms).norm() < _type_2_force_threshold) {
        //         _joint_strategy_torques = _posture_projected_jacobian.transpose() * _Lambda_joint_s_modified * \
        //                                     _joint_task_range_s.transpose() * (- _kv_type_2 * _robot->dq());

        //         std::cout << "type 2 damping\n";

        //         // // reset type 2 direction during this reset
        //         // for (int i = 0; i < _joint_task_range_s.rows(); ++i) {
        //         //     if (_joint_task_range_s(i, 0) != 0) {
        //         //         if (std::abs(_robot->q()(i) - _q_upper(i)) > std::abs(_robot->q()(i) - _q_lower(i))) {
        //         //             _type_2_direction(i) = 1;
        //         //         } else {
        //         //             _type_2_direction(i) = - 1;
        //         //         }
        //         //     }
        //         // }

        //         // for (int i = 0; i < _dof; ++i) {
        //         //     if (std::abs(_robot->q()(i) - _q_upper(i)) > std::abs(_robot->q()(i) - _q_lower(i))) {
        //         //         _type_2_direction(i) = 1;
        //         //     } else {
        //         //         _type_2_direction(i) = - 1;
        //         //     }
        //         // }

        //         // std::cout << "type 2 directions: " << _type_2_direction.transpose() << "\n";

        //     } else {

        //         // change direction if angle threshold is met 
        //         for (int i = 0; i < _dof; ++i) {

        //             // set direction to the direction of the current joint velocities 
        //             _type_2_direction(i) = sign(_robot->dq()(i));

        //             if (std::abs(_robot->q()(i) - _q_upper(i)) < _type_2_angle_threshold) {
        //                 _type_2_direction(i) = - 1;
        //             } else if (std::abs(_robot->q()(i) - _q_lower(i)) < _type_2_angle_threshold) {
        //                 _type_2_direction(i) = 1;
        //             } 
        //         }

        //         double fTd = ((unit_mass_force + force_related_terms).normalized()).dot(_task_range_s.col(0));
        //         // std::cout << "type 2 ftd: " << fTd << "\n";
        //         // VectorXd magnitude_unit_torques = std::abs(fTd) * _type_2_torque_vector;
        //         VectorXd magnitude_unit_torques = std::abs(fTd) * _type_2_max_vel_vector;
        //         // std::cout << "type 2 mag unit torques: " << magnitude_unit_torques.transpose() << "\n";
        //         VectorXd unit_torques = _type_2_direction.array() * magnitude_unit_torques.array(); 

        //         // type 2 strategy; instead of open-loop torque, change to desired velocity 
        //         VectorXd dq_des = unit_torques;

        //         std::cout << "type 2 velocity\n";

        //         // // apply non-linear scaling to reduce velocity near 0 
        //         // for (int i = 0; i < _dof; ++i) {
        //         //     double alpha = std::clamp(std::abs(dq_des(i)) / _type_2_max_vel_vector(0), 0.0, 1.0);
        //         //     dq_des(i) *= std::pow(alpha, 2);
        //         // }

        //         // std::cout << "type 2 unit torques: " << unit_torques.transpose() << "\n";
        //         // std::cout << "type 2 dq max array: " << _dq_max.transpose() << "\n";
        //         // std::cout << "type 2 dq des: " << dq_des.transpose() << "\n";

        //         VectorXd delta_q_des = dq_des * (1. / 1000);
        //         std::cout << "delta q desired: \n" << delta_q_des.transpose() << "\n";

        //         _joint_strategy_torques = _posture_projected_jacobian.transpose() * _Lambda_joint_s_modified * \
        //                                     _joint_task_range_s.transpose() * (- _kp_type_2 * delta_q_des - _kv_type_2 * _robot->dq());

        //         // _joint_strategy_torques = _posture_projected_jacobian.transpose() * _Lambda_joint_s_modified * \
        //         //                             _joint_task_range_s.transpose() * (- _kv_type_2 * (_robot->dq() - dq_des));

        //         // std::cout << "type 2 directions: " << _type_2_direction.transpose() << "\n";

        //         // _joint_strategy_torques = _posture_projected_jacobian.transpose() * _joint_task_range_s.transpose() * unit_torques + \
        //         //                             _posture_projected_jacobian.transpose() * _Lambda_joint_s_modified * \
        //         //                             _joint_task_range_s.transpose() * (- _kv_type_2 * _robot->dq());

        //     }
        // }

        // combine non-singular torques and blended singular torques with joint strategy torques
        _singular_task_torques = _projected_jacobian_s.transpose() * (_Lambda_s_modified * _task_range_s.transpose() * unit_mass_force + \
                                            _task_range_s.transpose() * force_related_terms);

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

        // std::cout << "task range s: \n" << _task_range_s << "\n";

        // return tau_ns + _alpha * _singular_task_torques + (1 - _alpha) * _joint_strategy_torques;
        return tau_ns + _alpha * _singular_task_torques + 1 * _joint_strategy_torques;
    }
}

}  // namespace