/*
 * SingularityHandler.cpp
 *
 *      Author: William Chong 
 */

#include "SingularityHandler.h"

// debug 
#include <chrono>
using std::chrono::high_resolution_clock;
using std::chrono::duration_cast;
using std::chrono::duration;
using std::chrono::milliseconds;

namespace {
    // functions
    int sign(double x) {
        return (x > 0) - (x < 0);
    }

    bool contains(const std::vector<int>& v, int value) {
        return std::find(v.begin(), v.end(), value) != v.end();
    }

    VectorXd saturateBox(const VectorXd& x,
                         const VectorXd& min_vec,
                         const VectorXd& max_vec) {
        assert(x.size() == min_vec.size() && x.size() == max_vec.size());
        // clamp elementwise: first ensure ≥ min, then ensure ≤ max
        return x.cwiseMax(min_vec).cwiseMin(max_vec);
    }

    template <typename T>
    bool majorityElement(const std::deque<T>& dq, const T& value) {
        if (dq.empty()) return false;

        auto count = std::count(dq.begin(), dq.end(), value);
        return count * 2 >= dq.size(); // majority or tie
    }


    VectorXi majoritySign(const std::deque<VectorXi>& dq, const int dof) {
        if (dq.empty()) {
            return VectorXi::Ones(dof);  // empty
        }

        const int dim = dq.front().size();
        VectorXi result(dim);
        result.setZero();

        for (int i = 0; i < dim; ++i) {
            int pos = 0, neg = 0, zero = 0;

            for (const auto& v : dq) {
                int s = (v[i] > 0) - (v[i] < 0);  // sign: +1, 0, -1
                if (s > 0)      ++pos;
                else if (s < 0) ++neg;
                else            ++zero;
            }

            // majority vote (tie = + 1)
            if (pos >= neg && pos >= zero)
                result[i] = 1;
            else if (neg >= pos && neg >= zero)
                result[i] = -1;
            else
                result[i] = 1;
        }

        return result;
    }

    MatrixXd collectColumns(const VectorXd& M, const std::vector<int>& cols) {
        VectorXd result(M.size());
        for (size_t i = 0; i < cols.size(); ++i) {
            result(i) = M(cols[i]);
        }
        return result;
    }

    MatrixXd collectColumns(const MatrixXd& M, const std::vector<int>& cols) {
        MatrixXd result(M.rows(), cols.size());
        for (size_t i = 0; i < cols.size(); ++i) {
            result.col(i) = M.col(cols[i]);
        }
        return result;
    }

    MatrixXd removeColumns(const MatrixXd& M, const std::vector<int>& cols) {
        MatrixXd result(M.rows(), M.cols() - cols.size());
        int cnt = 0;
        for (size_t i = 0; i < M.cols(); ++i) {
            if (!contains(cols, i)) {
                result.col(cnt) = M.col(cols[i]);
                cnt++;
            }
        }
        return result;
    }

    std::vector<std::vector<int>> findCloseGroups(const VectorXd& v, 
                                                  const double tol = 1e-2) {
        int n = v.size();
        std::vector<bool> used(n, false);
        std::vector<std::vector<int>> groups;

        for (int i = 0; i < n; ++i) {
            if (used[i]) continue;

            std::vector<int> group;
            group.push_back(i);
            used[i] = true;

            // Compare this element with all others
            for (int j = i + 1; j < n; ++j) {
                if (!used[j] && (std::abs(v[i] - v[j]) <= tol)) {
                    group.push_back(j);
                    used[j] = true;
                } else if (!used[j] && (std::abs(v[i]) <= tol && std::abs(v[j]) <= tol)) {
                    group.push_back(j);
                    used[j] = true;
                }
            }

            if (group.size() > 1) {
                groups.push_back(group);
            }
        }

        return groups;
    }

    bool containsInGroups(const std::vector<std::vector<int>>& groups, int value) {
        for (const auto& group : groups) {
            for (int i : group) {
                if (i == value) {
                    return true;
                }
            }
        }
        return false;
    }

    MatrixXd vectorsToMatrix(const std::vector<VectorXd>& vecs) {
        if (vecs.empty()) return MatrixXd(0, 0);

        int rows = vecs[0].size();
        int cols = vecs.size();

        MatrixXd mat(rows, cols);

        for (int j = 0; j < cols; ++j) {
            if (vecs[j].size() != rows) {
                throw std::runtime_error("All VectorXd elements must have the same size");
            }
            mat.col(j) = vecs[j];
        }

        return mat;
    }

    MatrixXd removeUnitDirectionFromBasisSVD(const MatrixXd& B,
                                             const VectorXd& u,
                                             double tol = 1e-9) {
        const int n = B.rows();

        MatrixXd Bproj =
            (MatrixXd::Identity(n, n) - u * u.transpose()) * B;

        JacobiSVD<MatrixXd> svd(
            Bproj, ComputeThinU);

        const auto& S = svd.singularValues();
        int r = 0;
        for (int i = 0; i < S.size(); ++i)
            if (S(i) > tol) r++;

        return svd.matrixU().leftCols(r);
    }

    std::vector<int> argsort(const std::vector<double>& v) {
        std::vector<int> indices(v.size());
        std::iota(indices.begin(), indices.end(), 0); // fill with 0, 1, ..., n-1

        std::sort(indices.begin(), indices.end(),
                [&v](int i1, int i2) { return v[i1] < v[i2]; });

        return indices;
    }

    // sort active singularities based on singular value (smaller -> larger)
    std::vector<int> sortSingularityIndices(const std::vector<Sai2Primitives::Singularity> singularities) {
        std::vector<double> singular_values;
        for (auto singularity : singularities) {
            singular_values.push_back(singularity.sigma);
        }
        return argsort(singular_values);
    }
    
    std::vector<VectorXd> matrixToColumnVectors(const MatrixXd& M) {
        std::vector<VectorXd> cols;
        cols.reserve(M.cols());
        for (int i = 0; i < M.cols(); ++i) {
            cols.push_back(M.col(i));
        }
        return cols;
    }

    VectorXd vectorFromBasis(const std::vector<double>& coefficients, const MatrixXd& basis) {
        VectorXd result = VectorXd::Zero(basis.rows());
        for (int i = 0; i < coefficients.size(); ++i) {
            result += coefficients[i] * basis.col(i);
        }
        return result;
    }

}

namespace Sai2Primitives {

double SingularityHandler::objective(const std::vector<double> &x, std::vector<double> &grad, void* f_data) {

    auto* self = static_cast<SingularityHandler*>(f_data);

    VectorXd dq = VectorXd::Zero(self->_dof);
    for (int i = 0; i < x.size(); ++i) {
        dq += x[i] * self->_nl_opt_data->basis[i];
    }
    dq.normalize();
    self->_robot->setQ(self->_nl_opt_data->starting_q + self->_nl_opt_data->perturb_step_size * dq);
    self->_robot->updateKinematics();
    Vector3d deviation = self->_robot->position(self->_link_name, self->_compliant_frame.translation()) - 
                            self->_nl_opt_data->starting_position - 
                            self->getLinearTaylorExpansion(self->_nl_opt_data->projected_jacobian, dq).head(3);
                            // (self->_nl_opt_data->projected_jacobian * dq).head(3);
    return deviation.norm();
}

double SingularityHandler::equality(const std::vector<double> &x, std::vector<double> &grad, void* f_data) {
    double sum = 0;
    for (int i = 0; i < x.size(); ++i) {
        sum += x[i];
    }
    return 1 - sum;
}

VectorXd SingularityHandler::getLinearTaylorExpansion(const MatrixXd& projected_jacobian,
                                                      const VectorXd& dq) {
    return projected_jacobian * dq;
}

VectorXd SingularityHandler::getQuadraticTaylorExpansion(const MatrixXd& projected_jacobian,
                                                         const std::vector<MatrixXd>& kinematic_hessian,
                                                         const VectorXd& left_vector,
                                                         const VectorXd& right_vector) {
    VectorXd dx = projected_jacobian * right_vector;
    int dx_size = dx.size();
    // VectorXd dx = VectorXd::Zero(6, _dof);

    // return dx;

    // two step contraction (einsten notation)
    MatrixXd T(_dof, dx_size);
    T.setZero();

    for (int k = 0; k < _dof; ++k) {
        for (int i = 0; i < dx_size; ++i) {
            for (int j = 0; j < _dof; ++j) {
                T(k, i) += kinematic_hessian[k](i, j) * right_vector(j);
            }
        }
    }

    for (int i = 0; i < dx_size; ++i) {
        for (int k = 0; k < _dof; ++k) {
            dx(i) += 0.5 * left_vector(k) * T(k, i);
        }
    }   

    return dx;
}

bool SingularityHandler::checkBasisForType1(const VectorXd& curr_q,
                                            const Vector3d& curr_pos,
                                            const MatrixXd& projected_jacobian,
                                            const MatrixXd& singular_task_range,
                                            const MatrixXd& singular_joint_task_range,
                                            const double step_size) {

    for (int i = 0; i < singular_joint_task_range.cols(); ++i) {
        VectorXd dq = step_size * singular_joint_task_range.col(i);
        _robot->setQ(curr_q + dq);
        _robot->updateKinematics();
        VectorXd delta = VectorXd::Zero(6);
        delta.head(3) = _robot->position(_link_name, _compliant_frame.translation()) - curr_pos;

        double deviation = (singular_task_range * singular_task_range.transpose() * delta).norm();

        // double higher_order_deviation = (_robot->position(_link_name, _compliant_frame.translation()) - 
        //                                 curr_pos - 
        //                                 getLinearTaylorExpansion(projected_jacobian, dq).head(3)).norm();

        // {
        //     // debug
        //     std::cout << "type 1 deviation: " << higher_order_deviation << "\n";
        // }
        
        if (deviation > _type_1_tol) {
            return true;
        }
    }

    // test barycenter 
    {
        VectorXd bary_vector = VectorXd::Zero(singular_joint_task_range.rows());
        for (int i = 0; i < singular_joint_task_range.cols(); ++i) {
            bary_vector += singular_joint_task_range.col(i);
        }
        VectorXd dq = step_size * bary_vector / singular_joint_task_range.size();
        _robot->setQ(curr_q + dq);
        _robot->updateKinematics();
        VectorXd delta = VectorXd::Zero(6);
        delta.head(3) = _robot->position(_link_name, _compliant_frame.translation()) - curr_pos;

        double deviation = (singular_task_range * singular_task_range.transpose() * delta).norm();
        
        if (deviation > _type_1_tol) {
            return true;
        }
    }

    return false;
}

bool SingularityHandler::classifySingularityType(const VectorXd& curr_q,
                                                 const Vector3d& curr_pos,
                                                 const Matrix3d& curr_ori,
                                                 const VectorXd& u,
                                                 const VectorXd& step_direction,
                                                 const double step_size) {
    // increment in dsdq 
    _robot->setQ(curr_q + step_size * step_direction);
    _robot->updateKinematics();

    // double deviation = u.head(3).transpose() * (_robot->position(_link_name, _compliant_frame.translation()) - 
    //                                             curr_pos);

    VectorXd delta = VectorXd::Zero(6);
    delta.head(3) = _robot->position(_link_name, _compliant_frame.translation()) - curr_pos;
    delta.tail(3) = Sai2Model::orientationError(_robot->rotation(_link_name, _compliant_frame.linear()), curr_ori);

    // double deviation = u.head(3).transpose() * (_robot->position(_link_name, _compliant_frame.translation()) - 
    //                                             curr_pos);

    double deviation = u.transpose() * delta;

    {
        // debug 
        std::cout << "classify singularity type deviation: " << deviation << "\n";
    }


    if (std::abs(deviation) > _type_1_tol) {
        return true;
    }
    return false;

}

bool SingularityHandler::classifySingularityType(const MatrixXd& projected_jacobian,
                                                 const VectorXd& u,
                                                 const VectorXd& v,
                                                 const std::vector<MatrixXd>& kinematic_hessian) {
    // compute 
    VectorXd hessian_term = getQuadraticTaylorExpansion(0 * projected_jacobian, kinematic_hessian, v, v);
    double deviation = u.transpose() * hessian_term;

    {
        // debug 
        std::cout << "classify singularity type deviation: " << deviation << "\n";
    }

    if (std::abs(deviation) > _type_1_tol) {
        return true;
    }
    return false;

}

std::pair<VectorXd, VectorXd> SingularityHandler::getTowardSingularityDirection(const VectorXd& curr_q,
                                                                                const Vector3d& curr_pos,
                                                                                const VectorXd& u,
                                                                                const VectorXd& dsdq,
                                                                                const double step_size) {
    VectorXd u_toward_singularity = u;
    int cnt = 0;
    while (true) {

        // + dsdq direction 
        VectorXd pos_q_sample = curr_q + (cnt + 1) * step_size * dsdq;
        _robot->setQ(pos_q_sample);
        _robot->updateKinematics();
        Vector3d pos_delta_vector = _robot->position(_link_name, _compliant_frame.translation()) - curr_pos;
        double pos_deviation = u.head(3).dot(pos_delta_vector);

        // - dsdq direction
        VectorXd neg_q_sample = curr_q - (cnt + 1) * step_size * dsdq;
        _robot->setQ(neg_q_sample);
        _robot->updateKinematics();
        Vector3d neg_delta_vector = _robot->position(_link_name, _compliant_frame.translation()) - curr_pos;
        double neg_deviation = u.head(3).dot(neg_delta_vector);

        if (sign(pos_deviation) == sign(neg_deviation)) {
            // same sign - both pos_delta and neg_delta are in the retracting direction

            if (u.head(3).dot(pos_delta_vector) > 0) {

                u_toward_singularity = - u;

                // re-compute correct dsdq direction (+ or -)
                _robot->setQ(curr_q - step_size * dsdq);
                _robot->updateKinematics();
                neg_delta_vector = _robot->position(_link_name, _compliant_frame.translation()) - curr_pos;

                if (u_toward_singularity.head(3).dot(neg_delta_vector) < 0) {
                    // dsdq not aligned in decreasing singular value direction
                    return std::make_pair(u_toward_singularity, - dsdq);
                } else {
                    return std::make_pair(u_toward_singularity, dsdq);
                }

            } else {

                // re-compute correct dsdq direction (+ or -)
                _robot->setQ(curr_q - step_size * dsdq);
                _robot->updateKinematics();
                neg_delta_vector = _robot->position(_link_name, _compliant_frame.translation()) - curr_pos;

                if (u_toward_singularity.head(3).dot(neg_delta_vector) < 0) {
                    return std::make_pair(u_toward_singularity, - dsdq);
                } else {
                    return std::make_pair(u_toward_singularity, dsdq);
                }
            }
        }
        cnt++;

        if (cnt > _type_1_search_max_iter) {

            // choose direction with more deviation for the retracting direction
            if (std::abs(pos_deviation) > std::abs(neg_deviation)) {
                if (u.head(3).dot(pos_delta_vector) < 0) {
                    return std::make_pair(u_toward_singularity, dsdq);
                } else {
                    u_toward_singularity = - u;
                    return std::make_pair(u_toward_singularity, dsdq);
                }
            } else {
                if (u.head(3).dot(neg_delta_vector) < 0) {
                    return std::make_pair(u_toward_singularity, dsdq);
                } else {
                    u_toward_singularity = - u;
                    return std::make_pair(u_toward_singularity, dsdq);
                }
            }
        }
    }
} 

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
    _kv_damping = DefaultParameters::kv_damping;
    setSingularityHandlingGains(DefaultParameters::kp_type_1, 
                                DefaultParameters::kv_type_1, 
                                DefaultParameters::kp_type_2, 
                                DefaultParameters::kv_type_2);
    setDynamicDecouplingType(BOUNDED_INERTIA_ESTIMATES);
	setBoundedInertiaEstimateThreshold(DefaultParameters::bie_threshold, DefaultParameters::singular_bie_threshold);
    // _type_1_counter = 0;
    // _type_2_counter = 0;
    _type_2_direction = VectorXd::Ones(_dof);
    _enforce_type_1_strategy = false;
    _enforce_handling_strategy = true;

    // initialize singularity handling classification variables
    _s_abs_tol = DefaultParameters::s_abs_tol;
    _type_1_tol = DefaultParameters::type_1_tol; 
    _perturb_step_size = DefaultParameters::perturb_step_size;
    _min_blending = DefaultParameters::min_blending;

    // _type_2_force_threshold = DefaultParameters::type_2_force_threshold;  
    _type_2_angle_threshold = DefaultParameters::type_2_angle_threshold;
    _type_2_max_vel = DefaultParameters::type_2_max_vel;
    _type_2_max_vel_vector = DefaultParameters::type_2_max_vel * VectorXd::Ones(_dof);   
    _type_2_task_torque_buffer_size = DefaultParameters::type_2_task_torque_buffer_size;

    _buffer_size = DefaultParameters::buffer_size;

    _type_1_buffer_size = DefaultParameters::type_1_buffer_size;
    _type_1_max_vel_away_from_singularity = DefaultParameters::type_1_max_vel_away_from_singularity;
    _type_1_max_vel_towards_singularity = DefaultParameters::type_1_max_vel_towards_singularity;
    _type_1_step_size_control_towards_singularity = DefaultParameters::type_1_step_size_control_towards_singularity;
    _type_1_step_size_classification_towards_singularity = DefaultParameters::type_1_step_size_classification_towards_singularity;
    _type_1_step_size_for_line_search = DefaultParameters::type_1_step_size_for_line_search;
    _type_1_search_max_iter = DefaultParameters::type_1_search_max_iter;

    _max_force_norm = DefaultParameters::max_force_norm;

    _degenerate_singular_value_spacing = DefaultParameters::degenerate_singular_value_spacing;

    _enable_force_decoupling = true;
    _fully_singular_task = false;
    _handle_singularity_exit = false;
    _is_in_singularity = false;
    _use_goal_posture = false;

    _alpha_blending_matrix = MatrixXd::Zero(1, 1);
    _prev_singular_vector = VectorXd::Zero(_task_rank);
    _type_1_retracting = false;
    _type_1_search_tol = DefaultParameters::type_1_search_tol;

    _num_singularities = 0;
    _prev_num_singularities = 0;

    // _nm_tol = DefaultParameters::nm_tol;
    // _nm_max_iter = DefaultParameters::nm_max_iter;
    // _nm_step_size = DefaultParameters::nm_step_size;

    // setup nlopt (setup optimizer for all dimensionality cases between 2 and 6)
    for (int i = 2; i < 7; ++i) {
        _nl_opt[i] = std::make_unique<nlopt::opt>(nlopt::LN_COBYLA, i);

        // setup
        std::vector<double> lb, ub;
        std::vector<double> initial_step_size;
        for (int j = 0; j < i; ++j) {
            lb.push_back(0);
            ub.push_back(1);
            initial_step_size.push_back(0.1);
        }
        _nl_opt[i]->set_max_objective(objective, this);
        _nl_opt[i]->set_lower_bounds(lb);
        // _nl_opt[i]->set_upper_bounds(ub);
        _nl_opt[i]->add_equality_constraint(equality, this, 1e-2);

        _nl_opt[i]->set_xtol_rel(DefaultParameters::xtol_rel);  // 1e-12 default
        _nl_opt[i]->set_ftol_rel(DefaultParameters::ftol_rel);
        _nl_opt[i]->set_xtol_abs(DefaultParameters::xtol_abs);
        _nl_opt[i]->set_maxtime(DefaultParameters::max_time * 1e-3);
        // _nl_opt[i]->set_maxeval(5);
        // _nl_opt[i]->set_initial_step(initial_step_size);
    }

    _nl_opt_data = new OptimData(DefaultParameters::perturb_step_size);

}

void SingularityHandler::updateTaskModel(MatrixXd& projected_jacobian, const MatrixXd& N_prec) {
    
    // task range decomposition
    // auto t1 = high_resolution_clock::now();
    _J_svd.compute(projected_jacobian, ComputeThinU | ComputeThinV);  // 0.005 ms
    // auto t2 = high_resolution_clock::now();
    // duration<double, std::milli> ms_double = t2 - t1;
    // std::cout << ms_double.count() << "ms\n";

    _svd_U = _J_svd.matrixU();
    _svd_s = _J_svd.singularValues();  // descending order 
    _svd_V = _J_svd.matrixV();   

    // compute jacobian derivatives 
    _dJdq = _robot->getJacobianDerivative(_link_name, _compliant_frame.translation());  // dof x (6 x dof)

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
        _Lambda_s = Sai2Model::computePseudoInverse(_projected_jacobian_s *
                                  _robot->MInv() * 
                                  _projected_jacobian_s.transpose(), _s_abs_tol);
        _svd_s_singular = _svd_s;

        // sjs info
        _prev_num_singularities = _num_singularities;
        _num_singularities = _task_rank;
        _prev_num_zone_2_singularities = _num_zone_2_singularities;
        _num_zone_2_singularities = _task_rank;
        _alpha_blending_matrix = MatrixXd::Zero(_task_rank, _task_rank);
        _alpha_vec = VectorXd::Zero(_task_rank);
        _condition_ratio_vec = VectorXd::Zero(_task_rank);

        // update flags 
        _is_in_singularity = true;

    } else if (_task_rank == 1) {

        if (_verbose) {
            std::cout << "Fully non-singular 1 DOF task" << "\n";
        }

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
        _prev_num_zone_2_singularities = _num_zone_2_singularities;
        _num_zone_2_singularities = 0;
        _alpha_blending_matrix = MatrixXd::Zero(_task_rank, _task_rank);
        _alpha_vec = VectorXd::Zero(_task_rank);
        _condition_ratio_vec = VectorXd::Zero(_task_rank);

        // update flags 
        _is_in_singularity = false;

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
                _Lambda_s = Sai2Model::computePseudoInverse(_projected_jacobian_s * _robot->MInv() * _projected_jacobian_s.transpose(), _s_abs_tol);
                _svd_s_singular = _svd_s.tail(_task_rank - i);

                // sjs info
                _prev_num_singularities = _num_singularities;
                _num_singularities = _task_range_s.cols();
                _prev_num_zone_2_singularities = _num_zone_2_singularities;
                _num_zone_2_singularities = 0;  // will accumulate based on the condition number
                _alpha_blending_matrix = MatrixXd::Zero(_num_singularities, _num_singularities);
                _alpha_vec = VectorXd::Zero(_num_singularities);
                _condition_ratio_vec = VectorXd::Zero(_num_singularities);
                for (int j = 0; j < _num_singularities; ++j) {
                    double curr_inv_condition_number = _svd_s(i + j) / _svd_s(0);
                    _alpha_blending_matrix(j, j) = std::clamp((curr_inv_condition_number - _s_min) / (_s_max - _s_min), _min_blending, 1.);
                    _condition_ratio_vec(j) = curr_inv_condition_number;
                    if (curr_inv_condition_number < _s_min) {
                        _num_zone_2_singularities++;
                        _alpha_blending_matrix(j, j) = 0;
                    }
                    _alpha_vec(j) = _alpha_blending_matrix(j, j);
                }

                // update flags 
                _is_in_singularity = true;
                break;

            } else if (i == _task_rank - 1) {

                if (_verbose) {
                    std::cout << "Fully non-singular task" << "\n";
                }

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
                _prev_num_zone_2_singularities = _num_zone_2_singularities;
                _num_zone_2_singularities = 0;  
                _alpha_blending_matrix = MatrixXd::Zero(_task_rank, _task_rank);
                _alpha_vec = VectorXd::Zero(_task_rank);
                _condition_ratio_vec = VectorXd::Zero(_task_rank);

                // update flags 
                _is_in_singularity = false;
            }
        }
    }

    // exit transition check (check if exiting zone 2)
    // if (_num_singularities < _prev_num_singularities) {
    if (_num_zone_2_singularities < _prev_num_zone_2_singularities) {
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
            for (int i = 0; i < _dof; i++) {
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
                _Lambda_s_modified = Sai2Model::computePseudoInverse(Lambda_inv_BIE, _s_abs_tol);
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
            //     _Lambda_joint_s_modified = Lambda_inv_BIE.completeOrthogonalDecomposition().Sai2Model::computePseudoInverse();
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
    if (!_is_in_singularity) {
        _q_prior = _robot->q();
        _dq_prior = _robot->dq();
    }

    _active_singularities = {};
    _degenerate_indices = {};

    if (!_is_in_singularity) {
        // _singularity_types.resize(0);
        // _singularity_history.clear();
        // _motion_towards_singularity_history.clear();
        // _type_1_counter = 0;
        // _type_2_counter = 0;
        return;
    }

    VectorXd curr_q = _robot->q();
    Vector3d curr_pos = _robot->position(_link_name, _compliant_frame.translation());
    Matrix3d curr_ori = _robot->rotation(_link_name, _compliant_frame.linear());

    // handle degenerate singularities
    if (_is_in_singularity) {
        _degenerate_indices = findCloseGroups(_svd_s_singular, _degenerate_singular_value_spacing);

        // debug 
        {
            std::cout << "_svd_s: " << _svd_s_singular.transpose() << "\n";
        }
    } 

    _degenerate_singularities.clear();  // clear map
    _degenerate_singular_task_range = {};
    _degenerate_singular_joint_task_range = {};
    _degenerate_singular_values = {};  // track degenerate singular values for _active_singularity handling priority
    // _type_1_degenerate_singular_task_range = {};
    // _type_1_degenerate_singular_joint_task_range = {};
    // _type_2_degenerate_singular_task_range = {};
    // _type_2_degenerate_singular_joint_task_range = {};

    if (_degenerate_indices.size() > 0) {

        // store degenerate task ranges 
        for (auto group : _degenerate_indices) {
            _degenerate_singular_task_range.push_back(collectColumns(singular_task_range, group));
            _degenerate_singular_joint_task_range.push_back(collectColumns(singular_joint_task_range, group));
            _degenerate_singular_values.push_back(collectColumns(_svd_s_singular, group));
        }

        // if degenerate, then perform search in singular joint space 
        // excite the boundaries of the singular joint space to determine if type 1 singularity exists (only measure > order 2 deviation)
        // if no motions for boundaries, then all type 2 singularity
        // if there are motion, then at least one type 1 singularity exists
        // perform search over the coefficients of the singular joint space to find the direction of maximum deviation (via optimization)
        // get {u, v} for each type 1 from this search, removing the singular joint space contribution for each successive search
        // type 2 singularities are in the remaining space, with gram-schmidt decomposition for the {u} directions used in sjs strategy

        int group_id = 0;
        for (auto group : _degenerate_indices) {

            // check for type 1
            bool exists_type_1_singularity = 
                checkBasisForType1(curr_q, curr_pos, projected_jacobian, _degenerate_singular_task_range[group_id], _degenerate_singular_joint_task_range[group_id], _perturb_step_size);
            
            // perform search of singular joint space to find the coefficients in sjs for all type 1 directions
            if (exists_type_1_singularity) {

                MatrixXd curr_singular_task_range = _degenerate_singular_task_range[group_id];
                MatrixXd curr_singular_joint_task_range = _degenerate_singular_joint_task_range[group_id];
                std::vector<VectorXd> curr_type_1_singular_task_range;
                std::vector<VectorXd> curr_type_1_singular_joint_task_range;
                // VectorXd nm_search_step = _nm_step_size * VectorXd::Ones(_degenerate_singular_joint_task_range[group_id].size());

                // {
                //     // test 
                //     double angle = 0;
                //     MatrixXd tmp_range = curr_singular_joint_task_range;
                //     tmp_range.col(0) = curr_singular_joint_task_range.col(0) * cos(angle) + curr_singular_joint_task_range.col(1) * sin(angle);
                //     tmp_range.col(1) = curr_singular_joint_task_range.col(0) * sin(angle) + curr_singular_joint_task_range.col(1) * cos(angle);
                //     curr_singular_joint_task_range = tmp_range;
                // }

                _nl_opt_data->setData(matrixToColumnVectors(curr_singular_joint_task_range), projected_jacobian, curr_pos, curr_q);

                bool remaining_type_1_singularity = true;
                while (remaining_type_1_singularity) {

                    // nlopt routine
                    int n_coefficients = curr_singular_joint_task_range.cols();
                    std::vector<double> sjs_coefficients;
                    for (int i = 0; i < n_coefficients; ++i) {
                        sjs_coefficients.push_back(1.0 / n_coefficients);
                    }
                    double optimal_value = 0;

                    auto t1 = high_resolution_clock::now();
                    try {
                        _nl_opt[n_coefficients]->optimize(sjs_coefficients, optimal_value);  // 0.05 ms max
                    } catch (...) {
                        if (_verbose) {
                            std::cout << "Failed solve\n";
                        }
                    }

                    {
                        // debug
                        auto t2 = high_resolution_clock::now();
                        duration<double, std::milli> ms_double = t2 - t1;
                        std::cout << "nm solve time: " << ms_double.count() << "ms\n";
                        std::cout << "nm num evals: " << _nl_opt[n_coefficients]->get_numevals() << "\n";
                        // std::cout << "nm iterations: " << nelder_mead_result.icount << "\n";
                        // std::cout << "nm solve status: " << nelder_mead_result.ifault << "\n";
                        // std::cout << "sol: " << sjs_coefficients.transpose() << "\n";
                        std::cout << "Solution: ";
                        for (auto val : sjs_coefficients) {
                            std::cout << val << ", ";
                        }
                        std::cout << "\n";
                        std::cout << "Deviation: " << optimal_value << "\n";
                        // throw runtime_error("");
                    }

                    VectorXd v_proj = vectorFromBasis(sjs_coefficients, curr_singular_joint_task_range).normalized();
                    VectorXd u_proj = curr_singular_task_range.col(0);  // placeholder
                    // if (_degenerate_singular_values[group_id].minCoeff() / _svd_s(0) > _type_1_search_tol) {
                    if (_degenerate_singular_values[group_id].minCoeff() > _type_1_search_tol) {
                        // matching u_proj from coefficients
                        u_proj = vectorFromBasis(sjs_coefficients, curr_singular_task_range).normalized();
                    } else {
                        // get u_proj from the deviation, as {u, v} are individually rotated
                        _robot->setQ(curr_q + _type_1_step_size_classification_towards_singularity * v_proj);
                        _robot->updateKinematics();
                        u_proj.head(3) = _robot->position(_link_name, _compliant_frame.translation()) - curr_pos; 
                        u_proj.tail(3) = Sai2Model::orientationError(_robot->rotation(_link_name, _compliant_frame.linear()), curr_ori);

                        // project u_proj into the current task range
                        u_proj = (curr_singular_task_range * curr_singular_task_range.transpose() * u_proj).normalized();
                    }

                    // push {u, v} to current type 1 task ranges 
                    curr_type_1_singular_task_range.push_back(u_proj);
                    curr_type_1_singular_joint_task_range.push_back(v_proj);

                    // {
                    //     // debug
                    //     std::cout << curr_singular_task_range.cols() << "\n";
                    //     std::cout << curr_singular_joint_task_range.cols() << "\n";
                    //     std::cout << curr_singular_task_range << "n\n";
                    //     std::cout << curr_singular_joint_task_range << "\n";
                    //     std::cout << "u proj: " << u_proj.transpose() << "\n";
                    //     std::cout << "v proj: " << v_proj.transpose() << "\n";
                    // }

                    // remove {u, v} from current degenerate singular task and joint range
                    curr_singular_task_range = removeUnitDirectionFromBasisSVD(curr_singular_task_range, u_proj);
                    curr_singular_joint_task_range = removeUnitDirectionFromBasisSVD(curr_singular_joint_task_range, v_proj);

                    // if (curr_singular_task_range.cols() != curr_singular_joint_task_range.cols()) {
                    //     std::cout << curr_singular_task_range.cols() << "\n";
                    //     std::cout << curr_singular_joint_task_range.cols() << "\n";
                    //     std::cout << curr_singular_task_range << "n\n";
                    //     std::cout << curr_singular_joint_task_range << "\n";
                    //     throw runtime_error("not matching columns");
                    // }

                    if (curr_singular_joint_task_range.cols() == 0) {
                        // should never arrive here
                        break;
                    } else {
                        remaining_type_1_singularity = 
                            checkBasisForType1(curr_q, curr_pos, projected_jacobian, curr_singular_task_range, curr_singular_joint_task_range, _perturb_step_size);

                        if (!remaining_type_1_singularity) {
                            // all type 2 (continue)
                            break;
                        } else if (curr_singular_joint_task_range.cols() == 1) {
                            // one column (singularity) remaining
                            if (remaining_type_1_singularity) {
                                curr_type_1_singular_task_range.push_back(curr_singular_task_range);
                                curr_type_1_singular_joint_task_range.push_back(curr_singular_joint_task_range);

                                curr_singular_task_range = MatrixXd::Zero(1, 0);
                                curr_singular_joint_task_range = MatrixXd::Zero(1, 0);

                                break;
                            } else {
                                // all type 2 (continue)
                                break;
                            }
                        }
                    }

                    // {
                    //     // debug
                    //     std::cout << "curr singular task range: \n" << curr_singular_task_range << "\n";
                    //     std::cout << "curr singular joint task range: \n" << curr_singular_joint_task_range << "\n";
                    // }

                }

                // // collect remaining type 2 singularities (0 columns if no space)
                // _type_2_degenerate_singular_task_range.push_back(curr_singular_task_range);
                // _type_2_degenerate_singular_joint_task_range.push_back(curr_singular_joint_task_range);

                // // fill information
                // _type_1_degenerate_singular_task_range.push_back(vectorsToMatrix(curr_type_1_singular_task_range));
                // _type_1_degenerate_singular_joint_task_range.push_back(vectorsToMatrix(curr_type_1_singular_joint_task_range));
                // if (curr_singular_task_range.cols() > 0) {
                //     _type_2_degenerate_singular_task_range.push_back(curr_singular_task_range);
                //     _type_2_degenerate_singular_joint_task_range.push_back(curr_singular_joint_task_range);
                // }

                // fill active singularity information

                // type 1 singularities 
                for (int i = 0; i < curr_type_1_singular_task_range.size(); ++i) {
                    // recompute dsdq for type 1 singularities 
                    VectorXd dsdq = VectorXd::Zero(_dof);
                    for (int j = 0; j < _dof; ++j) {
                        dsdq(j) = curr_type_1_singular_task_range[i].transpose() * _dJdq[j] * curr_type_1_singular_joint_task_range[i];
                    }     
                    
                    VectorXd u = curr_type_1_singular_task_range[i];
                    VectorXd u_toward_singularity = u;  // placeholder
                    VectorXd dsdq_toward_singularity = dsdq;  // placeholder
                    
                    // compute direction toward singularity
                    double condition_ratio = _degenerate_singular_values[group_id].minCoeff() / _svd_s(0);
                    // if (condition_ratio < _type_1_search_tol) {
                    if (_degenerate_singular_values[group_id].minCoeff() < _type_1_search_tol) {
                        // line search since dsdq is indeterminate up to a sign
                        std::tie(u_toward_singularity, dsdq_toward_singularity) = 
                            getTowardSingularityDirection(curr_q, curr_pos, u, dsdq, _type_1_step_size_for_line_search);
                    } else {
                        // perform gradient descent with - dsdq
                        VectorXd q_toward_singularity = curr_q - _type_1_step_size_classification_towards_singularity * dsdq;
                        _robot->setQ(q_toward_singularity);
                        _robot->updateKinematics();
                        Vector3d delta_vector = _robot->position(_link_name, _compliant_frame.translation()) - curr_pos;
                        if (delta_vector.dot(u.head(3)) < 0) {
                            u_toward_singularity = - u;
                        }
                    }

                    _active_singularities.push_back(Singularity(curr_type_1_singular_task_range[i], 
                                                                curr_type_1_singular_joint_task_range[i], 
                                                                _degenerate_singular_values[group_id](i),  // assign rough order
                                                                dsdq_toward_singularity,
                                                                u_toward_singularity,
                                                                SingularityType::TYPE_1_SINGULARITY,
                                                                true));
                }

                // type 2 singularities 
                for (int i = 0; i < curr_singular_task_range.cols(); ++i) {
                    _active_singularities.push_back(Singularity(curr_singular_task_range.col(i), 
                                                                curr_singular_joint_task_range.col(i), 
                                                                _degenerate_singular_values[group_id](i + curr_type_1_singular_task_range.size()),  // assign rough order
                                                                SingularityType::TYPE_2_SINGULARITY,
                                                                true));
                }

            } else {

                // all type 2 singularities, use entire singular task space and singular joint space

                // // fill information
                // _type_1_degenerate_singular_task_range.push_back(MatrixXd::Zero(1, 0));
                // _type_1_degenerate_singular_joint_task_range.push_back(MatrixXd::Zero(1, 0));
                // _type_2_degenerate_singular_task_range.push_back(_degenerate_singular_task_range[group_id]);
                // _type_2_degenerate_singular_joint_task_range.push_back(_degenerate_singular_joint_task_range[group_id]);

                // active singularity classification
                for (int i = 0; i < _degenerate_singular_task_range[group_id].cols(); ++i) {
                    _active_singularities.push_back(Singularity(_degenerate_singular_task_range[group_id].col(i), 
                                                                _degenerate_singular_joint_task_range[group_id].col(i), 
                                                                _degenerate_singular_values[group_id](i), 
                                                                SingularityType::TYPE_2_SINGULARITY,
                                                                true));
                }
            }

            group_id++;

        }
    }

    // classify non-degenerate singularities
    // classify each singular direction based on the n-th order taylor expansion: smaller = type 2, larger = type 1
    // retain ds/dq for each singularity for gradient descent in posture space for type 1
    // _dsdq_vec = {};
    // _singularity_types.resize(_num_singularities);

    for (int i = 0; i < _num_singularities; ++i) {

        if (!containsInGroups(_degenerate_indices, i)) {

            VectorXd u = singular_task_range.col(i);
            VectorXd v = singular_joint_task_range.col(i);
            VectorXd u_toward_singularity = u;  // placeholder
            
            // compute gradient
            VectorXd dsdq = VectorXd::Zero(_dof);
            for (int j = 0; j < _dof; ++j) {
                dsdq(j) = u.transpose() * _dJdq[j] * v;
            }
            // _dsdq_vec.push_back(dsdq);
            VectorXd dsdq_toward_singularity = dsdq;  // placeholder

            // classify type 1 
            // bool is_type_1 = checkBasisForType1(curr_q, curr_pos, projected_jacobian, v, _perturb_step_size);
            bool is_type_1 = classifySingularityType(curr_q, curr_pos, curr_ori, u, v, _perturb_step_size);  // 0.06 vs. 0.8
            // bool is_type_1 = classifySingularityType(projected_jacobian, u, v, _dJdq);  // 0.01 vs. 0.05

            {
                // debug
                std::cout << "condition ratio: " << _svd_s_singular(i) / _svd_s(0) << "\n";
            }

            // if (is_type_1 && (_svd_s_singular(i) / _svd_s(0) < _type_1_search_tol)) {
            if (is_type_1 && (_svd_s_singular(i) < _type_1_search_tol)) {
                // perform line search
                std::tie(u_toward_singularity, dsdq_toward_singularity) = 
                    getTowardSingularityDirection(curr_q, curr_pos, u, dsdq, _type_1_step_size_for_line_search);

                {
                    // debug
                    // throw runtime_error("");
                    std::cout << "type 1 search\n";
                }

            } else {
                // perform gradient descent 
                VectorXd q_toward_singularity = curr_q - _type_1_step_size_classification_towards_singularity * dsdq_toward_singularity;
                _robot->setQ(q_toward_singularity);
                _robot->updateKinematics();
                Vector3d delta_vector = _robot->position(_link_name, _compliant_frame.translation()) - curr_pos;
                if (delta_vector.dot(u.head(3)) < 0) {
                    u_toward_singularity = - u;
                }
                
                // {
                //     // debug
                //     throw runtime_error("");
                // }

            }

            if (is_type_1) {
                _active_singularities.push_back(Singularity(u, v, _svd_s_singular(i), dsdq_toward_singularity, u_toward_singularity, SingularityType::TYPE_1_SINGULARITY, false));
            } else {
                _active_singularities.push_back(Singularity(u, v, _svd_s_singular(i), dsdq_toward_singularity, u_toward_singularity, SingularityType::TYPE_2_SINGULARITY, false));
            }

            {
                // debug
                std::cout << "dsdq: " << dsdq.transpose() << "\n";
                std::cout << "v: " << v.transpose() << "\n";
            }

        }
    }

    // reset robot
    _robot->setQ(curr_q);
    _robot->updateKinematics();

    // // add to buffer and counters
    // auto it = std::find(_singularity_types.begin(), _singularity_types.end(), TYPE_1_SINGULARITY);
    // if (it != _singularity_types.end()) {
    //     _singularity_history.push_back(TYPE_1_SINGULARITY);
    //     _alpha_history.push_back(_alpha_vec);
    //     _type_1_counter++;
    // } else {
    //     _singularity_history.push_back(TYPE_2_SINGULARITY);
    //     _alpha_history.push_back(_alpha_vec);
    //     _type_2_counter++;
    // }

    // // pop oldest if greater than buffer size
    // if (_singularity_history.size() > _buffer_size) {
    //     if (_singularity_history.front() == TYPE_1_SINGULARITY) {
    //         _type_1_counter--;
    //     } else if (_singularity_history.front() == TYPE_2_SINGULARITY) {
    //         _type_2_counter--;
    //     }
    //     _singularity_history.pop_front();
    //     _alpha_history.pop_front();
    // }

}

VectorXd SingularityHandler::computeTorques(const VectorXd& unit_mass_force, const VectorXd& force_related_terms) {
    if (_verbose) {
        if (_is_in_singularity && _enforce_handling_strategy) {
            for (auto type : _singularity_types) {
                std::cout << "Singularity: " << singularity_labels[type] << " | ";
            }
            std::cout << "n---\n";
        }
    }

    // reset containers
    _non_singular_task_torques = VectorXd::Zero(_dof);
    _singular_task_torques = VectorXd::Zero(_dof);
    _joint_strategy_torques = VectorXd::Zero(_dof);
    _unmodified_singular_task_torques = VectorXd::Zero(_dof);

    if (!_is_in_singularity || !_enforce_handling_strategy) {
        if (_enable_force_decoupling) {
            _non_singular_task_torques = _projected_jacobian_ns.transpose() * 
                                            _Lambda_ns_modified * _task_range_ns.transpose() * (unit_mass_force + force_related_terms);
        } else {
            _non_singular_task_torques = _projected_jacobian_ns.transpose() * 
                                            (_Lambda_ns_modified * _task_range_ns.transpose() * unit_mass_force + 
                                             _task_range_ns.transpose() * force_related_terms);
        }
        return _non_singular_task_torques;
    } else {
        // compute non-singular torques 
        if (_fully_singular_task) {
            return _non_singular_task_torques;  // pass through task if fully singular (zero torques)
        } else {
            if (_dynamic_decoupling_type == IMPEDANCE) {
                _non_singular_task_torques = _projected_jacobian_ns.transpose() * 
                                                _task_range_ns.transpose() * (unit_mass_force + force_related_terms);
            } else {
                if (_enable_force_decoupling) {
                    _non_singular_task_torques = _projected_jacobian_ns.transpose() * 
                                                    _Lambda_ns_modified * _task_range_ns.transpose() * (unit_mass_force + force_related_terms); 
                } else {
                    _non_singular_task_torques = _projected_jacobian_ns.transpose() * 
                                                    (_Lambda_ns_modified * _task_range_ns.transpose() * unit_mass_force + 
                                                     _task_range_ns.transpose() * force_related_terms);
                }
                if (!_enforce_handling_strategy) {
                    return _non_singular_task_torques;
                }
            }
        } 

        // compute singular task torques 
        if (_is_in_singularity) {
            _singular_task_torques = _projected_jacobian_s.transpose() * 
                                        (_Lambda_s_modified * _alpha_blending_matrix * _task_range_s.transpose() * unit_mass_force + 
                                        _task_range_s.transpose() * force_related_terms);
        }

        if (_is_in_singularity) {
            // debug for experimental baseline
            _unmodified_singular_task_torques = _projected_jacobian_s.transpose() * 
                                                    ((_projected_jacobian_s * _robot->MInv() * _projected_jacobian_s.transpose()).inverse() * _task_range_s.transpose() * unit_mass_force + 
                                                    _task_range_s.transpose() * force_related_terms);
        }

        for (int i = 0; i < _dof; ++i) {
            if (isnan(_singular_task_torques(i))) {
                _singular_task_torques(i) = 0;  
            }
        }

        VectorXd impedance_singular_task_torques = 
            _projected_jacobian_s.transpose() * _Lambda_s_modified * _task_range_s.transpose() * (unit_mass_force + force_related_terms);

        VectorXi sign_impedance_singular_task_torques(_dof);
        for (int i = 0; i < _dof; ++i) {
            sign_impedance_singular_task_torques(i) = sign(impedance_singular_task_torques(i));
        }
        _singular_task_torque_history.push_back(sign_impedance_singular_task_torques);
        // pop oldest if greater than buffer size
        if (_singular_task_torque_history.size() > _type_2_task_torque_buffer_size) {
            _singular_task_torque_history.pop_front();
        }

        // singularity handling setup
        VectorXd curr_q = _robot->q();
        Vector3d curr_pos = _robot->position(_link_name, _compliant_frame.translation());
        Matrix3d curr_ori = _robot->rotation(_link_name, _compliant_frame.linear());
        VectorXd unit_torques = VectorXd::Zero(_dof);
        VectorXd singular_joint_task_torques = VectorXd::Zero(_dof);

        // VectorXd projected_unit_torques = VectorXd::Zero(_dof);
        // VectorXd projected_forces = VectorXd::Zero(_active_singularities.size());

        // form new singular joint space from active singularities 
        MatrixXd active_singular_joint_space = MatrixXd::Zero(_dof, _active_singularities.size());

        // information
        VectorXd normalized_force_moment = (unit_mass_force + force_related_terms).normalized();
        // VectorXd singular_task_force = _task_range_s * _task_range_s.transpose() * (unit_mass_force + force_related_terms);
        // VectorXd impedance_singular_task_torque = _projected_jacobian_s.transpose() * _task_range_s.transpose() * (unit_mass_force + force_related_terms);

        // containers 
        MatrixXd N_prec = _N_sjs_init;

        // handle each singularity recursively, and forward-compensate for disturbance torques based on hierarchy
        // handle smaller -> larger singular values in order
        std::vector<int> sorted_indices = sortSingularityIndices(_active_singularities);

        {
            // debug
            // if (_verbose) {
                int cnt = 0;
                for (auto singularity : _active_singularities) {
                    std::cout << "Singularity " << cnt << ": " << singularity.type << "\n";
                }
            // }
        }

        for (auto ind : sorted_indices) {

            active_singular_joint_space.col(ind) = _active_singularities[ind].v;  // collect singular joint space basis

            if (_active_singularities[ind].type == TYPE_1_SINGULARITY) {

                double motion_toward_singularity = 
                    _active_singularities[ind].u_toward_singularity.head(3).dot((unit_mass_force + force_related_terms).head(3));
                
                // _motion_towards_singularity_history.push_back(motion_toward_singularity > 0);

                // // pop oldest if greater than buffer size
                // if (_motion_towards_singularity_history.size() > _type_1_buffer_size) {
                //     _motion_towards_singularity_history.pop_front();
                // }
                // bool is_moving_towards_singularity = majorityElement(_motion_towards_singularity_history);
                bool is_moving_towards_singularity = motion_toward_singularity > 0;

                // {
                    // debug
                    // std::cout << "is moving towards singularity: " << is_moving_towards_singularity << "\n";
                    // std::cout << "u max: " << u_proj_maximizing.transpose() << "\n";
                    // std::cout << "force: " << (unit_mass_force + force_related_terms).transpose() << "\n";
                // }

                // compute control for approaching or leaving type 1 singularity
                if (is_moving_towards_singularity) {
                    // command is towards singularity, thus should approach in singular joint space
                    _type_1_retracting = false;

                    if (_active_singularities[ind].getConditionRatio(_s_min, _s_max, _svd_s(0)) < _s_min) {
                        // gradient descent towards singularity
                        // scaled by the minimum of the force magnitude and condition number
                        // reduce velocity to zero as robot gets closer to singularity to avoid oscillations

                        // VectorXd singular_task_force = _task_range_s * _task_range_s.transpose() * (unit_mass_force + force_related_terms);
                        // double curr_singular_task_force = 
                        //     (_active_singularities[ind].u * _active_singularities[ind].u.transpose() * (unit_mass_force + force_related_terms)).norm();
                        double curr_singular_task_force = 
                            std::abs(_active_singularities[ind].u.transpose() * (unit_mass_force + force_related_terms));
                        double force_vel_scaling = std::clamp(curr_singular_task_force / _max_force_norm, 0.0, 1.0);
                        double condition_number_scaling = std::clamp(_active_singularities[ind].getConditionRatioWithMin(_s_min), 0.0, 1.0);  // starts at 1 at _s_min, then goes to 0 towards 0
                        double vel_scaling = std::min(force_vel_scaling, condition_number_scaling);

                        VectorXd q_des = curr_q - _type_1_step_size_control_towards_singularity * _active_singularities[ind].dsdq;
                        q_des = saturateBox(q_des, _q_lower, _q_upper);  // saturate within joint limits 

                        // compute velocity-saturated control
                        VectorXd dq_des = (_kp_type_1 / _kv_type_1) * (q_des - curr_q);
                        if (dq_des.norm() > vel_scaling * _type_1_max_vel_towards_singularity) {
                            dq_des = vel_scaling * _type_1_max_vel_towards_singularity * dq_des.normalized();
                        } 
                        unit_torques = - _kv_type_1 * (_robot->dq() - dq_des) * 1;

                    } else {
                        // damping, as the singular task force is non-zero (let this dictate control towards singularity)
                        unit_torques = - _kv_damping * _robot->dq();
                    }

                } else {

                    // command to move away from singularity
                    // in this case, prefer to move towards the entering conditions (q_prior) OR defined goal posture (if enabled)
                    _type_1_retracting = true;

                    {
                        // debug
                        // throw runtime_error("retract");
                    }

                    if (_active_singularities[ind].getConditionRatio(_s_min, _s_max, _svd_s(0)) < _s_min) {
                        
                        // velocity-saturated towards holding posture (entering posture) 
                        VectorXd dq_des = (_kp_type_1 / _kv_type_1) * (_q_prior - curr_q);
                        if (_use_goal_posture) {
                            dq_des = (_kp_type_1 / _kv_type_1) * (_q_goal_posture - curr_q);
                        }

                        // double curr_singular_task_force = 
                        //     (_active_singularities[ind].u * _active_singularities[ind].u.transpose() * (unit_mass_force + force_related_terms)).norm();
                        double curr_singular_task_force = 
                            std::abs(_active_singularities[ind].u.transpose() * (unit_mass_force + force_related_terms));

                        double vel_scaling = std::clamp(curr_singular_task_force / _max_force_norm, 0.0, 1.0);

                        if (dq_des.norm() > vel_scaling * _type_1_max_vel_away_from_singularity) {
                            dq_des = vel_scaling * _type_1_max_vel_away_from_singularity * dq_des.normalized();
                        } 
                        unit_torques = - _kv_type_1 * (_robot->dq() - dq_des);

                    } else {
                        // damping
                        unit_torques = - _kv_damping * _robot->dq();
                    }

                }

            } else if (_active_singularities[ind].type == TYPE_2_SINGULARITY) {
                // type 2 handling 
                double curr_singular_task_force = 
                            std::abs(_active_singularities[ind].u.transpose() * (unit_mass_force + force_related_terms));
                // if (_active_singularities[ind].is_degenerate) {
                //     curr_singular_task_force = 
                //         (_active_singularities[ind].U * _active_singularities[ind].U.transpose() * (unit_mass_force + force_related_terms)).norm();
                // }

                if (_active_singularities[ind].getConditionRatio(_s_min, _s_max, _svd_s(0)) < _s_min) {
                    // throw runtime_error("");

                    double force_dotted_singular_direction = std::abs(normalized_force_moment.transpose() * _active_singularities[ind].u);

                    // get majority element from past singular task torques 
                    VectorXi singular_task_torque_sign = majoritySign(_singular_task_torque_history, _dof);

                    // change direction if angle threshold is met 
                    for (int i = 0; i < _dof; ++i) {

                        // set direction to the direction of the current joint velocities
                        // _type_2_direction(i) = sign(_robot->dq()(i));

                        // set direction to the direction of the singular task torque
                        // _type_2_direction(i) = sign(_robot->dq()(i) + _dt * (_singular_task_torques(i) + _non_singular_task_torques(i)));
                        // _type_2_direction(i) = sign(_singular_task_torques(i) + _non_singular_task_torques(i));
                        // _type_2_direction(i) = sign(impedance_singular_task_torques(i));
                        _type_2_direction(i) = singular_task_torque_sign(i);
                        if (_type_2_direction(i) == 0) {
                            _type_2_direction(i) = sign(_robot->dq()(i));
                        }

                        // direction change
                        if (std::abs(curr_q(i) - _q_upper(i)) < _type_2_angle_threshold) {
                            _type_2_direction(i) = - 1;
                        } else if (std::abs(curr_q(i) - _q_lower(i)) < _type_2_angle_threshold) {
                            _type_2_direction(i) = 1;
                        } 
                    }

                    // double force_dotted_singular_direction = std::abs(normalized_force_moment.transpose() * _active_singularities[ind].u);
                    double force_vel_scaling = std::clamp(curr_singular_task_force / _max_force_norm, 0.0, 1.0);
                    double scaled_velocity_magnitude = std::min(force_dotted_singular_direction, force_vel_scaling);
                    // double scaled_velocity_magnitude = force_vel_scaling;

                    VectorXd dq_des = _type_2_direction.normalized() * _type_2_max_vel * scaled_velocity_magnitude;
                    // VectorXd q_des = saturateBox(curr_q + _type_2_direction.normalized() * _type_2_max_vel * scaled_velocity_magnitude * _dt, _q_lower, _q_upper);
                    unit_torques = - _kv_type_2 * (_robot->dq() - dq_des);

                    // velocity saturation in the singular joint space metric 

                    // {
                    //     // debug
                    //     std::cout << "robot velocity: " << _robot->dq().transpose() << "\n";
                    //     std::cout << "scaled velocity magnitude: " << scaled_velocity_magnitude << "\n";
                    //     std::cout << "kv type 2: " << _kv_type_2 << "\n";
                    //     std::cout << "type 2 unit torque: " << unit_torques.transpose() << "\n";
                    //     std::cout << "dq des: " << dq_des.transpose() << "\n";
                    //     std::cout << "type 2 direction: " << _active_singularities[ind].u.transpose() << "\n";
                    // }

                } else {
                    // damping
                    unit_torques = - _kv_damping * _robot->dq();
                }
            }

            // reset 
            _robot->setQ(curr_q);
            _robot->updateKinematics();
            // _robot->updateModel();

            // compute projection
            // projected_forces(ind) = _active_singularities[ind].v.transpose() * unit_torques;

            // compute torques and forward compensate disturbance torques for multi-singularity hierarchy
            MatrixXd sjs_jacobian = _active_singularities[ind].v.transpose() * N_prec;
            Sai2Model::OpSpaceMatrices op_matrices = _robot->operationalSpaceMatrices(sjs_jacobian);
            MatrixXd Lambda_sjs_modified = op_matrices.Lambda;
            if (_dynamic_decoupling_type == BOUNDED_INERTIA_ESTIMATES) { 
                MatrixXd Lambda_inv_BIE = sjs_jacobian * _M_inv_BIE_SINGULARITY * sjs_jacobian.transpose();
                // Lambda_sjs_modified = Lambda_inv_BIE.inverse();
                Lambda_sjs_modified = Sai2Model::computePseudoInverse(Lambda_inv_BIE, _s_abs_tol);
            } 
            singular_joint_task_torques -= (MatrixXd::Identity(_dof, _dof) - op_matrices.N).transpose() * singular_joint_task_torques;  // forward compensation 
            singular_joint_task_torques += sjs_jacobian.transpose() * Lambda_sjs_modified * _active_singularities[ind].v.transpose() * unit_torques;
            N_prec = op_matrices.N * N_prec;

            // {
            //     // debug
            //     std::cout << "Lambda sjs modified: \n" << Lambda_sjs_modified << "\n";
            //     std::cout << "unit torques: \n" << unit_torques.transpose() << "\n";
            //     std::cout << "singular joint task torques: \n" << singular_joint_task_torques.transpose() << "\n";
            // }

        }

        _joint_strategy_torques = singular_joint_task_torques;

        // // compute joint strategy torques 
        // MatrixXd sjs_jacobian = active_singular_joint_space.transpose() * _N_sjs_init;
        // // sjs_jacobian = _posture_projected_jacobian;

        // {
        //     // debug rank
        //     // std::cout << "active sjs jacobian rank: " << matrixRank(sjs_jacobian) << "\n";
        //     std::cout << "sjs jacobian: \n" << sjs_jacobian << "\n";
        //     std::cout << "projected forces: \n" << projected_forces.transpose() << "\n";
        //     // if (!isOrthonormal(sjs_jacobian)) {
        //         // throw runtime_error("ortho error");
        //     // }
        // }

        // Sai2Model::OpSpaceMatrices op_matrices = _robot->operationalSpaceMatrices(sjs_jacobian);
        // MatrixXd Lambda_sjs_modified = op_matrices.Lambda;
        // if (_dynamic_decoupling_type == BOUNDED_INERTIA_ESTIMATES) { 
        //     MatrixXd Lambda_inv_BIE = sjs_jacobian * _M_inv_BIE_SINGULARITY * sjs_jacobian.transpose();
        //     Lambda_sjs_modified = Lambda_inv_BIE.inverse();
        //     // Lambda_sjs_modified = Sai2Model::computePseudoInverse(Lambda_inv_BIE, _s_max);
        // } 
        // _joint_strategy_torques = sjs_jacobian.transpose() * Lambda_sjs_modified * projected_forces;

        // {
        //     // debug
        //     std::cout << "blending matrix: " << _alpha_blending_matrix << "\n";
        //     // std::cout << "lambda sjs: \n" << Lambda_sjs_modified << "\n";
        //     std::cout << "singular task torques: " << _singular_task_torques.transpose() << "\n";
        //     std::cout << "joint strategy torques: " << _joint_strategy_torques.transpose() << "\n";
        //     std::cout << "---\n";

        //     if (_joint_strategy_torques.norm() > 5) {
        //         // throw runtime_error("");
        //     }
        // }

        return _non_singular_task_torques + _singular_task_torques + _joint_strategy_torques;
    }
}

}  // namespace