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

    std::ostream& operator<<(std::ostream& os,
                            const std::vector<double>& v) {
        os << "[ ";
        for (double x : v) {
            os << x << " ";
        }
        os << "]";
        return os;
    }

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
                result[i] = 0;  
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

    Eigen::MatrixXd vstack(const std::vector<Eigen::MatrixXd>& mats) {
        if (mats.empty()) {
            return Eigen::MatrixXd(0, 0);
        }

        const int cols = mats[0].cols();
        int total_rows = 0;

        for (const auto& m : mats) {
            if (m.cols() != cols) {
                throw std::runtime_error("vstack: column size mismatch");
            }
            total_rows += m.rows();
        }

        Eigen::MatrixXd out(total_rows, cols);

        int row = 0;
        for (const auto& m : mats) {
            out.middleRows(row, m.rows()) = m;
            row += m.rows();
        }

        return out;
    }

    Eigen::VectorXd vstack(const std::vector<Eigen::VectorXd>& vecs) {
        if (vecs.empty()) {
            return Eigen::VectorXd(0);
        }

        int total_size = 0;
        for (const auto& v : vecs) {
            total_size += v.size();
        }

        Eigen::VectorXd out(total_size);

        int offset = 0;
        for (const auto& v : vecs) {
            out.segment(offset, v.size()) = v;
            offset += v.size();
        }

        return out;
    }

    double getRatio(const double min, const double max, const double val) {
        return (val - min) / (max - min);
    }

    // Sort v in descending order and apply same permutation to vecs
    void sortDescendingWithPermutation(
        VectorXd& v,
        std::vector<VectorXd>& vecs)
    {
        const int n = v.size();

        // 1. Create index vector
        std::vector<int> idx(n);
        std::iota(idx.begin(), idx.end(), 0);

        // 2. Sort indices by v (descending)
        std::sort(idx.begin(), idx.end(),
                [&](int i, int j) {
                    return v(i) > v(j);
                });

        // 3. Apply permutation to v
        VectorXd v_sorted(n);
        for (int i = 0; i < n; ++i)
            v_sorted(i) = v(idx[i]);
        v = v_sorted;

        // 4. Apply permutation to each VectorXd in vecs
        for (auto& x : vecs) {
            assert(x.size() == n && "Vector size mismatch");
            VectorXd x_sorted(n);
            for (int i = 0; i < n; ++i)
                x_sorted(i) = x(idx[i]);
            x = x_sorted;
        }
    }

    std::vector<VectorXd> barycenterCoefficients(int n) {
        std::vector<VectorXd> coeffs;
        coeffs.reserve(n + 1 + n * (n - 1) / 2);

        // 1. Vertices (standard basis)
        for (int i = 0; i < n; ++i) {
            VectorXd v = VectorXd::Zero(n);
            v(i) = 1.0;
            coeffs.push_back(v);
        }

        // 2. Overall barycenter
        VectorXd center = VectorXd::Constant(n, 1.0 / n);
        coeffs.push_back(center);

        // 3. Pairwise barycenters
        for (int i = 0; i < n; ++i) {
            for (int j = i + 1; j < n; ++j) {
                VectorXd v = VectorXd::Zero(n);
                v(i) = 0.5;
                v(j) = 0.5;
                coeffs.push_back(v);
            }
        }

        return coeffs;
    }

}

namespace Sai2Primitives {

/*
    Base classification functions
*/
bool SingularityHandler::classifySingularityType(const std::vector<MatrixXd>& kinematic_hessian,
                                                 const VectorXd& u,
                                                 const VectorXd& v) {
    double deviation = std::abs(u.transpose() * getSecondOrderExpansion(kinematic_hessian, v));
    // double deviation = std::abs(v.dot(getProjectedHessian(kinematic_hessian, u) * v));

    // {
    //     // debug 
    //     std::cout << "classify u: " << u.transpose() << "\n";
    //     std::cout << "classify v: " << v.transpose() << "\n";
    //     std::cout << "classify deviation: " << deviation << "\n";
    // }

    if (deviation > _type_1_tol) {
        return true;
    } else {
        return false;
    }
}

VectorXd SingularityHandler::getSecondOrderExpansion(const std::vector<MatrixXd>& kinematic_hessian,
                                                     const VectorXd& dq) {
    const int dof  = dq.size();
    const int task = kinematic_hessian[0].rows();
    Eigen::VectorXd dx2 = Eigen::VectorXd::Zero(task);
    for (int j = 0; j < dof; ++j) {
        dx2.noalias() += dq(j) * (kinematic_hessian[j] * dq);
    }
    return dx2;
}

MatrixXd SingularityHandler::getProjectedHessian(const std::vector<MatrixXd>& kinematic_hessian, //  dof x (6 x dof)
                                                 const VectorXd& direction) {
    MatrixXd projected_hessian = MatrixXd::Zero(_robot->dof(), _robot->dof());
    for (int i = 0; i < _robot->dof(); ++i) {
        projected_hessian.row(i) = direction.transpose() * kinematic_hessian[i];
    }
    return projected_hessian;
}

/*
    Functions to solve the revised degenerate handling problem
*/
std::vector<MatrixXd> SingularityHandler::getTaskProjectedHessian(const std::vector<MatrixXd>& kinematic_hessian,
                                                                  const MatrixXd& U,
                                                                  const MatrixXd& V) {
    // compute sum of squares hessian
    int task_dimension = U.rows();
    std::vector<Eigen::MatrixXd> sos_hessians(U.cols(), Eigen::MatrixXd::Zero(task_dimension, task_dimension));
    // std::vector<Eigen::MatrixXd> sos_hessians(U.cols(), Eigen::MatrixXd::Zero(_robot->dof(), _robot->dof()));
    for (int j = 0; j < U.cols(); ++j) {
        for (int i = 0; i < _robot->dof(); ++i) {
            sos_hessians[j].noalias() += U(i, j) * kinematic_hessian[i];
        }
        sos_hessians[j] = V.transpose() * sos_hessians[j] * V;
    }
    return sos_hessians;
}

std::pair<bool, VectorXd> SingularityHandler::checkBasisForType1(const std::vector<MatrixXd>& kinematic_hessian,
                                                                 const MatrixXd& U,
                                                                 const MatrixXd& V) {

    // non-convex approach requires rough simplex search across V
    std::vector<VectorXd> sampling_coefficients = barycenterCoefficients(V.cols());
    MatrixXd U_basis = U * U.transpose();

    for (int i = 0; i < sampling_coefficients.size(); ++i) {
        double deviation = (U_basis * getSecondOrderExpansion(kinematic_hessian, V * sampling_coefficients[i])).norm();
        if (deviation > _type_1_tol) {
            return std::make_pair(true, sampling_coefficients[i]);
        }
    }

    return std::make_pair(false, VectorXd::Zero(V.cols()));
}

double SingularityHandler::objective(const std::vector<double> &x, std::vector<double> &grad, void* f_data) {

    auto* self = static_cast<SingularityHandler*>(f_data);

    VectorXd v = VectorXd::Zero(self->_dof);
    for (int i = 0; i < x.size(); ++i) {
        v += x[i] * self->_nl_opt_data->basis[i];
    }
    v.normalize();

    // compute matching u direction
    // self->_robot->setQ(self->_nl_opt_data->starting_q + self->_nl_opt_data->perturb_step_size * dq);
    // self->_robot->updateKinematics();
    // Vector3d deviation = self->_robot->position(self->_link_name, self->_compliant_frame.translation()) - 
    //                         self->_nl_opt_data->starting_position - 
    //                         self->getLinearTaylorExpansion(self->_nl_opt_data->projected_jacobian, dq).head(3);
    //                         // (self->_nl_opt_data->projected_jacobian * dq).head(3);

    // compute matching singular direction
    VectorXd u = vectorFromBasis(x, self->_nl_opt_data->singular_task_range).normalized();
    if (self->_nl_opt_data->flag_zero_value) {
        // get u_proj from the deviation, as {u, v} are individually rotated
        self->_robot->setQ(self->_nl_opt_data->starting_q + self->_nl_opt_data->perturb_step_size * v);
        self->_robot->updateKinematics();
        u.head(3) = self->_robot->position(self->_link_name, self->_compliant_frame.translation()) - self->_nl_opt_data->starting_position; 
        u.tail(3) = Sai2Model::orientationError(self->_robot->rotation(self->_link_name, self->_compliant_frame.linear()), self->_nl_opt_data->starting_orientation);

        // project u_proj into the current task range
        u = (self->_nl_opt_data->singular_task_range * self->_nl_opt_data->singular_task_range.transpose() * u).normalized();
    }

    double deviation = std::abs(u.transpose() * self->getSecondOrderExpansion(self->_dJdq, v));

    return deviation;

}

double SingularityHandler::equality(const std::vector<double> &x, std::vector<double> &grad, void* f_data) {
    double sum = 0;
    for (int i = 0; i < x.size(); ++i) {
        sum += x[i];
    }
    return 1 - sum;

    // double sum = 0;
    // for (int i = 0; i < x.size(); ++i) {
    //     sum += x[i] * x[i];
    // }
    // return 1 - sqrt(sum);
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
    _max_joint_vel_sf = VectorXd::Ones(_dof);
    auto joint_limits = _robot->jointLimits();
    for (int i = 0; i < joint_limits.size(); ++i) {
        _q_upper(i) = joint_limits[i].position_upper - DefaultParameters::joint_limit_buffer;
        _q_lower(i) = joint_limits[i].position_lower + DefaultParameters::joint_limit_buffer;
        _dq_max(i) = joint_limits[i].velocity;
        _joint_midrange(i) = 0.5 * (joint_limits[i].position_lower + joint_limits[i].position_upper);
        _tau_upper(i) = joint_limits[i].effort;
        _tau_lower(i) = - joint_limits[i].effort;
        _max_joint_vel_sf(i) = 1;
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
    _type_2_min_force = DefaultParameters::type_2_min_force;

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
    _num_zone_2_singularities = 0;
    _prev_num_zone_2_singularities = 0;

    // debug
    _force_dotted_singular_direction = 0;
    _dq_des = VectorXd::Zero(_dof);
    _enable_joint_strategy = true;
    _dsdq_norm = 1;
    _min_magnitude_thresh = 0.5;
    _type_2_vel_scheduling = DefaultParameters::type_2_vel_scheduling;

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
        // _nl_opt[i]->add_equality_constraint(equality, this);

        _nl_opt[i]->set_xtol_rel(DefaultParameters::xtol_rel);  // 1e-12 default
        _nl_opt[i]->set_ftol_rel(DefaultParameters::ftol_rel);
        _nl_opt[i]->set_xtol_abs(DefaultParameters::xtol_abs);
        _nl_opt[i]->set_maxtime(DefaultParameters::max_time * 1e-3);
        // _nl_opt[i]->set_maxeval(5);
        _nl_opt[i]->set_initial_step(initial_step_size);
    }

    _nl_opt_data = new OptimData(DefaultParameters::type_1_step_size_classification_towards_singularity);

}

void SingularityHandler::updateTaskModel(MatrixXd& projected_jacobian, const MatrixXd& N_prec) {
    
    // task range decomposition
    _projected_jacobian = projected_jacobian;

    // compute eigen-decomposition of task inertia matrix 
    _eig_solver.compute(projected_jacobian * _robot->MInv() * projected_jacobian.transpose());
    _eig_values = _eig_solver.eigenvalues().reverse();  // switch to descending order 
    _eig_vectors = _eig_solver.eigenvectors().rowwise().reverse();

    // compute svd of jacobian
    _J_svd.compute(projected_jacobian, ComputeThinU | ComputeThinV);
    _svd_U = _J_svd.matrixU();
    _svd_s = _J_svd.singularValues();  // descending order 
    _svd_V = _J_svd.matrixV();   

    // compute jacobian derivatives 
    _dJdq = _robot->getJacobianDerivative(_link_name, _compliant_frame.translation());  // dof x (6 x dof)

    // compute singular task range 
    _fully_singular_task = false;
    _is_in_singularity = false;

    if (_eig_values(0) < _s_abs_tol) {
        if (_verbose) {
            std::cout << "WARNING: Fully Singular Task\n";
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
        _Lambda_s = Sai2Model::computePseudoInverse(
                                  _projected_jacobian_s *
                                  _robot->MInv() * 
                                  _projected_jacobian_s.transpose(), _s_abs_tol);
        _svd_s_singular = _svd_s;
        _eig_s_singular = _eig_values;

        // sjs info
        _prev_num_singularities = _num_singularities;
        _num_singularities = _task_rank;
        _alpha_vec = VectorXd::Zero(_task_rank);
        _condition_ratio_vec = VectorXd::Zero(_task_rank);

        // update flags 
        _is_in_singularity = true;

    } else if (_task_rank == 1) {
        if (_verbose) {
            std::cout << "WARNING: Fully Non-Singular 1 DOF Task\n";
        }

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
        _eig_s_singular = VectorXd::Zero(_task_rank);

        // sjs info
        _prev_num_singularities = _num_singularities;
        _num_singularities = 0;
        _alpha_vec = VectorXd::Zero(_task_rank);
        _condition_ratio_vec = VectorXd::Zero(_task_rank);

        // update flags 
        _is_in_singularity = false;

    } else {

        // check up to _task_rank, as zero eigenvalues after 
        for (int i = 1; i < _task_rank; ++i) {
            if (_eig_values(i) < _s_max) {

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
                // _projected_jacobian_s = _task_range_s.transpose() * projected_jacobian * _N_ns;
                _projected_jacobian_s = _task_range_s.transpose() * projected_jacobian;
                _Lambda_s = (_projected_jacobian_s * _robot->MInv() * _projected_jacobian_s.transpose()).inverse();
                // _Lambda_s = Sai2Model::computePseudoInverse(_projected_jacobian_s * _robot->MInv() * _projected_jacobian_s.transpose(), _s_abs_tol);
                _svd_s_singular = _svd_s.tail(_task_rank - i);
                _eig_s_singular = _eig_values.tail(_task_rank - i);

                // SJS information 
                _prev_num_singularities = _num_singularities;
                _num_singularities = _task_range_s.cols();
                _alpha_vec = VectorXd::Zero(_num_singularities);
                _condition_ratio_vec = VectorXd::Zero(_num_singularities);

                // update flags 
                _is_in_singularity = true;
                break;

            } else if (i == _task_rank - 1) {

                if (_verbose) {
                    std::cout << "Fully Non-Singular Task\n";;
                }

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
                _eig_s_singular = VectorXd::Zero(_task_rank);

                // sjs info
                _prev_num_singularities = _num_singularities;
                _num_singularities = 0;
                _alpha_vec = VectorXd::Zero(_task_rank);
                _condition_ratio_vec = VectorXd::Zero(_task_rank);

                // update flags 
                _is_in_singularity = false;
            }
        }
    }

    // exit transition check
    if (_num_singularities < _prev_num_singularities) {
        _singularity_exit_transition = true;
        _singularity_enter_transition = false;
    } else if (_num_singularities > _prev_num_singularities) {
        _singularity_enter_transition = true;
        _singularity_exit_transition = false;
    } else {
        _singularity_enter_transition = false;
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
        _N = op_space_matrices.N * _N_ns;
        _N_sjs_init = _N_ns * N_prec;
    }

    // dynamic decoupling 
    switch (_dynamic_decoupling_type) {
        case FULL_DYNAMIC_DECOUPLING: {
            _Lambda_ns_modified = _Lambda_ns;
            _Lambda_s_modified = _Lambda_s;
            break;
        }

        case IMPEDANCE: {
            _Lambda_ns_modified = MatrixXd::Identity(_task_range_ns.cols(), _task_range_ns.cols());
            _Lambda_s_modified = MatrixXd::Identity(_task_range_s.cols(), _task_range_s.cols());
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

            break;
        }

        default: {
            _Lambda_s_modified = _Lambda_s;
            _Lambda_ns_modified = _Lambda_ns;
            break;
        }
	}

    classifySingularity(projected_jacobian, _task_range_s, _joint_task_range_s);
}

void SingularityHandler::classifySingularity(const MatrixXd& projected_jacobian,
                                             const MatrixXd& singular_task_range,
                                             const MatrixXd& singular_joint_task_range) {

    // {
    //     // debug 
    //     std::cout << "\n----------------------\nClassification: \n----------------------\n";
    // }

    // memory of entering singularity state
    if (!_is_in_singularity) {
        _q_prior = _robot->q();
        _dq_prior = _robot->dq();
    }

    _active_singularities = {};
    _degenerate_indices = {};

    if (!_is_in_singularity) {
        return;
    }

    VectorXd curr_q = _robot->q();
    Vector3d curr_pos = _robot->position(_link_name, _compliant_frame.translation());
    Matrix3d curr_ori = _robot->rotation(_link_name, _compliant_frame.linear());

    // handle degenerate singularities
    if (_is_in_singularity) {
        _degenerate_indices = findCloseGroups(_svd_s_singular, _degenerate_singular_value_spacing);
    } 

    _degenerate_singularities.clear();  // clear map
    _degenerate_singular_task_range = {};
    _degenerate_singular_joint_task_range = {};
    _degenerate_singular_values = {};  // track degenerate singular values for _active_singularity handling priority
    _degenerate_eigen_values = {};

    if (_degenerate_indices.size() > 0) {

        // store degenerate task ranges 
        for (auto group : _degenerate_indices) {
            _degenerate_singular_task_range.push_back(collectColumns(singular_task_range, group));
            _degenerate_singular_joint_task_range.push_back(collectColumns(singular_joint_task_range, group));
            _degenerate_singular_values.push_back(collectColumns(_svd_s_singular, group));
            _degenerate_eigen_values.push_back(collectColumns(_eig_s_singular, group));
        }

        // if degenerate, then perform search in singular joint space 
        // excite the boundaries of the singular joint space to determine if type 1 singularity exists 
        // if no motions for boundaries, then all type 2 singularity
        // if there are motion, then at least one type 1 singularity exists
        // perform search over the coefficients of the singular joint space to find the direction of maximum deviation (via optimization)
        // get {u, v} for each type 1 from this search, removing the singular joint space contribution for each successive search
        // type 2 singularities are in the remaining space, with gram-schmidt decomposition for the {u} directions used in sjs strategy
        
        int group_id = 0;
        for (auto group : _degenerate_indices) {

            // check for type 1
            auto [exists_type_1_singularity, basis_coefficients] = 
                checkBasisForType1(_dJdq, _degenerate_singular_task_range[group_id], _degenerate_singular_joint_task_range[group_id]);
                
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

                bool remaining_type_1_singularity = true;
                while (remaining_type_1_singularity) {

                    // update optimization parameters
                    _nl_opt_data->setData(matrixToColumnVectors(curr_singular_joint_task_range), 
                                          curr_singular_task_range, 
                                          projected_jacobian, 
                                          curr_pos, 
                                          curr_ori, 
                                          curr_q,
                                          _degenerate_singular_values[group_id].maxCoeff() < _type_1_search_tol);

                    // nlopt routine
                    int n_coefficients = curr_singular_joint_task_range.cols();
                    std::vector<double> sjs_coefficients;
                    for (int i = 0; i < n_coefficients; ++i) {
                        // sjs_coefficients.push_back(1.0 / n_coefficients);
                        sjs_coefficients.push_back(basis_coefficients[i]);
                    }

                    {
                        // debug
                        std::cout << "nm initial vector: " << vectorFromBasis(sjs_coefficients, curr_singular_task_range).transpose() << "\n";
                        std::cout << "nm initial sol: " << sjs_coefficients << "\n";
                        VectorXd _u = singular_task_range.col(0);
                        VectorXd _v = singular_joint_task_range.col(0); 
                        std::cout << "nm initial deviation: " << std::abs(_v.dot(getProjectedHessian(_dJdq, _u) * _v)) << "\n";
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
                        std::cout << "Solution: ";
                        for (auto val : sjs_coefficients) {
                            std::cout << val << ", ";
                        }
                        std::cout << "\n";
                        // auto v = vectorFromBasis(sjs_coefficients, curr_singular_joint_task_range);
                        // auto u = vectorFromBasis(sjs_coefficients, curr_singular_task_range);
                        // std::cout << "nm vector: " << v.transpose() << "\n";
                        std::cout << "deviation: " << optimal_value << "\n";
                        // std::cout << "deviation from projected: " << std::abs(v.dot(getProjectedHessian(_dJdq, u) * v)) << "\n";
                        // std::cout << (curr_singular_task_range * curr_singular_task_range.transpose() * v.dot(getProjectedHessian(_dJdq, u) * v)).norm() << "\n";
                        // throw runtime_error("");
                    }

                    VectorXd v_proj = vectorFromBasis(sjs_coefficients, curr_singular_joint_task_range).normalized();
                    VectorXd u_proj = curr_singular_task_range.col(0);  // placeholder
                    // if (_degenerate_singular_values[group_id].minCoeff() / _svd_s(0) > _type_1_search_tol) {
                    if (_degenerate_singular_values[group_id].maxCoeff() > _type_1_search_tol) {
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

                    if (curr_singular_joint_task_range.cols() == 1) {
                        // normal classification and break
                        if (classifySingularityType(_dJdq, curr_singular_task_range, curr_singular_joint_task_range)) {
                            curr_type_1_singular_task_range.push_back(curr_singular_task_range);
                            curr_type_1_singular_joint_task_range.push_back(curr_singular_joint_task_range);

                            curr_singular_task_range = MatrixXd::Zero(1, 0);
                            curr_singular_joint_task_range = MatrixXd::Zero(1, 0);
                            remaining_type_1_singularity = false;
                        } else {
                            remaining_type_1_singularity = false;
                        }
                    }

                }

                /*
                    Assign to active singularities 
                */

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
                    // double condition_ratio = _degenerate_singular_values[group_id].minCoeff() / _svd_s(0);
                    // if (condition_ratio < _type_1_search_tol) {
                    if (_degenerate_singular_values[group_id].maxCoeff() < _type_1_search_tol) {
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
                                                                _degenerate_eigen_values[group_id](i),
                                                                dsdq_toward_singularity,
                                                                u_toward_singularity,
                                                                SingularityType::TYPE_1_SINGULARITY,
                                                                true));
                }

                // type 2 singularities 
                for (int i = 0; i < curr_singular_task_range.cols(); ++i) {

                    // recompute dsdq for type 2 singularities 
                    VectorXd dsdq = VectorXd::Zero(_dof);
                    for (int j = 0; j < _dof; ++j) {
                        dsdq(j) = curr_singular_task_range.col(i).transpose() * _dJdq[j] * curr_singular_joint_task_range.col(i);
                    }    
                    VectorXd u = curr_singular_task_range.col(i);
                    VectorXd u_toward_singularity = u;  // placeholder
                    VectorXd dsdq_toward_singularity = dsdq;  // placeholder

                    _active_singularities.push_back(Singularity(curr_singular_task_range.col(i), 
                                                                curr_singular_joint_task_range.col(i), 
                                                                _degenerate_singular_values[group_id](i + curr_type_1_singular_task_range.size()),  // assign rough order
                                                                _degenerate_eigen_values[group_id](i + curr_type_1_singular_task_range.size()),
                                                                dsdq_toward_singularity,
                                                                u_toward_singularity,
                                                                SingularityType::TYPE_2_SINGULARITY,
                                                                true));
                }

            } else {

                // all type 2 singularities, use entire singular task space and singular joint space

                // active singularity classification
                for (int i = 0; i < _degenerate_singular_task_range[group_id].cols(); ++i) {

                    // recompute dsdq for type 2 singularities 
                    VectorXd dsdq = VectorXd::Zero(_dof);
                    for (int j = 0; j < _dof; ++j) {
                        dsdq(j) = _degenerate_singular_task_range[group_id].col(i).transpose() * _dJdq[j] * _degenerate_singular_joint_task_range[group_id].col(i);
                    }    
                    VectorXd u = _degenerate_singular_task_range[group_id].col(i);
                    VectorXd u_toward_singularity = u;  // placeholder
                    VectorXd dsdq_toward_singularity = dsdq;  // placeholder

                    _active_singularities.push_back(Singularity(_degenerate_singular_task_range[group_id].col(i), 
                                                                _degenerate_singular_joint_task_range[group_id].col(i), 
                                                                _degenerate_singular_values[group_id](i), 
                                                                _degenerate_eigen_values[group_id](i),
                                                                dsdq_toward_singularity,
                                                                u_toward_singularity,
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
            bool is_type_1 = classifySingularityType(_dJdq, u, v);

            // if (is_type_1 && (_svd_s_singular(i) / _svd_s(0) < _type_1_search_tol)) {
            if (is_type_1 && (_svd_s_singular(i) < _type_1_search_tol)) {
                // perform line search
                std::tie(u_toward_singularity, dsdq_toward_singularity) = 
                    getTowardSingularityDirection(curr_q, curr_pos, u, dsdq, _type_1_step_size_for_line_search);

            } else {
                // perform gradient descent 
                VectorXd q_toward_singularity = curr_q - _type_1_step_size_classification_towards_singularity * dsdq_toward_singularity;
                _robot->setQ(q_toward_singularity);
                _robot->updateKinematics();
                Vector3d delta_vector = _robot->position(_link_name, _compliant_frame.translation()) - curr_pos;
                if (delta_vector.dot(u.head(3)) < 0) {
                    u_toward_singularity = - u;
                }

            }

            if (is_type_1) {
                _active_singularities.push_back(Singularity(u, v, _svd_s_singular(i), _eig_s_singular(i), dsdq_toward_singularity, u_toward_singularity, SingularityType::TYPE_1_SINGULARITY, false));
            } else {
                _active_singularities.push_back(Singularity(u, v, _svd_s_singular(i), _eig_s_singular(i), dsdq_toward_singularity, u_toward_singularity, SingularityType::TYPE_2_SINGULARITY, false));
            }

        }
    }

    // reset robot
    _robot->setQ(curr_q);
    _robot->updateKinematics();

    // collect for _task_range_ns_with_blending
    // std::vector<VectorXd> task_range_ns_with_blending = {};
    // for (auto singularity : _active_singularities) {
    //     // if (singularity.getConditionRatio(_s_min, _s_max, singularity.sigma) > _s_min) {
    //     if (singularity.lambda > _s_max) {
    //         task_range_ns_with_blending.push_back(singularity.u);
    //     }
    // }
    // _task_range_ns_with_blending = vectorsToMatrix(task_range_ns_with_blending);

    _task_range_ns_with_blending = _task_range_ns; 

}

VectorXd SingularityHandler::computeTorques(const VectorXd& unit_mass_force, const VectorXd& force_related_terms) {
    // if (_verbose) {
    if (true) {
        if (_is_in_singularity && _enforce_handling_strategy) {
            int i = 0;
            for (auto singularity : _active_singularities) {
                std::cout << "Singularity " << i << ": " << singularity.type << "\n";
                std::cout << "u: " << singularity.u.transpose() << "\n";
                std::cout << "v: " << singularity.v.transpose() << "\n";
                ++i;
            }
            std::cout << "\n";
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

        // // compute singular task torques 
        // if (_is_in_singularity) {
        //     _singular_task_torques = _projected_jacobian_s.transpose() * 
        //                                 (_Lambda_s_modified * _alpha_blending_matrix * _task_range_s.transpose() * unit_mass_force + 
        //                                 _task_range_s.transpose() * force_related_terms);
        // }

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

        // VectorXd combined_torque_vector = VectorXd::Zero(_robot->dof());
        // {
        //     // debug 
        //     MatrixXd J_stacked(6, _robot->dof());
        //     J_stacked.topRows(_projected_jacobian_ns.rows()) = _projected_jacobian_ns;
        //     J_stacked.bottomRows(_projected_jacobian_s.rows()) = _projected_jacobian_s;
        //     MatrixXd Lambda_stacked = _robot->taskInertiaMatrix(J_stacked);
        //     VectorXd control_force(6);
        //     control_force.topRows(_projected_jacobian_ns.rows()) = _task_range_ns.transpose() * (unit_mass_force + force_related_terms); 
        //     control_force.bottomRows(_projected_jacobian_s.rows()) = _alpha_blending_matrix * _task_range_s.transpose() * unit_mass_force + 
        //                                                                     _task_range_s.transpose() * force_related_terms;
        //     combined_torque_vector = J_stacked.transpose() * Lambda_stacked * control_force;

        //     std::cout << "Lambda stacked: \n" << Lambda_stacked << "\n";
        // }

        // VectorXd impedance_singular_task_torques = 
        //     _projected_jacobian_s.transpose() * _Lambda_s_modified * _task_range_s.transpose() * (unit_mass_force + force_related_terms);

        // VectorXi sign_impedance_singular_task_torques(_dof);
        // for (int i = 0; i < _dof; ++i) {
        //     sign_impedance_singular_task_torques(i) = sign(impedance_singular_task_torques(i));
        // }
        // _singular_task_torque_history.push_back(sign_impedance_singular_task_torques);
        // // pop oldest if greater than buffer size
        // if (_singular_task_torque_history.size() > _type_2_task_torque_buffer_size) {
        //     _singular_task_torque_history.pop_front();
        // }

        // VectorXi sign_non_singular_task_torques(_dof);
        // for (int i = 0; i < _dof; ++i) {
        //     sign_non_singular_task_torques(i) = sign(_non_singular_task_torques(i));
        // }
        // _non_singular_task_torque_history.push_back(sign_non_singular_task_torques);
        // // pop oldest if greater than buffer size
        // if (_non_singular_task_torque_history.size() > _type_2_task_torque_buffer_size) {
        //     _non_singular_task_torque_history.pop_front();
        // }

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

        // {
        //     // debug
        //     // if (_verbose) {
        //         int cnt = 0;
        //         for (auto singularity : _active_singularities) {
        //             std::cout << "Singularity " << cnt << ": " << singularity.type << "\n";
        //             std::cout << "Direction: " << singularity.u.transpose() << "\n";
        //             std::cout << "Joint direction: " << singularity.v.transpose() << "\n";
        //         }
        //     // }
        // }

        std::vector<MatrixXd> sjs_jacobian_list = {};
        std::vector<VectorXd> torque_list = {};
        std::vector<MatrixXd> lambda_list = {};

        for (auto ind : sorted_indices) {

            active_singular_joint_space.col(ind) = _active_singularities[ind].v;  // collect singular joint space basis

            if (_active_singularities[ind].type == TYPE_1_SINGULARITY) {
                // type 1 handling
                // double motion_toward_singularity = 
                //     _active_singularities[ind].u_toward_singularity.head(3).dot((unit_mass_force + force_related_terms).head(3));

                double motion_toward_singularity = 
                    _active_singularities[ind].u_toward_singularity.dot((unit_mass_force + force_related_terms));

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

                    double curr_singular_task_force = 
                        std::abs(_active_singularities[ind].u.transpose() * (unit_mass_force + force_related_terms));
                    double force_vel_scaling = std::clamp(curr_singular_task_force / _max_force_norm, 0.0, 1.0);
                    double condition_number_scaling = 1 - std::pow((_s_max - _active_singularities[ind].lambda) / _s_max, 2);
                        // std::clamp(_active_singularities[ind].lambda / _s_max, 0.0, 1.0);  // starts at 1 at _s_min, then goes to 0 towards s = 0
                    double vel_scaling = std::min(force_vel_scaling, condition_number_scaling);

                    // VectorXd q_des = curr_q - _type_1_step_size_control_towards_singularity * _active_singularities[ind].dsdq;
                    // q_des = saturateBox(q_des, _q_lower, _q_upper);  // saturate within joint limits 

                    // compute velocity control
                    VectorXd dq_des = - _active_singularities[ind].v * _active_singularities[ind].v.transpose() * _active_singularities[ind].dsdq;
                    dq_des = vel_scaling * _type_1_max_vel_towards_singularity * dq_des.normalized();
                    unit_torques = - _kv_type_1 * (_robot->dq() - dq_des);

                } else {

                    // command to move away from singularity
                    // in this case, prefer to move towards the entering conditions (q_prior) OR defined goal posture (if enabled)
                    _type_1_retracting = true;
                        
                    // velocity-saturated towards holding posture (entering posture) 
                    VectorXd dq_des = (_kp_type_1 / _kv_type_1) * (_q_prior - curr_q);
                    if (_use_goal_posture) {
                        dq_des = (_kp_type_1 / _kv_type_1) * (_q_goal_posture - curr_q);
                    }

                    // project
                    dq_des = _active_singularities[ind].v * _active_singularities[ind].v.transpose() * dq_des;

                    double curr_singular_task_force = 
                        std::abs(_active_singularities[ind].u.transpose() * (unit_mass_force + force_related_terms));

                    double vel_scaling = std::clamp(curr_singular_task_force / _max_force_norm, 0.0, 1.0);

                    if (dq_des.norm() > vel_scaling * _type_1_max_vel_away_from_singularity) {
                        dq_des = vel_scaling * _type_1_max_vel_away_from_singularity * dq_des.normalized();
                    } 
                    unit_torques = - _kv_type_1 * (_robot->dq() - dq_des);

                }

            } else if (_active_singularities[ind].type == TYPE_2_SINGULARITY) {
               
                // type 2 handling 
                double curr_singular_task_force = 
                            std::abs(_active_singularities[ind].u.transpose() * (unit_mass_force + force_related_terms));

                double force_dotted_singular_direction = std::abs((normalized_force_moment.transpose() * _active_singularities[ind].u));
                _force_dotted_singular_direction = force_dotted_singular_direction;  // experimental logging

                // put force scaling through non-linear function for smoothing 
                force_dotted_singular_direction = std::clamp((1 - exp(-_type_2_vel_scheduling * force_dotted_singular_direction) / (1 - exp(-_type_2_vel_scheduling))), 0.0, 1.0);
                // force_dotted_singular_direction = std::clamp(std::pow(force_dotted_singular_direction, 1), 0.0, 1.0);
                // force_dotted_singular_direction = std::clamp(std::pow(force_dotted_singular_direction, 2), 0.0, 1.0);
                // force_dotted_singular_direction = std::clamp(std::pow(force_dotted_singular_direction, 3), 0.0, 1.0);
                // force_dotted_singular_direction = (1 - std::cos(M_PI * force_dotted_singular_direction)) / 2;

                // // get majority element from past singular task torques 
                // VectorXi singular_task_torque_sign = majoritySign(_singular_task_torque_history, _dof);
                // VectorXi non_singular_task_torque_sign = majoritySign(_non_singular_task_torque_history, _dof);

                // compute singular task torque component for this singularity with highest priority 
                // compute with pseudo-inverse of Lambda_s near singularity
                MatrixXd curr_projected_jacobian = _active_singularities[ind].u.transpose() * _projected_jacobian;
                MatrixXd curr_lambda = (curr_projected_jacobian * _robot->MInv() * curr_projected_jacobian.transpose()).inverse();
                    // Sai2Model::computePseudoInverse(curr_projected_jacobian * _robot->MInv() * curr_projected_jacobian.transpose(), _s_abs_tol);
                VectorXd singular_torque_component = 
                    curr_projected_jacobian.transpose() * curr_lambda * _active_singularities[ind].u.transpose() * unit_mass_force + 
                    curr_projected_jacobian.transpose() * _active_singularities[ind].u.transpose() * force_related_terms;

                // change direction if angle threshold is met 
                for (int i = 0; i < _dof; ++i) {

                    // set direction to the direction of the singular task torque
                    _type_2_direction(i) = sign(singular_torque_component(i));

                    // // direction change near joint limit 
                    // if (std::abs(curr_q(i) - _q_upper(i)) < _type_2_angle_threshold) {
                    //     _type_2_direction(i) = - 1;
                    // } else if (std::abs(curr_q(i) - _q_lower(i)) < _type_2_angle_threshold) {
                    //     _type_2_direction(i) = 1;
                    // } 
                }

                double scaled_velocity_magnitude = force_dotted_singular_direction;

                // if (curr_singular_task_force < _type_2_min_force) {
                //     scaled_velocity_magnitude = 0;
                // }

                VectorXd dq_des = _active_singularities[ind].v * _active_singularities[ind].v.transpose() * _type_2_direction;
                dq_des = _type_2_max_vel * scaled_velocity_magnitude * dq_des.normalized();

                unit_torques = - _kv_type_2 * (_robot->dq() - dq_des);

                _dsdq_norm = _active_singularities[ind].dsdq.norm();

                {
                //     // debug
                //     std::cout << "robot velocity: " << _robot->dq().transpose() << "\n";
                //     std::cout << "scaled velocity magnitude: " << scaled_velocity_magnitude << "\n";
                //     std::cout << "kv type 2: " << _kv_type_2 << "\n";
                //     std::cout << "type 2 unit torque: " << unit_torques.transpose() << "\n";
                    std::cout << "dq des: " << dq_des.transpose() << "\n";
                //     std::cout << "type 2 direction: " << _active_singularities[ind].u.transpose() << "\n";
                }

            }

            // reset 
            // _robot->setQ(curr_q);
            // _robot->updateKinematics();
            // _robot->updateModel();

            // compute projection
            // projected_forces(ind) = _active_singularities[ind].v.transpose() * unit_torques;

            // compute torques and forward compensate disturbance torques for multi-singularity hierarchy
            MatrixXd sjs_jacobian = _active_singularities[ind].v.transpose() * N_prec;
            Sai2Model::OpSpaceMatrices op_matrices = _robot->operationalSpaceMatrices(sjs_jacobian);
            MatrixXd Lambda_sjs_modified = op_matrices.Lambda;
            if (_dynamic_decoupling_type == BOUNDED_INERTIA_ESTIMATES) { 
                MatrixXd Lambda_inv_BIE = sjs_jacobian * _M_inv_BIE_SINGULARITY * sjs_jacobian.transpose();
                Lambda_sjs_modified = Lambda_inv_BIE.inverse();
                // Lambda_sjs_modified = op_matrices.Lambda;
                // Lambda_sjs_modified = Sai2Model::computePseudoInverse(Lambda_inv_BIE, _s_abs_tol);
            } 
            singular_joint_task_torques -= (MatrixXd::Identity(_dof, _dof) - op_matrices.N).transpose() * (singular_joint_task_torques + _non_singular_task_torques);  // forward compensation 
            singular_joint_task_torques += sjs_jacobian.transpose() * Lambda_sjs_modified * _active_singularities[ind].v.transpose() * unit_torques;
            // N_prec = op_matrices.N * N_prec;  // orthogonal tasks 

            // {
            //     // debug
            //     std::cout << "Lambda sjs modified: \n" << Lambda_sjs_modified << "\n";
            //     std::cout << "unit torques: \n" << unit_torques.transpose() << "\n";
            //     std::cout << "projected unit torques: " << _active_singularities[ind].v.transpose() * unit_torques << "\n";
            //     std::cout << "singular joint task torques: \n" << singular_joint_task_torques.transpose() << "\n";

            //     // stacking matrix 
            //     sjs_jacobian_list.push_back(sjs_jacobian);
            //     torque_list.push_back(_active_singularities[ind].v.transpose() * unit_torques);
            //     lambda_list.push_back(Lambda_sjs_modified);
            // }

        }

        _joint_strategy_torques = singular_joint_task_torques;

        // if (sjs_jacobian_list.size() > 0) {
        //     // debug check block form 
        //     MatrixXd J_sjs_stack = vstack(sjs_jacobian_list);
        //     MatrixXd Lambda_sjs_stack = _robot->taskInertiaMatrix(J_sjs_stack);
        //     VectorXd torque_stack = vstack(torque_list);
        //     VectorXd torque = J_sjs_stack.transpose() * Lambda_sjs_stack * torque_stack;

        //     if ((torque - _joint_strategy_torques).norm() > 1e-8) {
        //         std::cout << torque.transpose() <<" \n";
        //         std::cout << _joint_strategy_torques.transpose() << "\n";
        //         throw runtime_error("");
        //     }
        // }

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

        if (_is_in_singularity) {
            // std::cout << "joint strategy torques: \n" << _joint_strategy_torques.transpose() << "\n";
            return _non_singular_task_torques + _joint_strategy_torques * _enable_joint_strategy;
        } else {
            return _non_singular_task_torques;
        }

        // return combined_torque_vector + _joint_strategy_torques * _enable_joint_strategy;
    }
}

}  // namespace