/**
 * @file SingularityHandler.cpp
 * @author William Chong (wmchong@stanford.edu)
 * @brief Singularity handler class
 * @version 0.1
 * @date 2026-02-10
 * 
 * @copyright Copyright (c) 2026
 * 
 */

#include "SingularityHandler.h"

namespace {
    const std::vector<std::string> singularity_labels = 
        {"No Singularity", "Type 1 Singularity", "Type 2 Singularity"};

    int sign(double x) {
        return (x > 0) - (x < 0);
    }

    VectorXd saturateBox(
        const VectorXd& x,
        const VectorXd& min_vec,
        const VectorXd& max_vec) {
            
        assert(x.size() == min_vec.size() && x.size() == max_vec.size());
        // clamp elementwise: first ensure ≥ min, then ensure ≤ max
        return x.cwiseMax(min_vec).cwiseMin(max_vec);
    }

    double maxIgnoringNaN(const VectorXd& v) {
        double max_val = -std::numeric_limits<double>::infinity();
        for (int i = 0; i < v.size(); ++i) {
            double val = v[i];
            if (!std::isnan(val)) {
                max_val = std::max(max_val, val);
            }
        }
        return max_val;
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

    std::vector<std::vector<int>> findCloseGroups(
        const VectorXd& v, 
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

    bool containsInGroups(
        const std::vector<std::vector<int>>& groups, 
        int value) {

        for (const auto& group : groups) {
            for (int i : group) {
                if (i == value) {
                    return true;
                }
            }
        }
        return false;
    }

    MatrixXd removeUnitDirectionFromBasis(
        const MatrixXd& B,
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

    std::vector<int> sortSingularityIndices(const std::vector<SaiPrimitives::Singularity>& singularities) {
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

    double smoothExpSin(double x, double beta) {
        constexpr double half_pi = M_PI * 0.5;
        x = std::clamp(x, 0.0, 1.0); // clamp x to [0,1]
        return (1.0 - std::exp(-beta * std::sin(half_pi * x)))
            / (1.0 - std::exp(-beta));
    }

    MatrixXd stabilizedInverse(const MatrixXd& A, const double lambda = 1e-6) {
        MatrixXd AtA = A.transpose() * A;
        MatrixXd identity = MatrixXd::Identity(A.rows(), A.cols());
        // add a small epsilon (lambda^2) to the diagonal to ensure invertibility
        return (AtA + lambda * lambda * identity).ldlt().solve(A.transpose());
    }

    MatrixXd lltInverse(const MatrixXd& A_inv) {
        return A_inv.llt().solve(MatrixXd::Identity(A_inv.rows(), A_inv.rows()));
    }
}

namespace SaiPrimitives {

bool SingularityHandler::classifySingularityType(
    const std::vector<MatrixXd>& kinematic_hessian,
    const VectorXd& u,
    const VectorXd& v) {

    double deviation = std::abs(u.transpose() * getSecondOrderExpansion(kinematic_hessian, v));
    // double deviation = std::abs(v.dot(getProjectedHessian(kinematic_hessian, u) * v));
    {
        // debug
        // std::cout << "classify dev: " << deviation << "\n";

        // // cross-check with virtual perturbation
        // Vector3d x_curr = _robot->positionInWorld(_link_name, _compliant_frame.translation());
        // VectorXd q_init = _robot->q();
        // _robot->setQ(q_init + v);
        // _robot->updateKinematics();
        // Vector3d x_dev = _robot->positionInWorld(_link_name, _compliant_frame.translation());
        // double robot_dev = u.head(3).transpose() * (x_dev - x_curr);
        // std::cout << "robot perturb: " << robot_dev << "\n";
        // _robot->setQ(q_init);
        // _robot->updateKinematics();
    }
    return deviation > _type_1_tol;
}

VectorXd SingularityHandler::getSecondOrderExpansion(
    const std::vector<MatrixXd>& kinematic_hessian,
    const VectorXd& dq) {

    const int dof  = dq.size();
    const int task = kinematic_hessian[0].rows();
    VectorXd dx2 = VectorXd::Zero(task);
    for (int j = 0; j < dof; ++j) {
        dx2 += dq(j) * (kinematic_hessian[j] * dq);
    }
    return dx2;
}

MatrixXd SingularityHandler::getProjectedHessian(
    const std::vector<MatrixXd>& kinematic_hessian, // dof x (6 x dof)
    const VectorXd& direction) {

    MatrixXd projected_hessian = MatrixXd::Zero(_robot->dof(), _robot->dof());
    for (int i = 0; i < _robot->dof(); ++i) {
        projected_hessian.row(i) = direction.transpose() * kinematic_hessian[i];
    }
    return projected_hessian;
}

VectorXd SingularityHandler::getInitialVector(
    const std::vector<MatrixXd>& kinematic_hessian,
    const MatrixXd& U,
    const MatrixXd& V) {

    const int dof = _robot->dof();
    const int rU  = U.rows();
    const int cU  = U.cols();

    // pre-compute V H V^{T}
    std::vector<MatrixXd> VHVT(rU, MatrixXd::Zero(cU, cU));
    for (int i = 0; i < rU; ++i) {
        MatrixXd Hi = MatrixXd::Zero(dof, dof);
        for (int j = 0; j < dof; ++j) {
            Hi.row(j) = kinematic_hessian[j].row(i);
        }
        VHVT[i].noalias() = V.transpose() * Hi * V;
    }

    // compute A_{j} matrices
    std::vector<MatrixXd> A_weighted(cU, MatrixXd::Zero(cU, cU));
    for (int i = 0; i < rU; ++i) {
        for (int j = 0; j < cU; ++j) {
            A_weighted[j].noalias() += U(i, j) * VHVT[i];
        }
    }

    // compute sum of squares
    MatrixXd H_sum_squared = MatrixXd::Zero(cU, cU);
    for (const auto& A : A_weighted) {
        H_sum_squared.noalias() += A * A;
    }

    // eigenvalue solution and initial solution evaluation
    _eig_solver.compute(H_sum_squared);

    VectorXd deviation(cU);

    for (int i = 0; i < cU; ++i) {
        const VectorXd coeff = _eig_solver.eigenvectors().col(i);

        VectorXd u = U * coeff;
        VectorXd v = V * coeff;
        u.normalize();
        v.normalize();

        deviation(i) =
            std::abs(v.dot(getProjectedHessian(kinematic_hessian, u) * v));
    }

    Eigen::Index min_index;
    deviation.minCoeff(&min_index);
    return _eig_solver.eigenvectors().col(min_index);
}

double SingularityHandler::equality(
    const std::vector<double> &x, 
    std::vector<double> &grad, 
    void* f_data) {

    double sum = 0;
    for (int i = 0; i < x.size(); ++i) {
        sum += x[i] * x[i];
    }
    return 1 - sqrt(sum);
}

double SingularityHandler::objective(
    const std::vector<double>& x,
    std::vector<double>& grad,
    void* f_data) {

    auto* self = static_cast<SingularityHandler*>(f_data);
    const auto& data = *self->_nl_opt_data;
    const int n = static_cast<int>(x.size());

    // compute v vector
    VectorXd V = VectorXd::Zero(self->_dof);
    for (int i = 0; i < n; ++i) {
        V.noalias() += x[i] * data.basis[i];
    }
    
    const double V_norm = V.norm();
    const double safe_V_norm = (V_norm < 1e-12) ? 1e-12 : V_norm;
    const VectorXd v = V / safe_V_norm;

    // compute u vector
    VectorXd u = vectorFromBasis(x, data.singular_task_range);
    u.normalize();

    if (data.flag_zero_value) {
        // use forward kinematics to compute u at zero singular value
        self->_robot->setQ(data.starting_q + data.perturb_step_size * v);
        self->_robot->updateKinematics();

        u.head<3>() = self->_robot->position(
                        self->_link_name,
                        self->_compliant_frame.translation()) - data.starting_position;

        u.tail<3>() = SaiModel::orientationError(
                        self->_robot->rotation(
                            self->_link_name,
                            self->_compliant_frame.linear()), data.starting_orientation);

        // project onto singular direction
        u = data.singular_task_range * (data.singular_task_range.transpose() * u);
        u.normalize();
    }

    // compute objective function
    const MatrixXd M = self->getProjectedHessian(self->_dJdq, u);
    const VectorXd Mv = M * v;
    const double quadratic_form = v.dot(Mv);
    
    const double obj_val = quadratic_form * quadratic_form;
    const double result = data.flag_type_1_search ? -obj_val : obj_val;  // switch to maximization for type 1

    return result;
}

std::pair<VectorXd, VectorXd> SingularityHandler::getTowardSingularityDirection(
    const VectorXd& curr_q,
    const Vector3d& curr_pos,
    const Matrix3d& curr_ori,
    const VectorXd& u,
    const VectorXd& dsdq,
    const double step_size) {

    VectorXd u_toward = u;

    auto evalDeviation = [&](const VectorXd& q_sample,
                             VectorXd& delta) -> double {
        _robot->setQ(q_sample);
        _robot->updateKinematics();
        Vector3d delta_pos = _robot->position(
                                _link_name,
                                _compliant_frame.translation()) - curr_pos;
        Vector3d delta_ori = SaiModel::orientationError(
                                _robot->rotation(
                                _link_name,
                                _compliant_frame.linear()), curr_ori);
        delta.head(3) = delta_pos;
        delta.tail(3) = delta_ori;
        return u.dot(delta);
    };

    VectorXd pos_delta(6);
    VectorXd neg_delta(6);
    double pos_dev = 0.0, neg_dev = 0.0;

    for (int cnt = 0; cnt < _type_1_search_max_iter; ++cnt) {

        const double scale = (cnt + 1) * step_size;

        pos_dev = evalDeviation(curr_q + scale * dsdq, pos_delta);
        neg_dev = evalDeviation(curr_q - scale * dsdq, neg_delta);

        // case 1: same sign 
        // the deviation direction is in the maximizing direction
        if (sign(pos_dev) == sign(neg_dev)) {

            // align the u_toward_singularity opposite to pos_delta direction
            if (u_toward.dot(pos_delta.normalized()) > 0) {
                u_toward = - u_toward;
            }

            // choose sign of dsdq (+ sign is away from singularity) based on desired elbow crossing
            // defaults to the direction with the largest deviation for now
            bool flip_dsdq = false;
            if (std::abs(pos_dev) < std::abs(neg_dev)) {
                flip_dsdq = true;
            }

            return {u_toward, flip_dsdq ? -dsdq : dsdq};
        }
    }

    // case 2: bidirection search yields different signs
    // take the direction with largest deviation if the signs aren't the same
    if (std::abs(pos_dev) > std::abs(neg_dev)) {
        if (u_toward.dot(pos_delta.normalized()) > 0.0) {
            u_toward = - u_toward;
        }
        return {u_toward, dsdq};
    } else {
        if (u_toward.dot(neg_delta.normalized()) > 0.0) {
            u_toward = - u_toward;
        }
        return {u_toward, dsdq};
    }
}

SingularityHandler::SingularityHandler(
    std::shared_ptr<SaiModel::SaiModel> robot,
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
    _verbose(verbose) {
        
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
    _q_prior = _joint_midrange;
    _dq_prior = VectorXd::Zero(_dof);
    setSingularityHandlingGains(
        DefaultParameters::kv_type_1, 
        DefaultParameters::kv_type_2);
    setDynamicDecouplingType(BOUNDED_INERTIA_ESTIMATES);
	setBoundedInertiaEstimateThreshold(
        DefaultParameters::bie_threshold, 
        DefaultParameters::singular_bie_threshold);
    
    // flags
    _enforce_type_1_strategy = false;
    _enforce_handling_strategy = true;
    _enable_joint_strategy = true;
    _enable_force_damping_decoupling = false;
    _fully_singular_task = false;
    _handle_singularity_exit = false;
    _is_in_singularity = false;
    _type_1_retracting = false;

    // initialize singularity handling classification variables
    _s_abs_tol = DefaultParameters::s_abs_tol;
    _type_1_tol = DefaultParameters::type_1_tol; 

    _type_2_max_vel = DefaultParameters::type_2_max_vel;
    _type_2_min_force = DefaultParameters::type_2_min_force;

    _type_1_max_vel_away_from_singularity = DefaultParameters::type_1_max_vel_away_from_singularity;
    _type_1_max_vel_towards_singularity = DefaultParameters::type_1_max_vel_towards_singularity;
    _type_1_step_size_classification_towards_singularity = DefaultParameters::type_1_step_size_classification_towards_singularity;
    _type_1_step_size_for_line_search = DefaultParameters::type_1_step_size_for_line_search;
    _type_1_search_max_iter = DefaultParameters::type_1_search_max_iter;

    _max_force_norm = DefaultParameters::max_force_norm;

    _degenerate_singular_value_spacing = DefaultParameters::degenerate_singular_value_spacing;

    _type_1_search_tol = DefaultParameters::type_1_search_tol;

    _num_singularities = 0;
    _prev_num_singularities = 0;

    _type_1_vel_ramp_factor = DefaultParameters::type_1_vel_ramp_factor;
    _type_2_vel_ramp_factor = DefaultParameters::type_2_vel_ramp_factor;
    _type_1_alignment_factor = DefaultParameters::type_1_alignment_factor;

    // setup nlopt
    for (int i = 2; i < _task_rank; ++i) {
        _nl_opt[i] = std::make_unique<nlopt::opt>(nlopt::LN_COBYLA, i);
        _nl_opt[i]->set_min_objective(objective, this);
        _nl_opt[i]->add_equality_constraint(equality, this, 1e-2);

        _nl_opt[i]->set_xtol_rel(DefaultParameters::xtol_rel);  // 1e-12 default
        _nl_opt[i]->set_ftol_rel(DefaultParameters::ftol_rel);
        _nl_opt[i]->set_xtol_abs(DefaultParameters::xtol_abs);
        _nl_opt[i]->set_maxtime(DefaultParameters::max_time * 1e-3);
    }

    _nl_opt_data = std::make_unique<NloptData>(DefaultParameters::type_1_step_size_classification_towards_singularity);

    // initialize posture jacobian
    _posture_projected_jacobian = MatrixXd::Zero(1, _dof);
}

void SingularityHandler::updateTaskModel(MatrixXd& projected_jacobian, const MatrixXd& N_prec) {
    
    // task range decomposition
    _N_prec = N_prec;
    _projected_jacobian = projected_jacobian;

    // compute eigen-decomposition of task inertia matrix 
    _eig_solver.compute(projected_jacobian * _robot->MInv() * projected_jacobian.transpose());
    _eig_values = _eig_solver.eigenvalues().reverse();  // switch to descending order 
    _eig_vectors = _eig_solver.eigenvectors().rowwise().reverse();

    // compute svd of jacobian
    _svd_solver.compute(projected_jacobian, ComputeThinU | ComputeThinV);
    _svd_U = _svd_solver.matrixU();
    _svd_s = _svd_solver.singularValues();  // descending order 
    _svd_V = _svd_solver.matrixV();   

    // compute jacobian derivatives 
    _dJdq = _robot->getJacobianDerivative(_link_name, _compliant_frame.translation());  // dof x (6 x dof)

    // compute singular task range 
    _fully_singular_task = false;
    _is_in_singularity = false;

    if (_eig_values(0) < _s_abs_tol) {
        if (_verbose) {
            std::cout << "[WARNING] Task is fully singular\n";
        }
        _fully_singular_task = true;

        // placeholder non-singular terms 
        _task_range_ns = MatrixXd::Zero(_task_rank, 1);
        _projected_jacobian_ns = MatrixXd::Zero(_task_rank, _dof);
        _Lambda_ns = MatrixXd::Zero(_task_rank, _task_rank);
        _N_ns = N_prec;

        // singular task 
        _task_range_s = _svd_U.leftCols(_task_rank);
        _joint_task_range_s = _svd_V.leftCols(_task_rank);
        _projected_jacobian_s = _task_range_s.transpose() * projected_jacobian;
        _Lambda_s = SaiModel::computePseudoInverse(
                                  _projected_jacobian_s *
                                  _robot->MInv() * 
                                  _projected_jacobian_s.transpose(), _s_abs_tol);
        _svd_s_singular = _svd_s;
        _eig_s_singular = _eig_values;

        // sjs info
        _prev_num_singularities = _num_singularities;
        _num_singularities = _task_rank;
        _alpha_vec = VectorXd::Zero(_task_rank);
        // _condition_ratio_vec = VectorXd::Zero(_task_rank);

        // update flags 
        _is_in_singularity = true;

    } else if (_task_rank == 1) {
        if (_verbose) {
            std::cout << "[WARNING] Fully non-singular 1 dof task\n";
        }

        // non-singular task
        _task_range_ns = _svd_U.leftCols(_task_rank); 
        _projected_jacobian_ns = _task_range_ns.transpose() * projected_jacobian;
        SaiModel::OpSpaceMatrices ns_matrices =
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
        // _condition_ratio_vec = VectorXd::Zero(_task_rank);

        // update flags 
        _is_in_singularity = false;

    } else {

        // check up to _task_rank, as zero eigenvalues after 
        for (int i = 1; i < _task_rank; ++i) {
            if (_eig_values(i) < _s_max) {

                // non-singular task
                _task_range_ns = _svd_U.leftCols(i);
                _projected_jacobian_ns = _task_range_ns.transpose() * projected_jacobian;
                SaiModel::OpSpaceMatrices ns_matrices =
                    _robot->operationalSpaceMatrices(_projected_jacobian_ns);
                _Lambda_ns = ns_matrices.Lambda;
                _Jbar_ns = ns_matrices.Jbar;
                _N_ns = ns_matrices.N;

                // singular task: task range only collects columns of U up to size task_rank - non-singular task rank
                _task_range_s = _svd_U.block(0, i, _svd_U.rows(), _task_rank - i);  
                _joint_task_range_s = _svd_V.block(0, i, _svd_V.rows(), _task_rank - i);
                // _projected_jacobian_s = _task_range_s.transpose() * projected_jacobian * _N_ns;
                // _Lambda_s = SaiModel::computePseudoInverse(_projected_jacobian_s * _robot->MInv() * _projected_jacobian_s.transpose(), _s_abs_tol);
                _projected_jacobian_s = _task_range_s.transpose() * projected_jacobian;
                // _Lambda_s = (_projected_jacobian_s * _robot->MInv() * _projected_jacobian_s.transpose()).inverse();
                _Lambda_s = lltInverse(_projected_jacobian_s * _robot->MInv() * _projected_jacobian_s.transpose());
                _svd_s_singular = _svd_s.tail(_task_rank - i);
                _eig_s_singular = _eig_values.tail(_task_rank - i);

                // sjs info
                _prev_num_singularities = _num_singularities;
                _num_singularities = _task_range_s.cols();
                _alpha_vec = VectorXd::Zero(_num_singularities);
                // _condition_ratio_vec = VectorXd::Zero(_num_singularities);

                // update flags 
                _is_in_singularity = true;
                break;

            } else if (i == _task_rank - 1) {

                if (_verbose) {
                    std::cout << "Fully non-singular task\n";;
                }

                // non-singular task
                _task_range_ns = _svd_U.leftCols(_task_rank); 
                _projected_jacobian_ns = _task_range_ns.transpose() * projected_jacobian;
                SaiModel::OpSpaceMatrices ns_matrices =
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
                // _condition_ratio_vec = VectorXd::Zero(_task_rank);

                // update flags 
                _is_in_singularity = false;
            }
        }
    }

    // exit transition flags
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
        SaiModel::OpSpaceMatrices op_space_matrices =
            _robot->operationalSpaceMatrices(_posture_projected_jacobian);
        _Lambda_sjs = op_space_matrices.Lambda;
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
            // MatrixXd M_inv_BIE = M_BIE.inverse();
            // _M_inv_BIE_SINGULARITY = M_BIE_SINGULARITY.inverse();
            MatrixXd M_inv_BIE = lltInverse(M_BIE);
            _M_inv_BIE_SINGULARITY = lltInverse(M_BIE_SINGULARITY);

            // non-singular lambda
            if (!_task_range_ns.isZero(1e-8)) {
                MatrixXd Lambda_inv_BIE =
                    _projected_jacobian_ns *
                    M_inv_BIE * 
                    _projected_jacobian_ns.transpose();
                // _Lambda_ns_modified = Lambda_inv_BIE.inverse();
                _Lambda_ns_modified = lltInverse(Lambda_inv_BIE);
            } else {
                _Lambda_ns_modified = _Lambda_ns;
            }

            // singular lambda
            if (!_task_range_s.isZero(1e-8)) {
                MatrixXd Lambda_inv_BIE =
                    _projected_jacobian_s *
                    M_inv_BIE * 
                    _projected_jacobian_s.transpose();
                _Lambda_s_modified = SaiModel::computePseudoInverse(Lambda_inv_BIE, _s_abs_tol);
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

void SingularityHandler::classifySingularity(
    const MatrixXd& projected_jacobian,
    const MatrixXd& singular_task_range,
    const MatrixXd& singular_joint_task_range) {

    // memory of entering singularity state
    if (!_is_in_singularity) {
        _q_prior = _robot->q();
        _dq_prior = _robot->dq();
    }

    _active_singularities = {};
    _degenerate_indices = {};

    if (!_is_in_singularity || !_enforce_handling_strategy) {
        return;
    }

    VectorXd curr_q = _robot->q();
    Vector3d curr_pos = _robot->position(_link_name, _compliant_frame.translation());
    Matrix3d curr_ori = _robot->rotation(_link_name, _compliant_frame.linear());

    // handle degenerate singularities
    if (_is_in_singularity) {
        _degenerate_indices = findCloseGroups(_svd_s_singular, _degenerate_singular_value_spacing);
    }

    _degenerate_singular_task_range = {};
    _degenerate_singular_joint_task_range = {};
    _degenerate_singular_values = {};  // used to check for zero/very close to zero values
    _degenerate_eigen_values = {};

    if (_degenerate_indices.size() > 0) {

        // store degenerate task ranges 
        for (auto group : _degenerate_indices) {
            _degenerate_singular_task_range.push_back(collectColumns(singular_task_range, group));
            _degenerate_singular_joint_task_range.push_back(collectColumns(singular_joint_task_range, group));
            _degenerate_singular_values.push_back(collectColumns(_svd_s_singular, group));
            _degenerate_eigen_values.push_back(collectColumns(_eig_s_singular, group));
        }

        int group_id = 0;
        for (auto group : _degenerate_indices) {

            // run minimization
            MatrixXd curr_singular_task_range = _degenerate_singular_task_range[group_id];
            MatrixXd curr_singular_joint_task_range = _degenerate_singular_joint_task_range[group_id];
            std::vector<VectorXd> curr_type_1_singular_task_range, curr_type_2_singular_task_range;
            std::vector<VectorXd> curr_type_1_singular_joint_task_range, curr_type_2_singular_joint_task_range;
            bool search_type_1 = false;

            while (true) {

                // update optimization parameters
                _nl_opt_data->setData(
                    matrixToColumnVectors(curr_singular_joint_task_range), 
                    curr_singular_task_range, 
                    projected_jacobian, 
                    curr_pos, 
                    curr_ori, 
                    curr_q,
                    _degenerate_singular_values[group_id].maxCoeff() < _type_1_search_tol,
                    search_type_1);

                // initial guess from eigenvalue problem
                VectorXd initial_coeff = getInitialVector(_dJdq, curr_singular_task_range, curr_singular_joint_task_range);

                int n_coefficients = initial_coeff.size();
                std::vector<double> sjs_coefficients;
                for (int i = 0; i < n_coefficients; ++i) {
                    sjs_coefficients.push_back(initial_coeff(i));
                }
                double optimal_value = 0;

                // solve 
                try {
                    _nl_opt[n_coefficients]->optimize(sjs_coefficients, optimal_value);
                } catch (...) {
                    if (_verbose) {
                        std::cout << "[WARNING] Failed nlopt solve\n";
                    }
                }

                // compute (u, v)
                VectorXd v_proj = vectorFromBasis(sjs_coefficients, curr_singular_joint_task_range).normalized();
                VectorXd u_proj = curr_singular_task_range.col(0);  // placeholder

                if (_degenerate_singular_values[group_id].maxCoeff() > _type_1_search_tol) {
                    // matching u_proj from coefficients
                    u_proj = vectorFromBasis(sjs_coefficients, curr_singular_task_range).normalized();
                } else {
                    // get u_proj from the deviation, as {u, v} are individually rotated
                    _robot->setQ(curr_q + _type_1_step_size_classification_towards_singularity * v_proj);
                    _robot->updateKinematics();
                    u_proj.head(3) = _robot->position(_link_name, _compliant_frame.translation()) - curr_pos; 
                    u_proj.tail(3) = SaiModel::orientationError(_robot->rotation(_link_name, _compliant_frame.linear()), curr_ori);

                    // project u_proj into the current task range
                    u_proj = (curr_singular_task_range * curr_singular_task_range.transpose() * u_proj).normalized();
                }

                // process vectors
                bool flag_process_vectors = true;
                if (sqrt(optimal_value) > _type_1_tol) {
                    // if 2nd order displacement is above type 1 threshold, switch to type 1 search
                    if (!search_type_1) {
                        search_type_1 = true;
                        flag_process_vectors = false;
                    } else {
                        // push {u, v} to current type 1 task ranges 
                        curr_type_1_singular_task_range.push_back(u_proj);
                        curr_type_1_singular_joint_task_range.push_back(v_proj);
                    }
                } else {
                    // add to type 2
                    curr_type_2_singular_task_range.push_back(u_proj);
                    curr_type_2_singular_joint_task_range.push_back(v_proj);
                }
                
                if (flag_process_vectors) {
                    // remove (u, v) from current degenerate singular task and joint range
                    curr_singular_task_range = removeUnitDirectionFromBasis(curr_singular_task_range, u_proj);
                    curr_singular_joint_task_range = removeUnitDirectionFromBasis(curr_singular_joint_task_range, v_proj);

                    if (curr_singular_joint_task_range.cols() == 1) {
                        // normal classification and break
                        if (classifySingularityType(_dJdq, curr_singular_task_range, curr_singular_joint_task_range)) {
                            curr_type_1_singular_task_range.push_back(curr_singular_task_range);
                            curr_type_1_singular_joint_task_range.push_back(curr_singular_joint_task_range);
                        } else {
                            curr_type_2_singular_task_range.push_back(curr_singular_task_range);
                            curr_type_2_singular_joint_task_range.push_back(curr_singular_joint_task_range);
                        }

                        curr_singular_task_range = MatrixXd::Zero(1, 0);  // placeholder
                        curr_singular_joint_task_range = MatrixXd::Zero(1, 0);
                        break;
                    }
                }
            }

            // assign singularities
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
                if (_degenerate_singular_values[group_id].maxCoeff() < _type_1_search_tol) {
                    // line search since dsdq is indeterminate up to a sign
                    std::tie(u_toward_singularity, dsdq_toward_singularity) = 
                        getTowardSingularityDirection(curr_q, curr_pos, curr_ori, u, dsdq, _type_1_step_size_for_line_search);
                } else {
                    // perform gradient descent with -dsdq
                    VectorXd q_toward_singularity = curr_q - _type_1_step_size_classification_towards_singularity * dsdq;
                    _robot->setQ(q_toward_singularity);
                    _robot->updateKinematics();
                    // Vector3d delta_vector = _robot->position(_link_name, _compliant_frame.translation()) - curr_pos;
                    // if (delta_vector.dot(u.head(3)) < 0) {
                    //     u_toward_singularity = - u;
                    // }
                    VectorXd delta_vector(6);
                    delta_vector.head(3) = _robot->position(_link_name, _compliant_frame.translation()) - curr_pos;
                    delta_vector.tail(3) = SaiModel::orientationError(_robot->rotation(_link_name, _compliant_frame.linear()), curr_ori);
                    if (delta_vector.normalized().dot(u) < 0) {
                        u_toward_singularity = - u;
                    }
                }

            _active_singularities.push_back(
                Singularity(curr_type_1_singular_task_range[i], 
                            curr_type_1_singular_joint_task_range[i], 
                            _degenerate_singular_values[group_id](i),  // assign same value for all
                            _degenerate_eigen_values[group_id](i),
                            dsdq_toward_singularity,
                            u_toward_singularity,
                            SingularityType::TYPE_1_SINGULARITY,
                            true));
            }

            // type 2 singularities 
            for (int i = 0; i < curr_type_2_singular_task_range.size(); ++i) {

                // recompute dsdq for type 2 singularities 
                VectorXd dsdq = VectorXd::Zero(_dof);
                for (int j = 0; j < _dof; ++j) {
                    dsdq(j) = curr_type_2_singular_task_range[i].transpose() * _dJdq[j] * curr_type_2_singular_joint_task_range[i];
                }    
                VectorXd u = curr_type_2_singular_task_range[i];
                VectorXd u_toward_singularity = u;  // placeholder
                VectorXd dsdq_toward_singularity = dsdq;  // placeholder

                _active_singularities.push_back(
                    Singularity(curr_type_2_singular_task_range[i], 
                                curr_type_2_singular_joint_task_range[i], 
                                _degenerate_singular_values[group_id](i + curr_type_1_singular_task_range.size()),  // assign same value
                                _degenerate_eigen_values[group_id](i + curr_type_1_singular_task_range.size()),
                                dsdq_toward_singularity,
                                u_toward_singularity,
                                SingularityType::TYPE_2_SINGULARITY,
                                true));
            }

            group_id++;

        }
    }

    // normal classification
    std::vector<bool> is_degenerate(_num_singularities, false);
    for (const auto& group : _degenerate_indices) {
        for (int idx : group) {
            is_degenerate[idx] = true;
        }
    }

    for (int i = 0; i < _num_singularities; ++i) {

        // if (!containsInGroups(_degenerate_indices, i)) {
        if (!is_degenerate[i]) {

            VectorXd u = singular_task_range.col(i);
            VectorXd v = singular_joint_task_range.col(i);
            VectorXd u_toward_singularity = u;  // placeholder
            
            // compute gradient
            VectorXd dsdq = VectorXd::Zero(_dof);
            for (int j = 0; j < _dof; ++j) {
                dsdq(j) = u.transpose() * _dJdq[j] * v;
            }
            VectorXd dsdq_toward_singularity = dsdq;  // placeholder

            // classify type 1 
            bool is_type_1 = classifySingularityType(_dJdq, u, v);

            if (is_type_1 && (_svd_s_singular(i) < _type_1_search_tol)) {
                // perform line search
                std::tie(u_toward_singularity, dsdq_toward_singularity) = 
                    getTowardSingularityDirection(curr_q, curr_pos, curr_ori, u, dsdq, _type_1_step_size_for_line_search);
            } else {
                // perform gradient descent 
                VectorXd q_toward_singularity = curr_q - _type_1_step_size_classification_towards_singularity * dsdq_toward_singularity;
                _robot->setQ(q_toward_singularity);
                _robot->updateKinematics();
                VectorXd delta_vector(6);
                delta_vector.head(3) = _robot->position(_link_name, _compliant_frame.translation()) - curr_pos;
                delta_vector.tail(3) = SaiModel::orientationError(_robot->rotation(_link_name, _compliant_frame.linear()), curr_ori);
                if (delta_vector.normalized().dot(u) < 0) {
                    u_toward_singularity = - u;
                }

            }

            if (is_type_1) {
                _active_singularities.push_back(
                    Singularity(u, v, _svd_s_singular(i), _eig_s_singular(i), dsdq_toward_singularity, u_toward_singularity, TYPE_1_SINGULARITY, false));
            } else {
                _active_singularities.push_back(
                    Singularity(u, v, _svd_s_singular(i), _eig_s_singular(i), dsdq_toward_singularity, u_toward_singularity, TYPE_2_SINGULARITY, false));
            }

        }
    }

    // reset robot
    _robot->setQ(curr_q);
    _robot->updateKinematics();

    return;
}

VectorXd SingularityHandler::computeTorques(const VectorXd& unit_mass_force, const VectorXd& force_related_terms) {
    if (_verbose) {
        if (_is_in_singularity && _enforce_handling_strategy) {
            int i = 0;
            for (auto singularity : _active_singularities) {
                std::cout << "Singularity " << i << ": " << singularity.type << "\n";
                std::cout << "u: " << singularity.u.transpose() << "\n";
                std::cout << "v: " << singularity.v.transpose() << "\n";
                std::cout << "s-value: " << singularity.sigma << "\n";
                std::cout << "e-value: " << singularity.lambda << "\n";
                ++i;
            }
            std::cout << "-----------------------------------\n";
        }
    }

    // reset containers
    _non_singular_task_torques = VectorXd::Zero(_dof);
    _singular_task_torques = VectorXd::Zero(_dof);
    _joint_strategy_torques = VectorXd::Zero(_dof);
    _unmodified_singular_task_torques = VectorXd::Zero(_dof);

    if (!_is_in_singularity || !_enforce_handling_strategy) {
        if (_enable_force_damping_decoupling) {
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
                                                _task_range_ns.transpose() * 
                                                (unit_mass_force + force_related_terms);
            } else {
                if (_enable_force_damping_decoupling) {
                    _non_singular_task_torques = _projected_jacobian_ns.transpose() * 
                                                    _Lambda_ns_modified * _task_range_ns.transpose() * 
                                                    (unit_mass_force + force_related_terms); 
                } else {
                    _non_singular_task_torques = _projected_jacobian_ns.transpose() * 
                                                    (_Lambda_ns_modified * _task_range_ns.transpose() * 
                                                    unit_mass_force + _task_range_ns.transpose() * force_related_terms);
                }
                if (!_enforce_handling_strategy) {
                    return _non_singular_task_torques;
                }
            }
        } 

        {
            // experimental
            if (_is_in_singularity) {
                // debug for experimental baseline
                _unmodified_singular_task_torques = _projected_jacobian_s.transpose() * 
                                                        ((_projected_jacobian_s * _robot->MInv() * _projected_jacobian_s.transpose()).inverse() * 
                                                        _task_range_s.transpose() * unit_mass_force + 
                                                        _task_range_s.transpose() * force_related_terms);
            }

            for (int i = 0; i < _dof; ++i) {
                if (isnan(_singular_task_torques(i))) {
                    _singular_task_torques(i) = 0;  
                }
            }
        }

        // singularity handling setup
        VectorXd curr_q = _robot->q();
        Vector3d curr_pos = _robot->position(_link_name, _compliant_frame.translation());
        Matrix3d curr_ori = _robot->rotation(_link_name, _compliant_frame.linear());
        VectorXd unit_torques = VectorXd::Zero(_dof);
        VectorXd singular_joint_task_torques = VectorXd::Zero(_dof);

        VectorXd normalized_force_moment = unit_mass_force.normalized();
        if (_enable_force_damping_decoupling) {
            normalized_force_moment = (unit_mass_force + force_related_terms).normalized();
        }

        MatrixXd N_prec = _N_sjs_init;
        std::vector<int> sorted_indices = sortSingularityIndices(_active_singularities);  // sort priority in increasing singular value

        for (auto ind : sorted_indices) {

            // compute saturated singular joint space torque
            MatrixXd curr_projected_jacobian = _active_singularities[ind].u.transpose() * _projected_jacobian * _N_ns;
            MatrixXd curr_lambda = 
                lltInverse(curr_projected_jacobian * _robot->MInv() * curr_projected_jacobian.transpose());
                // SaiModel::computePseudoInverse(curr_projected_jacobian * _robot->MInv() * curr_projected_jacobian.transpose(), _type_1_search_tol);
                // (curr_projected_jacobian * _robot->MInv() * curr_projected_jacobian.transpose()).inverse();
            VectorXd singular_torque_component = 
                curr_projected_jacobian.transpose() * curr_lambda * _active_singularities[ind].u.transpose() * unit_mass_force + 
                curr_projected_jacobian.transpose() * _active_singularities[ind].u.transpose() * force_related_terms;
            if (_enable_force_damping_decoupling) {
                singular_torque_component = 
                    curr_projected_jacobian.transpose() * curr_lambda * _active_singularities[ind].u.transpose() * (unit_mass_force + force_related_terms); 
            }

            // use Tikhonov regularization for nan torques for approximate joint acceleration vector
            if (singular_torque_component.array().isNaN().any()) {
                curr_lambda = stabilizedInverse(curr_projected_jacobian * _robot->MInv() * curr_projected_jacobian.transpose());
                singular_torque_component = 
                    curr_projected_jacobian.transpose() * curr_lambda * _active_singularities[ind].u.transpose() * unit_mass_force + 
                    curr_projected_jacobian.transpose() * _active_singularities[ind].u.transpose() * force_related_terms;
            }

            VectorXd singular_unit_acceleration_component = _robot->MInv() * singular_torque_component;

            // handle each singularity type
            if (_active_singularities[ind].type == TYPE_1_SINGULARITY || _enforce_type_1_strategy) {

                // compute alignment 
                double singular_alignment = 
                    singular_unit_acceleration_component.normalized().dot(_active_singularities[ind].dsdq.normalized());

                // joint-space classification of control towards/away from singularity
                bool is_moving_towards_singularity = singular_alignment < 0;
                // if (singular_alignment > 0) {
                    // is_moving_towards_singularity = false;
                // } else {
                    // is_moving_towards_singularity = true;
                // }

                // compute scale factor based on singular alignment 
                double scale_factor_singular_alignment = 
                    smoothExpSin(std::abs(singular_alignment), _type_1_alignment_factor);

                // compute control for approaching or leaving type 1 singularity
                if (is_moving_towards_singularity) {
                    
                    if (_verbose) {
                        std::cout << "Type 1 singularity moving towards singularity\n";
                    }
                    _type_1_retracting = false;

                    double curr_singular_task_force = 
                        std::abs(_active_singularities[ind].u.transpose() * (unit_mass_force + force_related_terms));
                    double force_scaling = std::clamp(curr_singular_task_force / _max_force_norm, 0.0, 1.0);
                    double condition_number_scaling = 
                        smoothExpSin(_active_singularities[ind].lambda / _s_max, _type_1_vel_ramp_factor);
                        // std::clamp(smoothExpSin(_active_singularities[ind].lambda / _s_max, _type_1_vel_ramp_factor), 0.0, 1.0);
                        // std::clamp((1 - exp(-_type_1_vel_ramp_factor * (_active_singularities[ind].lambda / _s_max))) / (1 - exp(-_type_1_vel_ramp_factor)), 0.0, 1.0);
                    double vel_scaling = std::min(scale_factor_singular_alignment, std::min(force_scaling, condition_number_scaling));

                    VectorXd dq_des = - _active_singularities[ind].dsdq;
                    dq_des = vel_scaling * _type_1_max_vel_towards_singularity * dq_des.normalized();
                    unit_torques = - _kv_type_1 * (_robot->dq() - dq_des);

                } else {

                    if (_verbose) {
                        std::cout << "Type 1 singularity retracting from singularity\n";
                    }
                    _type_1_retracting = true;
                    
                    double curr_singular_task_force = 
                        std::abs(_active_singularities[ind].u.transpose() * (unit_mass_force + force_related_terms));

                    double vel_scaling = 
                        std::min(std::clamp(curr_singular_task_force / _max_force_norm, 0.0, 1.0), scale_factor_singular_alignment);

                    VectorXd dq_des = _active_singularities[ind].dsdq;
                    dq_des = vel_scaling * _type_1_max_vel_away_from_singularity * dq_des.normalized();
                    unit_torques = - _kv_type_1 * (_robot->dq() - dq_des);

                }

            } else if (_active_singularities[ind].type == TYPE_2_SINGULARITY) {

                double force_dotted_singular_direction = 
                    std::abs((normalized_force_moment.transpose() * _active_singularities[ind].u));

                double weighted_force_dotted_singular_direction = 
                    std::clamp(smoothExpSin(force_dotted_singular_direction, _type_2_vel_ramp_factor), 0.0, 1.0);
                    // std::clamp((1 - exp(-_type_2_vel_ramp_factor * force_dotted_singular_direction)) / (1 - exp(-_type_2_vel_ramp_factor)), 0.0, 1.0);
  
                VectorXd dq_des = 
                    _type_2_max_vel * weighted_force_dotted_singular_direction * singular_unit_acceleration_component.normalized();
                unit_torques = - _kv_type_2 * (_robot->dq() - dq_des);

            }

            // compute torques and forward compensate disturbance torques for multi-singularity hierarchy
            MatrixXd sjs_jacobian = _active_singularities[ind].v.transpose() * N_prec;
            SaiModel::OpSpaceMatrices op_matrices = _robot->operationalSpaceMatrices(sjs_jacobian);
            MatrixXd Lambda_sjs_modified = op_matrices.Lambda;
            if (_dynamic_decoupling_type == BOUNDED_INERTIA_ESTIMATES) { 
                MatrixXd Lambda_inv_BIE = sjs_jacobian * _M_inv_BIE_SINGULARITY * sjs_jacobian.transpose();
                // Lambda_sjs_modified = Lambda_inv_BIE.inverse();
                Lambda_sjs_modified = lltInverse(Lambda_inv_BIE);
            } 
            VectorXd unit_disturbance_force = sjs_jacobian * _robot->MInv() * (singular_joint_task_torques + _non_singular_task_torques);
            // singular_joint_task_torques -= 
                // (MatrixXd::Identity(_dof, _dof) - op_matrices.N).transpose() * (singular_joint_task_torques + _non_singular_task_torques);  // forward compensation 
            singular_joint_task_torques -= sjs_jacobian.transpose() * op_matrices.Lambda * unit_disturbance_force;
            singular_joint_task_torques += 
                sjs_jacobian.transpose() * Lambda_sjs_modified * _active_singularities[ind].v.transpose() * unit_torques;
            N_prec = op_matrices.N * N_prec;

        }

        _joint_strategy_torques = singular_joint_task_torques;

        if (_is_in_singularity) {
            return _non_singular_task_torques + _joint_strategy_torques * _enable_joint_strategy;
        } else {
            return _non_singular_task_torques;
        }
    }
}

}  // namespace