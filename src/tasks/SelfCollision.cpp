/**
 * @file SelfCollision.cpp
 * @author William Chong (wmchong@stanford.edu)
 * @brief 
 * @version 0.1
 * @date 2025-02-28
 * 
 * @copyright Copyright (c) 2025
 * 
 */

#include "SelfCollision.h"

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
// alpha from 1 -> 0 as approaching constraint 
inline double getMaxVelFunction(double alpha, double entry_vel, double exit_vel) {
    // std::cout << "alpha: " << alpha << "\n";
    // return exit_vel + (entry_vel - exit_vel) * ((1 - cos(M_PI * (1 - alpha))) / 2);
    // if (getSign(entry_vel) != getSign(exit_vel)) {
    //     exit_vel *= -1;
    // }
    return exit_vel + (entry_vel - exit_vel) * ((1 - cos(M_PI * (alpha))) / 2);
}

inline double** convertToDoublePointer(const vector<Vector3d>& vec) {
    int rows = vec.size();
    
    // Allocate memory for double**
    // double** array = new double*[rows];  
    double** array = (double**) malloc(rows * sizeof(double*));
    for (int i = 0; i < rows; ++i) {
        array[i] = (double*) malloc(3 * sizeof(double*));
    }

    for (int i = 0; i < rows; i++) {
        // array[i] = new double[3];  // Each Vector3d has 3 elements
        array[i][0] = vec[i].x();
        array[i][1] = vec[i].y();
        array[i][2] = vec[i].z();
    }
    
    return array;
}

vector<Vector3d> readDatFile(const string& filename) {
    vector<Vector3d> points;
    ifstream file(filename);

    if (!file.is_open()) {
        cerr << "Error: Could not open file " << filename << endl;
        return points;
    }

    int numVertices;
    file >> numVertices;  // Read the first line (number of vertices)

    if (file.fail() || numVertices <= 0) {
        cerr << "Error: Invalid number of vertices in file " << filename << endl;
        return points;
    }

    points.reserve(numVertices);  // Pre-allocate space for efficiency

    double x, y, z;
    while (file >> x >> y >> z) {  // Read (x, y, z) per line
        points.emplace_back(x, y, z);
    }

    file.close();

    if (points.size() != static_cast<size_t>(numVertices)) {
        cerr << "Warning: Expected " << numVertices << " vertices, but read " << points.size() << "." << endl;
    }

    return points;
}

SelfCollision::SelfCollision(std::shared_ptr<Sai2Model::Sai2Model> robot,
                             const std::string& mesh_yaml,
                             const bool& verbose,
                             const double& distance_zone_1,
                             const double& distance_zone_2,
                             const double& f_thresh) : 
                             _robot(robot),
                             _verbose(verbose),
                             _distance_zone_1(distance_zone_1),
                             _distance_zone_2(distance_zone_2),
                             _F_thresh(f_thresh) {

    // parse yaml file (mesh filenames (.dat files), and candidate pairs)
    /*
        File is in the format: 
        link_names: ["a", "b", ...]
        mesh_names: ["a.dat", "b.dat", ...]
        pairs: [(0, 1), (1, 2), ...]
    */
    std::cout << "Loading yaml file\n";
    YAML::Node config = YAML::LoadFile(mesh_yaml);

    // Read link_names
    std::vector<std::string> link_names = config["collision_config"]["link_names"].as<std::vector<std::string>>();

    // Read mesh prefix
    std::string mesh_prefix = config["collision_config"]["mesh_prefix"].as<std::string>();
    
    // Read mesh_names
    std::vector<std::string> mesh_fnames;
    for (const auto& name : config["collision_config"]["mesh_names"]) {
        mesh_fnames.push_back(mesh_prefix + name.as<std::string>());
    }

    // Read pairs
    std::vector<std::pair<int, int>> pairs;
    for (const auto& pair : config["collision_config"]["pairs"]) {
        pairs.push_back(std::make_pair(pair[0].as<int>(), pair[1].as<int>()));
    }
    
    // load into meshes 
    for (auto mesh : mesh_fnames) {
        std::cout << "Reading file: " << mesh << "\n";
        int nvrtx;
        gkFloat(**vrtx) = NULL;
        if (readMeshFile(mesh.c_str(), &vrtx, &nvrtx)) {
            throw std::runtime_error("Invalid mesh file read");
        }
        gkPolytope bd;
        bd.coord = vrtx;
        bd.numpoints = nvrtx;
        _bodies_polytope.push_back(bd);

        std::vector<Vector3d> pts = readDatFile(mesh);

        // create std::pair<int, std::vector<Vector3d>> object 
        // std::vector<Vector3d> pts;
        // for (int i = 0; i < nvrtx; ++i) {
            // pts.push_back(Vector3d(vrtx[i][0], vrtx[i][1], vrtx[i][2]));
        // }
        _bodies.push_back(pts);
        _bodies_centered.push_back(pts);
    }

    // safety settings 
    _link_names = link_names;
    _candidate_meshes = pairs;
    _n_collision_checks = pairs.size();
    _F_max = 100;
    _kv = 20;
    // _kv = 50;
    _n_meshes = _link_names.size();
    _eta = 0.1;

    _max_vel = 0.6;
    _min_vel = 0.2;

    for (int i = 0; i < _n_collision_checks; ++i) {
        _mesh_pair_flag.push_back(SAFE_COLLISION);
        _mesh_pair_distance.push_back(std::numeric_limits<double>::infinity());
        _mesh_pair_constraint_direction.push_back(Vector3d::Zero());
        _mesh_pair_projected_jacobian.push_back(MatrixXd::Zero(1, 1));
        _mesh_pair_body_points.push_back(std::make_pair(Vector3d::Zero(), Vector3d::Zero()));
        _mesh_pair_linear_jacobian_a.push_back(MatrixXd::Zero(1, 1));
        _mesh_pair_linear_jacobian_b.push_back(MatrixXd::Zero(1, 1));
    }

    for (int i = 0; i < _n_meshes; ++i) {
        _T_meshes.push_back(Affine3d::Identity());
    }

    // collision handling 
    _t_collision = 0.4;
    // _entry_velocity = VectorXd::Zero(_n_collision_checks);
    _entry_velocity = VectorXd::Zero(_n_collision_checks);
    _exit_velocity = VectorXd::Zero(_n_collision_checks);
    // _exit_velocity = VectorXd::Zero(_n_collision_checks);
    // _exit_velocity = - 0.1 * VectorXd::Ones(_n_collision_checks);  // always negative pointing inwards 
    _max_vel_vector = VectorXd::Zero(_n_collision_checks);
    _alpha = VectorXd::Zero(_n_collision_checks);
    _pos_zone_1_threshold = _distance_zone_1 * VectorXd::Ones(_n_collision_checks);
    _pos_zone_2_threshold = _distance_zone_2 * VectorXd::Ones(_n_collision_checks);

}

int SelfCollision::readMeshFile(const char* inputfile, gkFloat*** pts, int* out) {

    int npoints = 0;
    int idx = 0;
    FILE* fp;

    /* Open file. */
    if ((fp = fopen(inputfile, "r")) == NULL) {
        fprintf(stdout, "ERROR: input file %s not found!\n", inputfile);
        fprintf(stdout, "  -> The file must be in the folder from which this "
                        "program is launched\n\n");
        return 1;
    }

    /* Read number of input vertices. */
    if (fscanf(fp, "%d", &npoints) != 1) {
        return 1;
    }

    /* Allocate memory. */
    gkFloat** arr = (gkFloat**)malloc(npoints * sizeof(gkFloat*));
    for (int i = 0; i < npoints; i++) {
        arr[i] = (gkFloat*)malloc(3 * sizeof(gkFloat));
    }

    /* Read and store vertices' coordinates. */
    for (idx = 0; idx < npoints; idx++) {
        if (fscanf(fp, "%lf %lf %lf\n", &arr[idx][0], &arr[idx][1], &arr[idx][2]) != 3) {
            return 1;
        }
    }
    fclose(fp);

    *pts = arr;
    *out = idx;

    return 0;
}

void SelfCollision::updateTaskModel(const MatrixXd& N_prec, const bool flag_baseline) {

    // push forward 
    _N_prec = N_prec;

    // update robot mesh transforms 
    for (int i = 0; i < _n_meshes; ++i) {
        _T_meshes[i] = _robot->transform(_link_names[i]);
        if (i == _n_meshes - 1) {
            Matrix3d rot_in_link = AngleAxisd(-M_PI / 4, Vector3d::UnitZ()).toRotationMatrix();
            _T_meshes[i] = _robot->transform(_link_names[i], Vector3d(0, 0, -0.107), rot_in_link);
        }

        // debug transforms
        // std::cout << _link_names[i] << "\n";
        // std::cout << i << ": \n" << _T_meshes[i].matrix() << "\n";

        for (int j = 0; j < _bodies[i].size(); ++j) {
            _bodies_centered[i][j] = _T_meshes[i] * _bodies[i][j];
        } 

        // transfer to polytope
        gkPolytope bd;
        bd.coord = convertToDoublePointer(_bodies_centered[i]);
        bd.numpoints = _bodies[i].size();
        _bodies_polytope[i] = bd;
        
    }

    // _alpha_in_violation = {};
    _max_vel_vector = _max_vel * VectorXd::Ones(_n_collision_checks);

    // compute constraint jacobians from the mesh checks
    int i = 0;
    for (auto mesh_pairs : _candidate_meshes) {
        gkSimplex s;
        s.nvrtx = 0;
        int mesh_a_id = mesh_pairs.first;
        int mesh_b_id = mesh_pairs.second;

        double distance = compute_minimum_distance(_bodies_polytope[mesh_a_id], _bodies_polytope[mesh_b_id], &s);
        if (isnan(distance)) {
            // throw runtime_error("NAN distance");
            distance = -1;
            std::cout << "nan distance\n";
        }
        
        // // DEBUG 
        // std::cout << "pair: " << mesh_a_id << ", " << mesh_b_id << "\n";
        // std::cout << "distance: " << distance << "\n";

        Vector3d constraint_direction = (Vector3d(s.witnesses[1][0], s.witnesses[1][1], s.witnesses[1][2]) - \
                                            Vector3d(s.witnesses[0][0], s.witnesses[0][1], s.witnesses[0][2])).normalized();

        // Vector3d body_a_pos_in_link = _T_meshes[mesh_a_id].linear().transpose() * Vector3d(s.witnesses[0][0], s.witnesses[0][1], s.witnesses[0][2]) - \
                                            // _T_meshes[mesh_a_id].linear().transpose() * _T_meshes[mesh_a_id].translation();

        // Vector3d body_b_pos_in_link = _T_meshes[mesh_b_id].linear().transpose() * Vector3d(s.witnesses[1][0], s.witnesses[1][1], s.witnesses[1][2]) - \
                                            // _T_meshes[mesh_b_id].linear().transpose() * _T_meshes[mesh_b_id].translation();     

        Vector3d body_a_pos_in_link = _T_meshes[mesh_a_id].inverse() * Vector3d(s.witnesses[0][0], s.witnesses[0][1], s.witnesses[0][2]);
        Vector3d body_b_pos_in_link = _T_meshes[mesh_b_id].inverse() * Vector3d(s.witnesses[1][0], s.witnesses[1][1], s.witnesses[1][2]);

        _mesh_pair_body_points[i] = std::make_pair( Vector3d(s.witnesses[0][0], s.witnesses[0][1], s.witnesses[0][2]), \
                                                    Vector3d(s.witnesses[1][0], s.witnesses[1][1], s.witnesses[1][2]) );

        // collect information 
        _mesh_pair_distance[i] = distance;
        _mesh_pair_constraint_direction[i] = constraint_direction;
        // _mesh_pair_projected_jacobian[i] = constraint_direction.transpose() * \
        //     (_robot->Jv(_link_names[mesh_b_id], body_b_pos_in_link) - _robot->Jv(_link_names[mesh_a_id], body_a_pos_in_link)) * _N_prec;
        _mesh_pair_projected_jacobian[i] = 
            (_robot->Jv(_link_names[mesh_b_id], body_b_pos_in_link) - _robot->Jv(_link_names[mesh_a_id], body_a_pos_in_link)) * _N_prec;
        _mesh_pair_linear_jacobian_a[i] = _robot->Jv(_link_names[mesh_a_id], body_a_pos_in_link) * _N_prec;
        _mesh_pair_linear_jacobian_b[i] = _robot->Jv(_link_names[mesh_b_id], body_b_pos_in_link) * _N_prec;

        // check if current velocity will hit the boundary 
        double curr_vel = (_mesh_pair_projected_jacobian[i] * _robot->dq())(0);
        double distance_future = distance + curr_vel * _t_collision;

        if (flag_baseline) {
            if (distance < _pos_zone_2_threshold(i)) {
                std::cout << "Zone 2 classification\n";
                // throw runtime_error("");
                // zone 2
                _mesh_pair_flag[i] = ZONE_2_COLLISION;
            } else {
                _mesh_pair_flag[i] = SAFE_COLLISION;
            }
        } else if (_mesh_pair_flag[i] == SAFE_COLLISION && false) {

            // if (distance_future < _distance_zone_2) {
            if (distance_future < _pos_zone_2_threshold(i) && distance > _pos_zone_2_threshold(i)) {
                // trigger zone 1 collision
                std::cout << "UPDATE TASK MODEL: ZONE 1 RESET\n";
                _mesh_pair_flag[i] = ZONE_1_COLLISION;
                setPosZone1ThresholdIndex(distance, i);
                _alpha(i) = 0;
                _entry_velocity(i) = curr_vel;
                _max_vel_vector(i) = _max_vel;
            }
        } else {

            // if (distance < _distance_zone_2) {
            if (distance < _pos_zone_2_threshold(i)) {
                std::cout << "UPDATE TASK MODEL: ZONE 2 CLASSIFICATION\n";
                // throw runtime_error("");
                // zone 2
                _mesh_pair_flag[i] = ZONE_2_COLLISION;
                _alpha(i) = 0;
                // _alpha_in_violation.push_back(0);
                _max_vel_vector(i) = _min_vel;
                // std::cout << "min vel\n" << _min_vel << "\n";

            // } else if (distance < _distance_zone_1 && _mesh_pair_flag[i] != ZONE_2_COLLISION) {
            } else if (distance < _pos_zone_1_threshold(i)) {
                std::cout << "UDPATE TASK MODEL: ZONE 1 CLASSIFICATION\n";

                // if (_mesh_pair_flag[i] == ZONE_2_COLLISION) {
                //     _alpha(i) = 0;
                //     _max_vel_vector(i) = _min_vel;
                //     continue;
                // } 

                _alpha(i) = \
                    std::clamp(std::abs((distance - _pos_zone_2_threshold(i)) / (_pos_zone_1_threshold(i) - _pos_zone_2_threshold(i))), 0.0, 1.0);

                // _alpha_in_violation.push_back(_alpha(i));

                _max_vel_vector(i) = getMaxVelFunction(_alpha(i), _max_vel, _min_vel);
                
                // zone 1
                _mesh_pair_flag[i] = ZONE_1_COLLISION;

            } else {

                // std::cout << "Safe classification\n";
                _mesh_pair_flag[i] = SAFE_COLLISION;  // only register safe collision when exiting zone 1 collision 
                _alpha(i) = 1;
                // _max_vel_vector(i) = _max_vel;

            }

        }
        i++;
    }
   
}

VectorXd SelfCollision::computeTorques(const VectorXd& torques, 
                                       const bool constraint_only, 
                                       const bool flag_baseline) {
    
    // compute each self-collision in priority of closest distances
    VectorXd self_collision_torques = VectorXd::Zero(_robot->dof());
    // MatrixXd N = N_prec;  // collect nullspace 
    // bool constrained_direction = false;

    // Create an index vector and sort it based on values using a lambda function
    std::vector<std::size_t> indices(_mesh_pair_distance.size());
    std::iota(indices.begin(), indices.end(), 0); // Fill with 0, 1, 2, ...

    std::sort(indices.begin(), indices.end(),
              [&](std::size_t i, std::size_t j) { return _mesh_pair_distance[i] < _mesh_pair_distance[j]; });

    // Go through each index in order 
    for (int i = 0; i < indices.size(); ++i) {
        if (_mesh_pair_flag[i] != SAFE_COLLISION) {

            if (_verbose) {
                std::cout << "----\n";
                std::cout << "Collision handling for pair " << _candidate_meshes[i].first << ", " << _candidate_meshes[i].second <<" \n";
                std::cout << "----\n";
            }

            /*
            *********************************
                ZONE HANDLING
            *********************************
            */
            if (_mesh_pair_flag[i] == ZONE_1_COLLISION && !flag_baseline) {

                if (_verbose) {
                    std::cout << "Zone 1 Handling\n";
                    // throw runtime_error("");
                }

                if (_mesh_pair_distance[i] == -1) {
                    std::cout << "Skipping collision\n";
                    continue; 
                }

                /*
                *************************************************
                    APPROACH WITH TASK VELOCITY SATURATION
                *************************************************
                */

                continue;
                
                // // task elements                 
                // MatrixXd projected_jacobian = _mesh_pair_projected_jacobian[i] * _N_prec;
                // // MatrixXd projected_jacobian = _mesh_pair_projected_jacobian[i];
                // MatrixXd directed_projected_jacobian = _mesh_pair_constraint_direction[i].transpose() * projected_jacobian;

                // // using 1-dof constraint jacobian
                // MatrixXd task_inertia = _robot->taskInertiaMatrix(directed_projected_jacobian);
                // VectorXd task_force = _robot->dynConsistentInverseJacobian(directed_projected_jacobian).transpose() * torques;
                // // Vector3d task_force = _robot->dynConsistentInverseJacobian(_mesh_pair_linear_jacobian_b[i]).transpose() * torques;
                // double task_force_along_constraint = task_force(0);
                // // Vector3d unit_mass_constraint_force = - (_kv * projected_jacobian * _robot->dq()) * _mesh_pair_constraint_direction[i];

                // // velocity saturation approaching
                // double unit_mass_constraint_force = - (_kv * directed_projected_jacobian * _robot->dq())(0);
                // double max_vel = getMaxVelFunction(_alpha(i), _entry_velocity(i), _exit_velocity(i));
                // double vel_along_constraint = (directed_projected_jacobian * _robot->dq())(0);
                // double forward_vel = vel_along_constraint + (task_force_along_constraint / task_inertia(0, 0)) * (1 / _kv);

                // // if (std::abs(vel_along_constraint) > max_vel) {
                // if (std::abs(forward_vel) > max_vel) {
                //     double vel_saturation_force = - _kv * (vel_along_constraint - max_vel);
                //     self_collision_torques += directed_projected_jacobian.transpose() * task_inertia * vel_saturation_force;
                //     unit_mass_constraint_force *= 0;
                // } 

                // // using full task jacobian
                // // MatrixXd task_inertia = _robot->taskInertiaMatrix(projected_jacobian);
                // // Vector3d task_force = _robot->dynConsistentInverseJacobian(projected_jacobian).transpose() * torques;
                // // // Vector3d task_force = _robot->dynConsistentInverseJacobian(_mesh_pair_linear_jacobian_b[i]).transpose() * torques;
                // // double task_force_along_constraint = task_force.dot(_mesh_pair_constraint_direction[i]);
                // // Vector3d unit_mass_constraint_force = - (_kv * projected_jacobian * _robot->dq());

                // if (task_force_along_constraint > _F_thresh) {
                //     std::cout << "Zone 1 free\n";
                //     _mesh_pair_flag[i] = SAFE_COLLISION;  // exit nullspace 
                // } else if (!flag_baseline) {
                //     // self_collision_torques += 1 * projected_jacobian.transpose() * task_inertia * unit_mass_constraint_force;
                //     self_collision_torques += 1 * directed_projected_jacobian.transpose() * task_inertia * unit_mass_constraint_force;

                //     // nullspace 
                //     _N_prec = _robot->nullspaceMatrix(directed_projected_jacobian) * _N_prec;

                // }

            } else if (_mesh_pair_flag[i] == ZONE_2_COLLISION) {

                if (_verbose) {
                    std::cout << "Zone 2 Handling\n";
                    // throw runtime_error("");
                }
                // throw runtime_error("");

                if (_mesh_pair_distance[i] == -1) {
                    std::cout << "Skipping collision\n";
                    continue; 
                }

                /*
                ****************************
                    APF Torque Handling
                ****************************
                */
                // MatrixXd ee_projected_jacobian = _mesh_pair_constraint_direction[i].transpose() * \
                //                                     _mesh_pair_linear_jacobian_b[i] * _N_prec;
                // MatrixXd ee_task_inertia = _robot->taskInertiaMatrix(ee_projected_jacobian);                

                MatrixXd projected_jacobian = _mesh_pair_projected_jacobian[i];  // Jv_b - Jv_a
                // MatrixXd projected_jacobian = _mesh_pair_projected_jacobian[i];
                MatrixXd directed_projected_jacobian = _mesh_pair_constraint_direction[i].transpose() * projected_jacobian;

                // matrix basis
                MatrixXd U = Sai2Model::matrixRangeBasis(directed_projected_jacobian);
                directed_projected_jacobian = U.transpose() * directed_projected_jacobian;
                
                // using 1-dof constraint jacobian
                MatrixXd task_inertia = _robot->taskInertiaMatrix(directed_projected_jacobian);

                VectorXd task_force = _robot->dynConsistentInverseJacobian(projected_jacobian).transpose() * torques;
                double task_force_along_constraint = _mesh_pair_constraint_direction[i].dot(task_force);

                // VectorXd task_force = _robot->dynConsistentInverseJacobian(directed_projected_jacobian).transpose() * torques;  // using 1 dof 
                // double task_force_along_constraint = task_force(0);
                // Vector3d unit_mass_constraint_force = - (_kv * projected_jacobian * _robot->dq()) * _mesh_pair_constraint_direction[i];
                double unit_mass_constraint_force = - (_kv * directed_projected_jacobian * _robot->dq())(0);
                
                // // using full task jacobian 
                // MatrixXd task_inertia = _robot->taskInertiaMatrix(projected_jacobian);
                // Vector3d task_force = _robot->dynConsistentInverseJacobian(projected_jacobian).transpose() * torques;
                // // Vector3d task_force = _robot->dynConsistentInverseJacobian(_mesh_pair_linear_jacobian_b[i]).transpose() * torques;
                // double task_force_along_constraint = task_force.dot(_mesh_pair_constraint_direction[i]);
                // Vector3d unit_mass_constraint_force = - (_kv * projected_jacobian * _robot->dq());

                // apf force 
                double eta = _eta;
                double rho = std::clamp(_mesh_pair_distance[i], 1e-3, _pos_zone_2_threshold(i));
                // double rho = _mesh_pair_distance[i];
                // std::cout << "rho: " << rho << "\n";
                // double rho_0 = _distance_zone_2;
                double rho_0 = _pos_zone_2_threshold(i);
                double apf_force = eta * ((1 / rho) - (1 / rho_0)) * (1 / std::pow(rho, 2));

                // if (task_inertia(0) > 5) {
                //     task_inertia(0) = 5;
                // }

                // task_inertia.setIdentity();
                // std::cout << "task inertia: " << task_inertia << "\n";
                // std::cout << "apf force: " << apf_force << "\n";

                // self_collision_torques += 1 * projected_jacobian.transpose() * apf_force * _mesh_pair_constraint_direction[i];
                // self_collision_torques += 1 * projected_jacobian.transpose() * task_inertia * apf_force * _mesh_pair_constraint_direction[i];
                
                self_collision_torques += 1 * _N_prec.transpose() * directed_projected_jacobian.transpose() * U.transpose() * task_inertia * apf_force;

                // apply apf torque on one side only 
                // self_collision_torques += 1 * _N_prec.transpose() * ee_projected_jacobian.transpose() * ee_task_inertia * apf_force;

                std::cout << "task force along constraint: " << task_force_along_constraint << "\n";

                // if (std::abs(task_force_along_constraint / task_inertia(0)) > _F_thresh && getSign(task_force_along_constraint) > 0 && !flag_baseline) {
                if (std::abs(task_force_along_constraint) > _F_thresh && getSign(task_force_along_constraint) > 0 && !flag_baseline) {
                // if (std::abs(task_force_along_constraint) > _F_thresh && getSign(task_force_along_constraint) < 0 && !flag_baseline) {

                    std::cout << "ZONE 2 FREE\n";
                    // throw runtime_error("");
                    // _mesh_pair_flag[i] = SAFE_COLLISION;

                    _mesh_pair_flag[i] = ZONE_1_COLLISION;

                    self_collision_torques += 1 * _N_prec.transpose() * directed_projected_jacobian.transpose() * U.transpose() * task_inertia * unit_mass_constraint_force;

                    // nullspace 
                    // if (!flag_baseline) {
                        // _N_prec = _robot->nullspaceMatrix(directed_projected_jacobian) * _N_prec;
                    // }

                } else {
                    // damping 
                    // self_collision_torques += 1 * projected_jacobian.transpose() * task_inertia * unit_mass_constraint_force;
                    // self_collision_torques += 1 * directed_projected_jacobian.transpose() * task_inertia * apf_force;
                    self_collision_torques += 1 * _N_prec.transpose() * directed_projected_jacobian.transpose() * U.transpose() * task_inertia * unit_mass_constraint_force;

                    // nullspace 
                    if (!flag_baseline) {
                        _N_prec = _robot->nullspaceMatrix(directed_projected_jacobian) * _N_prec;
                        // _N_prec = _robot->nullspaceMatrix(ee_projected_jacobian) * _N_prec;
                    }
                }

            }
        }
    }    

    if (flag_baseline) {
        return self_collision_torques + torques;
    } else if (constraint_only) {
        return self_collision_torques;
    } else {
        return self_collision_torques + _N_prec.transpose() * torques;
    }
}

} // namespace 

// for (int i = 0; i < _mesh_pair_flag.size(); ++i) {
//         if (constrained_direction) {
//             continue;
//         } else if (_mesh_pair_flag[i] != SAFE_COLLISION) {

//             if (_verbose) {
//                 std::cout << "Collision handling for pair " << _candidate_meshes[i].first << ", " << _candidate_meshes[i].second <<" \n";
//             }
//             constrained_direction = true;

//             /**
//              * Zone 1 collision 
//              */
//             if (_mesh_pair_flag[i] == ZONE_1_COLLISION) {

//                 if (_mesh_pair_distance[i] == -1) {
//                     std::cout << "Skipping collision\n";
//                     continue; 
//                 }

//                 // task elements 
//                 // double alpha = std::clamp((_mesh_pair_distance[i] - _distance_zone_2) / (_distance_zone_1 - _distance_zone_2), 0.0, 1.0);
                
//                 // MatrixXd projected_jacobian = _mesh_pair_projected_jacobian[i] * _N_prec;
//                 MatrixXd projected_jacobian = _mesh_pair_projected_jacobian[i];
//                 MatrixXd directed_projected_jacobian = _mesh_pair_constraint_direction[i].transpose() * projected_jacobian;

//                 MatrixXd task_inertia = _robot->taskInertiaMatrix(projected_jacobian);
//                 Vector3d task_force = _robot->dynConsistentInverseJacobian(projected_jacobian).transpose() * torques;
//                 // Vector3d task_force = _robot->dynConsistentInverseJacobian(_mesh_pair_linear_jacobian_b[i]).transpose() * torques;
//                 double task_force_along_constraint = task_force.dot(_mesh_pair_constraint_direction[i]);

//                 // compute range basis 
//                 // MatrixXd U = Sai2Model::matrixRangeBasis(projected_jacobian);
//                 // std::cout << "Projected jacobian: \n" << projected_jacobian << "\n";
//                 // std::cout << "U: \n" << U << "\n";

//                 // double task_inertia = _robot->taskInertiaMatrix(projected_jacobian)(0);
//                 // double task_force_along_constraint = (_robot->dynConsistentInverseJacobian(projected_jacobian).transpose() * torques)(0);

//                 // Vector3d task_force_vector_along_constraint = _robot->dynConsistentInverseJacobian(projected_jacobian).transpose() * torques;
//                 // Vector3d task_force_vector_along_constraint = \
//                     // (_robot->dynConsistentInverseJacobian(_mesh_pair_linear_jacobian_b[i]).transpose() + \
//                     //  _robot->dynConsistentInverseJacobian(_mesh_pair_linear_jacobian_a[i]).transpose()) * torques;

//                 // std::cout << "constraint direction: " << _mesh_pair_constraint_direction[i].transpose() << "\n";
//                 // std::cout << "task force: " << task_force.transpose() << "\n";
//                 // std::cout << "task force along constraint: " << task_force_along_constraint << "\n";
//                 // std::cout << "task force vector along constraint: " << task_force_vector_along_constraint.transpose() << "\n";
//                 // std::cout << "dot product: " << task_force_vector_along_constraint.normalized().dot(_mesh_pair_constraint_direction[i]) << "\n";

//                 // constraint force
//                 // double unit_mass_constraint_force = - (std::pow(1 - alpha, 2) * _kv * projected_jacobian * _robot->dq())(0);
//                 // double unit_mass_constraint_force = - (_kv * projected_jacobian * _robot->dq())(0);
//                 Vector3d unit_mass_constraint_force = - (_kv * projected_jacobian * _robot->dq());

//                 // if (task_force_along_constraint.dot(_mesh_pair_constraint_direction[i]) > 0 \
//                     // && task_force_along_constraint > _F_thresh) {
//                 if (task_force_along_constraint > _F_thresh) {
//                 // if (task_force_vector_along_constraint.normalized().dot(_mesh_pair_constraint_direction[i]) > 0) {                    
//                     std::cout << "Zone 1 free\n";
//                     // throw runtime_error("");
//                     // self_collision_torques += 1 * directed_projected_jacobian.transpose() * task_force_along_constraint * _mesh_pair_constraint_direction[i];
//                     // self_collision_torques += 1 * projected_jacobian.transpose() * task_force;

//                     _mesh_pair_flag[i] = SAFE_COLLISION;  // exit nullspace 
//                 } else {
//                     self_collision_torques += 1 * projected_jacobian.transpose() * task_inertia * unit_mass_constraint_force;

//                     // nullspace 
//                     _N_prec = _robot->nullspaceMatrix(directed_projected_jacobian) * _N_prec;

//                 }
//                 // self_collision_torques += projected_jacobian.transpose() * task_inertia * unit_mass_constraint_force;

//             } else if (_mesh_pair_flag[i] == ZONE_2_COLLISION) {

//                 if (_verbose) {
//                     std::cout << "Zone 2 Handling\n";
//                     // throw runtime_error("");
//                 }

//                 if (_mesh_pair_distance[i] == -1) {
//                     std::cout << "Skipping collision\n";
//                     continue; 
//                 }

//                 // task elements 
                
//                 // MatrixXd projected_jacobian = _mesh_pair_projected_jacobian[i] * _N_prec;
//                 MatrixXd projected_jacobian = _mesh_pair_projected_jacobian[i];
//                 MatrixXd directed_projected_jacobian = _mesh_pair_constraint_direction[i].transpose() * projected_jacobian;

//                 MatrixXd task_inertia = _robot->taskInertiaMatrix(projected_jacobian);
//                 Vector3d task_force = _robot->dynConsistentInverseJacobian(projected_jacobian).transpose() * torques;
//                 // Vector3d task_force = _robot->dynConsistentInverseJacobian(_mesh_pair_linear_jacobian_b[i]).transpose() * torques;
//                 double task_force_along_constraint = task_force.dot(_mesh_pair_constraint_direction[i]);
//                 Vector3d unit_mass_constraint_force = - (_kv * projected_jacobian * _robot->dq());

//                 // apf force 
//                 double eta = 0.1;
//                 double rho = _mesh_pair_distance[i];
//                 double rho_0 = _distance_zone_2;
//                 double apf_force = eta * ((1 / rho) - (1 / rho_0)) * (1 / std::pow(rho, 2));

//                 self_collision_torques += 1 * projected_jacobian.transpose() * apf_force * _mesh_pair_constraint_direction[i];

//                 // if (task_force_along_constraint.dot(_mesh_pair_constraint_direction[i]) > 0 \
//                     // && task_force_along_constraint.norm() > _F_thresh) {
//                 if (task_force_along_constraint > _F_thresh) {
//                     std::cout << "Zone 2 free\n";
//                     // self_collision_torques += 1 * directed_projected_jacobian.transpose() * task_force_along_constraint * _mesh_pair_constraint_direction[i];
//                     // self_collision_torques += 1 * projected_jacobian.transpose() * task_force_along_constraint;
//                     // self_collision_torques += 1 * projected_jacobian.transpose() * task_force;
//                     _mesh_pair_flag[i] = SAFE_COLLISION;
//                 } else {
//                     // damping 
//                     self_collision_torques += 1 * projected_jacobian.transpose() * task_inertia * unit_mass_constraint_force;

//                     // nullspace 
//                     _N_prec = _robot->nullspaceMatrix(directed_projected_jacobian) * _N_prec;
//                 }

//             }
//         }
//     }