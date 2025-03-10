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

double** convertToDoublePointer(const vector<Vector3d>& vec) {
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
    _kv = 10;
    _n_meshes = _link_names.size();

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

void SelfCollision::updateTaskModel(const MatrixXd& N_prec) {

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

        if (distance < _distance_zone_2) {
            // zone 2
            _mesh_pair_flag[i] = ZONE_2_COLLISION;
        } else if (distance < _distance_zone_1) {
            // zone 1
            _mesh_pair_flag[i] = ZONE_1_COLLISION;
        } else {
            // _mesh_pair_flag[i] = SAFE_COLLISION;  // only register safe collision when exiting zone 1 collision 

        }
        i++;
    }
   
}

VectorXd SelfCollision::computeTorques(const VectorXd& torques, const bool constraint_only) {
    
    // compute each self-collision in priority of closest distances
    VectorXd self_collision_torques = VectorXd::Zero(_robot->dof());
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
                std::cout << "Collision handling for pair " << _candidate_meshes[i].first << ", " << _candidate_meshes[i].second <<" \n";
            }

            /**
             * Zone handling 
             */
            if (_mesh_pair_flag[i] == ZONE_1_COLLISION) {

                if (_verbose) {
                    std::cout << "Zone 2 Handling\n";
                    // throw runtime_error("");
                }

                if (_mesh_pair_distance[i] == -1) {
                    std::cout << "Skipping collision\n";
                    continue; 
                }

                // task elements                 
                MatrixXd projected_jacobian = _mesh_pair_projected_jacobian[i] * _N_prec;
                // MatrixXd projected_jacobian = _mesh_pair_projected_jacobian[i];
                MatrixXd directed_projected_jacobian = _mesh_pair_constraint_direction[i].transpose() * projected_jacobian;

                MatrixXd task_inertia = _robot->taskInertiaMatrix(projected_jacobian);
                Vector3d task_force = _robot->dynConsistentInverseJacobian(projected_jacobian).transpose() * torques;
                // Vector3d task_force = _robot->dynConsistentInverseJacobian(_mesh_pair_linear_jacobian_b[i]).transpose() * torques;
                double task_force_along_constraint = task_force.dot(_mesh_pair_constraint_direction[i]);
                Vector3d unit_mass_constraint_force = - (_kv * projected_jacobian * _robot->dq());

                if (task_force_along_constraint > _F_thresh) {
                    std::cout << "Zone 1 free\n";
                    _mesh_pair_flag[i] = SAFE_COLLISION;  // exit nullspace 
                } else {
                    self_collision_torques += 1 * projected_jacobian.transpose() * task_inertia * unit_mass_constraint_force;

                    // nullspace 
                    _N_prec = _robot->nullspaceMatrix(directed_projected_jacobian) * _N_prec;

                }

            } else if (_mesh_pair_flag[i] == ZONE_2_COLLISION) {

                if (_verbose) {
                    std::cout << "Zone 2 Handling\n";
                    // throw runtime_error("");
                }

                if (_mesh_pair_distance[i] == -1) {
                    std::cout << "Skipping collision\n";
                    continue; 
                }

                // task elements 
                
                MatrixXd projected_jacobian = _mesh_pair_projected_jacobian[i] * _N_prec;
                // MatrixXd projected_jacobian = _mesh_pair_projected_jacobian[i];
                MatrixXd directed_projected_jacobian = _mesh_pair_constraint_direction[i].transpose() * projected_jacobian;

                MatrixXd task_inertia = _robot->taskInertiaMatrix(projected_jacobian);
                Vector3d task_force = _robot->dynConsistentInverseJacobian(projected_jacobian).transpose() * torques;
                // Vector3d task_force = _robot->dynConsistentInverseJacobian(_mesh_pair_linear_jacobian_b[i]).transpose() * torques;
                double task_force_along_constraint = task_force.dot(_mesh_pair_constraint_direction[i]);
                Vector3d unit_mass_constraint_force = - (_kv * projected_jacobian * _robot->dq());

                // apf force 
                double eta = 0.1;
                double rho = _mesh_pair_distance[i];
                double rho_0 = _distance_zone_2;
                double apf_force = eta * ((1 / rho) - (1 / rho_0)) * (1 / std::pow(rho, 2));

                // self_collision_torques += 1 * projected_jacobian.transpose() * apf_force * _mesh_pair_constraint_direction[i];
                self_collision_torques += 1 * projected_jacobian.transpose() * task_inertia * apf_force * _mesh_pair_constraint_direction[i];

                if (task_force_along_constraint > _F_thresh) {
                    std::cout << "Zone 2 free\n";
                    _mesh_pair_flag[i] = SAFE_COLLISION;
                } else {
                    // damping 
                    self_collision_torques += 1 * projected_jacobian.transpose() * task_inertia * unit_mass_constraint_force;

                    // nullspace 
                    _N_prec = _robot->nullspaceMatrix(directed_projected_jacobian) * _N_prec;
                }

            }
        }
    }    

    if (constraint_only) {
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