/*
 * Collision.h
 *
 *      This class creates a collision handling.
 *
 *      Author: William Chong 
 */

#ifndef SAI2_PRIMITIVES_COLLISION_H_
#define SAI2_PRIMITIVES_COLLISION_H_

#include <helper_modules/Sai2PrimitivesCommonDefinitions.h>
#include "Sai2Model.h"
#include <Eigen/Dense>
#include <openGJK/openGJK.h>
#include <yaml-cpp/yaml.h>
#include <cstdio>
#include <fstream>
#include <algorithm>
#include <numeric>

using namespace Eigen;
namespace Sai2Primitives {

enum CollisionState {
    SAFE_OBJECT_COLLISION = 0,
    ZONE_1_OBJECT_COLLISION,  // 1
    ZONE_2_OBJECT_COLLISION,  // 2
    CHECK_STATIONARY_COLLISION
};

class Collision {
public:

    Collision(std::shared_ptr<Sai2Model::Sai2Model> robot,
                const std::string& mesh_yaml,
                const bool& verbose = true,
                const double& distance_zone_1 = 0.1,
                const double& distance_zone_2 = 0.05,
                const double& f_thresh = 1.0,
                const double& f_min = 0.5,
                const double& dx_min = 0.005);
                //   const double& distance_zone_1 = 0.05,
                //   const double& distance_zone_2 = 0.02);

    int readMeshFile(const char* inputfile, gkFloat*** pts, int* out);

    void updateTaskModel(const MatrixXd& N_prec);
    VectorXd computeTorques(const VectorXd& torques,
                            const bool constraint_only = false);

    void enableCollisionFlag() {
        _enable_limit_flag = true; 
    }

    void disableCollisionFlag() {
        _enable_limit_flag = false;
    }

    MatrixXd getTaskAndPreviousNullspace() {
        return _N_prec;
    }

    void setObjectTransform(const Affine3d object_transform, const int ind);
    /*
        Collision information
    */
    std::vector<std::pair<Vector3d, Vector3d>> getCollisionPoints() {
        return _mesh_pair_body_points;
    }

    std::vector<CollisionState> getCollisionStates() {
        return _mesh_pair_flag;
    }

    std::vector<Vector3d> getConstraintDirections() {
        return _mesh_pair_constraint_direction;
    }

    std::vector<double> getDistances() {
        return _mesh_pair_distance;
    }

    // void setJointSelection(const std::vector<int>& joint_selection) {
    //     _joint_selection = joint_selection;
    // }

    // MatrixXd getBlendingMatrix() {
    //     return _blending_matrix;
    // }

    // VectorXd getNonTaskSafetyTorques() {
    //     return _non_task_safety_torques;
    // }

    // MatrixXd getTaskJacobian() {
    //     return _projected_jacobian;
    // }

    // MatrixXd getTaskAndPreviousNullspace() {
    //     return _Nc * _N_prec;
    // }

    // MatrixXd getNullspaceMatrix() {
    //     return _Nc;
    // }

    /*
        Threshold and parameter setting 
    */
    void setPosZone1Threshold(const double& distance_zone_1) {
        _distance_zone_1 = distance_zone_1;
    }

    void setPosZone2Threshold(const double& distance_zone_2) {
        _distance_zone_2 = distance_zone_2;
    }

    void setDampingCoeff(const double& kv) {
        _kv = kv;
    }

    void setForceMagnitude(const double& F_max) {
        _F_max = F_max;
    }

private:

    std::shared_ptr<Sai2Model::Sai2Model> _robot;

    // openGJK variables
    /* Squared distance computed by openGJK.                                 */
    // gkFloat dd, dx, dy, dz;
    /* Structure of simplex used by openGJK.                                 */
    // gkSimplex s;
    /* Number of vertices defining body 1 and body 2, respectively.          */
    // int nvrtx1, nvrtx2;
    /* Structures of body 1 and body 2, respectively.                        */
    // gkPolytope bd1;
    // gkPolytope bd2;
    /* Specify name of input files for body 1 and body 2, respectively.      */
    // char inputfileA[40] = "userP.dat", inputfileB[40] = "userQ.dat";
    /* Pointers to vertices' coordinates of body 1 and body 2, respectively. */
    // gkFloat(**vrtx1) = NULL, (**vrtx2) = NULL;

    // std::vector<gkPolytope> _bodies_centered;
    std::vector<gkPolytope> _bodies_polytope;
    std::vector<gkPolytope> _object_bodies_polytope;

    std::vector<std::vector<Vector3d>> _bodies_centered;
    std::vector<std::vector<Vector3d>> _bodies;

    std::vector<std::vector<Vector3d>> _object_bodies_centered;
    std::vector<std::vector<Vector3d>> _object_bodies;

    // self-collision settings
    std::vector<std::pair<int, int>> _candidate_meshes;  // checking pairs of meshes
    std::vector<Affine3d> _T_meshes;
    std::vector<Affine3d> _T_object_meshes;
    std::vector<std::string> _link_names;    
    int _n_collision_checks;  // number of candidate mesh pairs 
    int _n_meshes;
    int _n_objects;

    std::vector<CollisionState> _mesh_pair_flag;
    std::vector<int> _check_mesh_pair_flag;
    std::vector<double> _mesh_pair_distance;
    std::vector<Vector3d> _mesh_pair_constraint_direction;
    std::vector<MatrixXd> _mesh_pair_projected_jacobian;
    std::vector<std::pair<Vector3d, Vector3d>> _mesh_pair_body_points;
    std::vector<MatrixXd> _mesh_pair_linear_jacobian_a;
    std::vector<MatrixXd> _mesh_pair_linear_jacobian_b;

    // task models
    std::vector<MatrixXd> _projected_jacobians;
    MatrixXd _N_prec;
    double _kv;
    double _F_max;
    // double _F_max;
    double _F_thresh;
    double _F_min;
    double _dx_min;

    // safety
    double _distance_zone_1, _distance_zone_2;

    std::vector<int> _joint_selection;
    bool _verbose;
    bool _enable_limit_flag;
    bool _is_floating;
    int _dof;
    VectorXd _q_min;
    VectorXd _q_max;
    VectorXd _dq_abs_max;
    VectorXd _tau_abs_max;
    VectorXd _pos_zone_1_threshold;
    VectorXd _pos_zone_2_threshold;
    VectorXd _vel_zone_1_threshold;
    VectorXd _vel_zone_2_threshold;
    VectorXi _joint_state;
    VectorXd _vel_gamma;
    VectorXd _kv_pos_limit;
    bool _truncation_flag;

    // verbose output 
    std::vector<std::string> _constraint_description;

    // task parameters
    std::vector<double> _blending_coefficients;
    MatrixXd _blending_matrix;
    VectorXd _non_task_safety_torques;
    // MatrixXd _N_prec;
    MatrixXd _Jc;
    MatrixXd _Nc;
    MatrixXd _Lambda_c;
    MatrixXd _projected_jacobian;
    MatrixXd _current_task_range;

};

}  // namespace

#endif 