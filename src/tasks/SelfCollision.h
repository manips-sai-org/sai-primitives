/*
 * SelfCollision.h
 *
 *      This class creates a self-collision handling.
 *
 *      Author: William Chong 
 */

#ifndef SAI2_PRIMITIVES_SELF_COLLISION_H_
#define SAI2_PRIMITIVES_SELF_COLLISION_H_

#include <helper_modules/Sai2PrimitivesCommonDefinitions.h>
#include "Sai2Model.h"
#include <Eigen/Dense>
#include <openGJK/openGJK.h>
#include <yaml-cpp/yaml.h>
#include <cstdio>
#include <fstream>

using namespace Eigen;
namespace Sai2Primitives {

enum SelfCollisionState {
    SAFE_COLLISION = 0,
    ZONE_1_COLLISION,  // 1
    ZONE_2_COLLISION,  // 2
};

class SelfCollision {
public:

    SelfCollision(std::shared_ptr<Sai2Model::Sai2Model> robot,
                  const std::string& mesh_yaml,
                  const bool& verbose = true,
                  const double& distance_zone_1 = 0.15,
                  const double& distance_zone_2 = 0.1);
                //   const double& distance_zone_1 = 0.05,
                //   const double& distance_zone_2 = 0.02);

    int readMeshFile(const char* inputfile, gkFloat*** pts, int* out);

    void updateTaskModel(const MatrixXd& N_prec);
    VectorXd computeTorques(const VectorXd& torques);

    void enableCollisionFlag() {
        _enable_limit_flag = true; 
    }

    void disableCollisionFlag() {
        _enable_limit_flag = false;
    }

    /*
        Collision information
    */
    std::vector<std::pair<Vector3d, Vector3d>> getCollisionPoints() {
        return _mesh_pair_body_points;
    }

    std::vector<SelfCollisionState> getCollisionStates() {
        return _mesh_pair_flag;
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

    std::vector<std::vector<Vector3d>> _bodies_centered;
    std::vector<std::vector<Vector3d>> _bodies;

    // self-collision settings
    std::vector<std::pair<int, int>> _candidate_meshes;  // checking pairs of meshes
    std::vector<Affine3d> _T_meshes;
    std::vector<std::string> _link_names;    
    int _n_collision_checks;  // number of candidate mesh pairs 
    int _n_meshes;

    std::vector<SelfCollisionState> _mesh_pair_flag;
    std::vector<double> _mesh_pair_distance;
    std::vector<Vector3d> _mesh_pair_constraint_direction;
    std::vector<MatrixXd> _mesh_pair_projected_jacobian;
    std::vector<std::pair<Vector3d, Vector3d>> _mesh_pair_body_points;

    // task models
    std::vector<MatrixXd> _projected_jacobians;
    MatrixXd _N_prec;
    double _kv;
    double _F_max;
    // double _F_max;

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