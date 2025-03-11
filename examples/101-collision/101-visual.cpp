/*
* Example of singularity handling by smoothing Lambda in/out of singularities.
*/

#include <math.h>
#include <signal.h>

#include <iostream>
#include <mutex>
#include <string>
#include <thread>

#include "Sai2Graphics.h"
#include "Sai2Model.h"
#include "Sai2Simulation.h"
#include "tasks/JointTask.h"
#include "tasks/MotionForceTask.h"
#include "tasks/JointHandler.h"
#include "tasks/Collision.h"
#include "timer/LoopTimer.h"
#include "logger/Logger.h"
#include "redis/RedisClient.h"
bool fSimulationRunning = false;
void sighandler(int) { fSimulationRunning = false; }

using namespace std;
using namespace Eigen;
using namespace chai3d;

// config file names and object names
const string world_file = "${EXAMPLE_101_FOLDER}/world.urdf";
const string robot_file =
    "${SAI2_MODEL_URDF_FOLDER}/panda/panda_arm_gripper_fixed.urdf";
const std::string mesh_yaml = "/home/src0/OpenSai/core/sai2-primitives/examples/101-collision/config.yaml";
const string robot_name = "PANDA";

// ui torques and control torques
VectorXd ui_torques;
VectorXd control_torques;
std::vector<std::pair<Vector3d, Vector3d>> collision_pairs;
std::vector<Sai2Primitives::CollisionState> collision_states;

// mutex for global variables between different threads
mutex mutex_torques;
mutex mutex_robot;
mutex mutex_collision;
mutex mutex_object;

// simulation and control loop
void control(shared_ptr<Sai2Model::Sai2Model> robot,
            shared_ptr<Sai2Simulation::Sai2Simulation> sim);
void simulation(shared_ptr<Sai2Model::Sai2Model> robot,
                shared_ptr<Sai2Simulation::Sai2Simulation> sim);

// graphics 
void deleteSphere(cWorld* world, cShapeSphere*& sphere) {
    if (sphere != nullptr) {
        world->removeChild(sphere); // Remove from the world
        delete sphere;              // Free memory
        sphere = nullptr;           // Avoid dangling pointer
    }
}

// // Function to create and add a sphere to the world
// cShapeSphere* addSphere(cWorld* world, double radius, cVector3d position, cColorf color) {
//     cShapeSphere* sphere = new cShapeSphere(radius); // Create sphere
//     sphere->setLocalPos(position);                  // Set position
//     sphere->m_material->setColor(color);            // Set color
//     world->addChild(sphere);                        // Add to world
//     return sphere;                                  // Return pointer to the sphere
// }

// Function to create and add a sphere to the world
void addSphere(cWorld* world, cShapeSphere* sphere, cVector3d position, cColorf color) {
    // cShapeSphere* sphere = new cShapeSphere(radius); // Create sphere
    sphere->setLocalPos(position);                  // Set position
    sphere->m_material->setColor(color);            // Set color
    world->addChild(sphere);                        // Add to world
    // return sphere;                                  // Return pointer to the sphere
}

// // Function to create and add a line to the world
// cShapeLine* addLine(cWorld* world, cVector3d pointA, cVector3d pointB, cColorf colorA, cColorf colorB) {
//     cShapeLine* line = new cShapeLine(pointA, pointB);  // Create line
//     line->m_colorPointA = colorA;  // Set color at point A
//     line->m_colorPointB = colorB;  // Set color at point B
//     world->addChild(line);         // Add to world
//     return line;                   // Return pointer to the line
// }

// Function to create and add a line to the world
cShapeLine* addLine(cWorld* world, cShapeLine* line, cVector3d pointA, cVector3d pointB, cColorf colorA, cColorf colorB) {
    // cShapeLine* line = new cShapeLine(pointA, pointB);  // Create line
    line->m_colorPointA = colorA;  // Set color at point A
    line->m_colorPointB = colorB;  // Set color at point B
    world->addChild(line);         // Add to world
    return line;                   // Return pointer to the line
}

/*
    Control
*/
// bool flag_simulation = true;
bool flag_simulation = false;
Sai2Common::RedisClient* redis_client;
std::string JOINT_ANGLES_KEY = "sai2::FrankaPanda::Juliet::sensors::q";
std::string JOINT_VELOCITIES_KEY = "sai2::FrankaPanda::Juliet::sensors::dq";
std::string JOINT_TORQUES_COMMANDED_KEY = "sai2::FrankaPanda::Juliet::actuators::fgc";
std::string MASS_MATRIX_KEY = "sai2::FrankaPanda::Juliet::sensors::model::massmatrix";
VectorXd q_init = VectorXd::Zero(7);

enum State {
    POSTURE = 0,
    MOTION
};

/*
Optitrack
*/
std::string OBJECT_POS_KEY = "optitrack::rigid_body_pos::0";
std::string OBJECT_ORI_KEY = "optitrack::rigid_body_ori::0";
std::string ROBOT_POS_KEY = "optitrack::rigid_body_pos::1";
std::string ROBOT_ORI_KEY = "optitrack::rigid_body_ori::1";

Eigen::Matrix3d quaternionToRotationMatrix(const VectorXd& quat) {
Eigen::Quaterniond q;
q.x() = quat[0];
q.y() = quat[1];
q.z() = quat[2];
q.w() = quat[3];
return q.toRotationMatrix();
}

const double ROBOT_ROTATION_IN_WORLD = -90;

//------------ main function
int main(int argc, char** argv) {
    Sai2Model::URDF_FOLDERS["EXAMPLE_101_FOLDER"] =
        string(EXAMPLES_FOLDER) + "/101-collision";
    cout << "Loading URDF world model file: " << world_file << endl;

    std::cout << "Loading mesh file: " << mesh_yaml << "\n";

    // set up signal handler
    signal(SIGABRT, &sighandler);
    signal(SIGTERM, &sighandler);
    signal(SIGINT, &sighandler);

    // redis client for graphics
    auto redis_client_graphics = new Sai2Common::RedisClient();
    redis_client_graphics->connect();

    Matrix3d R_robot_in_opti = AngleAxisd(ROBOT_ROTATION_IN_WORLD * M_PI / 180, Vector3d::UnitZ()).toRotationMatrix(); 

    // load graphics scene
    auto graphics = make_shared<Sai2Graphics::Sai2Graphics>(world_file);
    graphics->addUIForceInteraction(robot_name);
    // graphics->showTransparency(true, robot_name, 0.5);
    graphics->setBackgroundColor(.678, .847, .902);

    // add spheres for the number of collision pairs 
    const int n_collision_pairs = 9;
    double radius = 0.01;
    std::vector<std::pair<cShapeSphere*, cShapeSphere*>> chai_spheres;
    std::vector<cShapeLine*> chai_lines;
    for (int i = 0; i < n_collision_pairs; ++i) {
        chai_spheres.push_back(std::make_pair(new cShapeSphere(radius), new cShapeSphere(radius)));
        chai_lines.push_back(new cShapeLine(cVector3d(0, 0, 0), cVector3d(0, 0, 0)));
        addSphere(graphics->getWorld(), chai_spheres.back().first, cVector3d(0, 0, 0), cColorf(1, 0, 0));
        addSphere(graphics->getWorld(), chai_spheres.back().second, cVector3d(0, 0, 0), cColorf(1, 0, 0));
        addLine(graphics->getWorld(), chai_lines.back(), cVector3d(0, 0, 0), cVector3d(0, 0, 0), cColorf(1, 0, 0), cColorf(1, 0, 0));

        collision_pairs.push_back(std::make_pair(Vector3d::Zero(), Vector3d::Zero()));
        collision_states.push_back(Sai2Primitives::CollisionState::SAFE_OBJECT_COLLISION);
    }	

    // load simulation world
    auto sim = make_shared<Sai2Simulation::Sai2Simulation>(world_file);
    // q_init = sim->getJointPositions(robot_name);
    // q_init(6) = - M_PI / 2;
    // q_init << 0, -25, 0, -135, 0, 105, 45;
    // q_init << 0.0877147,-1.36757,-0.042176,-2.23712,-0.0722175,2.401,0.0175131;
    // q_init << -0.112934,-0.39119,0.0967503,-1.99967,-0.0318264,1.7468,0.839011;
    // q_init << 0.134576,-1.11298,-0.0839856,-1.46402,0.00621745,1.69818,-0.0694349;
    q_init << -0.0209861,-0.309057,0.0192885,-1.92814,0.0472275,1.63134,0.77224;
    // q_init *= M_PI / 180;

    // 0.134576, -1.11298, -0.0839856, -1.46402, 0.00621745, 1.69818, -0.0694349
    // 0.0877147, -1.36757, -0.042176, -2.23712, -0.0722175, 2.401, 0.0175131,

    Affine3d object_init_pose = Affine3d::Identity();
    object_init_pose.translation() = Vector3d(1e5, 1e5, 1e5);
    sim->setObjectPose("object", object_init_pose);
    sim->setJointPositions(robot_name, q_init);

    // load robots
    auto robot = make_shared<Sai2Model::Sai2Model>(robot_file, false);
    robot->setQ(q_init);
    robot->updateModel();

    // sim->setJointPositions(robot_name, 0 * robot->q());

    // intitialize global torques variables
    ui_torques = VectorXd::Zero(robot->dof());
    control_torques = VectorXd::Zero(robot->dof());

    // redis client
    redis_client = new Sai2Common::RedisClient();
    redis_client->connect();

    // start the simulation thread first
    fSimulationRunning = true;
    thread sim_thread(simulation, robot, sim);

    // next start the control thread
    thread ctrl_thread(control, robot, sim);

    // while window is open:
    while (graphics->isWindowOpen()) {

        {
            lock_guard<mutex> lock(mutex_object);

            // get robot pose in optitrack 
            Affine3d robot_pose;
            robot_pose.translation() = redis_client_graphics->getEigen(ROBOT_POS_KEY);
            robot_pose.linear() = quaternionToRotationMatrix(redis_client_graphics->getEigen(ROBOT_ORI_KEY));

            // get object pose in optitrack
            Affine3d object_pose;
            object_pose.translation() = redis_client_graphics->getEigen(OBJECT_POS_KEY);
            object_pose.linear() = quaternionToRotationMatrix(redis_client_graphics->getEigen(OBJECT_ORI_KEY));

            object_pose.translation() += Vector3d(0.125, 0, 0.035);  // object offset from marker arrangement 

            // compute relative transform from object to robot pose, in optitrack
            Affine3d object_rel_pose;
            object_rel_pose.translation() = object_pose.translation() - robot_pose.translation();
            object_rel_pose.linear() = object_pose.linear() * robot_pose.linear().transpose();

            // rotate to fit simulation
            object_rel_pose.translation() = R_robot_in_opti * object_rel_pose.translation();
            object_rel_pose.linear() = R_robot_in_opti * object_rel_pose.linear() * R_robot_in_opti.transpose();

            // rotate another rpy (90, 0, 0) to get to collision mesh

            graphics->updateObjectGraphics("object", object_rel_pose);
            sim->setObjectPose("object", object_rel_pose);
        }
        
        {
            lock_guard<mutex> lock(mutex_robot);
            graphics->updateRobotGraphics(robot_name, robot->q());
            // std::cout << "update robot graphics: " << robot->q().transpose() << "\n";
            // graphics->updateObjectGraphics("object", sim->getObjectPose("object"));
        }

        {
            lock_guard<mutex> lock(mutex_collision);
            int cnt = 0;
            for (auto pair : collision_pairs) {
                // change line color if inside distance threshold
                chai_spheres[cnt].first->setLocalPos(pair.first(0), pair.first(1), pair.first(2));
                chai_spheres[cnt].second->setLocalPos(pair.second(0), pair.second(1), pair.second(2));
                chai_lines[cnt]->m_pointA = cVector3d(pair.first(0), pair.first(1), pair.first(2));
                chai_lines[cnt]->m_pointB = cVector3d(pair.second(0), pair.second(1), pair.second(2));
                chai_lines[cnt]->setLineWidth(5);
                if (collision_states[cnt] == Sai2Primitives::CollisionState::SAFE_OBJECT_COLLISION) {
                    chai_lines[cnt]->m_colorPointA = cColorf(0, 1, 0);
                    chai_lines[cnt]->m_colorPointB = cColorf(0, 1, 0);
                } else {
                    chai_lines[cnt]->m_colorPointA = cColorf(1, 0, 0);
                    chai_lines[cnt]->m_colorPointB = cColorf(1, 0, 0);
                }
                cnt++;
            }
        }

        graphics->renderGraphicsWorld();
        {
            lock_guard<mutex> lock(mutex_torques);
            ui_torques = graphics->getUITorques(robot_name);
        }
    }

    // stop simulation
    fSimulationRunning = false;
    sim_thread.join();
    ctrl_thread.join();

    return 0;
}

//------------------------------------------------------------------------------
void control(shared_ptr<Sai2Model::Sai2Model> robot,
            shared_ptr<Sai2Simulation::Sai2Simulation> sim) {
    // update robot model and initialize control vectors
    robot->updateModel();
    int dof = robot->dof();
    MatrixXd N_prec = MatrixXd::Identity(dof, dof);

    // joint handler 
    auto joint_handler = make_unique<Sai2Primitives::JointHandler>(robot);

    // self collision
    auto collision_handler = make_unique<Sai2Primitives::Collision>(robot, mesh_yaml);

    // Position plus orientation task
    string link_name = "end-effector";
    Vector3d pos_in_link = Vector3d(0.0, 0.0, 0.107 + 0.1);
    Affine3d compliant_frame = Affine3d(Translation3d(pos_in_link));

    // Full motion force task
    auto motion_force_task = make_unique<Sai2Primitives::MotionForceTask>(
        robot, link_name, compliant_frame);

    // // Partial motion force task
    // vector<Vector3d> controlled_directions_translation = {
    // 	Vector3d::UnitX(), Vector3d::UnitY(), Vector3d::UnitZ()};
    // vector<Vector3d> controlled_directions_rotation = {};
    // auto motion_force_task = make_shared<Sai2Primitives::MotionForceTask>(
    // 	robot, link_name, controlled_directions_translation,
    // 	controlled_directions_rotation);
    // // motion_force_task->setSingularityGains(20, 20);

    motion_force_task->setPosControlGains(100, 20, 0);
    motion_force_task->setOriControlGains(400, 20, 0);
    motion_force_task->disableInternalOtg();
    motion_force_task->enableVelocitySaturation(0.3);
    // motion_force_task->disableSingularityHandling();
    VectorXd motion_force_task_torques = VectorXd::Zero(dof);

    // no gains setting here, using the default task values
    Matrix3d initial_orientation = robot->rotation(link_name);
    Vector3d initial_position = robot->position(link_name, pos_in_link);

    // joint task to control the redundancy
    // using default gains and interpolation settings
    auto joint_task = make_unique<Sai2Primitives::JointTask>(robot);
    joint_task->setGoalPosition(q_init);
    joint_task->disableInternalOtg();
    joint_task->enableVelocitySaturation(M_PI / 3);
    joint_task->setGains(200, 20);
    VectorXd joint_task_torques = VectorXd::Zero(dof);

    VectorXd initial_q = robot->q();
    // joint_task->setGoalPosition(initial_q);

    // apf joint limits 
    auto joint_limits = robot->jointLimits();
    VectorXd q_min(robot->dof()), q_max(robot->dof());
    int cnt = 0;
    for (auto limit : joint_limits) {
        q_min(cnt) = limit.position_lower + 0.5;
        q_max(cnt) = limit.position_upper - 0.5;
        cnt++;
    }
    double eta = 0.5;
    // VectorXd q_init = robot->q();
    VectorXd q_delta = VectorXd::Zero(robot->dof());
    q_delta(5) = 5;

    // desired position offsets 
    vector<Vector3d> desired_offsets {Vector3d(0, 0.5, 0), Vector3d(0, 0, 0), 
                                    Vector3d(0, 0.5, 0), Vector3d(0, 0, 0), 
                                    Vector3d(0, 0.5, 0), Vector3d(0, 0, 0),
                                    Vector3d(0, 0.5, 0), Vector3d(0, 0, 0)};
    // vector<Vector3d> desired_offsets {Vector3d(2, 0, 0)};
    double t_initial = 5;
    vector<double> t_wait {5, 5};
    // double t_wait = 10;  // wait between switching desired positions 
    // double t_reset_wait = 5;  // wait when resetting position 
    double prev_time = 0;
    // int cnt = 6 * 1;
    cnt = 0;
    int max_cnt = desired_offsets.size();
    int state = POSTURE;

    int n_meshes = collision_handler->getCollisionStates().size();
    int constraint_flag = 0;

    // create logger
    Sai2Common::Logger logger("collision-avoidance-floating", false);
    VectorXd svalues = VectorXd::Zero(6);
    VectorXd robot_q = robot->q();
    Vector3d ee_pos = motion_force_task->getCurrentPosition();
    Vector3d goal_pos = motion_force_task->getGoalPosition();
    VectorXi collision_state = VectorXi::Zero(n_meshes);
    VectorXd collision_distance = VectorXd::Zero(n_meshes);	
    Vector3d closest_point_in_world = Vector3d::Zero();
    double closest_distance = 0;

    // logger.addToLog(svalues, "svalues");
    logger.addToLog(robot_q, "robot_q");
    logger.addToLog(ee_pos, "ee_pos");
    logger.addToLog(goal_pos, "goal_pos");
    logger.addToLog(constraint_flag, "constraint_flag");
    logger.addToLog(collision_state, "collision_state");
    logger.addToLog(collision_distance, "collision_distance");
    logger.addToLog(closest_distance, "collision_closest_distance");
    logger.addToLog(closest_point_in_world, "collision_closest_point_in_world");
    logger.start();

    // create a loop timer
    double control_freq = 1000;
    Sai2Common::LoopTimer timer(control_freq, 1e6);

    while (fSimulationRunning) {
        timer.waitForNextLoop();
        const double time = timer.elapsedSimTime();

        // read joint positions, velocities, update model
        if (flag_simulation) {
            { 
                lock_guard<mutex> lock(mutex_robot);
                robot->setQ(sim->getJointPositions(robot_name));
                robot->setDq(sim->getJointVelocities(robot_name));
                robot->updateModel();

                robot_q = robot->q();
            }
        } else {
            robot->setQ(redis_client->getEigen(JOINT_ANGLES_KEY));
            robot->setDq(redis_client->getEigen(JOINT_VELOCITIES_KEY));
            MatrixXd M = redis_client->getEigen(MASS_MATRIX_KEY);
            M(4, 4) += 0.15;
            M(5, 5) += 0.15;
            M(6, 6) += 0.15;
            robot->updateModel(M);

            robot_q = robot->q();
        }

        // state machine
        if (state == POSTURE) {

            // std::cout << "Posture: " << joint_task->getGoalPosition().transpose() << "\n";;

            N_prec = MatrixXd::Identity(dof, dof);
            joint_task->updateTaskModel(N_prec);
            joint_task->setGains(200, 20, 10);

            {
                lock_guard<mutex> lock(mutex_torques);
                control_torques = joint_task->computeTorques();
                redis_client->setEigen(JOINT_TORQUES_COMMANDED_KEY, control_torques);
            }			

            if ((robot->q() - joint_task->getGoalPosition()).norm() < 5e-2) {
                state = MOTION;
                motion_force_task->reInitializeTask();
                joint_task->reInitializeTask();

                joint_task->setGains(100, 20, 0);

                initial_position = motion_force_task->getCurrentPosition();
            }

        } else if (state == MOTION) {

            // log 
            ee_pos = motion_force_task->getCurrentPosition();
            goal_pos = motion_force_task->getGoalPosition();

            // update dynamic object 
            auto object_transform = sim->getObjectPose("object");
            Affine3d collision_transform = Affine3d::Identity();
            collision_transform.linear() = AngleAxisd(M_PI / 2, Vector3d::UnitX()).toRotationMatrix();
            collision_transform.translation() = Vector3d(0, 0, 0);
            collision_handler->setObjectTransform(object_transform * collision_transform, 0);

            // update tasks model. Order is important to define the hierarchy
            N_prec = MatrixXd::Identity(dof, dof);
            {
                lock_guard<mutex> lock(mutex_robot);
                joint_handler->updateTaskModel(N_prec);
                collision_handler->updateTaskModel(N_prec);
                motion_force_task->updateTaskModel(N_prec);
            }
            N_prec = motion_force_task->getTaskAndPreviousNullspace();
            // after each task, need to update the nullspace
            // of the previous tasks in order to garantee
            // the dynamic consistency

            joint_task->updateTaskModel(N_prec);

            // -------- set task goals and compute control torques
            // position: move to workspace extents 
            // motion_force_task->setGoalPosition(initial_position + desired_offsets[0]);
            if (time - prev_time > t_wait[cnt % 2]) {
                // motion_force_task->setGoalPosition(initial_position + desired_offsets[cnt]);
                cnt++;
                prev_time = time;
                if (cnt == max_cnt) cnt = max_cnt - 1;
            }
            // motion_force_task->setGoalLinearVelocity(Vector3d::Zero());
            // motion_force_task->setGoalLinearAcceleration(Vector3d::Zero());

            // joint_task->setGoalPosition(q_init + q_delta);

            // compute torques without self collision handler nullspace 
            VectorXd motion_force_task_torques_without_handler = motion_force_task->computeTorques();
            VectorXd joint_task_torques_without_handler = joint_task->computeTorques();

            // // compute torques with self collision handler nullspace
            // motion_force_task->updateTaskModel(self_collision_handler->getTaskAndPreviousNullspace());
            // joint_task->updateTaskModel(motion_force_task->getTaskAndPreviousNullspace());
            // motion_force_task_torques = motion_force_task->computeTorques();
            // joint_task_torques = joint_task->computeTorques();
            // VectorXd collision_handler_constraint_torques = \
                // self_collision_handler->computeTorques(motion_force_task_torques_without_handler + joint_task_torques_without_handler, true);

            // // compute torques for the different tasks
            // {
            // 	lock_guard<mutex> lock(mutex_robot);
            // 	motion_force_task_torques = motion_force_task->computeTorques();
            // 	joint_task_torques = joint_task->computeTorques();
            // }

            // // compute apf torques 
            // VectorXd apf_force = VectorXd::Zero(robot->dof());
            // for (int i = 0; i < robot->dof(); ++i) {
            //     // if (i == 3) {
            //         double rho_lower = robot->q()(i) - q_min(i);
            //         double rho_upper = q_max(i) - robot->q()(i);
            //         if (rho_lower < 0.08) {
            //             apf_force(i) = eta * std::abs(((1 / rho_lower) - (1 / 0.08))) * (1 / (rho_lower * rho_lower));
            //         } else if (rho_upper < -0.08) {
            //             apf_force(i) = - eta * std::abs(((1 / rho_upper) - (1 / (-0.08)))) * (1 / (rho_upper * rho_upper));
            //         }
            //     // }
            // }

            //------ compute the final torques
            {
                lock_guard<mutex> lock(mutex_torques);
                // control_torques = joint_handler->computeTorques(self_collision_handler->computeTorques(motion_force_task_torques_without_handler + joint_task_torques_without_handler));
                control_torques = 1 * collision_handler->computeTorques(0 * motion_force_task_torques_without_handler + 0 * joint_task_torques_without_handler);
                // control_torques = collision_handler->computeTorques(joint_task_torques_without_handler);
                // control_torques += joint_task_torques_without_handler;
                // control_torques = motion_force_task_torques_without_handler + joint_task_torques_without_handler;
                // control_torques = collision_handler_constraint_torques + motion_force_task_torques + joint_task_torques;
                // control_torques = motion_force_task_torques + joint_task_torques;
                if (!flag_simulation) {
                    redis_client->setEigen(JOINT_TORQUES_COMMANDED_KEY, 1 * control_torques);
                }
            }
        }

        //------ draw collision points with line between the two 
        {
            lock_guard<mutex> lock(mutex_collision);
            collision_pairs = collision_handler->getCollisionPoints();
            collision_states = collision_handler->getCollisionStates();
            auto collision_distances = collision_handler->getDistances();
            if (std::accumulate(collision_states.begin(), collision_states.end(), 0) != 0) {
                constraint_flag = 1;
            } else {
                constraint_flag = 0;
            }
            // convert collision state to VectorXi
            for (int i = 0; i < collision_states.size(); ++i) {
                collision_state(i) = static_cast<int>(collision_states[i]);
            }
            // convert collision distances to VectorXd
            for (int i = 0; i < collision_distances.size(); ++i) {
                collision_distance(i) = collision_distances[i];
            }
            
            // only extract the closest point in the world 
            Eigen::Index min_index;
            double min_distance = collision_distance.minCoeff(&min_index);
            closest_point_in_world = collision_pairs[min_index].first;
            closest_distance = min_distance;
        }

        // MatrixXd Jc = MatrixXd::Zero(1, robot->dof());
        // Jc(0) = 1;
        // MatrixXd force_projection = Jc * robot->dynConsistentInverseJacobian(Jc);
        // std::cout << force_projection.transpose() << "\n";

        // -------------------------------------------
        if (timer.elapsedCycles() % 500 == 0) {
            cout << "time: " << time << endl;
            cout << "position error : "
                << (motion_force_task->getGoalPosition() -
                    motion_force_task->getCurrentPosition())
                        .norm()
                << endl;
            cout << endl;
        }
    }
    logger.stop();
    timer.stop();
    cout << "\nControl loop timer stats:\n";
    timer.printInfoPostRun();
}

//------------------------------------------------------------------------------
void simulation(shared_ptr<Sai2Model::Sai2Model> robot,
                shared_ptr<Sai2Simulation::Sai2Simulation> sim) {
    fSimulationRunning = true;

    sim->disableJointLimits(robot_name);

    // create a timer
    double sim_freq = 2000;
    Sai2Common::LoopTimer timer(sim_freq);

    sim->setTimestep(1.0 / sim_freq);

    if (flag_simulation) {
        while (fSimulationRunning) {
            timer.waitForNextLoop();
            {
                lock_guard<mutex> lock(mutex_torques);
                sim->setJointTorques(robot_name, control_torques + ui_torques);
            }
            sim->integrate();
        }
        timer.stop();
        cout << "\nSimulation loop timer stats:\n";
        timer.printInfoPostRun();
    } else {
        while (fSimulationRunning) {
            timer.waitForNextLoop();
        }
    }
}