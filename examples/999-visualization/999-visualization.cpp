/**
 * @file 999-visualization.cpp 
 * @author William Chong (wmchong@stanford.edu)
 * @brief 
 * @version 0.1
 * @date 2025-03-08
 * 
 * @copyright Copyright (c) 2025
 * 
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
#include "timer/LoopTimer.h"
#include "logger/Logger.h"
bool fSimulationRunning = false;
void sighandler(int) { fSimulationRunning = false; }

using namespace std;
using namespace Eigen;
using namespace chai3d;

// config file names and object names
const string world_file = "${EXAMPLE_999_FOLDER}/world.urdf";
// const string robot_file =
//     "${SAI2_MODEL_URDF_FOLDER}/panda/panda_arm_gripper_fixed.urdf";
// const string robot_file =
//     "${SAI2_MODEL_URDF_FOLDER}/mmp_panda/mmp_panda.urdf";
const string robot_file =
    "${SAI2_MODEL_URDF_FOLDER}/mmp_panda/mmp_newarm.urdf";
const string robot_name = "PANDA";

// ui torques and control torques
VectorXd ui_torques;
VectorXd control_torques;

// mutex for global variables between different threads
mutex mutex_torques;
mutex mutex_robot;

// simulation and control loop
void control(shared_ptr<Sai2Model::Sai2Model> robot,
			 shared_ptr<Sai2Simulation::Sai2Simulation> sim);
void simulation(shared_ptr<Sai2Model::Sai2Model> robot,
				shared_ptr<Sai2Simulation::Sai2Simulation> sim);

		// Function to get mesh by name
	cMesh* getMeshByName(cMultiMesh* multiMesh, const std::string& name) {
		for (size_t i = 0; i < multiMesh->getNumMeshes(); i++) {
			cMesh* mesh = multiMesh->getMesh(i);
			std::cout << mesh->m_name << "\n";
			if (mesh->m_name == name) { // Compare name
				return mesh;
			}
		}
		return nullptr; // Not found
	}

//------------ main function
int main(int argc, char** argv) {
	Sai2Model::URDF_FOLDERS["EXAMPLE_999_FOLDER"] =
		string(EXAMPLES_FOLDER) + "/999-visualization";
	cout << "Loading URDF world model file: " << world_file << endl;

	// set up signal handler
	signal(SIGABRT, &sighandler);
	signal(SIGTERM, &sighandler);
	signal(SIGINT, &sighandler);

	// load graphics scene
	auto graphics = make_shared<Sai2Graphics::Sai2Graphics>(world_file);
	graphics->addUIForceInteraction(robot_name);
    graphics->setBackgroundColor(.678, .847, .902);
	// graphics->showTransparency(true, robot_name, 0.5);

    // add zone boundary meshes 
    
    // Create an ellipsoid (which will appear as an ellipse)
    // Vector3d ee_pos = Vector3d(0.39947, -5.59271e-12, 0.381585);
	Vector3d ee_pos = Vector3d(0.895759, -1.62915 + 0.03, 0.849251);
    Vector3d vector_direction = Vector3d(1, -0.1, -1).normalized();
    double x_radius = 0.1;
    double y_radius = 0.1;
    double z_radius = 0.2;
    chai3d::cShapeEllipsoid* inner_ellipse = new chai3d::cShapeEllipsoid(x_radius, y_radius, z_radius);
    chai3d::cShapeEllipsoid* outer_ellipse = new chai3d::cShapeEllipsoid(x_radius + 0.05, y_radius + 0.05, z_radius + 0.05);
    chai3d::cShapeLine* line = new chai3d::cShapeLine(chai3d::cVector3d(ee_pos), chai3d::cVector3d(ee_pos + 0.5 * vector_direction));
    chai3d::cShapeSphere* sphere = new chai3d::cShapeSphere(0.005);

	chai3d::cShapeSphere* workspace_sphere = new chai3d::cShapeSphere(0.88);
	workspace_sphere->setLocalPos(chai3d::cVector3d(0.878256, -1.12523, 0.435379));
	workspace_sphere->m_material->setBlue();
	workspace_sphere->setWireMode(true);

    // Set wireframe mode if needed
    inner_ellipse->setWireMode(true); // Enable wireframe view
    inner_ellipse->m_material->setRed(); // Set color

    outer_ellipse->setWireMode(true); // Enable wireframe view
    outer_ellipse->m_material->setOrange(); // Set color

    line->m_colorPointA.setGreen();
    line->m_colorPointB.setGreen();
    line->setLineWidth(10.0);
    sphere->m_material->setGreen();
    sphere->setLocalPos(chai3d::cVector3d(ee_pos + 0.5 * vector_direction));
    // sphere->setRadius(0.05);

    // inner_ellipse->setLocalPos(chai3d::cVector3d(0.5, 0, 0.08));
    // outer_ellipse->setLocalPos(chai3d::cVector3d(0.5, 0, 0.08));
    inner_ellipse->setLocalPos(chai3d::cVector3d(ee_pos));
    outer_ellipse->setLocalPos(chai3d::cVector3d(ee_pos));

    // Add the ellipse to the world
    graphics->_world->addChild(inner_ellipse);
    graphics->_world->addChild(outer_ellipse);
    graphics->_world->addChild(line);
	// graphics->_world->addChild(workspace_sphere);
    // graphics->_world->addChild(sphere);

    // create robot workspace 
    double x_work_outer = 0.9;
    double y_work_outer = 0.9;
    double z_work_outer = 0.9;
    chai3d::cShapeEllipsoid* outer_workspace = new chai3d::cShapeEllipsoid(x_work_outer, y_work_outer, z_work_outer);

    double x_work_inner = 0.4;
    double y_work_inner = 0.4;
    double z_work_inner = 0.4;
    chai3d::cShapeEllipsoid* inner_workspace = new chai3d::cShapeEllipsoid(x_work_inner, y_work_inner, z_work_inner);

	inner_workspace->setLocalPos(cVector3d(0.854406, -1.14437, 0.720639));
    inner_workspace->setWireMode(true);
    inner_workspace->m_material->setRed();

	outer_workspace->setLocalPos(cVector3d(0.854406, -1.14437, 0.720639));
    outer_workspace->setWireMode(true);
    outer_workspace->m_material->setRed();

	// add mesh for workspace
	// Create a new mesh object
    cMultiMesh* object = new cMultiMesh();

    // Load the OBJ file
    bool success = object->loadFromFile("../../../openGJK/utils/object_meshes/workspace_full_red.obj");

    if (!success) {
        std::cerr << "Failed to load OBJ file!" << std::endl;
        return -1;
    }

    // Add the object to the world
    graphics->_world->addChild(object);

    // Set object properties (optional)
    object->setLocalPos(0.854406, -1.14437, 0.720639 - 0.3); // Position
    // object->setUseCulling(true);        // Enable back-face culling
    // object->setUseTransparency(true);   // Enable transparency if needed
	object->setWireMode(true);
	object->scale(1.0);

    // graphics->_world->addChild(inner_workspace);
    // graphics->_world->addChild(outer_workspace);

	// load simulation world
	auto sim = make_shared<Sai2Simulation::Sai2Simulation>(world_file);

	// load robots
	auto robot = make_shared<Sai2Model::Sai2Model>(robot_file, false);
	robot->setQ(sim->getJointPositions(robot_name));
	robot->updateModel();

    // std::cout << "ee pos: " << robot->position("link7", Vector3d(0, 0, 0)).transpose();
	// std::cout << "link 2 transform: " << robot->transform("link2").linear() << "\n" << robot->transform("link2").translation().transpose() << "\n";


	// intitialize global torques variables
	ui_torques = VectorXd::Zero(robot->dof());
	control_torques = VectorXd::Zero(robot->dof());

	// start the simulation thread first
	fSimulationRunning = true;
	thread sim_thread(simulation, robot, sim);

	// next start the control thread
	// thread ctrl_thread(control, robot, sim);

	// while window is open:
	while (graphics->isWindowOpen()) {
		{
			lock_guard<mutex> lock(mutex_robot);
			graphics->updateRobotGraphics(robot_name, robot->q());
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
	// ctrl_thread.join();

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

	// Position plus orientation task
	string link_name = "end-effector";
	Vector3d pos_in_link = Vector3d(0.0, 0.0, 0.07);
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
	// motion_force_task->setSingularityGains(20, 20);

    motion_force_task->disableInternalOtg();
    motion_force_task->enableVelocitySaturation(0.2);
	VectorXd motion_force_task_torques = VectorXd::Zero(dof);

	// no gains setting here, using the default task values
	const Matrix3d initial_orientation = robot->rotation(link_name);
	const Vector3d initial_position = robot->position(link_name, pos_in_link);

	// joint task to control the redundancy
	// using default gains and interpolation settings
	auto joint_task = make_unique<Sai2Primitives::JointTask>(robot);
    joint_task->disableInternalOtg();
    // joint_task->enableVelocitySaturation(1.5 * M_PI);
    joint_task->enableVelocitySaturation(1.0);
    joint_task->setGains(200, 20);
	joint_task->setDynamicDecouplingType(Sai2Primitives::FULL_DYNAMIC_DECOUPLING);
	VectorXd joint_task_torques = VectorXd::Zero(dof);

	VectorXd initial_q = robot->q();
    joint_task->setGoalPosition(initial_q);

    // apf joint limits
    auto joint_limits = robot->jointLimits();
    VectorXd q_min(robot->dof()), q_max(robot->dof());
    int cnt = 0;
    for (auto limit : joint_limits) {
        q_min(cnt) = limit.position_lower + 0.4;
        q_max(cnt) = limit.position_upper - 0.4;
        cnt++;
    }
    // double eta = 0.5;
    double eta = 0.5;
    // double eta = 10.0;
    VectorXd q_init = robot->q();
	q_init(5) = M_PI / 2;
    VectorXd q_delta = VectorXd::Zero(robot->dof());
	q_delta(3) = 5;
    // q_delta(5) = 5;
	double sign_switch = 1;
	VectorXd q_oscillation = VectorXd::Zero(robot->dof());

    // desired position offsets
    // vector<Vector3d> desired_offsets {Vector3d(2, 0, 0), Vector3d(0, 0, 0),
    //                                   Vector3d(0, 2, 0), Vector3d(0, 0, 0),
    //                                   Vector3d(0, -2, 0), Vector3d(0, 0, 0),
    //                                   Vector3d(0, 0, 2), Vector3d(0, 0, 0)};
	vector<Vector3d> desired_offsets {Vector3d(0, 2, 0), Vector3d(0, 0, 0),
                                      Vector3d(0, -2, 0), Vector3d(0, 0, 0),
                                      Vector3d(0, 0, 2), Vector3d(0, 0, 0)};
    // vector<Vector3d> desired_offsets {Vector3d(2, 0, 0)};
	double t_initial = 2;
	vector<double> t_wait {5, 5};
    // double t_wait = 10;  // wait between switching desired positions
	// double t_reset_wait = 5;  // wait when resetting position
    double prev_time = 0;
    // int cnt = 6 * 1;
	cnt = 0;
    int max_cnt = desired_offsets.size();

	// create logger
	Sai2Common::Logger logger("joints", false);
	VectorXd svalues = VectorXd::Zero(6);
    VectorXd robot_q = robot->q();
	VectorXd robot_dq = robot->dq();
	VectorXd robot_torque = VectorXd::Zero(robot->dof());
	VectorXi joint_pos_state = VectorXi::Zero(robot->dof());
	VectorXi joint_vel_state = VectorXi::Zero(robot->dof());
	// VectorXd joint_task_torques = VectorXd::Zero(robot->dof());
	int constraint_flag = 0;
	// logger.addToLog(svalues, "svalues");
    logger.addToLog(robot_q, "robot_q");
	logger.addToLog(robot_dq, "robot_dq");
	logger.addToLog(robot_torque, "robot_torque");
	logger.addToLog(joint_pos_state, "joint_pos_state");
	logger.addToLog(joint_vel_state, "joint_vel_state");
	logger.start(1000);

	// create a loop timer
	double control_freq = 1000;
	Sai2Common::LoopTimer timer(control_freq, 1e6);

	while (fSimulationRunning) {
		timer.waitForNextLoop();
		const double time = timer.elapsedSimTime();

		// read joint positions, velocities, update model
		robot->setQ(sim->getJointPositions(robot_name));
		robot->setDq(sim->getJointVelocities(robot_name));
		robot->updateModel();
		// MatrixXd M = robot->M();
		// M.bottomRightCorner(3, 3) += 0.15 * Matrix3d::Identity();
		// robot->updateModel(M);

        robot_q = robot->q();
		robot_dq = robot->dq();

		// update tasks model. Order is important to define the hierarchy
		N_prec = MatrixXd::Identity(dof, dof);
		joint_handler->updateTaskModel(N_prec);
		{
			lock_guard<mutex> lock(mutex_robot);
			joint_task->updateTaskModel(N_prec);
			// motion_force_task->updateTaskModel(N_prec);
		}
		// N_prec = motion_force_task->getTaskAndPreviousNullspace();
		// after each task, need to update the nullspace
		// of the previous tasks in order to garantee
		// the dynamic consistency

		double freq = 0.2;
		// q_oscillation(0) = 0.2 * sin(2 * M_PI * freq * time);
		// q_oscillation(1) = 0.2 * sin(2 * M_PI * freq * time);
		// q_oscillation(5) = 0.1 * sin(2 * M_PI * freq * time);

		// -------- set task goals and compute control torques
		// position: move to workspace extents
        if (time - prev_time > t_wait[cnt % 2]) {
            // motion_force_task->setGoalPosition(initial_position + desired_offsets[cnt]);
			sign_switch *= -1;
			q_delta(3) = sign_switch * 5;
			// q_delta(5) = - sign_switch * 5;
            cnt++;
            prev_time = time;
            if (cnt == max_cnt) cnt = max_cnt - 1;
        }
        // motion_force_task->setGoalLinearVelocity(Vector3d::Zero());
        // motion_force_task->setGoalLinearAcceleration(Vector3d::Zero());

        joint_task->setGoalPosition(q_init + q_delta + q_oscillation);
		joint_task_torques = joint_task->computeTorques();

        // // compute apf torques
        // VectorXd apf_force = VectorXd::Zero(robot->dof());
		// VectorXd custom_apf_force = VectorXd::Zero(robot->dof());
		// MatrixXd J_c = MatrixXd::Zero(1, robot->dof());
		// VectorXd apf_damping_force = VectorXd::Zero(robot->dof());
		// MatrixXd selection = MatrixXd::Zero(robot->dof(), robot->dof());
		// double zone_width = 0.16;
		// double alpha = 0;
		// bool constraint_flag = false;
        // for (int i = 0; i < robot->dof(); ++i) {
        //     if (i == 5) {
        //         double rho_lower = robot->q()(i) - q_min(i);
        //         double rho_upper = q_max(i) - robot->q()(i);
        //         if (rho_lower < zone_width) {
        //             apf_force(i) = eta * std::abs(((1 / rho_lower) - (1 / zone_width))) * (1 / (rho_lower * rho_lower));
		// 			J_c(0, i) = 1;
		// 			alpha = std::clamp(std::abs((robot->q()(i) - q_min(i) - zone_width) / zone_width), 0.0, 1.0);
		// 			// alpha = std::clamp(std::abs(zone_width / rho_lower), 0.0, 1.0);
		// 			// alpha = std::clamp(std::abs((robot->q()(i) - q_min(i)) / zone_width), 0.0, 1.0);
		// 			// std::cout << "alpha lower: " << alpha << "\n";
		// 			// alpha = std::abs(rho_lower / 0.08);
		// 			custom_apf_force(i) = 12. * alpha * alpha;
		// 			constraint_flag = true;
		// 			selection(i, i) = 1;
        //         } else if (rho_upper < -zone_width) {
        //             apf_force(i) = - eta * std::abs(((1 / rho_upper) - (1 / (zone_width)))) * (1 / (rho_upper * rho_upper));
		// 			J_c(0, i) = 1;
		// 			alpha = std::clamp(std::abs((robot->q()(i) - q_max(i) - zone_width) / zone_width), 0.0, 1.0);
		// 			// alpha = std::clamp(std::abs(rho_upper / zone_width), 0.0, 1.0);
		// 			// alpha = std::clamp(std::abs((robot->q()(i) - q_max(i)) / zone_width), 0.0, 1.0);
		// 			// std::cout << "value: " << std::abs(zone_width / rho_upper) << "\n";
		// 			// std::cout << "alpha upper: " << alpha << "\n";
		// 			// alpha = std::abs(rho_upper / 0.08);
		// 			custom_apf_force(i) = -12. * alpha * alpha;
		// 			constraint_flag = true;
		// 			selection(i, i) = 1;
        //         }
        //     }
        // }
        // // std::cout << "apf force: " << apf_force.transpose() << "\n";

        // joint_task->updateTaskModel(MatrixXd::Identity(robot->dof(), robot->dof()));  // original
		// VectorXd joint_task_torques_without_nullspace = joint_task->computeTorques();

		// // if (J_c.sum() != 0) {
		// if (constraint_flag) {
		// 	// std::cout << "truncation\n";
		// 	joint_task->updateTaskModel(robot->nullspaceMatrix(J_c));  // nullspace of constraint
		// } else {
	    //     joint_task->updateTaskModel(MatrixXd::Identity(robot->dof(), robot->dof()));  // original
		// }

		// // compute torques for the different tasks
		// // motion_force_task_torques = motion_force_task->computeTorques();
		// joint_task_torques = joint_task->computeTorques();
		// VectorXd leaky_torque = VectorXd::Zero(robot->dof());
		// // if (J_c.sum() != 0) {
		// if (constraint_flag) {
		// 	leaky_torque = (std::pow(1 - alpha, 2)) * J_c.transpose() * robot->dynConsistentInverseJacobian(J_c).transpose() * joint_task_torques_without_nullspace;
		// 	// leaky_torque = (std::pow(alpha, 2)) * joint_task_torques_without_nullspace;
		// 	apf_damping_force = - 20 * robot->M() * robot->dq();
		// 	for (int i = 0; i < robot->dof(); ++i) {
		// 		if (J_c(0, i) != 1) {
		// 			// leaky_torque(i) = 0;
		// 			apf_damping_force(i) = 0;
		// 		}
		// 	}
		// }

		//------ compute the final torques
		{
			lock_guard<mutex> lock(mutex_torques);
			// control_torques = joint_handler->computeTorques(motion_force_task_torques + joint_task_torques);
			// control_torques = motion_force_task_torques + joint_task_torques + apf_force;
			// control_torques = joint_task_torques + J_c.transpose() * robot->M() * apf_force;
			// control_torques = joint_task_torques + custom_apf_force + leaky_torque + apf_damping_force;  // proposed method 
			// control_torques = joint_task_torques + J_c.transpose() * selection * robot->M() * apf_force + leaky_torque;
			// control_torques = joint_task_torques_without_nullspace + J_c.transpose() * selection * robot->M() * apf_force + apf_damping_force;  // normal method
			// control_torques = joint_task_torques + J_c.transpose() * selection * robot->M() * apf_force + 1 * apf_damping_force;  // truncation
			// control_torques = joint_task_torques + custom_apf_force + apf_damping_force;  // truncation with custom apf 
			// control_torques = joint_task_torques;
			control_torques = joint_handler->computeTorques(joint_task_torques) + 1 * robot->coriolisForce();
			// control_torques = joint_task_torques_without_nullspace;
			// saturate control torques
			// std::cout << "apf forces: " << apf_force.transpose() << "\n";
			// std::cout << "control torques: " << control_torques.transpose() << "\n";
			// std::cout << "alpha: " << alpha << "\n";
			// std::cout << "custom apf force: " << custom_apf_force.transpose() << "\n";
		}
		robot_torque = control_torques;

		// log joint state 
		auto joint_states = joint_handler->getJointState();
		joint_pos_state = joint_states.first;
		joint_vel_state = joint_states.second;

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
}