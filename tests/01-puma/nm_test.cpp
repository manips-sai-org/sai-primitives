/**
 * @file main.cpp
 * @author William Chong (wmchong@stanford.edu)
 * @brief 
 * @version 0.1
 * @date 2025-12-12
 * 
 * @copyright Copyright (c) 2025
 * 
 */

/*
    Test nelder-mead routine at multi-singularity
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
#include "timer/LoopTimer.h"
#include "logger/Logger.h"
bool fSimulationRunning = false;
void sighandler(int) { fSimulationRunning = false; }

using namespace std;
using namespace Eigen;

const string world_file = "${TEST_1_FOLDER}/world.urdf";
const string robot_file = "${SAI2_MODEL_URDF_FOLDER}/puma/puma.urdf";
const string robot_name = "PUMA";  // name in the world file

// ui torques and control torques
VectorXd ui_torques;
VectorXd control_torques;

// mutex for global variables between different threads
mutex mutex_torques;
mutex mutex_robot;

// switch for which type 2 singularity
bool flag_overhead_singularity = false;

enum State {
	GO_TO_SINGULARITY,
	EXIT_SINGULARITY
};

// simulation and control loop
void control(shared_ptr<Sai2Model::Sai2Model> robot,
			 shared_ptr<Sai2Simulation::Sai2Simulation> sim);
void simulation(shared_ptr<Sai2Model::Sai2Model> robot,
				shared_ptr<Sai2Simulation::Sai2Simulation> sim);

//------------ main function
int main(int argc, char** argv) {
	Sai2Model::URDF_FOLDERS["TEST_1_FOLDER"] =
		string(TESTS_FOLDER) + "/01-puma";
	cout << "Loading URDF world model file: " << world_file << endl;

	// set up signal handler
	signal(SIGABRT, &sighandler);
	signal(SIGTERM, &sighandler);
	signal(SIGINT, &sighandler);

	// load graphics scene
	auto graphics = make_shared<Sai2Graphics::Sai2Graphics>(world_file);
	graphics->addUIForceInteraction(robot_name);
	// graphics->showTransparency(true, robot_name, 0.5);
	graphics->showLinkFrame(true, robot_name, "end-effector", 0.25);

	// load simulation world
	auto sim = make_shared<Sai2Simulation::Sai2Simulation>(world_file);
	VectorXd init_q = sim->getJointPositions(robot_name);

	/*
		Type 2 described in Marcelo's paper 
	*/
	init_q(1) = - M_PI / 4;
	init_q(2) = M_PI / 4 + M_PI / 2;
	init_q(3) = M_PI / 2;
	init_q(4) = 0;
	init_q(5) = M_PI;

    /*
        Simultaneous type 1 and 2
    */
    init_q << 0, 0, M_PI / 2, M_PI / 2, 0, 0;

	// /*
	// 	Extended elbow condition 
	// */
	// init_q(1) = 0;
	// init_q(2) = M_PI / 2;
	// init_q(3) = 0;
	// init_q(4) = 0;

	sim->setJointPositions(robot_name, init_q);

	// load robots
	auto robot = make_shared<Sai2Model::Sai2Model>(robot_file, false);
	robot->setQ(sim->getJointPositions(robot_name));
	robot->updateModel();

    // motion force task
	string link_name = "end-effector";
	// Vector3d pos_in_link = Vector3d(0.0, 0.0, 0.07);
	Vector3d pos_in_link = Vector3d(0.0, 0.0, 0.0);
	Affine3d compliant_frame = Affine3d(Translation3d(pos_in_link));

	// Full motion force task
	auto motion_force_task = make_unique<Sai2Primitives::MotionForceTask>(
		robot, link_name, compliant_frame);

    // Test classification update 
    MatrixXd N_prec = MatrixXd::Identity(robot->dof(), robot->dof());
    motion_force_task->updateTaskModel(N_prec);

	return 0;
}
