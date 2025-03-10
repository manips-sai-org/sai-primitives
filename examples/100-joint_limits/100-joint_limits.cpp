/**
 * @file 100-joint_pos.cpp
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
#include "redis/RedisClient.h"
bool fSimulationRunning = false;
void sighandler(int) { fSimulationRunning = false; }

using namespace std;
using namespace Eigen;

// config file names and object names
const string world_file = "${EXAMPLE_100_FOLDER}/world.urdf";
const string robot_file =
	"${SAI2_MODEL_URDF_FOLDER}/panda/panda_arm_sphere.urdf";
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

/*
	Control
*/
// bool flag_simulation = true;
bool flag_simulation = false;
Sai2Common::RedisClient* redis_client;
std::string JOINT_ANGLES_KEY = "sai2::FrankaPanda::Romeo::sensors::q";
std::string JOINT_VELOCITIES_KEY = "sai2::FrankaPanda::Romeo::sensors::dq";
std::string JOINT_TORQUES_COMMANDED_KEY = "sai2::FrankaPanda::Romeo::actuators::fgc";
std::string MASS_MATRIX_KEY = "sai2::FrankaPanda::Romeo::sensors::model::massmatrix";
VectorXd q_init = VectorXd::Zero(7);

enum State {
	POSTURE = 0,
	MOTION
};

//------------ main function
int main(int argc, char** argv) {
	Sai2Model::URDF_FOLDERS["EXAMPLE_100_FOLDER"] =
		string(EXAMPLES_FOLDER) + "/100-joint_limits";
	cout << "Loading URDF world model file: " << world_file << endl;

	// set up signal handler
	signal(SIGABRT, &sighandler);
	signal(SIGTERM, &sighandler);
	signal(SIGINT, &sighandler);

	// load graphics scene
	auto graphics = make_shared<Sai2Graphics::Sai2Graphics>(world_file);
	graphics->addUIForceInteraction(robot_name);
	// graphics->showTransparency(true, robot_name, 0.5);

	// load simulation world
	auto sim = make_shared<Sai2Simulation::Sai2Simulation>(world_file);

	// load robots
	auto robot = make_shared<Sai2Model::Sai2Model>(robot_file, false);
	robot->setQ(sim->getJointPositions(robot_name));
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

	// // Position plus orientation task
	// string link_name = "end-effector";
	// Vector3d pos_in_link = Vector3d(0.0, 0.0, 0.07);
	// Affine3d compliant_frame = Affine3d(Translation3d(pos_in_link));

	// // Full motion force task
	// auto motion_force_task = make_unique<Sai2Primitives::MotionForceTask>(
	// 	robot, link_name, compliant_frame);

	// // // Partial motion force task
	// // vector<Vector3d> controlled_directions_translation = {
	// // 	Vector3d::UnitX(), Vector3d::UnitY(), Vector3d::UnitZ()};
	// // vector<Vector3d> controlled_directions_rotation = {};
	// // auto motion_force_task = make_shared<Sai2Primitives::MotionForceTask>(
	// // 	robot, link_name, controlled_directions_translation,
	// // 	controlled_directions_rotation);
	// // motion_force_task->setSingularityGains(20, 20);

    // motion_force_task->disableInternalOtg();
    // motion_force_task->enableVelocitySaturation(0.2);
	// VectorXd motion_force_task_torques = VectorXd::Zero(dof);

	// // no gains setting here, using the default task values
	// const Matrix3d initial_orientation = robot->rotation(link_name);
	// const Vector3d initial_position = robot->position(link_name, pos_in_link);

	// joint task 
	auto joint_task = make_unique<Sai2Primitives::JointTask>(robot);
    // joint_task->disableInternalOtg();
    joint_task->enableVelocitySaturation(0.4);
    joint_task->setGains(200, 20);
	VectorXd joint_task_torques = VectorXd::Zero(dof);

	VectorXd q_des = robot->q();
    q_des << 0,-0.607897,-0.0140497,-2.27666,-0.031351,1.71403,0.71785;
    joint_task->setGoalPosition(q_des);

    // offsets 
    VectorXd q_delta = VectorXd::Zero(robot->dof());
    q_delta(3) = -5;
    double sign_switch = 1;

    // // apf joint limits
    // auto joint_limits = robot->jointLimits();
    // VectorXd q_min(robot->dof()), q_max(robot->dof());
    // int cnt = 0;
    // for (auto limit : joint_limits) {
    //     q_min(cnt) = limit.position_lower + 0.4;
    //     q_max(cnt) = limit.position_upper - 0.4;
    //     cnt++;
    // }
    // // double eta = 0.5;
    // double eta = 0.5;
    // // double eta = 10.0;
    // VectorXd q_init = robot->q();
	// q_init(5) = M_PI / 2;
    // VectorXd q_delta = VectorXd::Zero(robot->dof());
	// q_delta(3) = 5;
    // // q_delta(5) = 5;
	// double sign_switch = 1;
	// VectorXd q_oscillation = VectorXd::Zero(robot->dof());

    // desired position offsets
    // vector<Vector3d> desired_offsets {Vector3d(2, 0, 0), Vector3d(0, 0, 0),
    //                                   Vector3d(0, 2, 0), Vector3d(0, 0, 0),
    //                                   Vector3d(0, -2, 0), Vector3d(0, 0, 0),
    //                                   Vector3d(0, 0, 2), Vector3d(0, 0, 0)};
	// vector<Vector3d> desired_offsets {Vector3d(0, 2, 0), Vector3d(0, 0, 0),
    //                                   Vector3d(0, -2, 0), Vector3d(0, 0, 0),
    //                                   Vector3d(0, 0, 2), Vector3d(0, 0, 0)};
    // vector<Vector3d> desired_offsets {Vector3d(2, 0, 0)};
	double t_initial = 2;
	// vector<double> t_wait {10, 10};
	vector<double> t_wait {5, 5};
    // double t_wait = 10;  // wait between switching desired positions
	// double t_reset_wait = 5;  // wait when resetting position
    double prev_time = 0;
    // int cnt = 6 * 1;
	int cnt = 0;
    // int max_cnt = desired_offsets.size();
    int state = POSTURE;

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
    logger.addToLog(robot_q, "robot_q");
	logger.addToLog(robot_dq, "robot_dq");
	logger.addToLog(robot_torque, "robot_torque");
	logger.addToLog(joint_pos_state, "joint_pos_state");
	logger.start(100);

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
                robot_dq = robot->dq();
        	}
		} else {
			robot->setQ(redis_client->getEigen(JOINT_ANGLES_KEY));
			robot->setDq(redis_client->getEigen(JOINT_VELOCITIES_KEY));
			MatrixXd M = redis_client->getEigen(MASS_MATRIX_KEY);
            M.bottomRightCorner(4, 4) += 0.15 * Matrix3d::Identity();
			robot->updateModel(M);

			robot_q = robot->q();
            robot_dq = robot->dq();
		}

        // state machine
		if (state == POSTURE) {

			N_prec = MatrixXd::Identity(dof, dof);
			joint_task->updateTaskModel(N_prec);
			joint_task->setGains(200, 20, 20);

			{
				lock_guard<mutex> lock(mutex_torques);
				control_torques = joint_task->computeTorques();
                if (!flag_simulation) {
				    redis_client->setEigen(JOINT_TORQUES_COMMANDED_KEY, control_torques);
                }
            }

			if ((robot->q() - joint_task->getGoalPosition()).norm() < 5e-2) {
				state = MOTION;
				joint_task->reInitializeTask();

                joint_task->disableInternalOtg();
                joint_task->enableVelocitySaturation(1.2);
				joint_task->setGains(300, 20, 0);
			}

		} else if (state == MOTION) {

            // apply delta joint position 
            if (time - prev_time > t_wait[cnt % 2]) {
                sign_switch *= -1;
                // q_delta(3) = sign_switch * 5;
                q_des(3) = q_init(3) + sign_switch * q_delta(3);
                // q_delta(5) = - sign_switch * 5;
                cnt++;
                prev_time = time;
                // if (cnt == max_cnt) cnt = max_cnt - 1;
                joint_task->setGoalPosition(q_des);
            }

            // update task models 
            N_prec = MatrixXd::Identity(dof, dof);
            {
                lock_guard<mutex> lock(mutex_robot);
                joint_handler->updateTaskModel(N_prec);                
                joint_task->updateTaskModel(N_prec);
            }

            // compute torques 
            {
                lock_guard<mutex> lock(mutex_torques);
                control_torques = joint_handler->computeTorques(joint_task->computeTorques());
                if (!flag_simulation) {
				    redis_client->setEigen(JOINT_TORQUES_COMMANDED_KEY, control_torques);
                }
            }

            robot_torque = control_torques;

            // log joint state 
            auto joint_states = joint_handler->getJointState();
            joint_pos_state = joint_states.first;
            joint_vel_state = joint_states.second;
        }

		// -------------------------------------------
		if (timer.elapsedCycles() % 500 == 0) {
			cout << "time: " << time << endl;
			// cout << "position error : "
				//  << (motion_force_task->getGoalPosition() -
					//  motion_force_task->getCurrentPosition())
						// .norm()
				//  << endl;
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
        if (flag_simulation) {
		    {
    			lock_guard<mutex> lock(mutex_torques);
			    sim->setJointTorques(robot_name, control_torques + ui_torques);
		    }
		    sim->integrate();
        } 
	}
	timer.stop();
	cout << "\nSimulation loop timer stats:\n";
	timer.printInfoPostRun();
}