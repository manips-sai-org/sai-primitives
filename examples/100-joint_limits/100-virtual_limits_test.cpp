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

int getSign(double value) {
    if (value > 0) {
        return 1;
    } else if (value < 0) {
        return -1;
    } else {
        return 0;
    }
}

// sigmoid function for velocity saturation
// alpha from 0 -> 1 as approaching constraint 
double getMaxVelFunction(double alpha, double entry_vel, double exit_vel) {
    // std::cout << "alpha: " << alpha << "\n";
    if (getSign(entry_vel) != getSign(exit_vel)) {
        exit_vel *= -1;
    }
    return exit_vel + (entry_vel - exit_vel) * ((1 - cos(M_PI * (1 - alpha))) / 2);
}
/*
	Control
*/
bool flag_simulation = true;
// bool flag_simulation = false;

// bool flag_baseline = true;
bool flag_baseline = false;

bool flag_joint_method = true;
// bool flag_joint_method = false;

// bool flag_constraint_only = true;
bool flag_constraint_only = false;

Sai2Common::RedisClient* redis_client;
std::string JOINT_ANGLES_KEY = "sai2::FrankaPanda::Romeo::sensors::q";
std::string JOINT_VELOCITIES_KEY = "sai2::FrankaPanda::Romeo::sensors::dq";
std::string JOINT_TORQUES_COMMANDED_KEY = "sai2::FrankaPanda::Romeo::actuators::fgc";
std::string MASS_MATRIX_KEY = "sai2::FrankaPanda::Romeo::sensors::model::massmatrix";
std::string CORIOLIS_KEY = "sai2::FrankaPanda::Romeo::sensors::model::coriolis";
VectorXd q_init = VectorXd::Zero(7);
int sign_switch = -1;

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
    VectorXd pos_zone_1_threshold = joint_handler->getPosZone1Threshold();
    VectorXd pos_zone_2_threshold = joint_handler->getPosZone2Threshold();

	// joint_handler->setEta(0.1);
	// joint_handler->setPosZone1ThresholdIndex(7 * M_PI / 180, 2);
	// joint_handler->setPosZone2ThresholdIndex(7 * M_PI / 180, 2);

	// Position plus orientation task
	string link_name = "end-effector";
	Vector3d pos_in_link = Vector3d(0.0, 0.0, 0.107 + 0.1);
	Affine3d compliant_frame = Affine3d(Translation3d(pos_in_link));

	// Full motion force task
	auto motion_force_task = make_unique<Sai2Primitives::MotionForceTask>(
		robot, link_name, compliant_frame);
    motion_force_task->disableInternalOtg();

	// double max_saturation_velocity = 1.4;  // upper limit for velocity saturation 
	// double min_saturation_velocity = 0.5;

	// double max_saturation_velocity = 1.0;  // upper limit for velocity saturation 
	double max_saturation_velocity = 0.5 + 0.1 * 0;  
	double min_saturation_velocity = 0.2;

	// double max_saturation_velocity = 0.5;
	// double min_saturation_velocity = 0.2;
	// int CONSTRAINED_JOINT_IDX = 2;
	int CONSTRAINED_JOINT_IDX = 3;

	// double min_saturation_velocity = 0.4;
	// motion_force_task->enableVelocitySaturation(max_saturation_velocity, M_PI / 3);
	motion_force_task->enableVelocitySaturation(max_saturation_velocity);

    motion_force_task->setPosControlGains(200, 20, 0);
    motion_force_task->setOriControlGains(200, 20, 0);
    VectorXd motion_force_task_torques = VectorXd::Zero(dof);

	Vector3d initial_position = robot->position(link_name, pos_in_link);
    Matrix3d initial_orientation = robot->rotation(link_name);

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
    joint_task->enableVelocitySaturation(0.8);
    joint_task->setGains(400, 20);
	VectorXd joint_task_torques = VectorXd::Zero(dof);

	VectorXd q_des = robot->q();
    // q_des << 0,-0.607897,-0.0140497,-2.27666,-0.031351,1.71403,0.71785;  // LEFT/RIGHT MOTION
	// q_des << -0.021456,-0.311432,0.0319844,-1.89182,-0.0400231,1.60057,0.798106;  // UP/DOWN MOTION 
	q_des << -0.0592664,-0.067921,0.0443692,-2.28513,-0.0393825,2.17235,0.814726;  // ONLY UP MOTION 
    joint_task->setGoalPosition(q_des);

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
	vector<double> t_wait {5, 5};
    // double t_wait = 10;  // wait between switching desired positions
	// double t_reset_wait = 5;  // wait when resetting position
    double prev_time = 0;
    // int cnt = 6 * 1;
	int cnt = 0;
    // int max_cnt = desired_offsets.size();
    int state = POSTURE;
    double start_time = 0;

    bool locked_joint = false;
    double t_lock = 4;
    double t_free = 20;

    // get joint limits 
    VectorXd q_min = joint_handler->getMinJointLimit();
    VectorXd q_max = joint_handler->getMaxJointLimit();

	VectorXd coriolis = VectorXd::Zero(robot->dof());

	bool first_reset = false;

	// create logger
	Sai2Common::Logger logger("virtual", false);
	VectorXd svalues = VectorXd::Zero(6);
    VectorXd robot_q = robot->q();
	VectorXd robot_dq = robot->dq();
	VectorXd robot_torque = VectorXd::Zero(robot->dof());
	VectorXi joint_pos_state = VectorXi::Zero(robot->dof());
	VectorXi joint_vel_state = VectorXi::Zero(robot->dof());
    Vector3d ee_pos = motion_force_task->getCurrentPosition();
    Vector3d goal_pos = motion_force_task->getGoalPosition();
    Matrix3d ee_ori = motion_force_task->getCurrentOrientation();
	// VectorXd joint_task_torques = VectorXd::Zero(robot->dof());
	int constraint_flag = 0;
    logger.addToLog(robot_q, "robot_q");
	logger.addToLog(robot_dq, "robot_dq");
	logger.addToLog(robot_torque, "robot_torque");
	logger.addToLog(joint_pos_state, "joint_pos_state");
    logger.addToLog(ee_pos, "ee_pos");
    logger.addToLog(goal_pos, "goal_pos");
    logger.addToLog(ee_ori, "ee_ori");
    logger.addToLog(locked_joint, "locked_joint");
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
            M.bottomRightCorner(3, 3) += 0.3 * Matrix3d::Identity();
			robot->updateModel(M);

			// coriolis = redis_client->getEigen(CORIOLIS_KEY);

			robot_q = robot->q();
            robot_dq = robot->dq();
		}

        // state machine
		if (state == POSTURE) {

			N_prec = MatrixXd::Identity(dof, dof);
			joint_task->updateTaskModel(N_prec);
			// joint_task->setGains(200, 20, 20);

			{
				lock_guard<mutex> lock(mutex_torques);
				control_torques = joint_task->computeTorques();
                if (!flag_simulation) {
				    redis_client->setEigen(JOINT_TORQUES_COMMANDED_KEY, control_torques);
                }
            }

			std::cout << "joint error: " << (robot->q() - joint_task->getGoalPosition()).norm() << "\n";

			if ((robot->q() - joint_task->getGoalPosition()).norm() < 5e-1 && !first_reset) {
				joint_task->setGains(400, 20, 100);
				joint_task->resetIntegrators();
				first_reset = true;
			}

			if ((robot->q() - joint_task->getGoalPosition()).norm() < 8e-2) {
				state = MOTION;
				joint_task->reInitializeTask();
                motion_force_task->reInitializeTask();

                initial_position = motion_force_task->getCurrentPosition();

                joint_task->disableInternalOtg();
                // joint_task->enableVelocitySaturation(1.5);
                // joint_task->disableVelocitySaturation();
				joint_task->setGains(200, 20, 0);
				joint_task->enableVelocitySaturation(0.4);

				// lock the joints now 
				VectorXd new_q_min = q_min;
				VectorXd new_q_max = q_max;
				// new_q_min(2) = -0.3;  // LEFT/RIGHT
				// new_q_max(2) = 0.3;

				// new_q_min(CONSTRAINED_JOINT_IDX) = robot->q()(CONSTRAINED_JOINT_IDX) - 0.3;
				// new_q_max(CONSTRAINED_JOINT_IDX) = robot->q()(CONSTRAINED_JOINT_IDX) + 0.3;
				new_q_max(CONSTRAINED_JOINT_IDX) = -1.89182 + 0.3;  // CURRENT SETUP

				// std::cout << "q min: " << new_q_min.transpose() << "\n";
				// std::cout << "q max: " << new_q_max.transpose() << "\n";

				joint_handler->setMinJointLimit(new_q_min);
				joint_handler->setMaxJointLimit(new_q_max);

				// joint_handler->setEta(0.008);
				// joint_handler->setEta(0.05);  // final method 
				// joint_handler->setEta(0.0013);  // final method 

				// joint_handler->setEta(0.0015);
				// joint_handler->setEta(0.0001);  // final baseline implementation
				joint_handler->setEta(0.0013);  // final baseline implementation in paper (LEFT/RIGHT)
				// joint_handler->setEta(0.1);  // final method 

				// joint_handler->setEta(0.001);  // final baseline

				// joint_handler->setPosZone1ThresholdIndex(4 * M_PI / 180, CONSTRAINED_JOINT_IDX);  // 4 for baseline 
				// joint_handler->setPosZone2ThresholdIndex(4 * M_PI / 180, CONSTRAINED_JOINT_IDX);

				joint_handler->setPosZone1ThresholdIndex(8 * M_PI / 180, CONSTRAINED_JOINT_IDX);  // 8 for baseline 
				joint_handler->setPosZone2ThresholdIndex(4 * M_PI / 180, CONSTRAINED_JOINT_IDX);

				// joint_handler->setPosZone1ThresholdIndex(12 * M_PI / 180, CONSTRAINED_JOINT_IDX);  // 8 for baseline 
				// joint_handler->setPosZone2ThresholdIndex(8 * M_PI / 180, CONSTRAINED_JOINT_IDX);
 
				// joint_handler->setPosZone1ThresholdIndex(8 * M_PI / 180, CONSTRAINED_JOINT_IDX);  // 8 for baseline 
				// joint_handler->setPosZone2ThresholdIndex(8 * M_PI / 180, CONSTRAINED_JOINT_IDX);
				
				// joint_handler->setCollisionTime(0.3);
				joint_handler->setPosTimeBuffer(0.15);
				joint_handler->enableVariableVelocityZone(true);
				// joint_handler->enableVariableVelocityZone(false);
				// joint_handler->setVelocityTol(0.1);
				// joint_handler->setTorqueTol(0.1);
				// joint_handler->setDamping(30);
				// joint_handler->setDamping(30);
				locked_joint = false; 

                start_time = time;
			}

		} else if (state == MOTION) {

            ee_pos = motion_force_task->getCurrentPosition();
            goal_pos = motion_force_task->getGoalPosition();

			// set goal position every N second
			if (time - start_time > t_lock) {
				// command motion in y direction to limit
				sign_switch *= -1;
				int switch_value = 1;
				if (sign_switch == -1) {
					switch_value = 0;
				}
				// motion_force_task->setGoalPosition(initial_position + sign_switch * Vector3d(0, 0.3, 0));
				// motion_force_task->setGoalPosition(initial_position + switch_value * sign_switch * Vector3d(-0.05, 0, 0.25));
				motion_force_task->setGoalPosition(initial_position + switch_value * sign_switch * Vector3d(0, 0, 0.4));  // 0.6 before
				start_time = time;
			}

            // update task models 
            N_prec = MatrixXd::Identity(dof, dof);
            {
                lock_guard<mutex> lock(mutex_robot);
                joint_handler->updateTaskModel(N_prec);
                motion_force_task->updateTaskModel(N_prec);
				// motion_force_task->updateTaskModel(joint_handler->getTaskAndPreviousNullspace());
                joint_task->updateTaskModel(motion_force_task->getTaskAndPreviousNullspace());
            }

			// compute joint handler torques to get the joint state 
			// VectorXd motion_force_torques = VectorXd::Zero(robot->dof());
			// VectorXd joint_task_torques = VectorXd::Zero(robot->dof());
			auto motion_force_torques = motion_force_task->computeTorques();
			auto joint_task_torques = joint_task->computeTorques();
			// auto _ = joint_handler->computeTorques(motion_force_torques + joint_task_torques);

			VectorXd motion_force_torques_without_handler = motion_force_torques;
			VectorXd joint_task_torques_without_handler = joint_task_torques;
			// auto exit_state = joint_handler->getExitState(motion_force_torques_without_handler + joint_task_torques_without_handler);
			// VectorXd joint_handler_constraint_torques = joint_handler->computeTorques(VectorXd::Zero(7), true);
            
			// {
            //     lock_guard<mutex> lock(mutex_robot);
			// 	motion_force_task->updateTaskModel(joint_handler->getTaskAndPreviousNullspace());
            //     joint_task->updateTaskModel(motion_force_task->getTaskAndPreviousNullspace());
            // }
			// motion_force_torques = motion_force_task->computeTorques();
			// joint_task_torques = joint_task->computeTorques();
		
			control_torques = joint_handler->computeTorques(motion_force_torques_without_handler + joint_task_torques_without_handler);

			if (!flag_simulation) {
				redis_client->setEigen(JOINT_TORQUES_COMMANDED_KEY, control_torques);
			}

            robot_torque = control_torques + coriolis;

            // // log joint state 
            // auto joint_states = joint_handler->getJointState();
            // joint_pos_state = joint_states.first;
            // joint_vel_state = joint_states.second;
        }

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