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
#include "timer/LoopTimer.h"
#include "logger/Logger.h"
#include "redis/RedisClient.h"
bool fSimulationRunning = false;
void sighandler(int) { fSimulationRunning = false; }

using namespace std;
using namespace Eigen;

// config file names and object names
const string world_file = "${EXAMPLE_18_FOLDER}/world.urdf";
// const string robot_file =
// 	"${SAI2_MODEL_URDF_FOLDER}/panda/panda_arm_sphere.urdf";
// const string robot_file =
	// "${SAI2_MODEL_URDF_FOLDER}/panda/panda_arm_gripper_fixed.urdf";
const string robot_file = std::string(SAI2_MODEL_URDF_FOLDER) + "/panda/panda_arm_gripper_fixed.urdf";
const string robot_name = "PANDA";

// ui torques and control torques
VectorXd ui_torques;
VectorXd control_torques;
VectorXd curr_robot_q;

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
	Sai2Model::URDF_FOLDERS["EXAMPLE_18_FOLDER"] =
		string(EXAMPLES_FOLDER) + "/18-panda_singularity";
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

	curr_robot_q = robot->q();

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
			graphics->updateRobotGraphics(robot_name, curr_robot_q);
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
	joint_handler->setEta(0.003);

    // partial joint task to hold joints 3 and 5 fixed 
    MatrixXd joint_selection_matrix = MatrixXd::Zero(2, dof);
    joint_selection_matrix(0, 2) = 1;
    joint_selection_matrix(1, 4) = 1;
    auto partial_joint_task = std::make_unique<Sai2Primitives::JointTask>(robot, joint_selection_matrix);
    VectorXd partial_joint_task_torques = VectorXd::Zero(dof);
	partial_joint_task->setGains(100, 20, 0);

	// Position plus orientation task
	string link_name = "end-effector";
	// string link_name = "link7";
	Vector3d pos_in_link = Vector3d(0.0, 0.0, 0.0);
	Affine3d compliant_frame = Affine3d(Translation3d(pos_in_link));

	// Full motion force task
	auto motion_force_task = make_unique<Sai2Primitives::MotionForceTask>(
		robot, link_name, compliant_frame);

	// Partial motion force task
	// vector<Vector3d> controlled_directions_translation = {
	// 	Vector3d::UnitX(), Vector3d::UnitY(), Vector3d::UnitZ()};
	// vector<Vector3d> controlled_directions_rotation = {};
	// auto motion_force_task = make_shared<Sai2Primitives::MotionForceTask>(
	// 	robot, link_name, controlled_directions_translation,
	// 	controlled_directions_rotation);
	// motion_force_task->setSingularityGains(20, 20);

    motion_force_task->setPosControlGains(200, 20, 0);
    motion_force_task->setOriControlGains(200, 20, 0);
	motion_force_task->disableInternalOtg();
	motion_force_task->enableTrackingMode();
    motion_force_task->disableVelocitySaturation();
    // motion_force_task->setSingularityHandlingBounds(1e-2, 7e-2);
    motion_force_task->setSingularityHandlingBounds(3e-2, 7e-2);
    // motion_force_task->enableVelocitySaturation(1.0, M_PI / 3);
    // motion_force_task->setSingularityHandlingBounds(5e-2, 5e-1);
	VectorXd motion_force_task_torques = VectorXd::Zero(dof);

	// // orientation task 
	// auto ori_task = make_shared<Sai2Primitives::MotionForceTask>(robot, link_name, \
	// 						controlled_directions_rotation, controlled_directions_translation, compliant_frame);
	// ori_task->disableSingularityHandling();

	// no gains setting here, using the default task values
	Matrix3d initial_orientation = robot->rotation(link_name);
	Vector3d initial_position = robot->position(link_name, pos_in_link);

	// joint task to control the redundancy
	// using default gains and interpolation settings
	auto joint_task = make_unique<Sai2Primitives::JointTask>(robot);
    joint_task->setGains(100, 20);
	joint_task->enableVelocitySaturation(M_PI / 3);
	VectorXd joint_task_torques = VectorXd::Zero(dof);

	VectorXd initial_q = robot->q();
    VectorXd q_des = initial_q;
    // q_des << 0,-1.57079632679,0,-2.35619449019,0,0.78539816339,0.78539816339; 
	// q_des << -0.0426864,-1.15974,-0.0811375,-1.84581,-0.198303,0.679431,0.636189;
	// q_des << -0.109943,-1.44935,-0.0982967,-2.21411,-0.100156,0.785034,0.756978;
    // q_des << 0,-0.919364,0.0103624,-2.38671,-0.0298135,3.0152,0.863642;  // wrist singularity 
    // q_des << -0.125263*0,-0.119768,0.104413,-2.16512,0.0216533 * 0,3.32598,0.787686;  // combined 
    // q_des << -0.00775566,-0.133565,0.0153017,-2.16814,0.0430064,3.29317,0.797755;  // starting
    q_des << 0,-0.133565,0.0153017,-2.16814,0,3.29317,0.797755;  // starting
    joint_task->setGoalPosition(q_des);
	motion_force_task->setTypeOnePosture(q_des);
	// partial_joint_task->setGoalPosition(q_des);

    // desired orientation offset 
    Matrix3d rotation_offset = AngleAxisd(M_PI / 2, Vector3d::UnitZ()).toRotationMatrix();

    // desired position offsets 

	// std::vector<Vector3d> desired_offsets {Vector3d(0, -0.15, 0), Vector3d(0, 0, 0),
	// 									 Vector3d(0.15, 0, 0), Vector3d(0, 0, 0),
	// 									 Vector3d(0, 0.15, 0), Vector3d(0, 0, 0),
	// 									 Vector3d(-0.15, 0, 0), Vector3d(0, 0, 0)};

    vector<Vector3d> desired_offsets {Vector3d(0, -0.25, 0), Vector3d(0, 0, 0),
                                      Vector3d(0, 0.25, 0), Vector3d(0, 0, 0)};
    // vector<Vector3d> desired_offsets {Vector3d(2, 0, 0)};
	double t_initial = 2;
	vector<double> t_wait {5, 5};
	// vector<double> t_wait {10, 10};
    // double t_wait = 10;  // wait between switching desired positions 
	// double t_reset_wait = 5;  // wait when resetting position 
    double prev_time = 0;
    // int cnt = 6 * 1;
	int cnt = 0;
    int max_cnt = desired_offsets.size();
    double time_transition = 0;

    int state = POSTURE;
    bool integrator_on = false;

	// create logger
	Sai2Common::Logger logger("type_2_wrist", false);
	VectorXd svalues = VectorXd::Zero(6);
    VectorXd robot_q = robot->q();
    VectorXd robot_dq = robot->dq();
	Vector3d pos_error = Vector3d::Zero();
    Vector3d ori_error = Vector3d::Zero();
    Vector3d ee_pos = Vector3d::Zero();
    Matrix3d ee_ori = Matrix3d::Identity();
	Vector3d goal_pos = Vector3d::Zero();
	VectorXd unmodified_singular_task_torques = VectorXd::Zero(robot->dof());
    VectorXd singular_task_torques = VectorXd::Zero(robot->dof());
	VectorXd non_singular_task_torques = VectorXd::Zero(robot->dof());
	VectorXd singular_joint_space_torques = VectorXd::Zero(robot->dof());
	VectorXd alpha = VectorXd::Ones(1);
	VectorXd condition_ratio = VectorXd::Ones(6);
    VectorXd singular_direction = VectorXd::Zero(6);
	VectorXd singular_joint_space = VectorXd::Zero(robot->dof());
	VectorXi classification = VectorXi::Zero(6);

	logger.addToLog(svalues, "svalues");
	logger.addToLog(robot_q, "robot_q");
	logger.addToLog(robot_dq, "robot_dq");
	logger.addToLog(pos_error, "pos_error");
	logger.addToLog(ori_error, "ori_error");
	logger.addToLog(ee_pos, "ee_pos");
	logger.addToLog(ee_ori, "ee_ori");
	logger.addToLog(goal_pos, "goal_pos");
	logger.addToLog(unmodified_singular_task_torques, "unmodified_singular_task_torques");
    logger.addToLog(singular_task_torques, "singular_task_torques");
	logger.addToLog(non_singular_task_torques, "non_singular_task_torques");
	logger.addToLog(singular_joint_space_torques, "singular_joint_space_torques");
	logger.addToLog(motion_force_task_torques, "motion_task_torques");
	logger.addToLog(alpha, "alpha");
	logger.addToLog(condition_ratio, "condition_ratio");
    logger.addToLog(singular_direction, "singular_direction");
	logger.addToLog(singular_joint_space, "singular_joint_space");
	logger.addToLog(classification, "classification");
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
                robot_dq = robot->dq();
        	}
		} else {
			robot->setQ(redis_client->getEigen(JOINT_ANGLES_KEY));
			robot->setDq(redis_client->getEigen(JOINT_VELOCITIES_KEY));
			MatrixXd M = redis_client->getEigen(MASS_MATRIX_KEY);
            M.bottomRightCorner(4, 4) += 0.15 * MatrixXd::Identity(4, 4);
            // M.bottomRightCorner(3, 3) += 0.15 * Matrix3d::Identity();  // use less fopr this 
			robot->updateModel(M);

			robot_q = robot->q();
            robot_dq = robot->dq();
		}

        // kinematics
        ee_pos = robot->position(link_name, pos_in_link);
        ee_ori = robot->rotation(link_name);
		goal_pos = motion_force_task->getGoalPosition();
		// svalues = motion_force_task->getSingularValues();
		// singular_task_torques = motion_force_task->getSingularTaskTorques();
		// singular_direction = motion_force_task->getSingularTaskRange().col(0);
		if (motion_force_task->getNumSingularities() > 0) {
			// alpha.head(motion_force_task->getNumSingularities()) = motion_force_task->getBlendingVector();
			alpha(0) = motion_force_task->getBlendingVector()(0);
		}
		pos_error = motion_force_task->getPositionError();
		ori_error = motion_force_task->getOrientationError();
		curr_robot_q = robot_q;

		// std::cout << "s values: " << svalues.transpose() << "\n";

        // state machine
		if (state == POSTURE) {

			N_prec = MatrixXd::Identity(dof, dof);
			joint_task->updateTaskModel(N_prec);
			joint_task->setGains(200, 20, 20);

            if (joint_task->goalPositionReached(1e0) && !integrator_on) {
                std::cout << "Turning on integrator\n";
                joint_task->resetIntegrators();
                joint_task->setGains(400, 20, 1000);
                integrator_on = true; 
            }

			{
				lock_guard<mutex> lock(mutex_torques);
				control_torques = joint_task->computeTorques();
                if (!flag_simulation) {
				    redis_client->setEigen(JOINT_TORQUES_COMMANDED_KEY, control_torques);
                }
            }

            std::cout << "joint error: " << (robot->q() - joint_task->getGoalPosition()).norm() << "\n";

			// if ((robot->q() - joint_task->getGoalPosition()).norm() < 5e-2) {
			if (joint_task->goalPositionReached(5e-2)) {
                std::cout << "Posture to Motion\n";
				state = MOTION;
				joint_task->reInitializeTask();

                joint_task->disableInternalOtg();
                // joint_task->disableVelocitySaturation();
                // joint_task->enableVelocitySaturation(0.5);
                // joint_task->enableVelocitySaturation(0.3);
				joint_task->setGains(100, 20, 0);
                joint_task->resetIntegrators();

                motion_force_task->reInitializeTask();
                joint_task->reInitializeTask();
                partial_joint_task->reInitializeTask();
				// ori_task->reInitializeTask();

                initial_position = ee_pos;
                initial_orientation = ee_ori;

                time_transition = time;

                // VectorXd type_2_direction = - VectorXd::Ones(robot->dof());
                // motion_force_task->setSingularityHandlingType2Direction(type_2_direction);

                continue;
			} 

		} else if (state == MOTION) {

            // update tasks model. Order is important to define the hierarchy
            N_prec = MatrixXd::Identity(dof, dof);
            {
                lock_guard<mutex> lock(mutex_robot);
				joint_handler->updateTaskModel(N_prec);
                // partial_joint_task->updateTaskModel(N_prec);            
				// joint_handler->updateTaskModel(partial_joint_task->getTaskAndPreviousNullspace());
                motion_force_task->updateTaskModel(N_prec);
                joint_task->updateTaskModel(motion_force_task->getTaskAndPreviousNullspace());
				// ori_task->updateTaskModel(motion_force_task->getTaskAndPreviousNullspace());
            }
            // N_prec = motion_force_task->getTaskAndPreviousNullspace();
            // after each task, need to update the nullspace
            // of the previous tasks in order to garantee
            // the dyamic consistency

            // joint_task->updateTaskModel(N_prec);

            // -------- set task goals and compute control torques
            // position: move to workspace extents 
            // if (time - prev_time > t_wait[cnt % 2]) {
            //     std::cout << "offset: " << desired_offsets[cnt].transpose() << "\n";
            //     motion_force_task->setGoalPosition(initial_position + desired_offsets[cnt]);
            //     // motion_force_task->setGoalOrientation(rotation_offset * initial_orientation);
            //     cnt++;
            //     prev_time = time;
            //     if (cnt == max_cnt) cnt = max_cnt - 1;
            // }

            Vector3d offset_trajectory = Vector3d(0, 0, 0);
            Vector3d offset_velocity_trajectory = Vector3d(0, 0, 0);
            Vector3d offset_acceleration_trajectory = Vector3d(0, 0, 0);
            double freq = 0.1;
            double amplitude = 0.25;  
            offset_trajectory(1) = amplitude * sin(2 * M_PI * freq * (time - time_transition));
            offset_velocity_trajectory(1) = 2 * M_PI * freq * amplitude * cos(2 * M_PI * freq * (time - time_transition));
            offset_acceleration_trajectory(1) = - std::pow(2 * M_PI * freq, 2) * amplitude * sin(2 * M_PI * freq * (time - time_transition));
            motion_force_task->setGoalPosition(initial_position + offset_trajectory);
            motion_force_task->setGoalLinearVelocity(offset_velocity_trajectory);
            // motion_force_task->setGoalLinearAcceleration(offset_acceleration_trajectory);

            // motion_force_task->setGoalLinearVelocity(Vector3d::Zero());
            // motion_force_task->setGoalLinearAcceleration(Vector3d::Zero());

            // compute torques for the different tasks
            motion_force_task_torques = motion_force_task->computeTorques();
            // partial_joint_task_torques = partial_joint_task->computeTorques();
            joint_task_torques = joint_task->computeTorques();
			// VectorXd ori_task_torques = ori_task->computeTorques();

            //------ compute the final torques
            {
                lock_guard<mutex> lock(mutex_torques);
                // control_torques = joint_handler->computeTorques(motion_force_task_torques + joint_task_torques);
                // control_torques = motion_force_task_torques + partial_joint_task_torques + joint_task_torques;
                // control_torques = joint_handler->computeTorques(motion_force_task_torques + joint_task_torques);
				control_torques = motion_force_task_torques + joint_task_torques;
				if (!flag_simulation) {
					redis_client->setEigen(JOINT_TORQUES_COMMANDED_KEY, control_torques);
				}
            }

            // log
			{
            	svalues = motion_force_task->getSingularValues();
				unmodified_singular_task_torques = motion_force_task->getUnmodifiedSingularTaskTorques();
		    	singular_task_torques = motion_force_task->getSingularTaskTorques();
				non_singular_task_torques = motion_force_task->getNonSingularTaskTorques();
				singular_joint_space_torques = motion_force_task->getSingularJointTaskTorques();
		    	singular_direction = motion_force_task->getSingularTaskRange().col(0);
				singular_joint_space = motion_force_task->getSingularJointTaskRange().col(0);
				classification.head(motion_force_task->getSingularityClassification().size()) = motion_force_task->getSingularityClassification();
				condition_ratio = motion_force_task->getConditionRatio();
			}

        }

		// MatrixXd Jc = MatrixXd::Zero(1, robot->dof());
		// Jc(0) = 1;
		// MatrixXd force_projection = Jc * robot->dynConsistentInverseJacobian(Jc);
		// std::cout << force_projection.transpose() << "\n";

		// // -------------------------------------------
		// if (timer.elapsedCycles() % 500 == 0) {
		// 	cout << "time: " << time << endl;
		// 	cout << "position error : "
		// 		 << (motion_force_task->getGoalPosition() -
		// 			 motion_force_task->getCurrentPosition())
		// 				.norm()
		// 		 << endl;
		// 	cout << endl;
		// }
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