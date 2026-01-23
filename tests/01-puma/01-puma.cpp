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

MatrixXd task_range_s;
Vector3d global_goal_position = Vector3d(100, 0, 0);
Matrix3d global_goal_orientation = Matrix3d::Identity();
Matrix3d global_control_orientation = Matrix3d::Identity();

// mutex for global variables between different threads
mutex mutex_torques;
mutex mutex_robot;
mutex mutex_sim;

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

std::vector<chai3d::cShapeCylinder*> singularity_cylinders;

using namespace chai3d;
void addCylinder(std::shared_ptr<Sai2Graphics::Sai2Graphics> graphics, 
			  	 chai3d::cShapeCylinder* cylinder,
			     chai3d::cColorf color) {
	cylinder->setLocalPos(cVector3d(100, 0, 0));
	cylinder->m_material->setColor(color);
	graphics->getWorld()->addChild(cylinder);
}

Eigen::Matrix3d rotationFromZ(const Eigen::Vector3d& z_in) {
    Eigen::Vector3d z = z_in.normalized();

    // Pick a vector not parallel to z
    Eigen::Vector3d arbitrary;
    if (std::abs(z.z()) < 0.9)
        arbitrary = Eigen::Vector3d::UnitZ();
    else
        arbitrary = Eigen::Vector3d::UnitY();

    // Project arbitrary vector onto plane orthogonal to z
    Eigen::Vector3d x = (arbitrary - arbitrary.dot(z) * z).normalized();
    Eigen::Vector3d y = z.cross(x);

    Eigen::Matrix3d R;
    R.col(0) = x;
    R.col(1) = y;
    R.col(2) = z;

    return R;
}

double cylinder_length = 0.2;

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
	graphics->showLinkFrame(true, robot_name, "end-effector-frame", 0.15);
	graphics->showObjectLinkFrame(true, "goal_frame", 0.15);
	// graphics->showObjectLinkFrame(true, "control_force", 0.25);
	graphics->setBackgroundColor(135./255, 206./255, 235./255);

	// add cylinders for singularity direction
	double radius = 0.005;
	cylinder_length = 0.2;
	chai3d::cColorf linear_singularity_color(1, 0, 0);
	chai3d::cColorf angular_singularity_color(0, 1, 0);
	for (int i = 0; i < 4; ++i) {
		singularity_cylinders.push_back(new chai3d::cShapeCylinder(radius, radius, cylinder_length));
		singularity_cylinders.back()->setUseTransparency(true);
		singularity_cylinders.back()->setTransparencyLevel(0.3);
		if (i % 2 == 0) {
			addCylinder(graphics, singularity_cylinders.back(), linear_singularity_color);
		} else {
			addCylinder(graphics, singularity_cylinders.back(), angular_singularity_color);
		}
	}

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

	// intitialize global torques variables
	ui_torques = VectorXd::Zero(robot->dof());
	control_torques = VectorXd::Zero(robot->dof());

	// start the simulation thread first
	fSimulationRunning = true;
	thread sim_thread(simulation, robot, sim);

	// next start the control thread
	thread ctrl_thread(control, robot, sim);

	// while window is open:
	while (graphics->isWindowOpen()) {
		{
			lock_guard<mutex> lock(mutex_sim);
			graphics->updateRobotGraphics(robot_name, sim->getJointPositions(robot_name));
		}

		Affine3d object_pose;
		object_pose.translation() = global_goal_position;
		object_pose.linear() = global_goal_orientation;
		graphics->updateObjectGraphics("goal_frame", object_pose);

		Affine3d control_pose;
		control_pose.translation() = global_goal_position;
		control_pose.linear() = global_control_orientation;
		// graphics->updateObjectGraphics("control_force", control_pose);

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

	// Position plus orientation task
	string link_name = "end-effector";
	// Vector3d pos_in_link = Vector3d(0.0, 0.0, 0.07);
	Vector3d pos_in_link = Vector3d(0.0, 0.0, 0.0);
	Affine3d compliant_frame = Affine3d(Translation3d(pos_in_link));

	// Full motion force task
	auto motion_force_task = make_unique<Sai2Primitives::MotionForceTask>(
		robot, link_name, compliant_frame);
	// motion_force_task->enableTrackingMode();
	motion_force_task->disableTrackingMode();
	// motion_force_task->setDynamicDecouplingType(Sai2Primitives::FULL_DYNAMIC_DECOUPLING);
    motion_force_task->enableVelocitySaturation(0.3, M_PI / 3);  // adjust for puma
	// motion_force_task->setSingularityHandlingBounds(7e-3, 7e-2);
	// motion_force_task->setSingularityHandlingBounds(4e-2, 7e-2);
	motion_force_task->disableInternalOtg();
	motion_force_task->setSingularityHandlingBound(7e-2);
	motion_force_task->setPosControlGains(100, 20, 0);
	motion_force_task->setOriControlGains(100, 20, 0);
	motion_force_task->setType1Posture(robot->q());
	motion_force_task->setSingularityHandlingGains(100, 20, 100, 20);
	// motion_force_task->setType1Tol(1e-3);
	motion_force_task->setBoundedInertiaEstimateThreshold(0, 0);

	// motion_force_task->setType1Velocity(M_PI, M_PI);
	// motion_force_task->setType2Velocity(M_PI);

	motion_force_task->setType2Velocity(M_PI / 2);
	motion_force_task->setType1Velocity(M_PI / 2, M_PI / 2);

	// // Partial motion force task
	// vector<Vector3d> controlled_directions_translation = {
	// 	Vector3d::UnitX(), Vector3d::UnitY(), Vector3d::UnitZ()};
	// vector<Vector3d> controlled_directions_rotation = {};
	// auto motion_force_task = make_shared<Sai2Primitives::MotionForceTask>(
	// 	robot, link_name, controlled_directions_translation,
	// 	controlled_directions_rotation);

	VectorXd motion_force_task_torques = VectorXd::Zero(dof);

	// no gains setting here, using the default task values
	const Matrix3d initial_orientation = robot->rotation(link_name);
	const Vector3d initial_position = robot->position(link_name, pos_in_link);

	// singularity cases
	// if overhead, then move robot to overhead, then move out
	// if wrist lock, then start robot in wrist lock, and translate in z-axis
	// and rotate about z-axis by 15 degrees 

	// desired position based on singularity testing
	Matrix3d desired_orientation = Matrix3d::Identity();
	if (flag_overhead_singularity) {	
		desired_orientation = AngleAxisd(M_PI / 2, Vector3d::UnitY()).toRotationMatrix();
		// motion_force_task->setGoalPosition(Vector3d(-0.12, 0.12, 1.3));
		motion_force_task->setGoalPosition(Vector3d(0, 0, 1.3));
	} else {
	}
	// motion_force_task->setGoalOrientation(desired_orientation);

	// joint task to control the redundancy
	// using default gains and interpolation settings
	auto joint_task = make_unique<Sai2Primitives::JointTask>(robot);
    joint_task->setGains(100, 20);
	joint_task->enableVelocitySaturation(M_PI / 3);
	VectorXd joint_task_torques = VectorXd::Zero(dof);

	VectorXd initial_q = robot->q();
    joint_task->setGoalPosition(initial_q);

	int state = GO_TO_SINGULARITY;
	double start_time = 0;
	Vector3d starting_ee_pos;
	Matrix3d starting_ee_ori;
	int cnt = 0;

	// create logger
	Sai2Common::Logger logger("puma", false);
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
		robot->setQ(sim->getJointPositions(robot_name));
		robot->setDq(sim->getJointVelocities(robot_name));
		robot->updateModel();

		// update kinematics
		ee_pos = robot->position(link_name, compliant_frame.translation());
		// std::cout << "ee position: " << ee_pos.transpose() << "\n";
		ee_ori = robot->rotation(link_name, compliant_frame.linear());

		// state transition logic 
		if (time - start_time > 5 && state == GO_TO_SINGULARITY) {
			// std::cout << "Updated direction\n";
			starting_ee_pos = ee_pos;
			starting_ee_ori = ee_ori;
			// motion_force_task->setGoalPosition(starting_ee_pos + Vector3d(-0.25, 0, 0));
			motion_force_task->setGoalPosition(starting_ee_pos + Vector3d(0, 0, -0.2));
			// motion_force_task->setGoalOrientation(starting_ee_ori * AngleAxisd(15 * M_PI / 180, Vector3d::UnitZ()));
			motion_force_task->setGoalOrientation(starting_ee_ori * AngleAxisd(90 * M_PI / 180, Vector3d::UnitZ()).toRotationMatrix());
			// motion_force_task->setGoalOrientation(AngleAxisd(15 * M_PI / 180, Vector3d::UnitZ()).toRotationMatrix() * starting_ee_ori);
			state = EXIT_SINGULARITY;
			start_time = time;
		}

		if (time - start_time > 5 && state == EXIT_SINGULARITY) {
			if (cnt == 0) {
				motion_force_task->setGoalPosition(starting_ee_pos);
				motion_force_task->setGoalOrientation(starting_ee_ori);
			} else if (cnt == 1) {
				// motion_force_task->setGoalPosition(starting_ee_pos + Vector3d(0, 0.25, 0));
				motion_force_task->setGoalPosition(starting_ee_pos + Vector3d(0, 0, -0.2));
				motion_force_task->setGoalOrientation(starting_ee_ori * AngleAxisd(90 * M_PI / 180, Vector3d::UnitZ()).toRotationMatrix());
			} else if (cnt == 2) {
				motion_force_task->setGoalPosition(starting_ee_pos);
				motion_force_task->setGoalOrientation(starting_ee_ori);
			} else if (cnt == 3) {
				// motion_force_task->setGoalPosition(starting_ee_pos + Vector3d(-0.25, 0, 0));
				motion_force_task->setGoalPosition(starting_ee_pos + Vector3d(0, 0, -0.2));
				motion_force_task->setGoalOrientation(starting_ee_ori * AngleAxisd(90 * M_PI / 180, Vector3d::UnitZ()).toRotationMatrix());
			}
			start_time = time;
			cnt++;
			if (cnt == 4) {
				cnt = 0;
			}
		}

		// std::cout << "pos error: " << motion_force_task->getPositionError().norm() << "\n";
		// std::cout << "ori error: " << motion_force_task->getOrientationError().norm() << "\n";

		// update tasks model. Order is important to define the hierarchy
		N_prec = MatrixXd::Identity(dof, dof);
		{
			lock_guard<mutex> lock(mutex_robot);
			motion_force_task->updateTaskModel(N_prec);
		}
		N_prec = motion_force_task->getTaskAndPreviousNullspace();
		// after each task, need to update the nullspace
		// of the previous tasks in order to garantee
		// the dyamic consistency

		joint_task->updateTaskModel(N_prec);

		// compute torques for the different tasks
		motion_force_task_torques = motion_force_task->computeTorques();
		joint_task_torques = joint_task->computeTorques();

		//------ compute the final torques
		{
			lock_guard<mutex> lock(mutex_torques);
			control_torques = motion_force_task_torques + joint_task_torques;
			// control_torques.setZero();
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

		// graphics
		{
			global_goal_position = motion_force_task->getGoalPosition();
			global_goal_orientation = motion_force_task->getGoalOrientation() * AngleAxisd(M_PI / 2, Vector3d::UnitZ()).toRotationMatrix();
			auto unit_mass_force = motion_force_task->getUnitMassForce();
			global_control_orientation = rotationFromZ(unit_mass_force.head(3));
		}

		// log 
		{
			ee_pos = motion_force_task->getCurrentPosition();
			ee_ori = motion_force_task->getCurrentOrientation();
			goal_pos = motion_force_task->getGoalPosition();
			pos_error = motion_force_task->getPositionError();
			ori_error = motion_force_task->getOrientationError();

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

		// graphics
		{	
			task_range_s = motion_force_task->getSingularTaskRange();
			// auto joint_task_range_s = motion_force_task->getSingularJointTaskRange();

			// std::cout << joint_task_range_s.transpose() << "\n";

			auto active_singularities = motion_force_task->getSingularities();
			int n_singularities = active_singularities.size();

			// // show only type 1
			// for (int i = 0; i < n_singularities; ++i) {
			// 	if (active_singularities[i].type == Sai2Primitives::TYPE_1_SINGULARITY) {

			// 		VectorXd u = active_singularities[i].u;

			// 		auto R_linear = rotationFromZ(u.head(3).normalized());
			// 		Vector3d center_position = ee_pos - cylinder_length / 2 * R_linear.col(2);
			// 		singularity_cylinders[0]->setLocalPos(chai3d::cVector3d(center_position * 100));
			// 		singularity_cylinders[0]->setLocalRot(chai3d::cMatrix3d(R_linear));
					
			// 		auto R_angular = rotationFromZ(u.tail(3).normalized());
			// 		center_position = ee_pos - cylinder_length / 2 * R_angular.col(2);
			// 		singularity_cylinders[1]->setLocalPos(chai3d::cVector3d(center_position * 100));
			// 		singularity_cylinders[1]->setLocalRot(chai3d::cMatrix3d(R_angular));
			// 	} else {
			// 		singularity_cylinders[0]->setLocalPos(chai3d::cVector3d(100, 0, 0));
			// 		singularity_cylinders[1]->setLocalPos(chai3d::cVector3d(100, 0, 0));
			// 	}
			// }

			// // first singularity
			// if (task_range_s.cols() > 0)
			// {
			// 	auto R_linear = rotationFromZ(task_range_s.col(0).head(3).normalized());
			// 	Vector3d center_position = ee_pos - cylinder_length / 2 * R_linear.col(2);
			// 	singularity_cylinders[0]->setLocalPos(chai3d::cVector3d(center_position));
			// 	singularity_cylinders[0]->setLocalRot(chai3d::cMatrix3d(R_linear));
				
			// 	auto R_angular = rotationFromZ(task_range_s.col(0).tail(3).normalized());
			// 	center_position = ee_pos - cylinder_length / 2 * R_angular.col(2);
			// 	singularity_cylinders[1]->setLocalPos(chai3d::cVector3d(center_position * 100));
			// 	singularity_cylinders[1]->setLocalRot(chai3d::cMatrix3d(R_angular));
			// } else {
			// 	singularity_cylinders[0]->setLocalPos(chai3d::cVector3d(100, 0, 0));
			// 	singularity_cylinders[1]->setLocalPos(chai3d::cVector3d(100, 0, 0));
			// }

			// // second singularity 
			// if (task_range_s.cols() > 1) 
			// {
			// 	auto R_linear = rotationFromZ(task_range_s.col(1).head(3).normalized());
			// 	Vector3d center_position = ee_pos - cylinder_length / 2 * R_linear.col(2);
			// 	singularity_cylinders[2]->setLocalPos(chai3d::cVector3d(cVector3d(center_position * 100)));
			// 	singularity_cylinders[2]->setLocalRot(chai3d::cMatrix3d(R_linear));
				
			// 	auto R_angular = rotationFromZ(task_range_s.col(1).tail(3).normalized());
			// 	center_position = ee_pos - cylinder_length / 2 * R_angular.col(2);
			// 	singularity_cylinders[3]->setLocalPos(chai3d::cVector3d(cVector3d(center_position)));
			// 	singularity_cylinders[3]->setLocalRot(chai3d::cMatrix3d(R_angular));
			// } else {
			// 	singularity_cylinders[2]->setLocalPos(chai3d::cVector3d(100, 0, 0));
			// 	singularity_cylinders[3]->setLocalPos(chai3d::cVector3d(100, 0, 0));
			// }

		}


	}
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
	double sim_freq = 1000;
	Sai2Common::LoopTimer timer(sim_freq);

	sim->setTimestep(1.0 / sim_freq);

	while (fSimulationRunning) {
		timer.waitForNextLoop();
		{
			lock_guard<mutex> lock(mutex_torques);
			sim->setJointTorques(robot_name, control_torques + ui_torques);
		}
		{
			lock_guard<mutex> lock(mutex_sim);
			sim->integrate();
		}
	}
	timer.stop();
	cout << "\nSimulation loop timer stats:\n";
	timer.printInfoPostRun();
}