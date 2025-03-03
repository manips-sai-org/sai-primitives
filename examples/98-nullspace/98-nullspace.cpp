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
bool fSimulationRunning = false;
void sighandler(int) { fSimulationRunning = false; }

using namespace std;
using namespace Eigen;

// config file names and object names
const string world_file = "${EXAMPLE_98_FOLDER}/world.urdf";
const string robot_file = "${EXAMPLE_98_FOLDER}/rrrbot.urdf";
const string robot_name = "RRRBOT";

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

// Function to calculate the 2D distance from point P to the capsule
// and return the direction vector to the closest point
std::pair<double, Eigen::Vector2d> distanceToCapsule(const Eigen::Vector2d& P, const Eigen::Vector2d& A, const Eigen::Vector2d& B, double r) {
    // Vector AB and AP
    Eigen::Vector2d AB = B - A;
    Eigen::Vector2d AP = P - A;

    // Compute the projection scalar t of AP onto AB
    double AB_dot_AB = AB.dot(AB);
    double AP_dot_AB = AP.dot(AB);
    double t = AP_dot_AB / AB_dot_AB;

    // Variables to store the closest point and direction vector
    Eigen::Vector2d closestPoint;
    Eigen::Vector2d direction;

    // If t is between 0 and 1, the perpendicular projection is within the segment
    if (t >= 0 && t <= 1) {
        // Perpendicular distance from P to the line AB
        double crossProduct = std::abs(AP.x() * AB.y() - AP.y() * AB.x());
        double distance = crossProduct / AB.norm();
        closestPoint = A + t * AB;  // Closest point on the line segment
        direction = (P - closestPoint).normalized();  // Direction vector from P to closest point
        return {std::max(distance - r, 0.0), direction};
    } else {
        // If t is outside [0, 1], compute the distance to the nearest endpoint (A or B)
        if (t < 0) {
            closestPoint = A;
        } else {
            closestPoint = B;
        }
        direction = (P - closestPoint).normalized();  // Direction vector from P to closest point
        return {(P - closestPoint).norm() - r, direction};
    }
}

//------------ main function
int main(int argc, char** argv) {
	Sai2Model::URDF_FOLDERS["EXAMPLE_98_FOLDER"] =
		string(EXAMPLES_FOLDER) + "/98-nullspace";
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

    // Capsule collision
    Vector2d capsule_a = Vector2d(1.0, 0.3);
    Vector2d capsule_b = Vector2d(1.0, 0.0);
    double capsule_radius = 0.1;
    double d_thresh = 0.005;

	// Position plus orientation task
	string link_name = "link3";
	Vector3d pos_in_link = Vector3d(0.5, 0, 0);
	Affine3d compliant_frame = Affine3d(Translation3d(pos_in_link));

	// // Full motion force task
	// auto motion_force_task = make_unique<Sai2Primitives::MotionForceTask>(
	// 	robot, link_name, compliant_frame);

	// Partial motion force task
	vector<Vector3d> controlled_directions_translation = {
		Vector3d::UnitX(), Vector3d::UnitY()};
	vector<Vector3d> controlled_directions_rotation = {Vector3d::UnitZ()};
	auto motion_force_task = make_shared<Sai2Primitives::MotionForceTask>(
		robot, link_name, controlled_directions_translation,
		controlled_directions_rotation);

    motion_force_task->disableInternalOtg();
    motion_force_task->disableSingularityHandling();
    motion_force_task->enableVelocitySaturation(0.5);
    motion_force_task->setPosControlGains(100, 20);
    motion_force_task->setDynamicDecouplingType(Sai2Primitives::FULL_DYNAMIC_DECOUPLING);
	VectorXd motion_force_task_torques = VectorXd::Zero(dof);

	// no gains setting here, using the default task values
	const Matrix3d initial_orientation = robot->rotation(link_name);
	const Vector3d initial_position = robot->position(link_name, pos_in_link);

	// joint task to control the redundancy
	// using default gains and interpolation settings
	auto joint_task = make_unique<Sai2Primitives::JointTask>(robot);
    joint_task->disableInternalOtg();
    joint_task->enableVelocitySaturation(M_PI);
    joint_task->setGains(100, 20);
	VectorXd joint_task_torques = VectorXd::Zero(dof);

	VectorXd initial_q = robot->q();
    // joint_task->setGoalPosition(Vector3d::Zero());

    // // apf joint limits 
    // auto joint_limits = robot->jointLimits();
    // VectorXd q_min(robot->dof()), q_max(robot->dof());
    // int cnt = 0;
    // for (auto limit : joint_limits) {
    //     q_min(cnt) = limit.position_lower + 0.5;
    //     q_max(cnt) = limit.position_upper - 0.5;
    //     cnt++;
    // }
    // double eta = 0.5;
    // VectorXd q_init = robot->q();
    // VectorXd q_delta = VectorXd::Zero(robot->dof());
    // q_delta(5) = 5;

    Vector3d x_init = robot->position(link_name, pos_in_link);
    Vector3d x_delta = Vector3d(0.15, 0, 0);

    // // desired position offsets 
    // vector<Vector3d> desired_offsets {Vector3d(2, 0, 0), Vector3d(0, 0, 0), 
    //                                   Vector3d(0, 2, 0), Vector3d(0, 0, 0), 
    //                                   Vector3d(0, -2, 0), Vector3d(0, 0, 0),
    //                                   Vector3d(0, 0, 2), Vector3d(0, 0, 0)};
    // // vector<Vector3d> desired_offsets {Vector3d(2, 0, 0)};
	// double t_initial = 2;
	// vector<double> t_wait {5, 5};
    // // double t_wait = 10;  // wait between switching desired positions 
	// // double t_reset_wait = 5;  // wait when resetting position 
    // double prev_time = 0;
    // // int cnt = 6 * 1;
	// cnt = 0;
    // int max_cnt = desired_offsets.size();

	// create logger
	Sai2Common::Logger logger("ee_pos", false);
	VectorXd svalues = VectorXd::Zero(6);
    VectorXd robot_q = robot->q();
    Vector3d ee_pos = robot->position(link_name, pos_in_link);
    Vector3d x_curr;
    Vector3d control_forces = Vector3d::Zero();
    Vector3d constraint_direction = Vector3d::Zero();
    int constraint_flag = 0;
	// logger.addToLog(svalues, "svalues");
    logger.addToLog(ee_pos, "ee_pos");
    logger.addToLog(constraint_flag, "constraint_flag");
    logger.addToLog(control_forces, "control_forces");
    logger.addToLog(constraint_direction, "constraint_direction");
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

        robot_q = robot->q();

        // compute distance 
        {
			lock_guard<mutex> lock(mutex_robot);
            x_curr = robot->position(link_name, pos_in_link);
        }
        ee_pos = x_curr;
        std::cout << "ee pos: " << ee_pos.transpose() << "\n";
        auto [distance, con_dir] = distanceToCapsule(ee_pos.head(2), capsule_a, capsule_b, capsule_radius);
        // std::cout << "con_dir: " << con_dir.transpose() << "\n";
        constraint_direction = Vector3d(con_dir(0), con_dir(1), 0).normalized();

        std::cout << "distance: " << distance << "\n";

		// update tasks model. Order is important to define the hierarchy
		N_prec = MatrixXd::Identity(dof, dof);

        // {
        //     lock_guard<mutex> lock(mutex_robot);
        //     MatrixXd Jv_con = Vector3d(1, 0, 0).transpose() * robot->Jv(link_name, pos_in_link);
        //     N_prec = robot->nullspaceMatrix(Jv_con);
        // }
        constraint_flag = 0;

        // add constraint direction if in zone of influence 
        if (distance < d_thresh) {
            // std::cout << "Jv_con: \n" << Jv_con << "\n";
            constraint_flag = 1;
            // {
			//     lock_guard<mutex> lock(mutex_robot);
                MatrixXd Jv_con = constraint_direction.transpose() * robot->Jv(link_name, pos_in_link);
                N_prec = robot->nullspaceMatrix(Jv_con);
            // }

            // constraint debug
            std::cout << "distance: " << distance << "\n";
            std::cout << "constraint direction: " << constraint_direction.transpose() << "\n";
            // std::cout << "nullspace matrix: \n" << N_prec << "\n";
        } else {
            constraint_direction = Vector3d::Zero();
        }

		{
			lock_guard<mutex> lock(mutex_robot);
			motion_force_task->updateTaskModel(N_prec);
		}
		N_prec = motion_force_task->getTaskAndPreviousNullspace();
		// after each task, need to update the nullspace
		// of the previous tasks in order to garantee
		// the dynamic consistency

		joint_task->updateTaskModel(N_prec);

		// -------- set task goals and compute control torques
		// position: move to workspace extents 
        // if (time - prev_time > t_wait[cnt % 2]) {
            // motion_force_task->setGoalPosition(initial_position + desired_offsets[cnt]);
            // cnt++;
            // prev_time = time;
            // if (cnt == max_cnt) cnt = max_cnt - 1;
        // }
        // motion_force_task->setGoalLinearVelocity(Vector3d::Zero());
        // motion_force_task->setGoalLinearAcceleration(Vector3d::Zero());

        motion_force_task->setGoalPosition(x_init + x_delta);

		// compute torques for the different tasks
		motion_force_task_torques = motion_force_task->computeTorques();
		joint_task_torques = joint_task->computeTorques();

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
        // std::cout << "apf force: " << apf_force.transpose() << "\n";

		//------ compute the final torques
		{
			lock_guard<mutex> lock(mutex_torques);
			// control_torques = joint_handler->computeTorques(motion_force_task_torques + joint_task_torques);
			control_torques = motion_force_task_torques + joint_task_torques;
		}

        control_forces = motion_force_task->getNonSingularLambda() * motion_force_task->getUnitControlForces();

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

