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
const string world_file = "${EXAMPLE_18_FOLDER}/world.urdf";
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

Eigen::MatrixXd pseudoInverse(const Eigen::MatrixXd& mat, double tolerance = 1e-6) {
    Eigen::JacobiSVD<Eigen::MatrixXd> svd(mat, Eigen::ComputeThinU | Eigen::ComputeThinV);
    Eigen::VectorXd singularValues = svd.singularValues();
    Eigen::MatrixXd singularValuesInv(mat.cols(), mat.rows());
    singularValuesInv.setZero();

    // Invert nonzero singular values
    for (int i = 0; i < singularValues.size(); ++i) {
        if (singularValues(i) > tolerance) {
            singularValuesInv(i, i) = 1.0 / singularValues(i);
        }
    }

    // Compute pseudoinverse: V * S⁺ * U^T
    return svd.matrixV() * singularValuesInv * svd.matrixU().transpose();
}

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
    motion_force_task->enableVelocitySaturation();
	VectorXd motion_force_task_torques = VectorXd::Zero(dof);

	// no gains setting here, using the default task values
	const Matrix3d initial_orientation = robot->rotation(link_name);
	const Vector3d initial_position = robot->position(link_name, pos_in_link);

	// joint task to control the redundancy
	// using default gains and interpolation settings
	auto joint_task = make_unique<Sai2Primitives::JointTask>(robot);
    joint_task->setGains(100, 20);
	VectorXd joint_task_torques = VectorXd::Zero(dof);

	VectorXd initial_q = robot->q();
    joint_task->setGoalPosition(initial_q);

    // desired position offsets 
    vector<Vector3d> desired_offsets {Vector3d(2, 0, 0), Vector3d(0, 0, 0), 
                                      Vector3d(0, 2, 0), Vector3d(0, 0, 0), 
                                      Vector3d(0, -2, 0), Vector3d(0, 0, 0),
                                      Vector3d(0, 0, 2), Vector3d(0, 0, 0)};
    // vector<Vector3d> desired_offsets {Vector3d(2, 0, 0)};
	double t_initial = 2;
	vector<double> t_wait {5, 5};
    // double t_wait = 10;  // wait between switching desired positions 
	// double t_reset_wait = 5;  // wait when resetting position 
    double prev_time = 0;
    // int cnt = 6 * 1;
	int cnt = 0;
    int max_cnt = desired_offsets.size();

	// create logger
	Sai2Common::Logger logger("singular_values", false);
	VectorXd svalues = VectorXd::Zero(6);
	logger.addToLog(svalues, "svalues");
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

		// -------- set task goals and compute control torques
		// position: move to workspace extents 
        if (time - prev_time > t_wait[cnt % 2]) {
            motion_force_task->setGoalPosition(initial_position + desired_offsets[cnt]);
            cnt++;
            prev_time = time;
            if (cnt == max_cnt) cnt = max_cnt - 1;
        }
        motion_force_task->setGoalLinearVelocity(Vector3d::Zero());
        motion_force_task->setGoalLinearAcceleration(Vector3d::Zero());

		// compute torques for the different tasks
		motion_force_task_torques = motion_force_task->computeTorques();
		joint_task_torques = joint_task->computeTorques();

		//------ compute the final torques
		{
			lock_guard<mutex> lock(mutex_torques);
			// control_torques = joint_handler->computeTorques(motion_force_task_torques + joint_task_torques);
			control_torques = motion_force_task_torques + joint_task_torques;
		}

		// debug compute the kinematics approach
		VectorXd unit_mass_force = motion_force_task->getUnitControlForces();
		MatrixXd U_s = motion_force_task->getSingularTaskRange();
		MatrixXd J_s = motion_force_task->getSingularJacobian();
		VectorXd JdotQdot = robot->jDotQDot(link_name, pos_in_link);
		// std::cout << "unit mass force projected into singular space: " << unit_mass_force.transpose() << "\n";
		// std::cout << "jdotqdot: " << JdotQdot << "\n";
		VectorXd dynamic_bias = U_s.transpose() * JdotQdot;
		VectorXd b = U_s.transpose() * unit_mass_force - U_s.transpose() * JdotQdot;
		VectorXd ddq_sol = J_s.colPivHouseholderQr().solve(b);
		// std::cout << "J_s: \n" << J_s << "\n";
		// std::cout << "b: \n" << b.transpose() << "\n";
		// std::cout << "ddq sol: " << ddq_sol.transpose() << "\n";

		// compute task inertia matrix 
		MatrixXd singular_task_lambda = (J_s * robot->MInv() * J_s.transpose()).inverse();
		// std::cout << "singular task lambda: \n" << singular_task_lambda << "\n";

		// solve closed-form 
		if (!J_s.isZero()) {
			MatrixXd A = J_s;
			std::cout << A << "\n";
			VectorXd ddq_opt = robot->MInv() * A.transpose() * (A * robot->MInv() * A.transpose()).inverse() * b;
			std::cout << "ddq opt: " << ddq_opt.transpose() << "\n";

			// compute torque from this 
			MatrixXd J_qs = motion_force_task->getSingularJointJacobian();
			MatrixXd singular_joint_lambda = (J_qs * robot->MInv() * J_qs.transpose()).inverse();
			MatrixXd singular_joint_range = motion_force_task->getSingularJointTaskRange();
			// std::cout << "singular joint lambda: \n" << singular_joint_lambda << "\n";
			// std::cout << "Js: " << J_s.transpose() << "\n";
			VectorXd torque_from_ddq = J_qs.transpose() * singular_joint_lambda * singular_joint_range.transpose() * ddq_opt;
			// std::cout << "1\n";	
			// torque_from_ddq = J_s.transpose() * torque_from_ddq;

			std::cout << "torque from ddq: " << torque_from_ddq.transpose() << "\n";

			// compute mp-inverse 
			MatrixXd test(2, 2);
			test << 0, 0, 2, 1;

			auto test_inv = pseudoInverse(test);
			std::cout << "test inv: \n" << test_inv << "\n";

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