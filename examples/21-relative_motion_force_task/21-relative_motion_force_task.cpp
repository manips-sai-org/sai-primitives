#include <math.h>
#include <signal.h>

#include <iostream>
#include <mutex>
#include <string>
#include <thread>

#include "SaiGraphics.h"
#include "SaiModel.h"
#include "SaiSimulation.h"
#include "tasks/JointTask.h"
#include "tasks/MotionForceTask.h"
#include "timer/LoopTimer.h"

bool fSimulationRunning = false;
void sighandler(int) { fSimulationRunning = false; }

using namespace std;
using namespace Eigen;

const string world_file = "${EXAMPLE_21_FOLDER}/world.urdf";
const string robot_file =
	"${SAI_MODEL_URDF_FOLDER}/panda/panda_arm_sphere.urdf";
const string robot_name = "panda";

// ui torques and control torques
VectorXd ui_torques;
VectorXd control_torques;

// mutex for global variables between different threads
mutex mutex_torques;

Vector3d goal_position_in_relative_frame;
Matrix3d goal_orientation_in_relative_frame;

// simulation and control loop
void control(shared_ptr<SaiModel::SaiModel> robot,
			 shared_ptr<SaiSimulation::SaiSimulation> sim);
void simulation(shared_ptr<SaiModel::SaiModel> robot,
				shared_ptr<SaiSimulation::SaiSimulation> sim);

//------------ main function
int main(int argc, char **argv)
{
	SaiModel::URDF_FOLDERS["EXAMPLE_21_FOLDER"] =
		string(EXAMPLES_FOLDER) + "/21-relative_motion_force_task";
	cout << "Loading URDF world model file: "
		 << SaiModel::ReplaceUrdfPathPrefix(world_file) << endl;

	// set up signal handler
	signal(SIGABRT, &sighandler);
	signal(SIGTERM, &sighandler);
	signal(SIGINT, &sighandler);

	// load graphics scene
	auto graphics = make_shared<SaiGraphics::SaiGraphics>(world_file);
	graphics->addUIForceInteraction(robot_name);

	// load simulation world
	auto sim = make_shared<SaiSimulation::SaiSimulation>(world_file);

	// load robots
	auto robot = make_shared<SaiModel::SaiModel>(robot_file, false);
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

	graphics->showLinkFrame(true, robot_name, "link2");
	graphics->showLinkFrame(true, robot_name, "end-effector");
	graphics->showLinkFrame(true, robot_name, "link0");

	// while window is open:
	while (graphics->isWindowOpen())
	{
		graphics->updateRobotGraphics(robot_name, robot->q());
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
void control(shared_ptr<SaiModel::SaiModel> robot,
			 shared_ptr<SaiSimulation::SaiSimulation> sim)
{
	// update robot model and initialize control vectors
	robot->updateModel();
	int dof = robot->dof();
	MatrixXd N_prec = MatrixXd::Identity(dof, dof);

	// Position plus orientation task
	string link_name = "end-effector";
	Affine3d compliant_frame = Affine3d::Identity();
	auto motion_force_task = make_unique<SaiPrimitives::MotionForceTask>(
		robot, link_name, compliant_frame);
	VectorXd motion_force_task_torques = VectorXd::Zero(dof);
	motion_force_task->disableInternalOtg();

	// no gains setting here, using the default task values
	const Matrix3d initial_orientation = robot->rotation(link_name);
	const Vector3d initial_position = robot->position(link_name);

	string relative_link_name = "link2";

	goal_position_in_relative_frame = Vector3d(0.2, 0.0, 0.2);
	goal_orientation_in_relative_frame = Matrix3d::Identity();

	// joint task to control the redundancy
	// using default gains and interpolation settings
	auto joint_task = make_unique<SaiPrimitives::JointTask>(robot);
	VectorXd joint_task_torques = VectorXd::Zero(dof);

	VectorXd initial_q = robot->q();

	// create a loop timer
	double control_freq = 1000;
	SaiCommon::LoopTimer timer(control_freq, 1e6);

	while (fSimulationRunning)
	{
		timer.waitForNextLoop();
		const double time = timer.elapsedSimTime();

		// read joint positions, velocities, update model
		robot->setQ(sim->getJointPositions(robot_name));
		robot->setDq(sim->getJointVelocities(robot_name));
		robot->updateModel();

		// update tasks model. Order is important to define the hierarchy
		N_prec = MatrixXd::Identity(dof, dof);

		motion_force_task->updateTaskModel(N_prec);
		N_prec = motion_force_task->getTaskAndPreviousNullspace();
		// after each task, need to update the nullspace
		// of the previous tasks in order to garantee
		// the dyamic consistency

		joint_task->updateTaskModel(N_prec);

		// first calculate the goal position and orientation in world frame
		Affine3d T_world_relative = robot->transformInWorld(relative_link_name, goal_position_in_relative_frame, goal_orientation_in_relative_frame);

		// set constant goal position and orientation
		motion_force_task->setGoalPosition(T_world_relative.translation());
		motion_force_task->setGoalOrientation(T_world_relative.rotation());

		// cout << "goal position: " << goal_position_in_relative_frame.transpose() << endl;
		// cout << "goal orientation: \n"
		// 	 << goal_orientation_in_relative_frame << endl;

		// compute torques for the different tasks
		motion_force_task_torques = motion_force_task->computeTorques();
		joint_task_torques = joint_task->computeTorques();

		//------ compute the final torques
		{
			lock_guard<mutex> lock(mutex_torques);
			control_torques = motion_force_task_torques + joint_task_torques;
		}
	}
	timer.stop();
	cout << "\nControl loop timer stats:\n";
	timer.printInfoPostRun();
}

//------------------------------------------------------------------------------
void simulation(shared_ptr<SaiModel::SaiModel> robot,
				shared_ptr<SaiSimulation::SaiSimulation> sim)
{
	fSimulationRunning = true;

	// create a timer
	double sim_freq = 2000;
	SaiCommon::LoopTimer timer(sim_freq);

	sim->setTimestep(1.0 / sim_freq);
	sim->disableJointLimits(robot_name);

	while (fSimulationRunning)
	{
		timer.waitForNextLoop();
		{
			lock_guard<mutex> lock(mutex_torques);
			sim->setJointTorques(robot_name, control_torques + 10 * ui_torques);
		}
		sim->integrate();
	}
	timer.stop();
	cout << "\nSimulation loop timer stats:\n";
	timer.printInfoPostRun();
}