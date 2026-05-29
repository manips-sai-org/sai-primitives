/*
 * Example of a controller for a Panda arm (7DoF robot) using a centroidal
 * linear momentum task and a joint posture task in the nullspace of the
 * centroidal linear momentum task.
 */

#include <math.h>
#include <signal.h>

#include <iostream>
#include <mutex>
#include <string>
#include <thread>

#include "SaiGraphics.h"
#include "SaiModel.h"
#include "SaiSimulation.h"
#include "tasks/CentroidalLinearMomentumTask.h"
#include "tasks/JointTask.h"
#include "timer/LoopTimer.h"

bool fSimulationRunning = false;
void sighandler(int) { fSimulationRunning = false; }

using namespace std;
using namespace Eigen;

const string world_file = "${EXAMPLE_13_FOLDER}/world.urdf";
const string robot_file =
	"${SAI_MODEL_URDF_FOLDER}/panda/panda_arm_sphere.urdf";
const string robot_name = "PANDA";

VectorXd ui_torques;
VectorXd control_torques;
mutex mutex_torques;

void control(shared_ptr<SaiModel::SaiModel> robot,
			 shared_ptr<SaiSimulation::SaiSimulation> sim);
void simulation(shared_ptr<SaiModel::SaiModel> robot,
				shared_ptr<SaiSimulation::SaiSimulation> sim);

int main(int argc, char** argv) {
	SaiModel::URDF_FOLDERS["EXAMPLE_13_FOLDER"] =
		string(EXAMPLES_FOLDER) + "/13-centroidal_linear_momentum";
	cout << "Loading URDF world model file: "
		 << SaiModel::ReplaceUrdfPathPrefix(world_file) << endl;

	signal(SIGABRT, &sighandler);
	signal(SIGTERM, &sighandler);
	signal(SIGINT, &sighandler);

	auto graphics = make_shared<SaiGraphics::SaiGraphics>(world_file);
	graphics->addUIForceInteraction(robot_name);

	auto sim = make_shared<SaiSimulation::SaiSimulation>(world_file);

	auto robot = make_shared<SaiModel::SaiModel>(robot_file, false);
	robot->setQ(sim->getJointPositions(robot_name));
	robot->updateModel();

	ui_torques = VectorXd::Zero(robot->dof());
	control_torques = VectorXd::Zero(robot->dof());

	fSimulationRunning = true;
	thread sim_thread(simulation, robot, sim);
	thread ctrl_thread(control, robot, sim);

	while (graphics->isWindowOpen()) {
		graphics->updateRobotGraphics(robot_name, robot->q());
		graphics->renderGraphicsWorld();
		{
			lock_guard<mutex> lock(mutex_torques);
			ui_torques = graphics->getUITorques(robot_name);
		}
	}

	fSimulationRunning = false;
	sim_thread.join();
	ctrl_thread.join();

	return 0;
}

//------------------------------------------------------------------------------
void control(shared_ptr<SaiModel::SaiModel> robot,
			 shared_ptr<SaiSimulation::SaiSimulation> sim) {
	robot->updateModel();
	const int dof = robot->dof();
	MatrixXd N_prec = MatrixXd::Identity(dof, dof);

	auto centroidal_linear_momentum_task =
		make_unique<SaiPrimitives::CentroidalLinearMomentumTask>(
			robot, "centroidal_linear_momentum_task");
	centroidal_linear_momentum_task->setGains(20.0);

	const VectorXd initial_q = robot->q();
	auto joint_task = make_unique<SaiPrimitives::JointTask>(robot);
	joint_task->disableInternalOtg();
	joint_task->setGains(25.0, 10.0);
	joint_task->setGoalPosition(initial_q);

	const double control_freq = 1000.0;
	SaiCommon::LoopTimer timer(control_freq, 1e6);

	while (fSimulationRunning) {
		timer.waitForNextLoop();
		const double time = timer.elapsedSimTime();

		robot->setQ(sim->getJointPositions(robot_name));
		robot->setDq(sim->getJointVelocities(robot_name));
		robot->updateModel();

		// Update task models in priority order. The joint task receives the
		// centroidal linear momentum task nullspace as N_prec.
		N_prec.setIdentity(dof, dof);
		centroidal_linear_momentum_task->updateTaskModel(N_prec);
		N_prec =
			centroidal_linear_momentum_task->getTaskAndPreviousNullspace();
		joint_task->updateTaskModel(N_prec);

		const double momentum_amplitude = 0.4;
		const double momentum_frequency = 2.0 * M_PI * 0.35;
		centroidal_linear_momentum_task->setGoalMomentum(
			momentum_amplitude * sin(momentum_frequency * time) *
			Vector3d::UnitY());
		centroidal_linear_momentum_task->setGoalMomentumVelocity(
			momentum_amplitude * momentum_frequency *
			cos(momentum_frequency * time) * Vector3d::UnitY());

		VectorXd centroidal_momentum_torques =
			centroidal_linear_momentum_task->computeTorques();
		VectorXd joint_torques =
			joint_task->computeTorques(centroidal_momentum_torques);

		{
			lock_guard<mutex> lock(mutex_torques);
			control_torques = centroidal_momentum_torques + joint_torques;
		}

		if (timer.elapsedCycles() % 500 == 0) {
			cout << "time: " << time << endl;
			cout << "goal centroidal linear momentum: "
				 << centroidal_linear_momentum_task->getGoalMomentum()
						.transpose()
				 << endl;
			cout << "current centroidal linear momentum: "
				 << centroidal_linear_momentum_task->getCurrentMomentum()
						.transpose()
				 << endl;
			cout << "joint position error: "
				 << (joint_task->getGoalPosition() -
					 joint_task->getCurrentPosition())
						.norm()
				 << endl;
			cout << endl;
		}
	}

	timer.stop();
	cout << "\nControl loop timer stats:\n";
	timer.printInfoPostRun();
}

//------------------------------------------------------------------------------
void simulation(shared_ptr<SaiModel::SaiModel> robot,
				shared_ptr<SaiSimulation::SaiSimulation> sim) {
	fSimulationRunning = true;

	const double sim_freq = 2000.0;
	SaiCommon::LoopTimer timer(sim_freq);

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
