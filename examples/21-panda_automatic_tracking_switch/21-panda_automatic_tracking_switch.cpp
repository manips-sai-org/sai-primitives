/*
 * Example of a Panda controller that automatically switches the internal OTG
 * between regular Ruckig OTG and Ruckig Trackig.
 *
 * Large discrete goal jumps remain in regular OTG. Small high-frequency goal
 * updates enter Trackig automatically. Holding the goal stable returns to
 * regular OTG.
 */

#include <cmath>
#include <csignal>
#include <iostream>
#include <memory>
#include <mutex>
#include <string>
#include <thread>
#include <vector>

#include "RobotController.h"
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

namespace {
constexpr double kPi = 3.14159265358979323846;

const string world_file = "${EXAMPLE_21_FOLDER}/world.urdf";
const string robot_file =
	"${SAI_MODEL_URDF_FOLDER}/panda/panda_arm_sphere.urdf";
const string robot_name = "PANDA";
const string link_name = "end-effector";

constexpr double kDiscreteDuration = 3.0;
constexpr double kStreamDuration = 4.0;
constexpr double kHoldDuration = 2.0;
constexpr double kHalfCycleDuration =
	kDiscreteDuration + kStreamDuration + kHoldDuration;
constexpr double kCycleDuration = 2.0 * kHalfCycleDuration;
constexpr double kStreamRadius = 0.05;
constexpr double kStreamOmega = 2.0 * kPi / kStreamDuration;

VectorXd ui_torques;
VectorXd control_torques;
mutex mutex_torques;
Vector3d displayed_goal_position = Vector3d::Zero();
mutex mutex_displayed_goal;

enum class ReferencePhase {
	DiscreteA,
	StreamA,
	HoldA,
	DiscreteB,
	StreamB,
	HoldB,
};

struct GoalReference {
	Vector3d position;
	Vector3d linear_velocity;
	Vector3d linear_acceleration;
	ReferencePhase phase;
};

Vector3d controlPointInLink() {
	return Vector3d(0.0, 0.0, 0.07);
}

Vector3d discreteGoalOffsetA() {
	return Vector3d(0.18, 0.08, 0.04);
}

Vector3d discreteGoalOffsetB() {
	return Vector3d(-0.12, -0.16, 0.08);
}

Affine3d markerPose(const Vector3d& position) {
	Affine3d pose = Affine3d::Identity();
	pose.translation() = position;
	return pose;
}

const char* phaseName(const ReferencePhase phase) {
	switch (phase) {
		case ReferencePhase::DiscreteA:
			return "large discrete goal A";
		case ReferencePhase::StreamA:
			return "streamed small goals around A";
		case ReferencePhase::HoldA:
			return "hold goal A";
		case ReferencePhase::DiscreteB:
			return "large discrete goal B";
		case ReferencePhase::StreamB:
			return "streamed small goals around B";
		case ReferencePhase::HoldB:
			return "hold goal B";
	}
	return "unknown";
}

GoalReference streamedReference(const Vector3d& center,
								const double stream_time,
								const ReferencePhase phase) {
	const double theta = kStreamOmega * stream_time;

	GoalReference reference;
	reference.phase = phase;
	reference.position =
		center + Vector3d(kStreamRadius * sin(theta),
						  kStreamRadius * (1.0 - cos(theta)), 0.0);
	reference.linear_velocity =
		Vector3d(kStreamRadius * kStreamOmega * cos(theta),
				 kStreamRadius * kStreamOmega * sin(theta), 0.0);
	reference.linear_acceleration =
		Vector3d(-kStreamRadius * kStreamOmega * kStreamOmega * sin(theta),
				 kStreamRadius * kStreamOmega * kStreamOmega * cos(theta),
				 0.0);

	return reference;
}

GoalReference desiredGoalReference(const double time,
								   const Vector3d& initial_position) {
	const Vector3d goal_a = initial_position + discreteGoalOffsetA();
	const Vector3d goal_b = initial_position + discreteGoalOffsetB();
	const double cycle_time = fmod(time, kCycleDuration);
	const bool second_half = cycle_time >= kHalfCycleDuration;
	const double half_time =
		second_half ? cycle_time - kHalfCycleDuration : cycle_time;
	const Vector3d center = second_half ? goal_b : goal_a;

	GoalReference reference;
	reference.position = center;
	reference.linear_velocity.setZero();
	reference.linear_acceleration.setZero();

	if (half_time < kDiscreteDuration) {
		reference.phase = second_half ? ReferencePhase::DiscreteB
									  : ReferencePhase::DiscreteA;
		return reference;
	}

	if (half_time < kDiscreteDuration + kStreamDuration) {
		return streamedReference(
			center, half_time - kDiscreteDuration,
			second_half ? ReferencePhase::StreamB : ReferencePhase::StreamA);
	}

	reference.phase =
		second_half ? ReferencePhase::HoldB : ReferencePhase::HoldA;
	return reference;
}

}  // namespace

void control(shared_ptr<SaiModel::SaiModel> robot,
			 shared_ptr<SaiSimulation::SaiSimulation> sim);
void simulation(shared_ptr<SaiModel::SaiModel> robot,
				shared_ptr<SaiSimulation::SaiSimulation> sim);

int main(int argc, char** argv) {
	SaiModel::URDF_FOLDERS["EXAMPLE_21_FOLDER"] =
		string(EXAMPLES_FOLDER) + "/21-panda_automatic_tracking_switch";
	cout << "Loading URDF world model file: "
		 << SaiModel::ReplaceUrdfPathPrefix(world_file) << endl;

	signal(SIGABRT, &sighandler);
	signal(SIGTERM, &sighandler);
	signal(SIGINT, &sighandler);

	auto graphics = make_shared<SaiGraphics::SaiGraphics>(world_file);
	graphics->addUIForceInteraction(robot_name);
	graphics->showLinkFrame(true, robot_name, link_name, 0.18);

	auto sim = make_shared<SaiSimulation::SaiSimulation>(world_file);

	auto robot = make_shared<SaiModel::SaiModel>(robot_file, false);
	robot->setTRobotBase(sim->getRobotBaseTransform(robot_name));
	robot->setQ(sim->getJointPositions(robot_name));
	robot->updateModel();

	const Vector3d initial_position =
		robot->positionInWorld(link_name, controlPointInLink());
	const Vector3d goal_a = initial_position + discreteGoalOffsetA();
	const Vector3d goal_b = initial_position + discreteGoalOffsetB();
	graphics->updateObjectGraphics("DiscreteGoalA", markerPose(goal_a));
	graphics->updateObjectGraphics("DiscreteGoalB", markerPose(goal_b));
	graphics->updateObjectGraphics("ActiveGoal", markerPose(goal_a));
	displayed_goal_position = goal_a;

	ui_torques = VectorXd::Zero(robot->dof());
	control_torques = VectorXd::Zero(robot->dof());

	fSimulationRunning = true;
	thread sim_thread(simulation, robot, sim);
	thread ctrl_thread(control, robot, sim);

	while (graphics->isWindowOpen()) {
		graphics->updateRobotGraphics(robot_name, robot->q());
		graphics->updateObjectGraphics("DiscreteGoalA", markerPose(goal_a));
		graphics->updateObjectGraphics("DiscreteGoalB", markerPose(goal_b));
		{
			lock_guard<mutex> lock(mutex_displayed_goal);
			graphics->updateObjectGraphics(
				"ActiveGoal", markerPose(displayed_goal_position));
		}
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

void control(shared_ptr<SaiModel::SaiModel> robot,
			 shared_ptr<SaiSimulation::SaiSimulation> sim) {
	robot->updateModel();

	const Vector3d pos_in_link = controlPointInLink();
	const Affine3d compliant_frame = Affine3d(Translation3d(pos_in_link));

	auto motion_force_task = make_shared<SaiPrimitives::MotionForceTask>(
		robot, link_name, compliant_frame);
	motion_force_task->setPosControlGains(100.0, 20.0);
	motion_force_task->setOriControlGains(100.0, 20.0);
	motion_force_task->enableInternalOtgJerkLimited(
		0.45, 1.5, 10.0, 1.2, 4.0, 20.0);
	motion_force_task->setInternalOtgTrackingTargetLimits(
		0.45, 1.5, 1.2, 4.0);
	motion_force_task->enableAutomaticInternalOtgTrackingModeSwitch(
		0.01, 2.0 * kPi / 180.0, 1e-6, 1e-5, 0.35, 4, 0.8, 6, 16);

	const Matrix3d initial_orientation = robot->rotationInWorld(link_name);
	const Vector3d initial_position =
		robot->positionInWorld(link_name, pos_in_link);
	motion_force_task->setGoalOrientation(initial_orientation);

	auto joint_task = make_shared<SaiPrimitives::JointTask>(robot);
	joint_task->setGains(80.0, 18.0);
	joint_task->setGoalPosition(robot->q());

	vector<shared_ptr<SaiPrimitives::TemplateTask>> task_list = {
		motion_force_task, joint_task};
	auto robot_controller =
		make_unique<SaiPrimitives::RobotController>(robot, task_list);

	const double control_freq = 1000.0;
	SaiCommon::LoopTimer timer(control_freq, 1e6);
	bool previous_tracking_mode =
		motion_force_task->getInternalOtgTrackingModeEnabled();
	ReferencePhase previous_phase = ReferencePhase::DiscreteA;

	cout << "automatic OTG/Trackig switch enabled: "
		 << motion_force_task
				->getAutomaticInternalOtgTrackingModeSwitchEnabled()
		 << endl;
	cout << "tracking mode starts disabled: " << previous_tracking_mode
		 << endl
		 << endl;

	while (fSimulationRunning) {
		timer.waitForNextLoop();
		const double time = timer.elapsedSimTime();

		robot->setQ(sim->getJointPositions(robot_name));
		robot->setDq(sim->getJointVelocities(robot_name));
		robot->updateModel();

		robot_controller->updateControllerTaskModels();

		const GoalReference reference =
			desiredGoalReference(time, initial_position);
		{
			lock_guard<mutex> lock(mutex_displayed_goal);
			displayed_goal_position = reference.position;
		}
		motion_force_task->setGoalPosition(reference.position);
		motion_force_task->setGoalLinearVelocity(reference.linear_velocity);
		motion_force_task->setGoalLinearAcceleration(
			reference.linear_acceleration);
		motion_force_task->setGoalOrientation(initial_orientation);
		motion_force_task->setGoalAngularVelocity(Vector3d::Zero());
		motion_force_task->setGoalAngularAcceleration(Vector3d::Zero());

		{
			lock_guard<mutex> lock(mutex_torques);
			control_torques = robot_controller->computeControlTorques();
		}

		const bool tracking_mode =
			motion_force_task->getInternalOtgTrackingModeEnabled();
		if (tracking_mode != previous_tracking_mode) {
			cout << "mode switch at t = " << time << " s: "
				 << (tracking_mode ? "Trackig" : "regular OTG") << endl;
			previous_tracking_mode = tracking_mode;
		}
		if (reference.phase != previous_phase) {
			cout << "phase at t = " << time << " s: "
				 << phaseName(reference.phase) << endl;
			previous_phase = reference.phase;
		}

		if (timer.elapsedCycles() % 1000 == 0) {
			cout << "time: " << time << endl;
			cout << "phase: " << phaseName(reference.phase) << endl;
			cout << "internal OTG enabled: "
				 << motion_force_task->getInternalOtgEnabled() << endl;
			cout << "tracking mode enabled: " << tracking_mode << endl;
			cout << "goal position: "
				 << motion_force_task->getGoalPosition().transpose() << endl;
			cout << "desired position: "
				 << motion_force_task->getDesiredPosition().transpose()
				 << endl;
			cout << "current position: "
				 << motion_force_task->getCurrentPosition().transpose()
				 << endl;
			cout << "goal velocity norm: "
				 << reference.linear_velocity.norm() << endl;
			cout << "position error: "
				 << motion_force_task->getPositionError().norm() << endl;
			cout << endl;
		}
	}

	timer.stop();
	cout << "\nControl loop timer stats:\n";
	timer.printInfoPostRun();
}

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
