/*
 * Example of a Panda controller using MotionForceTask tracking mode with large
 * Cartesian position waypoints and low tracking target velocity limits.
 */

#include <algorithm>
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
const string world_file = "${EXAMPLE_20_FOLDER}/world.urdf";
const string robot_file =
	"${SAI_MODEL_URDF_FOLDER}/panda/panda_arm_sphere.urdf";
const string robot_name = "PANDA";
const string link_name = "end-effector";

constexpr double kLinearVelocityLimit = 0.08;
constexpr double kLinearAccelerationLimit = 0.24;
constexpr double kLinearJerkLimit = 1.0;
constexpr double kRawWaypointVelocity = 0.80;
constexpr double kWaypointPeriod = 8.0;
constexpr double kWaypointArrivalTolerance = 0.02;
constexpr double kWaypointSlowdownDistance = 0.40;

VectorXd ui_torques;
VectorXd control_torques;
mutex mutex_torques;
Vector3d displayed_goal_position = Vector3d::Zero();
mutex mutex_displayed_goal;

struct WaypointReference {
	Vector3d position;
	Vector3d linear_velocity;
	Vector3d linear_acceleration;
	size_t index;
	size_t segment;
};

vector<Vector3d> waypointOffsets() {
	return {
		Vector3d(0.00, 0.00, 0.00),
		Vector3d(0.30, 0.00, 0.00),
		Vector3d(0.30, 0.25, 0.00),
		Vector3d(-0.10, 0.25, 0.08),
		Vector3d(-0.20, -0.15, 0.05),
	};
}

vector<string> waypointMarkerNames() {
	return {
		"GoalWaypoint0",
		"GoalWaypoint1",
		"GoalWaypoint2",
		"GoalWaypoint3",
		"GoalWaypoint4",
	};
}

Vector3d controlPointInLink() {
	return Vector3d(0.0, 0.0, 0.07);
}

Affine3d markerPose(const Vector3d& position) {
	Affine3d pose = Affine3d::Identity();
	pose.translation() = position;
	return pose;
}

WaypointReference desiredWaypointReference(
	const double time, const Vector3d& initial_position,
	const Vector3d& current_desired_position, size_t& active_segment,
	bool& waypoint_arrived) {
	const vector<Vector3d> offsets = waypointOffsets();
	const size_t waypoint_segment =
		static_cast<size_t>(floor(time / kWaypointPeriod));
	const size_t waypoint_index = waypoint_segment % offsets.size();
	if (waypoint_segment != active_segment) {
		active_segment = waypoint_segment;
		waypoint_arrived = false;
	}

	WaypointReference reference;
	reference.index = waypoint_index;
	reference.segment = waypoint_segment;
	reference.position = initial_position + offsets[waypoint_index];
	reference.linear_velocity.setZero();
	reference.linear_acceleration.setZero();

	const Vector3d waypoint_error =
		reference.position - current_desired_position;
	const double distance_to_waypoint = waypoint_error.norm();
	if (distance_to_waypoint <= kWaypointArrivalTolerance) {
		waypoint_arrived = true;
	}

	if (!waypoint_arrived && distance_to_waypoint > 1e-6) {
		const double velocity_scale =
			min(1.0, distance_to_waypoint / kWaypointSlowdownDistance);
		const double target_velocity = kRawWaypointVelocity * velocity_scale;
		reference.linear_velocity =
			target_velocity * waypoint_error.normalized();
		if (velocity_scale < 1.0 &&
			reference.linear_velocity.norm() < kLinearVelocityLimit) {
			reference.linear_acceleration =
				-(kRawWaypointVelocity / kWaypointSlowdownDistance) *
				reference.linear_velocity;
			const double acceleration_norm =
				reference.linear_acceleration.norm();
			const double velocity_limit_margin =
				kLinearVelocityLimit - reference.linear_velocity.norm();
			const double jerk_feasible_acceleration =
				sqrt(2.0 * kLinearJerkLimit * velocity_limit_margin);
			const double acceleration_limit =
				min(kLinearAccelerationLimit, jerk_feasible_acceleration);
			if (acceleration_norm > acceleration_limit) {
				reference.linear_acceleration *=
					acceleration_limit / acceleration_norm;
			}
		}
	}

	return reference;
}

}  // namespace

void control(shared_ptr<SaiModel::SaiModel> robot,
			 shared_ptr<SaiSimulation::SaiSimulation> sim);
void simulation(shared_ptr<SaiModel::SaiModel> robot,
				shared_ptr<SaiSimulation::SaiSimulation> sim);

int main(int argc, char** argv) {
	SaiModel::URDF_FOLDERS["EXAMPLE_20_FOLDER"] =
		string(EXAMPLES_FOLDER) + "/20-panda_tracking_velocity_limits";
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

	const vector<Vector3d> offsets = waypointOffsets();
	const vector<string> marker_names = waypointMarkerNames();
	for (size_t i = 0; i < offsets.size(); ++i) {
		graphics->updateObjectGraphics(
			marker_names[i], markerPose(initial_position + offsets[i]));
	}
	graphics->updateObjectGraphics("ActiveGoal", markerPose(initial_position));
	displayed_goal_position = initial_position;

	ui_torques = VectorXd::Zero(robot->dof());
	control_torques = VectorXd::Zero(robot->dof());

	fSimulationRunning = true;
	thread sim_thread(simulation, robot, sim);
	thread ctrl_thread(control, robot, sim);

	while (graphics->isWindowOpen()) {
		graphics->updateRobotGraphics(robot_name, robot->q());
		for (size_t i = 0; i < offsets.size(); ++i) {
			graphics->updateObjectGraphics(
				marker_names[i], markerPose(initial_position + offsets[i]));
		}
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
		kLinearVelocityLimit, kLinearAccelerationLimit, kLinearJerkLimit,
		1.0, 3.0, 20.0);
	motion_force_task->enableInternalOtgTrackingMode(1.0, 1, 16);
	motion_force_task->setInternalOtgTrackingTargetLimits(
		kLinearVelocityLimit, kLinearAccelerationLimit, 1.0, 3.0);

	const Matrix3d initial_orientation =
		robot->rotationInWorld(link_name);
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
	double max_observed_desired_velocity = 0.0;
	size_t active_waypoint_segment =
		static_cast<size_t>(floor(timer.elapsedSimTime() / kWaypointPeriod));
	bool waypoint_arrived = false;

	while (fSimulationRunning) {
		timer.waitForNextLoop();
		const double time = timer.elapsedSimTime();

		robot->setQ(sim->getJointPositions(robot_name));
		robot->setDq(sim->getJointVelocities(robot_name));
		robot->updateModel();

		robot_controller->updateControllerTaskModels();

		const WaypointReference reference =
			desiredWaypointReference(time, initial_position,
									 motion_force_task->getDesiredPosition(),
									 active_waypoint_segment, waypoint_arrived);
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

		const double desired_velocity_norm =
			motion_force_task->getDesiredLinearVelocity().norm();
		max_observed_desired_velocity =
			max(max_observed_desired_velocity, desired_velocity_norm);

		if (timer.elapsedCycles() % 1000 == 0) {
			cout << "time: " << time << endl;
			cout << "waypoint index: " << reference.index << endl;
			cout << "goal position: "
				 << motion_force_task->getGoalPosition().transpose() << endl;
			cout << "desired position: "
				 << motion_force_task->getDesiredPosition().transpose() << endl;
			cout << "current position: "
				 << motion_force_task->getCurrentPosition().transpose() << endl;
			cout << "raw waypoint velocity norm: "
				 << reference.linear_velocity.norm() << endl;
			cout << "raw waypoint acceleration norm: "
				 << reference.linear_acceleration.norm() << endl;
			cout << "waypoint arrived: " << waypoint_arrived << endl;
			cout << "desired velocity norm: " << desired_velocity_norm << endl;
			cout << "linear velocity limit: " << kLinearVelocityLimit << endl;
			cout << "max observed desired velocity norm: "
				 << max_observed_desired_velocity << endl;
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
