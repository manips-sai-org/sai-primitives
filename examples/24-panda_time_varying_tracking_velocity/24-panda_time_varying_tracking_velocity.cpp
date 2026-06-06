/*
 * Example of a Panda controller using MotionForceTask tracking mode with a
 * time-varying tracking target max linear velocity while following an
 * oscillatory Cartesian position reference.
 */

#include <algorithm>
#include <cmath>
#include <csignal>
#include <deque>
#include <iomanip>
#include <iostream>
#include <memory>
#include <mutex>
#include <sstream>
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

const string world_file = "${EXAMPLE_24_FOLDER}/world.urdf";
const string robot_file =
	"${SAI_MODEL_URDF_FOLDER}/panda/panda_arm_sphere.urdf";
const string robot_name = "PANDA";
const string link_name = "end-effector";

constexpr double kRegularOtgMaxLinearVelocity = 0.24;
constexpr double kRegularOtgMaxLinearAcceleration = 1.2;
constexpr double kRegularOtgMaxLinearJerk = 8.0;
constexpr double kRegularOtgMaxAngularVelocity = 1.2;
constexpr double kRegularOtgMaxAngularAcceleration = 4.0;
constexpr double kRegularOtgMaxAngularJerk = 20.0;

constexpr double kTrackingMinLinearVelocity = 0.04;
constexpr double kTrackingMaxLinearVelocity = 0.18;
constexpr double kTrackingLinearAcceleration = 1.0;
constexpr double kTrackingAngularVelocity = 1.0;
constexpr double kTrackingAngularAcceleration = 3.0;
constexpr double kTrackingVelocityFrequency = 0.08;

constexpr double kOscillationFrequencyX = 0.18;
constexpr double kOscillationFrequencyY = 0.13;
constexpr double kOscillationFrequencyZ = 0.10;
constexpr double kVelocityPlotHistoryDuration = 5.0;
constexpr size_t kVelocityPlotPointCount = 50;
constexpr double kVelocityPlotSamplePeriod =
	kVelocityPlotHistoryDuration / (kVelocityPlotPointCount - 1);
constexpr double kVelocityPlotMaxVelocity = 0.30;

VectorXd ui_torques;
VectorXd control_torques;
mutex mutex_torques;
Vector3d displayed_goal_position = Vector3d::Zero();
Vector3d displayed_desired_position = Vector3d::Zero();
mutex mutex_displayed_points;

struct DisplayState {
	double tracking_linear_velocity_limit = kTrackingMinLinearVelocity;
	double goal_linear_velocity_norm = 0.0;
	double limited_linear_velocity_norm = 0.0;
	double desired_linear_velocity_norm = 0.0;
	double current_linear_velocity_norm = 0.0;
	double position_error = 0.0;
	bool tracking_mode_enabled = false;
};

DisplayState displayed_state;
mutex mutex_displayed_state;

struct VelocityHistoryPoint {
	double time = 0.0;
	double current_velocity_norm = 0.0;
	double limited_velocity_norm = 0.0;
};

deque<VelocityHistoryPoint> displayed_velocity_history;
mutex mutex_velocity_history;
double last_velocity_history_sample_time = -kVelocityPlotSamplePeriod;

struct PositionReference {
	Vector3d position;
	Vector3d linear_velocity;
	Vector3d linear_acceleration;
};

Vector3d controlPointInLink() {
	return Vector3d(0.0, 0.0, 0.07);
}

Affine3d markerPose(const Vector3d& position) {
	Affine3d pose = Affine3d::Identity();
	pose.translation() = position;
	return pose;
}

double trackingLinearVelocityLimit(const double time) {
	const double wave =
		0.5 * (1.0 + sin(2.0 * kPi * kTrackingVelocityFrequency * time));
	return kTrackingMinLinearVelocity +
		   (kTrackingMaxLinearVelocity - kTrackingMinLinearVelocity) * wave;
}

PositionReference desiredPositionTrajectory(
	const double time, const Vector3d& initial_position) {
	const Vector3d amplitude(0.12, 0.08, 0.05);
	const Vector3d omega(
		2.0 * kPi * kOscillationFrequencyX,
		2.0 * kPi * kOscillationFrequencyY,
		2.0 * kPi * kOscillationFrequencyZ);
	const Vector3d phase = omega * time;

	PositionReference reference;
	reference.position =
		initial_position +
		Vector3d(amplitude.x() * sin(phase.x()),
				 amplitude.y() * (1.0 - cos(phase.y())),
				 amplitude.z() * sin(phase.z()));
	reference.linear_velocity =
		Vector3d(amplitude.x() * omega.x() * cos(phase.x()),
				 amplitude.y() * omega.y() * sin(phase.y()),
				 amplitude.z() * omega.z() * cos(phase.z()));
	reference.linear_acceleration =
		Vector3d(-amplitude.x() * omega.x() * omega.x() * sin(phase.x()),
				 amplitude.y() * omega.y() * omega.y() * cos(phase.y()),
				 -amplitude.z() * omega.z() * omega.z() * sin(phase.z()));

	return reference;
}

double normalizedTrackingVelocityLimit(const double velocity_limit) {
	return clamp(
		(velocity_limit - kTrackingMinLinearVelocity) /
			(kTrackingMaxLinearVelocity - kTrackingMinLinearVelocity),
		0.0, 1.0);
}

Vector3d velocityPlotOrigin() {
	return Vector3d(-0.55, -0.45, 0.32);
}

Vector3d hiddenPlotPosition() {
	return Vector3d(0.0, 0.0, -10.0);
}

Vector3d velocityPlotPosition(
	const double sample_age, const double velocity_norm) {
	constexpr double kPlotWidth = 0.90;
	constexpr double kPlotHeight = 0.45;
	const double horizontal_fraction =
		1.0 - clamp(sample_age / kVelocityPlotHistoryDuration, 0.0, 1.0);
	const double vertical_fraction =
		clamp(velocity_norm / kVelocityPlotMaxVelocity, 0.0, 1.0);
	const Vector3d origin = velocityPlotOrigin();
	return Vector3d(
		origin.x(), origin.y() + kPlotWidth * horizontal_fraction,
		origin.z() + kPlotHeight * vertical_fraction);
}

string velocityPlotObjectName(const string& prefix, const size_t index) {
	ostringstream name;
	name << prefix << setw(2) << setfill('0') << index;
	return name.str();
}

void publishVelocityHistory(
	const double time, const double current_velocity_norm,
	const double limited_velocity_norm) {
	if (time - last_velocity_history_sample_time <
		kVelocityPlotSamplePeriod) {
		return;
	}
	last_velocity_history_sample_time = time;

	lock_guard<mutex> lock(mutex_velocity_history);
	displayed_velocity_history.push_back(
		{time, current_velocity_norm, limited_velocity_norm});
	while (!displayed_velocity_history.empty() &&
		   time - displayed_velocity_history.front().time >
			   kVelocityPlotHistoryDuration) {
		displayed_velocity_history.pop_front();
	}
	while (displayed_velocity_history.size() > kVelocityPlotPointCount) {
		displayed_velocity_history.pop_front();
	}
}

void addDisplayLabels(const shared_ptr<SaiGraphics::SaiGraphics>& graphics) {
	graphics->addOverlayLabel(
		"TrackingState", "Interpolation: initializing", "camera", 20, 40,
		1.0);
	graphics->addOverlayLabel(
		"VelocityLimitState", "Tracking max linear velocity: initializing",
		"camera", 20, 64, 1.0);
	graphics->addOverlayLabel(
		"VelocityState", "Velocity: initializing", "camera", 20, 88, 1.0);
	graphics->addOverlayLabel(
		"VelocityPlotState",
		"Plot: current speed (orange), active velocity limit (cyan), last 5 s, 0-0.30 m/s",
		"camera", 20, 112, 0.82);
}

void publishDisplayState(
	const SaiPrimitives::MotionForceTask& motion_force_task,
	const PositionReference& reference, const double velocity_limit,
	const double limited_velocity_norm) {
	{
		lock_guard<mutex> lock(mutex_displayed_points);
		displayed_goal_position = reference.position;
		displayed_desired_position = motion_force_task.getDesiredPosition();
	}

	DisplayState state;
	state.tracking_linear_velocity_limit = velocity_limit;
	state.goal_linear_velocity_norm = reference.linear_velocity.norm();
	state.limited_linear_velocity_norm = limited_velocity_norm;
	state.desired_linear_velocity_norm =
		motion_force_task.getDesiredLinearVelocity().norm();
	state.current_linear_velocity_norm =
		motion_force_task.getCurrentLinearVelocity().norm();
	state.position_error = motion_force_task.getPositionError().norm();
	state.tracking_mode_enabled =
		motion_force_task.getInternalOtgTrackingModeEnabled();

	lock_guard<mutex> lock(mutex_displayed_state);
	displayed_state = state;
}

void updateDisplayLabels(
	const shared_ptr<SaiGraphics::SaiGraphics>& graphics) {
	DisplayState state;
	{
		lock_guard<mutex> lock(mutex_displayed_state);
		state = displayed_state;
	}

	graphics->updateOverlayLabel(
		"TrackingState",
		state.tracking_mode_enabled
			? "Interpolation: Ruckig Trackig tracking interpolation"
			: "Interpolation: regular Ruckig OTG interpolation",
		state.tracking_mode_enabled ? 1.0 : 0.0,
		state.tracking_mode_enabled ? 0.55 : 0.65,
		state.tracking_mode_enabled ? 0.0 : 1.0);

	ostringstream limit_label;
	limit_label << fixed << setprecision(3)
				<< "Tracking max linear velocity: "
				<< state.tracking_linear_velocity_limit << " m/s";
	const double limit_blend =
		normalizedTrackingVelocityLimit(state.tracking_linear_velocity_limit);
	graphics->updateOverlayLabel(
		"VelocityLimitState", limit_label.str(),
		0.1 + 0.9 * limit_blend, 0.85 - 0.3 * limit_blend,
		0.95 - 0.95 * limit_blend);

	ostringstream velocity_label;
	velocity_label << fixed << setprecision(3)
				   << "Goal speed: " << state.goal_linear_velocity_norm
				   << " m/s, active speed limit: "
				   << state.limited_linear_velocity_norm
				   << " m/s, desired speed: "
				   << state.desired_linear_velocity_norm
				   << " m/s, current speed: "
				   << state.current_linear_velocity_norm
				   << " m/s, position error: " << state.position_error
				   << " m";
	graphics->updateOverlayLabel(
		"VelocityState", velocity_label.str(), 0.86, 0.86, 0.86);
	graphics->updateOverlayLabel(
		"VelocityPlotState",
		"Plot: current speed (orange), active velocity limit (cyan), last 5 s, 0-0.30 m/s",
		0.86, 0.86, 0.86);
}

void updateVelocityHistoryPlot(
	const shared_ptr<SaiGraphics::SaiGraphics>& graphics) {
	deque<VelocityHistoryPoint> history;
	{
		lock_guard<mutex> lock(mutex_velocity_history);
		history = displayed_velocity_history;
	}

	if (history.empty()) {
		for (size_t i = 0; i < kVelocityPlotPointCount; ++i) {
			const Affine3d hidden_pose = markerPose(hiddenPlotPosition());
			graphics->updateObjectGraphics(
				velocityPlotObjectName("CurrentVelocityPlot", i),
				hidden_pose);
			graphics->updateObjectGraphics(
				velocityPlotObjectName("LimitedVelocityPlot", i),
				hidden_pose);
		}
		return;
	}

	const double latest_time = history.back().time;
	const size_t sample_count =
		min(history.size(), kVelocityPlotPointCount);
	const size_t first_sample = history.size() - sample_count;

	for (size_t i = 0; i < kVelocityPlotPointCount; ++i) {
		if (i >= sample_count) {
			const Affine3d hidden_pose = markerPose(hiddenPlotPosition());
			graphics->updateObjectGraphics(
				velocityPlotObjectName("CurrentVelocityPlot", i),
				hidden_pose);
			graphics->updateObjectGraphics(
				velocityPlotObjectName("LimitedVelocityPlot", i),
				hidden_pose);
			continue;
		}

		const VelocityHistoryPoint& sample = history[first_sample + i];
		const double sample_age = latest_time - sample.time;
		graphics->updateObjectGraphics(
			velocityPlotObjectName("CurrentVelocityPlot", i),
			markerPose(velocityPlotPosition(
				sample_age, sample.current_velocity_norm)));
		graphics->updateObjectGraphics(
			velocityPlotObjectName("LimitedVelocityPlot", i),
			markerPose(velocityPlotPosition(
				sample_age, sample.limited_velocity_norm)));
	}
}

}  // namespace

void control(shared_ptr<SaiModel::SaiModel> robot,
			 shared_ptr<SaiSimulation::SaiSimulation> sim);
void simulation(shared_ptr<SaiModel::SaiModel> robot,
				shared_ptr<SaiSimulation::SaiSimulation> sim);

int main(int argc, char** argv) {
	SaiModel::URDF_FOLDERS["EXAMPLE_24_FOLDER"] =
		string(EXAMPLES_FOLDER) + "/24-panda_time_varying_tracking_velocity";
	cout << "Loading URDF world model file: "
		 << SaiModel::ReplaceUrdfPathPrefix(world_file) << endl;

	signal(SIGABRT, &sighandler);
	signal(SIGTERM, &sighandler);
	signal(SIGINT, &sighandler);

	auto graphics = make_shared<SaiGraphics::SaiGraphics>(world_file);
	graphics->addUIForceInteraction(robot_name);
	graphics->showLinkFrame(true, robot_name, link_name, 0.18);
	addDisplayLabels(graphics);

	auto sim = make_shared<SaiSimulation::SaiSimulation>(world_file);

	auto robot = make_shared<SaiModel::SaiModel>(robot_file, false);
	robot->setTRobotBase(sim->getRobotBaseTransform(robot_name));
	robot->setQ(sim->getJointPositions(robot_name));
	robot->updateModel();

	const Vector3d initial_position =
		robot->positionInWorld(link_name, controlPointInLink());
	graphics->updateObjectGraphics("GoalCenter", markerPose(initial_position));
	graphics->updateObjectGraphics("ActiveGoal", markerPose(initial_position));
	graphics->updateObjectGraphics("DesiredPoint", markerPose(initial_position));
	displayed_goal_position = initial_position;
	displayed_desired_position = initial_position;

	ui_torques = VectorXd::Zero(robot->dof());
	control_torques = VectorXd::Zero(robot->dof());

	fSimulationRunning = true;
	thread sim_thread(simulation, robot, sim);
	thread ctrl_thread(control, robot, sim);

	while (graphics->isWindowOpen()) {
		graphics->updateRobotGraphics(robot_name, robot->q());
		graphics->updateObjectGraphics("GoalCenter", markerPose(initial_position));
		{
			lock_guard<mutex> lock(mutex_displayed_points);
			graphics->updateObjectGraphics(
				"ActiveGoal", markerPose(displayed_goal_position));
			graphics->updateObjectGraphics(
				"DesiredPoint", markerPose(displayed_desired_position));
		}
		updateDisplayLabels(graphics);
		updateVelocityHistoryPlot(graphics);
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
		kRegularOtgMaxLinearVelocity, kRegularOtgMaxLinearAcceleration,
		kRegularOtgMaxLinearJerk, kRegularOtgMaxAngularVelocity,
		kRegularOtgMaxAngularAcceleration, kRegularOtgMaxAngularJerk);
	motion_force_task->enableInternalOtgTrackingMode(1.0, 1, 16);

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
	double max_observed_desired_velocity = 0.0;

	while (fSimulationRunning) {
		timer.waitForNextLoop();
		const double time = timer.elapsedSimTime();

		robot->setQ(sim->getJointPositions(robot_name));
		robot->setDq(sim->getJointVelocities(robot_name));
		robot->updateModel();

		robot_controller->updateControllerTaskModels();

		const PositionReference reference =
			desiredPositionTrajectory(time, initial_position);
		const double velocity_limit = trackingLinearVelocityLimit(time);
		const double limited_velocity_norm = velocity_limit;
		motion_force_task->setInternalOtgTrackingTargetLimits(
			velocity_limit, kTrackingLinearAcceleration,
			kTrackingAngularVelocity, kTrackingAngularAcceleration);
		motion_force_task->enableVelocitySaturation(
			velocity_limit, kTrackingAngularVelocity);
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
		const double current_velocity_norm =
			motion_force_task->getCurrentLinearVelocity().norm();
		publishDisplayState(
			*motion_force_task, reference, velocity_limit,
			limited_velocity_norm);
		publishVelocityHistory(
			time, current_velocity_norm, limited_velocity_norm);

		const double desired_velocity_norm =
			motion_force_task->getDesiredLinearVelocity().norm();
		max_observed_desired_velocity =
			max(max_observed_desired_velocity, desired_velocity_norm);

		if (timer.elapsedCycles() % 1000 == 0) {
			cout << "time: " << time << endl;
			cout << "tracking target velocity limits enabled: "
				 << motion_force_task
						->getInternalOtgTrackingTargetVelocityLimitsEnabled()
				 << endl;
			cout << "tracking max linear velocity: " << velocity_limit
				 << endl;
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
			cout << "active velocity limit: "
				 << limited_velocity_norm << endl;
			cout << "desired velocity norm: " << desired_velocity_norm
				 << endl;
			cout << "current velocity norm: " << current_velocity_norm
				 << endl;
			cout << "max observed desired velocity norm: "
				 << max_observed_desired_velocity << endl;
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
