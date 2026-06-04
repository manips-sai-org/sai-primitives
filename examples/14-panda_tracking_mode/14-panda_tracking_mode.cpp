/*
 * Example of a Panda controller that tracks a moving position and orientation
 * reference with the MotionForceTask internal Ruckig tracking mode.
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

const string world_file = "${EXAMPLE_14_FOLDER}/world.urdf";
const string robot_file =
	"${SAI_MODEL_URDF_FOLDER}/panda/panda_arm_sphere.urdf";
const string robot_name = "PANDA";
const string link_name = "end-effector";

VectorXd ui_torques;
VectorXd control_torques;
mutex mutex_torques;
mutex mutex_interpolation_state_label;

struct InterpolationStateLabel {
	string text = "initializing";
	bool internal_otg_enabled = false;
	bool tracking_mode_enabled = false;
};

InterpolationStateLabel displayed_interpolation_state_label;
const string interpolation_state_label_name = "interpolation_state";

struct PoseReference {
	Vector3d position;
	Vector3d linear_velocity;
	Vector3d linear_acceleration;
	Matrix3d orientation;
	Vector3d angular_velocity;
	Vector3d angular_acceleration;
};

PoseReference desiredPoseTrajectory(const double time,
									const Vector3d& initial_position,
									const Matrix3d& initial_orientation) {
	PoseReference reference;
	reference.position = initial_position;
	reference.linear_velocity.setZero();
	reference.linear_acceleration.setZero();

	const Vector2d xy_amplitude(0.08, 0.08);
	const double xy_omega = 2.0 * kPi * 0.12;
	const double xy_phase = xy_omega * time;

	reference.position.x() += xy_amplitude.x() * sin(xy_phase);
	reference.position.y() += xy_amplitude.y() * (1.0 - cos(xy_phase));
	reference.linear_velocity.x() =
		xy_amplitude.x() * xy_omega * cos(xy_phase);
	reference.linear_velocity.y() =
		xy_amplitude.y() * xy_omega * sin(xy_phase);
	reference.linear_acceleration.x() =
		-xy_amplitude.x() * xy_omega * xy_omega * sin(xy_phase);
	reference.linear_acceleration.y() =
		xy_amplitude.y() * xy_omega * xy_omega * cos(xy_phase);

	const double cone_half_angle = 15.0 * kPi / 180.0;
	const double cone_omega = 2.0 * kPi * 0.10;
	const double cone_phase = cone_omega * time;
	const double cone_sin = sin(cone_half_angle);
	const double cone_cos = cos(cone_half_angle);
	const Matrix3d cone_heading =
		AngleAxisd(cone_phase, Vector3d::UnitZ()).toRotationMatrix();
	const Matrix3d cone_tilt =
		AngleAxisd(cone_half_angle, Vector3d::UnitY()).toRotationMatrix();
	const Matrix3d cone_orientation =
		cone_heading * cone_tilt * cone_heading.transpose() *
		cone_tilt.transpose();

	reference.orientation = cone_orientation * initial_orientation;
	reference.angular_velocity =
		cone_omega *
		Vector3d(-cone_sin * cos(cone_phase),
				 -cone_sin * sin(cone_phase),
				 1.0 - cone_cos);
	reference.angular_acceleration =
		cone_omega * cone_omega *
		Vector3d(cone_sin * sin(cone_phase),
				 -cone_sin * cos(cone_phase),
				 0.0);

	return reference;
}

string interpolationStateLabel(
	const SaiPrimitives::MotionForceTask& motion_force_task) {
	if (!motion_force_task.getInternalOtgEnabled()) {
		return "direct task goal, internal OTG disabled";
	}
	if (motion_force_task.getInternalOtgTrackingModeEnabled()) {
		return "Ruckig Trackig tracking interpolation";
	}
	return "regular Ruckig OTG interpolation";
}

InterpolationStateLabel interpolationStateLabelDisplay(
	const SaiPrimitives::MotionForceTask& motion_force_task) {
	return {
		interpolationStateLabel(motion_force_task),
		motion_force_task.getInternalOtgEnabled(),
		motion_force_task.getInternalOtgTrackingModeEnabled()};
}

void publishInterpolationStateLabel(
	const SaiPrimitives::MotionForceTask& motion_force_task) {
	lock_guard<mutex> lock(mutex_interpolation_state_label);
	displayed_interpolation_state_label =
		interpolationStateLabelDisplay(motion_force_task);
}

void addInterpolationStateLabel(
	const shared_ptr<SaiGraphics::SaiGraphics>& graphics) {
	graphics->addOverlayLabel(
		interpolation_state_label_name,
		"Interpolation: initializing", "camera", 20, 40, 1.0);
}

void updateInterpolationStateLabel(
	const shared_ptr<SaiGraphics::SaiGraphics>& graphics) {

	InterpolationStateLabel label_state;
	{
		lock_guard<mutex> lock(mutex_interpolation_state_label);
		label_state = displayed_interpolation_state_label;
	}

	if (!label_state.internal_otg_enabled) {
		graphics->updateOverlayLabel(
			interpolation_state_label_name,
			"Interpolation: " + label_state.text, 0.86, 0.86, 0.86);
	} else if (label_state.tracking_mode_enabled) {
		graphics->updateOverlayLabel(
			interpolation_state_label_name,
			"Interpolation: " + label_state.text, 1.0, 0.55, 0.0);
	} else {
		graphics->updateOverlayLabel(
			interpolation_state_label_name,
			"Interpolation: " + label_state.text, 0.0, 0.65, 1.0);
	}
}

}  // namespace

void control(shared_ptr<SaiModel::SaiModel> robot,
			 shared_ptr<SaiSimulation::SaiSimulation> sim);
void simulation(shared_ptr<SaiModel::SaiModel> robot,
				shared_ptr<SaiSimulation::SaiSimulation> sim);

int main(int argc, char** argv) {
	SaiModel::URDF_FOLDERS["EXAMPLE_14_FOLDER"] =
		string(EXAMPLES_FOLDER) + "/14-panda_tracking_mode";
	cout << "Loading URDF world model file: "
		 << SaiModel::ReplaceUrdfPathPrefix(world_file) << endl;

	signal(SIGABRT, &sighandler);
	signal(SIGTERM, &sighandler);
	signal(SIGINT, &sighandler);

	auto graphics = make_shared<SaiGraphics::SaiGraphics>(world_file);
	graphics->addUIForceInteraction(robot_name);
	graphics->showLinkFrame(true, robot_name, link_name, 0.18);
	addInterpolationStateLabel(graphics);

	auto sim = make_shared<SaiSimulation::SaiSimulation>(world_file);

	auto robot = make_shared<SaiModel::SaiModel>(robot_file, false);
	robot->setTRobotBase(sim->getRobotBaseTransform(robot_name));
	robot->setQ(sim->getJointPositions(robot_name));
	robot->updateModel();

	ui_torques = VectorXd::Zero(robot->dof());
	control_torques = VectorXd::Zero(robot->dof());

	fSimulationRunning = true;
	thread sim_thread(simulation, robot, sim);
	thread ctrl_thread(control, robot, sim);

	while (graphics->isWindowOpen()) {
		graphics->updateRobotGraphics(robot_name, robot->q());
		updateInterpolationStateLabel(graphics);
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
	const int dof = robot->dof();

	const Vector3d pos_in_link(0.0, 0.0, 0.07);
	const Affine3d compliant_frame = Affine3d(Translation3d(pos_in_link));

	auto motion_force_task = make_shared<SaiPrimitives::MotionForceTask>(
		robot, link_name, compliant_frame);
	motion_force_task->setPosControlGains(100.0, 20.0);
	motion_force_task->setOriControlGains(100.0, 20.0);
	motion_force_task->enableInternalOtgJerkLimited(
		0.8, 3.0, 20.0, 1.5, 6.0, 30.0);
	motion_force_task->enableInternalOtgTrackingMode(1, 1, 16);
	motion_force_task->setInternalOtgTrackingTargetLimits(0.8, 3.0, 1.5, 6.0);

	const Matrix3d initial_orientation =
		robot->rotationInWorld(link_name);
	const Vector3d initial_position =
		robot->positionInWorld(link_name, pos_in_link);

	auto joint_task = make_shared<SaiPrimitives::JointTask>(robot);
	joint_task->setGains(100.0, 20.0);
	joint_task->setGoalPosition(robot->q());

	vector<shared_ptr<SaiPrimitives::TemplateTask>> task_list = {
		motion_force_task, joint_task};
	auto robot_controller =
		make_unique<SaiPrimitives::RobotController>(robot, task_list);

	const double control_freq = 1000.0;
	SaiCommon::LoopTimer timer(control_freq, 1e6);

	while (fSimulationRunning) {
		timer.waitForNextLoop();
		const double time = timer.elapsedSimTime();

		robot->setQ(sim->getJointPositions(robot_name));
		robot->setDq(sim->getJointVelocities(robot_name));
		robot->updateModel();

		robot_controller->updateControllerTaskModels();

		const PoseReference reference =
			desiredPoseTrajectory(time, initial_position, initial_orientation);
		motion_force_task->setGoalPosition(reference.position);
		motion_force_task->setGoalLinearVelocity(reference.linear_velocity);
		motion_force_task->setGoalLinearAcceleration(
			reference.linear_acceleration);
		motion_force_task->setGoalOrientation(reference.orientation);
		motion_force_task->setGoalAngularVelocity(
			reference.angular_velocity);
		motion_force_task->setGoalAngularAcceleration(
			reference.angular_acceleration);

		{
			lock_guard<mutex> lock(mutex_torques);
			control_torques = robot_controller->computeControlTorques();
		}
		publishInterpolationStateLabel(*motion_force_task);

		if (timer.elapsedCycles() % 1000 == 0) {
			cout << "time: " << time << endl;
			cout << "interpolation state: "
				 << interpolationStateLabel(*motion_force_task) << endl;
			cout << "tracking mode: "
				 << motion_force_task->getInternalOtgTrackingModeEnabled()
				 << endl;
			cout << "goal position: "
				 << motion_force_task->getGoalPosition().transpose()
				 << endl;
			cout << "desired position: "
				 << motion_force_task->getDesiredPosition().transpose()
				 << endl;
			cout << "current position: "
				 << motion_force_task->getCurrentPosition().transpose()
				 << endl;
			cout << "position tracking error: "
				 << (motion_force_task->getGoalPosition() -
					 motion_force_task->getCurrentPosition()).norm()
				 << endl;
			cout << "orientation error: "
				 << motion_force_task->getOrientationError().norm() << endl;
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
