#include <signal.h>

#include <iostream>
#include <string>

#include "Sai2Graphics.h"
#include "Sai2Primitives.h"
#include "Sai2Simulation.h"
#include "redis/RedisClient.h"
#include "redis_keys.h"
#include "timer/LoopTimer.h"
bool fSimulationRunning = false;
void sighandler(int) { fSimulationRunning = false; }

using namespace std;
using namespace Eigen;

namespace {
const string world_file = "./resources/world.urdf";
const string robot_file = "./resources/panda_arm.urdf";
const string robot_name = "PANDA";
const string link_name = "end-effector";

// mutex for control torques
mutex mtx;

// haptic plane and line guidance
const Vector3d plane_normal = Vector3d(0, 0, 1);
const Vector3d plane_origin = Vector3d(0, 0, 0.15);

const Vector3d line_first_point = Vector3d(0, 0, 0.15);
const Vector3d line_second_point = Vector3d(0, 0, 0.25);

bool plane_guidance = false;
bool line_guidance = false;

bool p_was_pressed = false;
bool l_was_pressed = false;

}  // namespace

// Create simulation and control function
void runSim(shared_ptr<Sai2Simulation::Sai2Simulation> sim);
void runControl(shared_ptr<Sai2Simulation::Sai2Simulation> sim);

//// Robot global variables /////
// sensed task force from robot interaction
Vector3d sensed_force = Vector3d::Zero();
Vector3d sensed_moment = Vector3d::Zero();
// robot joint data
VectorXd robot_control_torques = Eigen::VectorXd::Zero(7);

int main() {
	// set up signal handler
	signal(SIGABRT, &sighandler);
	signal(SIGTERM, &sighandler);
	signal(SIGINT, &sighandler);

	// load simulation world
	auto sim = make_shared<Sai2Simulation::Sai2Simulation>(world_file);
	sim->addSimulatedForceSensor(robot_name, link_name, Affine3d::Identity(),
								 5.0);
	sim->setCoeffFrictionStatic(0.1);

	// load graphics scene
	auto graphics = make_shared<Sai2Graphics::Sai2Graphics>(world_file);
	graphics->addForceSensorDisplay(sim->getAllForceSensorData()[0]);

	// Run simulation and control threads
	thread sim_thread(runSim, sim);
	thread control_thread(runControl, sim);

	// graphics timer
	Sai2Common::LoopTimer graphicsTimer(30.0, 1e6);

	while (graphics->isWindowOpen()) {
		graphicsTimer.waitForNextLoop();

		if (graphics->isKeyPressed(GLFW_KEY_P) && !p_was_pressed) {
			plane_guidance = !plane_guidance;
			p_was_pressed = true;
		} else if (!graphics->isKeyPressed(GLFW_KEY_P)) {
			p_was_pressed = false;
		}

		if (graphics->isKeyPressed(GLFW_KEY_L) && !l_was_pressed) {
			line_guidance = !line_guidance;
			l_was_pressed = true;
		} else if (!graphics->isKeyPressed(GLFW_KEY_L)) {
			l_was_pressed = false;
		}

		graphics->updateRobotGraphics(robot_name,
									  sim->getJointPositions(robot_name));
		graphics->updateDisplayedForceSensor(sim->getAllForceSensorData()[0]);
		graphics->renderGraphicsWorld();
	}

	// stop simulation and control threads
	fSimulationRunning = false;
	sim_thread.join();
	control_thread.join();

	return 0;
}

//------------------------------------------------------------------------------
////// Simulation thread //////
//------------------------------------------------------------------------------
void runSim(shared_ptr<Sai2Simulation::Sai2Simulation> sim) {
	// create a timer
	Sai2Common::LoopTimer simTimer(1.0 / sim->timestep(), 1e6);

	fSimulationRunning = true;

	while (fSimulationRunning) {
		simTimer.waitForNextLoop();
		{
			lock_guard<mutex> lock(mtx);
			sim->setJointTorques(robot_name, robot_control_torques);
		}
		sim->integrate();
	}

	cout << "simulation timer stats:" << endl;
	simTimer.printInfoPostRun();
}

//------------------------------------------------------------------------------
////// Control thread //////
//------------------------------------------------------------------------------
void runControl(shared_ptr<Sai2Simulation::Sai2Simulation> sim) {
	// redis client
	auto redis_client = Sai2Common::RedisClient();
	redis_client.connect();

	// load robot
	const Affine3d T_world_robot = sim->getRobotBaseTransform(robot_name);
	auto robot = make_shared<Sai2Model::Sai2Model>(robot_file);
	robot->setTRobotBase(T_world_robot);
	robot->setQ(sim->getJointPositions(robot_name));
	robot->setDq(sim->getJointVelocities(robot_name));
	robot->updateModel();

	// create robot controller
	Affine3d compliant_frame = Affine3d::Identity();
	auto motion_force_task = make_shared<Sai2Primitives::MotionForceTask>(
		robot, link_name, compliant_frame);
	motion_force_task->disableInternalOtg();
	// motion_force_task->enableInternalOtgAccelerationLimited(0.8, 40.0, M_PI,
	// 50*M_PI);

	vector<shared_ptr<Sai2Primitives::TemplateTask>> task_list = {
		motion_force_task};
	auto robot_controller =
		make_unique<Sai2Primitives::RobotController>(robot, task_list);

	// create haptic controller
	Sai2Primitives::HapticDeviceController::DeviceLimits device_limits(
		redis_client.getEigen(MAX_STIFFNESS_KEY),
		redis_client.getEigen(MAX_DAMPING_KEY),
		redis_client.getEigen(MAX_FORCE_KEY));
	Affine3d device_home_pose = Affine3d(Translation3d(0, 0, 0.15));
	auto haptic_controller =
		make_shared<Sai2Primitives::HapticDeviceController>(
			device_limits, robot->transformInWorld(link_name),
			device_home_pose);
	haptic_controller->setScalingFactors(2.5);
	haptic_controller->setHapticControlType(
		Sai2Primitives::HapticControlType::HOMING);
	haptic_controller->setEnableOrientationTeleoperation(false);
	haptic_controller->enableHapticWorkspaceVirtualLimits(0.05, M_PI / 4);
	haptic_controller->setHapticFeedbackFromProxy(false);
	// haptic_controller->setSendHapticFeedback(false);

	Sai2Primitives::HapticControllerInput haptic_input;
	Sai2Primitives::HapticControllerOtuput haptic_output;
	bool haptic_button_was_pressed = false;
	int haptic_button_is_pressed = 0;
	redis_client.setInt(SWITCH_PRESSED_KEY, haptic_button_is_pressed);
	redis_client.setInt(USE_GRIPPER_AS_SWITCH_KEY, 1);

	// setup redis communication
	redis_client.addToSendGroup(COMMANDED_FORCE_KEY,
								haptic_output.device_command_force);
	redis_client.addToSendGroup(COMMANDED_TORQUE_KEY,
								haptic_output.device_command_moment);

	redis_client.addToReceiveGroup(POSITION_KEY, haptic_input.device_position);
	redis_client.addToReceiveGroup(ROTATION_KEY,
								   haptic_input.device_orientation);
	redis_client.addToReceiveGroup(LINEAR_VELOCITY_KEY,
								   haptic_input.device_linear_velocity);
	redis_client.addToReceiveGroup(ANGULAR_VELOCITY_KEY,
								   haptic_input.device_angular_velocity);
	redis_client.addToReceiveGroup(SWITCH_PRESSED_KEY,
								   haptic_button_is_pressed);

	// create a timer
	Sai2Common::LoopTimer controlTimer(1000.0, 1e6);

	while (fSimulationRunning) {
		// wait for next scheduled loop
		controlTimer.waitForNextLoop();

		// read robot data from simulation thread
		robot->setQ(sim->getJointPositions(robot_name));
		robot->setDq(sim->getJointVelocities(robot_name));
		robot->updateModel();

		robot_controller->updateControllerTaskModels();

		// read haptic device state from redis
		redis_client.receiveAllFromGroup();

		// compute haptic control
		haptic_input.robot_position = robot->positionInWorld(link_name);
		haptic_input.robot_orientation = robot->rotationInWorld(link_name);
		haptic_input.robot_linear_velocity =
			robot->linearVelocityInWorld(link_name);
		haptic_input.robot_angular_velocity =
			robot->angularVelocityInWorld(link_name);
		haptic_input.robot_sensed_force = -motion_force_task->getSensedForce();
		haptic_input.robot_sensed_moment =
			-motion_force_task->getSensedMoment();

		haptic_output = haptic_controller->computeHapticControl(haptic_input);

		redis_client.sendAllFromGroup();

		// compute robot control
		motion_force_task->updateSensedForceAndMoment(
			sim->getSensedForce(robot_name, link_name),
			sim->getSensedMoment(robot_name, link_name));
		motion_force_task->setDesiredPosition(
			haptic_output.robot_goal_position);
		motion_force_task->setDesiredOrientation(
			haptic_output.robot_goal_orientation);

		{
			lock_guard<mutex> lock(mtx);
			robot_control_torques = robot_controller->computeControlTorques();
		}

		if (haptic_controller->getHapticControlType() ==
				Sai2Primitives::HapticControlType::HOMING &&
			haptic_controller->getHomed()) {
			haptic_controller->setHapticControlType(
				Sai2Primitives::HapticControlType::MOTION_MOTION);
		} else {
			if (haptic_button_is_pressed && !haptic_button_was_pressed) {
				haptic_controller->setHapticControlType(
					Sai2Primitives::HapticControlType::DETACHED);
			} else if (!haptic_button_is_pressed && haptic_button_was_pressed) {
				haptic_controller->resetRobotOffset();
				haptic_controller->setHapticControlType(
					Sai2Primitives::HapticControlType::MOTION_MOTION);
			}
		}
		haptic_button_was_pressed = haptic_button_is_pressed;

		if (plane_guidance) {
			haptic_controller->enablePlaneGuidance(plane_origin, plane_normal);
		} else {
			haptic_controller->disablePlaneGuidance();
		}

		if (line_guidance) {
			haptic_controller->enableLineGuidance(line_first_point,
												  line_second_point);
		} else {
			haptic_controller->disableLineGuidance();
		}
	}

	redis_client.setEigen(COMMANDED_FORCE_KEY, Vector3d::Zero());
	redis_client.setEigen(COMMANDED_TORQUE_KEY, Vector3d::Zero());

	cout << "control timer stats:" << endl;
	controlTimer.printInfoPostRun();
}