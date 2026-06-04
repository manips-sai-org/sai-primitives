/**
 * Example of latency-aware predictive haptic contact rendering from a point
 * cloud.
 *
 * The impedance-type haptic controller handles homing, clutching, and
 * motion-motion teleoperation of the Panda end effector. A latency-aware
 * renderer predicts the robot tool's swept path over the haptic latency window
 * and adds a predictive contact force to the haptic device command. Delayed
 * robot contact from the simulated force sensor locally updates the learned
 * point-cloud bias.
 */

#include <signal.h>

#include <algorithm>
#include <array>
#include <cmath>
#include <iomanip>
#include <iostream>
#include <map>
#include <memory>
#include <mutex>
#include <sstream>
#include <string>
#include <thread>
#include <vector>

#include "SaiGraphics.h"
#include "SaiPrimitives.h"
#include "SaiSimulation.h"
#include "redis/RedisClient.h"
#include "redis/keys/chai_haptic_devices_driver.h"
#include "timer/LoopTimer.h"

using namespace std;
using namespace Eigen;
using namespace SaiCommon::ChaiHapticDriverKeys;

bool fSimulationRunning = false;
void sighandler(int) { fSimulationRunning = false; }

namespace {

const string world_file = "${EXAMPLE_23_FOLDER}/world.urdf";
const string robot_file =
	"${SAI_MODEL_URDF_FOLDER}/panda/panda_arm_sphere.urdf";
const string robot_name = "PANDA";
const string link_name = "end-effector";
const string overlay_label_name = "RendererStatus";

constexpr double kControlFrequency = 1000.0;
constexpr double kGraphicsFrequency = 30.0;
constexpr double kLatency = 0.150;
constexpr int kRendererPathSamples = 18;
constexpr size_t kSweepVisualSamples = 8;
constexpr size_t kOffsetPointVisualCount = 25;
constexpr int kPlaneGridCount = 5;
constexpr int kBoxTopGridCount = 5;
constexpr int kBoxSideGridCount = 3;

constexpr double kSurfaceCenterX = 0.40;
constexpr double kSurfaceCenterY = 0.00;
constexpr double kSurfaceZ = 0.240;
constexpr double kPointCloudSpacing = 0.060;
constexpr double kBoxCenterX = 0.33;
constexpr double kBoxCenterY = 0.17;
constexpr double kBoxCenterZ = 0.27;
constexpr double kBoxSizeX = 0.20;
constexpr double kBoxSizeY = 0.25;
constexpr double kBoxSizeZ = 0.06;
constexpr double kUncertaintyMargin = 0.003;
constexpr double kLearningRate = 0.55;
constexpr double kLearningSigma = 0.045;
constexpr double kMinLearnedBias = -0.012;
constexpr double kMaxLearnedBias = 0.018;
constexpr double kRendererStiffness = 3500.0;
constexpr double kRendererDamping = 8.0;
constexpr double kRendererMaxForce = 15.0;
constexpr double kRendererForceRampDuration = 0.080;
constexpr double kRendererForceFilterTimeConstant = 0.025;
constexpr double kRendererForceSlewRateLimit = 250.0;
constexpr bool kUseTimeToContactScaling = false;
constexpr double kEndEffectorSphereRadius = 0.040;
constexpr double kRobotContactForceThreshold = 0.75;
constexpr double kRobotContactReleaseThreshold = 0.20;

const Vector3d kHiddenMarkerPosition(0.0, 0.0, -10.0);

mutex mutex_torques;
VectorXd robot_control_torques = VectorXd::Zero(7);

// map of flags for key presses
map<int, bool> key_pressed = {
	{GLFW_KEY_P, false},
	{GLFW_KEY_L, false},
	{GLFW_KEY_W, false},
	{GLFW_KEY_O, false},
};
map<int, bool> key_was_pressed = key_pressed;

template <size_t Count>
array<string, Count> makeObjectNames(const string& prefix) {
	array<string, Count> names;
	for (size_t i = 0; i < Count; ++i) {
		ostringstream name;
		name << prefix << setw(2) << setfill('0') << i;
		names[i] = name.str();
	}
	return names;
}

const array<string, kOffsetPointVisualCount> kOffsetPointNames =
	makeObjectNames<kOffsetPointVisualCount>("OffsetPoint");
const array<string, kSweepVisualSamples> kSweepPointNames =
	makeObjectNames<kSweepVisualSamples>("SweepPoint");

struct RendererVisualizationState {
	array<Vector3d, kOffsetPointVisualCount> offset_points;
	array<Vector3d, kSweepVisualSamples> sweep_points;
	Vector3d x_star = kHiddenMarkerPosition;
	double phi_tau = 0.0;
	double penetration = 0.0;
	double eta = 1.0;
	double learned_bias = 0.0;
	double force_norm = 0.0;
	bool predicted_contact = false;
	bool haptic_force_enabled = false;
	bool learned_from_robot_contact = false;
};

RendererVisualizationState renderer_visualization;
mutex mutex_renderer_visualization;

struct RendererPointCloud {
	vector<Vector3d> points;
	vector<Vector3d> normals;
	array<size_t, kOffsetPointVisualCount> visual_indices;
};

void appendSurfaceGrid(
	RendererPointCloud& point_cloud,
	const Vector3d& center,
	const Vector3d& u_axis,
	const Vector3d& v_axis,
	const double half_u,
	const double half_v,
	const int u_count,
	const int v_count,
	const Vector3d& normal) {

	for (int i = 0; i < u_count; ++i) {
		const double u_alpha =
			u_count == 1 ?
				0.0 :
				static_cast<double>(i) / static_cast<double>(u_count - 1);
		const double u = -half_u + 2.0 * half_u * u_alpha;
		for (int j = 0; j < v_count; ++j) {
			const double v_alpha =
				v_count == 1 ?
					0.0 :
					static_cast<double>(j) /
						static_cast<double>(v_count - 1);
			const double v = -half_v + 2.0 * half_v * v_alpha;
			point_cloud.points.emplace_back(center + u * u_axis + v * v_axis);
			point_cloud.normals.emplace_back(normal.normalized());
		}
	}
}

RendererPointCloud makeRendererPointCloud() {
	RendererPointCloud point_cloud;
	point_cloud.visual_indices.fill(0);

	const size_t plane_start = point_cloud.points.size();
	appendSurfaceGrid(
		point_cloud,
		Vector3d(kSurfaceCenterX, kSurfaceCenterY, kSurfaceZ),
		Vector3d::UnitX(),
		Vector3d::UnitY(),
		0.5 * kPointCloudSpacing * static_cast<double>(kPlaneGridCount - 1),
		0.5 * kPointCloudSpacing * static_cast<double>(kPlaneGridCount - 1),
		kPlaneGridCount,
		kPlaneGridCount,
		Vector3d::UnitZ());

	const double half_box_x = 0.5 * kBoxSizeX;
	const double half_box_y = 0.5 * kBoxSizeY;
	const double half_box_z = 0.5 * kBoxSizeZ;
	const Vector3d box_center(kBoxCenterX, kBoxCenterY, kBoxCenterZ);

	const size_t box_top_start = point_cloud.points.size();
	appendSurfaceGrid(
		point_cloud,
		box_center + half_box_z * Vector3d::UnitZ(),
		Vector3d::UnitX(),
		Vector3d::UnitY(),
		half_box_x,
		half_box_y,
		kBoxTopGridCount,
		kBoxTopGridCount,
		Vector3d::UnitZ());

	const size_t box_pos_x_start = point_cloud.points.size();
	appendSurfaceGrid(
		point_cloud,
		box_center + half_box_x * Vector3d::UnitX(),
		Vector3d::UnitY(),
		Vector3d::UnitZ(),
		half_box_y,
		half_box_z,
		kBoxSideGridCount,
		kBoxSideGridCount,
		Vector3d::UnitX());

	const size_t box_neg_x_start = point_cloud.points.size();
	appendSurfaceGrid(
		point_cloud,
		box_center - half_box_x * Vector3d::UnitX(),
		Vector3d::UnitY(),
		Vector3d::UnitZ(),
		half_box_y,
		half_box_z,
		kBoxSideGridCount,
		kBoxSideGridCount,
		-Vector3d::UnitX());

	const size_t box_pos_y_start = point_cloud.points.size();
	appendSurfaceGrid(
		point_cloud,
		box_center + half_box_y * Vector3d::UnitY(),
		Vector3d::UnitX(),
		Vector3d::UnitZ(),
		half_box_x,
		half_box_z,
		kBoxSideGridCount,
		kBoxSideGridCount,
		Vector3d::UnitY());

	const size_t box_neg_y_start = point_cloud.points.size();
	appendSurfaceGrid(
		point_cloud,
		box_center - half_box_y * Vector3d::UnitY(),
		Vector3d::UnitX(),
		Vector3d::UnitZ(),
		half_box_x,
		half_box_z,
		kBoxSideGridCount,
		kBoxSideGridCount,
		-Vector3d::UnitY());

	const array<size_t, 9> sparse_grid_indices = {
		0, 2, 4,
		10, 12, 14,
		20, 22, 24};
	size_t visual_index = 0;
	for (const size_t index : sparse_grid_indices) {
		point_cloud.visual_indices[visual_index++] = plane_start + index;
	}
	for (const size_t index : sparse_grid_indices) {
		point_cloud.visual_indices[visual_index++] = box_top_start + index;
	}
	point_cloud.visual_indices[visual_index++] = box_pos_x_start + 4;
	point_cloud.visual_indices[visual_index++] = box_neg_x_start + 4;
	point_cloud.visual_indices[visual_index++] = box_pos_y_start + 4;
	point_cloud.visual_indices[visual_index++] = box_neg_y_start + 4;
	point_cloud.visual_indices[visual_index++] = box_pos_x_start + 1;
	point_cloud.visual_indices[visual_index++] = box_neg_x_start + 7;
	point_cloud.visual_indices[visual_index++] = box_pos_y_start + 3;

	return point_cloud;
}

Affine3d markerPose(const Vector3d& position) {
	Affine3d pose = Affine3d::Identity();
	pose.translation() = position;
	return pose;
}

Vector3d normalizedOrFallback(
	const Vector3d& vector,
	const Vector3d& fallback) {
	if (vector.norm() > 1e-6) {
		return vector.normalized();
	}
	return fallback;
}

Vector3d sphereContactPointInLink(
	const Matrix3d& link_rotation_world,
	const Vector3d& surface_normal_world) {

	const Vector3d normal_world =
		normalizedOrFallback(surface_normal_world, Vector3d::UnitZ());
	return link_rotation_world.transpose() *
		   (-kEndEffectorSphereRadius * normal_world);
}

Vector3d clampVectorNorm(const Vector3d& vector, const double max_norm) {
	if (max_norm <= 0.0) {
		return Vector3d::Zero();
	}
	if (vector.norm() <= max_norm) {
		return vector;
	}
	return max_norm * vector.normalized();
}

Vector3d smoothRendererForce(
	const Vector3d& previous_force,
	const Vector3d& target_force,
	const double dt) {

	const double safe_dt = max(dt, 1e-6);
	const double alpha =
		safe_dt / (kRendererForceFilterTimeConstant + safe_dt);
	Vector3d filtered_force =
		previous_force + alpha * (target_force - previous_force);

	const double max_delta = kRendererForceSlewRateLimit * safe_dt;
	const Vector3d force_delta = filtered_force - previous_force;
	if (force_delta.norm() > max_delta) {
		filtered_force =
			previous_force + max_delta * force_delta.normalized();
	}

	return filtered_force;
}

RendererVisualizationState initialRendererVisualizationState() {
	RendererVisualizationState state;
	const RendererPointCloud point_cloud = makeRendererPointCloud();
	for (size_t i = 0; i < kOffsetPointVisualCount; ++i) {
		const size_t point_index = point_cloud.visual_indices[i];
		state.offset_points[i] =
			point_cloud.points[point_index] +
			kUncertaintyMargin * point_cloud.normals[point_index];
	}
	for (size_t i = 0; i < kSweepVisualSamples; ++i) {
		state.sweep_points[i] = kHiddenMarkerPosition;
	}
	return state;
}

void publishRendererVisualization(
	const SaiPrimitives::LatencyAwareContactRenderer& renderer,
	const array<size_t, kOffsetPointVisualCount>& visual_point_indices,
	const SaiPrimitives::LatencyAwareContactRenderer::Query& query,
	const SaiPrimitives::LatencyAwareContactRenderer::ForceResult& result,
	const bool haptic_force_enabled,
	const bool learned_from_robot_contact) {

	RendererVisualizationState state;
	const auto& points = renderer.getPoints();
	const auto& normals = renderer.getNormals();
	const auto& biases = renderer.getLearnedBiases();

	for (size_t i = 0; i < kOffsetPointVisualCount; ++i) {
		const size_t point_index = visual_point_indices[i];
		const Vector3d normal =
			normals[point_index].norm() > 1e-6 ?
				normals[point_index].normalized() :
				Vector3d::UnitZ();
		state.offset_points[i] =
			points[point_index] +
			(biases[point_index] + kUncertaintyMargin) * normal;
	}

	for (size_t i = 0; i < kSweepVisualSamples; ++i) {
		const double alpha =
			kSweepVisualSamples == 1 ?
				0.0 :
				static_cast<double>(i) /
					static_cast<double>(kSweepVisualSamples - 1);
		state.sweep_points[i] =
			query.robot_position + alpha * query.latency * query.robot_velocity;
	}

	state.x_star = result.x_star;
	state.phi_tau = result.phi_tau;
	state.penetration = result.penetration;
	state.eta = result.eta;
	state.learned_bias = renderer.evaluateLearnedBias(result.x_star);
	state.force_norm = result.force.norm();
	state.predicted_contact = result.in_contact;
	state.haptic_force_enabled = haptic_force_enabled;
	state.learned_from_robot_contact = learned_from_robot_contact;

	lock_guard<mutex> lock(mutex_renderer_visualization);
	renderer_visualization = state;
}

string overlayLabel(const RendererVisualizationState& state) {
	ostringstream label;
	label << fixed << setprecision(4)
		  << "Latency renderer | phi_tau " << state.phi_tau
		  << " m | bias " << state.learned_bias
		  << " m | Fh " << setprecision(2) << state.force_norm
		  << " N | eta " << setprecision(2) << state.eta
		  << " | predicted "
		  << (state.predicted_contact ? "contact" : "free")
		  << " | force "
		  << (state.haptic_force_enabled ? "on" : "off")
		  << " | update "
		  << (state.learned_from_robot_contact ? "yes" : "no");
	return label.str();
}

void updateRendererGraphics(
	const shared_ptr<SaiGraphics::SaiGraphics>& graphics) {

	RendererVisualizationState state;
	{
		lock_guard<mutex> lock(mutex_renderer_visualization);
		state = renderer_visualization;
	}

	for (size_t i = 0; i < kOffsetPointVisualCount; ++i) {
		graphics->updateObjectGraphics(
			kOffsetPointNames[i], markerPose(state.offset_points[i]));
	}
	for (size_t i = 0; i < kSweepVisualSamples; ++i) {
		graphics->updateObjectGraphics(
			kSweepPointNames[i], markerPose(state.sweep_points[i]));
	}
	graphics->updateObjectGraphics(
		"PredictedContact", markerPose(state.x_star));
	graphics->updateOverlayLabel(
		overlay_label_name,
		overlayLabel(state),
		state.predicted_contact ? 1.0 : 0.82,
		state.predicted_contact ? 0.48 : 0.86,
		state.predicted_contact ? 0.12 : 0.86);
}

bool shouldRenderHapticContact(
	const SaiPrimitives::HapticControlType& haptic_control_type) {
	return haptic_control_type ==
		   SaiPrimitives::HapticControlType::MOTION_MOTION;
}

}  // namespace

// Create simulation and control function
void runSim(shared_ptr<SaiSimulation::SaiSimulation> sim);
void runControl(shared_ptr<SaiSimulation::SaiSimulation> sim);

int main() {
	SaiModel::URDF_FOLDERS["EXAMPLE_23_FOLDER"] =
		string(EXAMPLES_FOLDER) + "/23-latency_aware_contact_renderer";
	cout << "Loading URDF world model file: "
		 << SaiModel::ReplaceUrdfPathPrefix(world_file) << endl;

	signal(SIGABRT, &sighandler);
	signal(SIGTERM, &sighandler);
	signal(SIGINT, &sighandler);

	renderer_visualization = initialRendererVisualizationState();

	auto sim = make_shared<SaiSimulation::SaiSimulation>(world_file);
	sim->addSimulatedForceSensor(robot_name, link_name, Affine3d::Identity(),
								 10.0);
	sim->setCoeffFrictionStatic(0.0);

	auto graphics = make_shared<SaiGraphics::SaiGraphics>(world_file);
	graphics->addForceSensorDisplay(sim->getAllForceSensorData()[0]);
	graphics->showLinkFrame(true, robot_name, link_name, 0.14);
	graphics->addOverlayLabel(
		overlay_label_name,
		"Latency renderer: initializing", "camera", 20, 40, 0.86);

	fSimulationRunning = true;
	thread sim_thread(runSim, sim);
	thread control_thread(runControl, sim);

	SaiCommon::LoopTimer graphicsTimer(kGraphicsFrequency, 1e6);

	while (graphics->isWindowOpen()) {
		graphicsTimer.waitForNextLoop();

		for (auto& key : key_pressed) {
			key_pressed[key.first] = graphics->isKeyPressed(key.first);
		}

		graphics->updateRobotGraphics(robot_name,
									  sim->getJointPositions(robot_name));
		graphics->updateDisplayedForceSensor(sim->getAllForceSensorData()[0]);
		updateRendererGraphics(graphics);
		graphics->renderGraphicsWorld();
	}

	fSimulationRunning = false;
	sim_thread.join();
	control_thread.join();

	return 0;
}

//------------------------------------------------------------------------------
////// Simulation thread //////
//------------------------------------------------------------------------------
void runSim(shared_ptr<SaiSimulation::SaiSimulation> sim) {
	SaiCommon::LoopTimer simTimer(1.0 / sim->timestep(), 1e6);

	while (fSimulationRunning) {
		simTimer.waitForNextLoop();
		{
			lock_guard<mutex> lock(mutex_torques);
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
void runControl(shared_ptr<SaiSimulation::SaiSimulation> sim) {
	auto redis_client = SaiCommon::RedisClient();
	redis_client.connect();

	const Affine3d T_world_robot = sim->getRobotBaseTransform(robot_name);
	auto robot = make_shared<SaiModel::SaiModel>(robot_file);
	robot->setTRobotBase(T_world_robot);
	robot->setQ(sim->getJointPositions(robot_name));
	robot->setDq(sim->getJointVelocities(robot_name));
	robot->updateModel();

	cout << "\nLatency-aware haptic contact rendering example." << endl;
	cout << "The haptic controller homes the device, then maps device motion "
			"to the Panda end effector. The renderer predicts contact over "
		 << kLatency
		 << " s, renders a force from the point cloud, and learns a local "
			"surface bias when delayed robot contact arrives."
		 << endl;
	cout << "Provided options:" << endl;
	cout << "1. Press the device gripper/button after homing to start teleop."
		 << endl;
	cout << "2. Press the device gripper/button during teleop to clutch."
		 << endl;
	cout << "3. Press 'p' to enable/disable plane guidance." << endl;
	cout << "4. Press 'l' to enable/disable line guidance." << endl;
	cout << "5. Press 'w' to enable/disable haptic workspace virtual limits."
		 << endl;
	cout << "6. Orientation teleoperation is enabled by default. Press 'o' "
			"to enable/disable it."
		 << endl;

	Affine3d compliant_frame = Affine3d::Identity();
	auto motion_force_task = make_shared<SaiPrimitives::MotionForceTask>(
		robot, link_name, compliant_frame);
	motion_force_task->disableInternalOtg();
	motion_force_task->enableVelocitySaturation(0.7, M_PI);
	motion_force_task->setOriControlGains(400.0, 40.0);

	auto joint_task = make_shared<SaiPrimitives::JointTask>(robot);
	vector<shared_ptr<SaiPrimitives::TemplateTask>> task_list = {
		motion_force_task, joint_task};
	auto robot_controller =
		make_unique<SaiPrimitives::RobotController>(robot, task_list);

	SaiPrimitives::LatencyAwareContactRenderer renderer;
	const RendererPointCloud renderer_point_cloud = makeRendererPointCloud();
	renderer.setPointCloud(
		renderer_point_cloud.points,
		renderer_point_cloud.normals);
	renderer.setUncertaintyMargin(kUncertaintyMargin);
	renderer.setLearningRate(kLearningRate);
	renderer.setLearningSigma(kLearningSigma);
	renderer.setBiasClamp(kMinLearnedBias, kMaxLearnedBias);
	renderer.setMaxHistoryDuration(2.5);

	SaiPrimitives::HapticDeviceController::DeviceLimits device_limits(
		redis_client.getEigen(createRedisKey(MAX_STIFFNESS_KEY_SUFFIX, 0)),
		redis_client.getEigen(createRedisKey(MAX_DAMPING_KEY_SUFFIX, 0)),
		redis_client.getEigen(createRedisKey(MAX_FORCE_KEY_SUFFIX, 0)));
	auto haptic_controller =
		make_shared<SaiPrimitives::HapticDeviceController>(
			device_limits, robot->transformInWorld(link_name));
	haptic_controller->setScalingFactors(3.5);
	haptic_controller->setHapticControlType(
		SaiPrimitives::HapticControlType::HOMING);
	haptic_controller->enableOrientationTeleop();

	SaiPrimitives::HapticControllerInput haptic_input;
	SaiPrimitives::HapticControllerOutput haptic_output;
	bool haptic_button_was_pressed = false;
	int haptic_button_is_pressed = 0;
	redis_client.setInt(createRedisKey(SWITCH_PRESSED_KEY_SUFFIX, 0),
						haptic_button_is_pressed);
	redis_client.setInt(createRedisKey(USE_GRIPPER_AS_SWITCH_KEY_SUFFIX, 0), 1);

	redis_client.addToSendGroup(createRedisKey(COMMANDED_FORCE_KEY_SUFFIX, 0),
								haptic_output.device_command_force);
	redis_client.addToSendGroup(createRedisKey(COMMANDED_TORQUE_KEY_SUFFIX, 0),
								haptic_output.device_command_moment);

	redis_client.addToReceiveGroup(createRedisKey(POSITION_KEY_SUFFIX, 0),
								   haptic_input.device_position);
	redis_client.addToReceiveGroup(createRedisKey(ROTATION_KEY_SUFFIX, 0),
								   haptic_input.device_orientation);
	redis_client.addToReceiveGroup(
		createRedisKey(LINEAR_VELOCITY_KEY_SUFFIX, 0),
		haptic_input.device_linear_velocity);
	redis_client.addToReceiveGroup(
		createRedisKey(ANGULAR_VELOCITY_KEY_SUFFIX, 0),
		haptic_input.device_angular_velocity);
	redis_client.addToReceiveGroup(createRedisKey(SWITCH_PRESSED_KEY_SUFFIX, 0),
								   haptic_button_is_pressed);

	SaiCommon::LoopTimer controlTimer(kControlFrequency, 1e6);
	bool robot_contact_active = false;
	bool renderer_contact_was_active = false;
	double renderer_contact_start_time = 0.0;
	double previous_control_time = 0.0;
	Vector3d filtered_renderer_force = Vector3d::Zero();

	while (fSimulationRunning) {
		controlTimer.waitForNextLoop();
		const double time = controlTimer.elapsedSimTime();
		const double dt =
			previous_control_time > 0.0 ?
				time - previous_control_time :
				1.0 / kControlFrequency;
		previous_control_time = time;

		robot->setQ(sim->getJointPositions(robot_name));
		robot->setDq(sim->getJointVelocities(robot_name));
		robot->updateModel();

		robot_controller->updateControllerTaskModels();

		redis_client.receiveAllFromGroup();

		const Vector3d robot_position = robot->positionInWorld(link_name);
		const Matrix3d robot_orientation = robot->rotationInWorld(link_name);
		const Vector3d robot_linear_velocity =
			robot->linearVelocityInWorld(link_name);
		Vector3d surface_normal =
			renderer.evaluateGradientPhiHat(robot_position);
		const Vector3d renderer_point_in_link =
			sphereContactPointInLink(robot_orientation, surface_normal);
		Vector3d renderer_position =
			robot->positionInWorld(link_name, renderer_point_in_link);
		surface_normal = renderer.evaluateGradientPhiHat(renderer_position);
		const Vector3d refined_renderer_point_in_link =
			sphereContactPointInLink(robot_orientation, surface_normal);
		renderer_position =
			robot->positionInWorld(link_name, refined_renderer_point_in_link);
		const Vector3d renderer_linear_velocity =
			robot->linearVelocityInWorld(
				link_name,
				refined_renderer_point_in_link);

		haptic_input.robot_position = robot_position;
		haptic_input.robot_orientation = robot_orientation;
		haptic_input.robot_linear_velocity = robot_linear_velocity;
		haptic_input.robot_angular_velocity =
			robot->angularVelocityInWorld(link_name);
		haptic_input.robot_sensed_force = Vector3d::Zero();
		haptic_input.robot_sensed_moment = Vector3d::Zero();

		SaiPrimitives::LatencyAwareContactRenderer::Query renderer_query;
		renderer_query.time = time;
		renderer_query.robot_position = renderer_position;
		renderer_query.robot_velocity = renderer_linear_velocity;
		const double inward_normal_velocity =
			min(0.0, renderer_linear_velocity.dot(surface_normal));
		renderer_query.haptic_velocity =
			inward_normal_velocity * surface_normal;
		renderer_query.approach_direction =
			normalizedOrFallback(renderer_linear_velocity, -Vector3d::UnitZ());
		renderer_query.latency = kLatency;
		renderer_query.path_samples = kRendererPathSamples;
		renderer_query.stiffness = kRendererStiffness;
		renderer_query.damping = kRendererDamping;
		renderer_query.max_force =
			min(kRendererMaxForce, device_limits.max_force);
		renderer_query.use_time_to_contact_scaling =
			kUseTimeToContactScaling;

		auto renderer_result =
			renderer.computeLatencyAwareHapticForce(renderer_query);

		const bool render_contact =
			shouldRenderHapticContact(
				haptic_controller->getHapticControlType());
		const bool renderer_contact_active =
			render_contact && renderer_result.in_contact;
		if (!renderer_contact_active) {
			renderer_contact_was_active = false;
			filtered_renderer_force.setZero();
			renderer_result.force.setZero();
		} else {
			if (!renderer_contact_was_active) {
				renderer_contact_start_time = time;
				renderer_contact_was_active = true;
			}
			const double contact_ramp =
				std::clamp(
					(time - renderer_contact_start_time) /
						kRendererForceRampDuration,
					0.0,
					1.0);
			const Vector3d ramped_force =
				contact_ramp * renderer_result.force;
			filtered_renderer_force =
				smoothRendererForce(
					filtered_renderer_force,
					ramped_force,
					dt);
			renderer_result.force = filtered_renderer_force;
		}

		haptic_output = haptic_controller->computeHapticControl(haptic_input);
		const Vector3d renderer_force_device_frame =
			haptic_controller->getRotationWorldToDeviceBase().transpose() *
			renderer_result.force;
		haptic_output.device_command_force =
			clampVectorNorm(
				haptic_output.device_command_force + renderer_force_device_frame,
				min(kRendererMaxForce, device_limits.max_force));

		redis_client.sendAllFromGroup();

		motion_force_task->updateSensedForceAndMoment(
			sim->getSensedForce(robot_name, link_name),
			sim->getSensedMoment(robot_name, link_name));
		motion_force_task->setGoalPosition(haptic_output.robot_goal_position);
		motion_force_task->setGoalOrientation(
			haptic_output.robot_goal_orientation);

		{
			lock_guard<mutex> lock(mutex_torques);
			robot_control_torques = robot_controller->computeControlTorques();
		}

		const Vector3d sensed_force_world =
			sim->getSensedForce(robot_name, link_name, false);
		const double sensed_force_norm = sensed_force_world.norm();
		bool learned_from_robot_contact = false;
		if (!robot_contact_active &&
			sensed_force_norm > kRobotContactForceThreshold) {
			SaiPrimitives::LatencyAwareContactRenderer::RobotContact contact;
			contact.contact_time = time;
			contact.contact_point = renderer_position;
			contact.contact_normal = surface_normal;
			contact.approach_direction =
				normalizedOrFallback(
					renderer_linear_velocity,
					-Vector3d::UnitZ());
			contact.latency = kLatency;
			learned_from_robot_contact =
				renderer.updateFromRobotContact(contact);
			robot_contact_active = true;
		} else if (robot_contact_active &&
				   sensed_force_norm < kRobotContactReleaseThreshold) {
			robot_contact_active = false;
		}

		publishRendererVisualization(
			renderer,
			renderer_point_cloud.visual_indices,
			renderer_query,
			renderer_result,
			render_contact,
			learned_from_robot_contact);

		if (haptic_controller->getHapticControlType() ==
				SaiPrimitives::HapticControlType::HOMING &&
			haptic_controller->getHomed() && haptic_button_is_pressed) {
			haptic_controller->setHapticControlType(
				SaiPrimitives::HapticControlType::MOTION_MOTION);
			haptic_controller->setDeviceControlGains(200.0, 15.0);
			cout << "haptic device homed" << endl;
		}

		if (haptic_controller->getHapticControlType() ==
				SaiPrimitives::HapticControlType::MOTION_MOTION &&
			haptic_button_is_pressed && !haptic_button_was_pressed) {
			haptic_controller->setHapticControlType(
				SaiPrimitives::HapticControlType::CLUTCH);
		} else if (haptic_controller->getHapticControlType() ==
					   SaiPrimitives::HapticControlType::CLUTCH &&
				   !haptic_button_is_pressed && haptic_button_was_pressed) {
			haptic_controller->setHapticControlType(
				SaiPrimitives::HapticControlType::MOTION_MOTION);
		} else if (key_pressed.at(GLFW_KEY_P) &&
				   !key_was_pressed.at(GLFW_KEY_P)) {
			if (haptic_controller->getPlaneGuidanceEnabled()) {
				cout << "disabling plane guidance" << endl;
				haptic_controller->disablePlaneGuidance();
			} else {
				cout << "enabling plane guidance" << endl;
				haptic_controller->enablePlaneGuidance(
					haptic_input.device_position, Vector3d::UnitZ());
			}
		} else if (key_pressed.at(GLFW_KEY_L) &&
				   !key_was_pressed.at(GLFW_KEY_L)) {
			if (haptic_controller->getLineGuidanceEnabled()) {
				cout << "disabling line guidance" << endl;
				haptic_controller->disableLineGuidance();
			} else {
				cout << "enabling line guidance" << endl;
				haptic_controller->enableLineGuidance(
					haptic_input.device_position, Vector3d::UnitZ());
			}
		} else if (key_pressed.at(GLFW_KEY_W) &&
				   !key_was_pressed.at(GLFW_KEY_W)) {
			if (haptic_controller->getHapticWorkspaceVirtualLimitsEnabled()) {
				cout << "disabling haptic workspace virtual limits" << endl;
				haptic_controller->disableHapticWorkspaceVirtualLimits();
			} else {
				cout << "enabling haptic workspace virtual limits" << endl;
				haptic_controller->enableHapticWorkspaceVirtualLimits(
					haptic_input.device_position.norm(), M_PI / 3.0);
			}
		} else if (key_pressed.at(GLFW_KEY_O) &&
				   !key_was_pressed.at(GLFW_KEY_O)) {
			if (haptic_controller->getOrientationTeleopEnabled()) {
				cout << "disabling orientation teleoperation" << endl;
				haptic_controller->disableOrientationTeleop();
			} else {
				cout << "enabling orientation teleoperation" << endl;
				haptic_controller->enableOrientationTeleop();
			}
		}

		haptic_button_was_pressed = haptic_button_is_pressed;
		key_was_pressed = key_pressed;
	}

	redis_client.setEigen(createRedisKey(COMMANDED_FORCE_KEY_SUFFIX, 0),
						  Vector3d::Zero());
	redis_client.setEigen(createRedisKey(COMMANDED_TORQUE_KEY_SUFFIX, 0),
						  Vector3d::Zero());
	redis_client.setInt(createRedisKey(USE_GRIPPER_AS_SWITCH_KEY_SUFFIX, 0), 0);

	cout << "control timer stats:" << endl;
	controlTimer.printInfoPostRun();
}
