/**
 * Example of closed-loop force control with an Active Observer (AOB).
 *
 * Two Panda arms maintain contact with virtual tables whose stiffness varies
 * over time. The upper arm uses the AOB stiffness estimate to adapt the
 * MotionForceTask force-control gains. The lower arm sees the same stiffness
 * variation but keeps fixed nominal force-control gains for comparison.
 */

#include <signal.h>

#include <algorithm>
#include <array>
#include <cmath>
#include <iomanip>
#include <iostream>
#include <memory>
#include <mutex>
#include <sstream>
#include <string>
#include <thread>
#include <vector>

#include "SaiGraphics.h"
#include "SaiModel.h"
#include "SaiSimulation.h"

#include "RobotController.h"
#include "estimators/ActiveObserver.h"
#include "tasks/JointTask.h"
#include "tasks/MotionForceTask.h"
#include "timer/LoopTimer.h"

using namespace std;
using namespace Eigen;

bool fSimulationRunning = false;
void sighandler(int) { fSimulationRunning = false; }

namespace {

const string world_file = "${EXAMPLE_22_FOLDER}/world.urdf";
const string robot_file =
	"${SAI_MODEL_URDF_FOLDER}/panda/panda_arm_sphere.urdf";
const string link_name = "end-effector";

constexpr size_t kNumArms = 2;

constexpr double kControlFreq = 1000.0;
constexpr double kNominalEnvironmentStiffness = 700.0;	// N/m
constexpr double kMinEnvironmentStiffness = 100.0;		// N/m
constexpr double kMaxEnvironmentStiffness = 10000.0;	// N/m
constexpr double kEnvironmentDamping = 4.0;				// Ns/m
constexpr double kEnvironmentSurfaceZ = 0.315;			// m
constexpr double kMinimumEstimatorVelocity = 5e-4;		// m/s
constexpr double kMinimumEstimatorForce = 0.75;			// N
constexpr double kMinimumEstimatorPenetration = 2e-4;	// m
constexpr double kStiffnessLowPass = 0.04;
constexpr double kStiffnessMarkerHalfThickness = 0.012;	// m
constexpr double kStiffnessMarkerMinLift = 0.005;		// m
constexpr double kStiffnessMarkerMaxLift = 0.145;		// m
constexpr double kNominalForceFeedbackLimit = 30.0;		// N
constexpr double kMeanGoalForceZ = -7.0;				// N
constexpr double kGoalForceAmplitude = 4.0;			// N
constexpr double kGoalForceFrequency = 0.35;			// Hz

struct AdaptiveForceGains {
	double kp = 0.0;
	double kv = 0.0;
	double ki = 0.0;
};

AdaptiveForceGains gainsFromStiffness(const double stiffness_estimate) {
	const double normalized_stiffness =
		std::clamp(
			(stiffness_estimate - 300.0) /
				(kMaxEnvironmentStiffness - 300.0),
			0.0,
			1.0);

	AdaptiveForceGains gains;
	gains.kp = 1.15 - 0.95 * normalized_stiffness;
	gains.kv = 4.0 + 46.0 * normalized_stiffness;
	gains.ki = 1.25 - 1.15 * normalized_stiffness;
	return gains;
}

AdaptiveForceGains fixedComparisonGains() {
	return AdaptiveForceGains{1.15, 4.0, 1.25};
}

struct ArmConfig {
	string robot_name;
	string marker_name;
	string overlay_label_name;
	string display_name;
	Vector3d table_center;
	bool use_aob_gain_correction;
};

const array<ArmConfig, kNumArms> kArmConfigs = {{
	{"PANDA_AOB",
	 "StiffnessMarkerAOB",
	 "AOBStatus",
	 "AOB adaptive",
	 Vector3d(0.45, 0.40, 0.30),
	 true},
	{"PANDA_FIXED",
	 "StiffnessMarkerFixed",
	 "FixedStatus",
	 "Fixed gains",
	 Vector3d(0.45, -0.30, 0.30),
	 false},
}};

struct ArmSharedState {
	VectorXd control_torques;
	VectorXd ui_torques;
	Vector3d sensed_force = Vector3d::Zero();
	double true_environment_stiffness = kNominalEnvironmentStiffness;
	double environment_penetration = 0.0;
	double stiffness_estimate = kNominalEnvironmentStiffness;
	double goal_force_z = 0.0;
	double sensed_force_z = 0.0;
	double force_error_z = 0.0;
	AdaptiveForceGains force_gains;
};

double normalizedStiffness(const double stiffness) {
	return std::clamp(
		(stiffness - kMinEnvironmentStiffness) /
			(kMaxEnvironmentStiffness - kMinEnvironmentStiffness),
		0.0,
		1.0);
}

Affine3d stiffnessMarkerPose(
	const Vector3d& table_center,
	const double stiffness) {

	const double alpha = normalizedStiffness(stiffness);
	const double lift =
		kStiffnessMarkerMinLift +
		alpha * (kStiffnessMarkerMaxLift - kStiffnessMarkerMinLift);

	Affine3d marker_pose = Affine3d::Identity();
	marker_pose.translation() =
		Vector3d(
			table_center(0),
			table_center(1),
			kEnvironmentSurfaceZ + kStiffnessMarkerHalfThickness + lift);
	return marker_pose;
}

string overlayLabel(
	const ArmConfig& config,
	const ArmSharedState& state) {

	ostringstream label;
	label << fixed << setprecision(0)
		  << config.display_name
		  << " | true k " << state.true_environment_stiffness
		  << " N/m";
	if (config.use_aob_gain_correction) {
		label << " | AOB k " << state.stiffness_estimate << " N/m";
	} else {
		label << " | fixed nominal gains";
	}
	label << fixed << setprecision(2)
		  << " | err " << state.force_error_z << " N"
		  << " | kp/kv/ki "
		  << state.force_gains.kp << "/"
		  << state.force_gains.kv << "/"
		  << state.force_gains.ki;
	return label.str();
}

double smoothStep(const double x) {
	const double clamped_x = std::clamp(x, 0.0, 1.0);
	return clamped_x * clamped_x * (3.0 - 2.0 * clamped_x);
}

double trueEnvironmentStiffness(const double time) {
	constexpr double transition_duration = 1.0;
	constexpr double segment_duration = 6.0;
	const array<double, 4> stiffness_levels = {
		450.0, 2500.0, 6500.0, 10000.0};

	const double cycle_time =
		std::fmod(std::max(time, 0.0),
				  segment_duration * stiffness_levels.size());
	const int segment =
		static_cast<int>(cycle_time / segment_duration) %
		static_cast<int>(stiffness_levels.size());
	const int next_segment =
		(segment + 1) % static_cast<int>(stiffness_levels.size());
	const double time_in_segment =
		cycle_time - segment_duration * static_cast<double>(segment);

	if (time_in_segment < segment_duration - transition_duration) {
		return stiffness_levels[segment];
	}

	const double alpha = smoothStep(
		(time_in_segment - (segment_duration - transition_duration)) /
		transition_duration);
	return (1.0 - alpha) * stiffness_levels[segment] +
		   alpha * stiffness_levels[next_segment];
}

SaiPrimitives::ActiveObserver makeStiffnessObserver(
	const double loop_timestep) {

	// Nominal contact model: f_z,k = f_z,k-1 + dt * k_nominal * v_z,k-1.
	// The plant state is the filtered contact force, while the active state
	// estimates the velocity correction needed when the real stiffness differs.
	MatrixXd A = MatrixXd::Identity(1, 1);
	MatrixXd B(1, 1);
	B << loop_timestep * kNominalEnvironmentStiffness;
	MatrixXd C = MatrixXd::Identity(1, 1);
	MatrixXd L = MatrixXd::Zero(1, 1);

	const int aob_order = 2;
	SaiPrimitives::ActiveObserver observer(A, B, C, L, aob_order);

	const MatrixXd q_force =
		1e-4 * MatrixXd::Identity(1, 1);
	const MatrixXd q_active =
		4e-5 *
		SaiPrimitives::ActiveObserver::activeDerivativeVarianceScale(
			aob_order, 0.4) *
		MatrixXd::Identity(1, 1);
	const MatrixXd r_force =
		4e-2 * MatrixXd::Identity(1, 1);
	observer.setNoiseCovariances(q_force, q_active, r_force);

	return observer;
}

class ArmControlRuntime {
public:
	ArmControlRuntime(
		shared_ptr<SaiModel::SaiModel> robot,
		const bool use_aob_gain_correction) {

		vector<Vector3d> controlled_directions_translation = {
			Vector3d::UnitX(), Vector3d::UnitY(), Vector3d::UnitZ()};
		vector<Vector3d> controlled_directions_rotation;
		motion_force_task = make_shared<SaiPrimitives::MotionForceTask>(
			robot, link_name, controlled_directions_translation,
			controlled_directions_rotation);
		motion_force_task->setPosControlGains(100.0, 20.0);
		motion_force_task->setOriControlGains(100.0, 20.0);
		motion_force_task->setMaxForceControlFeedbackOutput(
			kNominalForceFeedbackLimit);

		force_gains = use_aob_gain_correction ?
			gainsFromStiffness(kNominalEnvironmentStiffness) :
			fixedComparisonGains();
		motion_force_task->setForceControlGains(
			force_gains.kp, force_gains.kv, force_gains.ki);

		goal_position = robot->positionInWorld(link_name);

		auto joint_task = make_shared<SaiPrimitives::JointTask>(robot);
		vector<shared_ptr<SaiPrimitives::TemplateTask>> task_list = {
			motion_force_task, joint_task};
		robot_controller =
			make_unique<SaiPrimitives::RobotController>(robot, task_list);

		if (use_aob_gain_correction) {
			stiffness_observer =
				make_unique<SaiPrimitives::ActiveObserver>(
					makeStiffnessObserver(1.0 / kControlFreq));
		}
	}

	shared_ptr<SaiPrimitives::MotionForceTask> motion_force_task;
	unique_ptr<SaiPrimitives::RobotController> robot_controller;
	unique_ptr<SaiPrimitives::ActiveObserver> stiffness_observer;
	Vector3d goal_position = Vector3d::Zero();
	double stiffness_estimate = kNominalEnvironmentStiffness;
	AdaptiveForceGains force_gains =
		gainsFromStiffness(kNominalEnvironmentStiffness);
	bool force_control_enabled = false;
};

}  // namespace

void control(
	array<shared_ptr<SaiModel::SaiModel>, kNumArms> robots,
	shared_ptr<SaiSimulation::SaiSimulation> sim);
void simulation(shared_ptr<SaiSimulation::SaiSimulation> sim);

array<ArmSharedState, kNumArms> arm_states;
mutex mutex_torques;

int main() {
	SaiModel::URDF_FOLDERS["EXAMPLE_22_FOLDER"] =
		string(EXAMPLES_FOLDER) + "/22-aob_force_control";
	cout << "Loading URDF world model file: "
		 << SaiModel::ReplaceUrdfPathPrefix(world_file) << endl;

	signal(SIGABRT, &sighandler);
	signal(SIGTERM, &sighandler);
	signal(SIGINT, &sighandler);

	auto graphics = make_shared<SaiGraphics::SaiGraphics>(world_file);

	auto sim = make_shared<SaiSimulation::SaiSimulation>(world_file);
	sim->setCoeffFrictionStatic(0.0);

	array<shared_ptr<SaiModel::SaiModel>, kNumArms> robots;
	for (size_t arm = 0; arm < kNumArms; ++arm) {
		const ArmConfig& config = kArmConfigs[arm];
		robots[arm] = make_shared<SaiModel::SaiModel>(robot_file, false);
		robots[arm]->setTRobotBase(
			sim->getRobotBaseTransform(config.robot_name));
		robots[arm]->setQ(sim->getJointPositions(config.robot_name));
		robots[arm]->updateModel();

		arm_states[arm].control_torques =
			VectorXd::Zero(robots[arm]->dof());
		arm_states[arm].ui_torques =
			VectorXd::Zero(robots[arm]->dof());
		arm_states[arm].force_gains =
			config.use_aob_gain_correction ?
				gainsFromStiffness(kNominalEnvironmentStiffness) :
				fixedComparisonGains();

		graphics->addUIForceInteraction(config.robot_name);
		graphics->updateObjectGraphics(
			config.marker_name,
			stiffnessMarkerPose(
				config.table_center,
				kNominalEnvironmentStiffness));
	}
	graphics->addOverlayLabel("AOBStatus", "", "", 20, 40, 0.82);
	graphics->addOverlayLabel("FixedStatus", "", "", 20, 64, 0.82);

	fSimulationRunning = true;
	thread sim_thread(simulation, sim);
	thread ctrl_thread(control, robots, sim);

	while (graphics->isWindowOpen()) {
		for (size_t arm = 0; arm < kNumArms; ++arm) {
			graphics->updateRobotGraphics(
				kArmConfigs[arm].robot_name,
				robots[arm]->q());
		}

		array<ArmSharedState, kNumArms> display_states;
		{
			lock_guard<mutex> guard(mutex_torques);
			for (size_t arm = 0; arm < kNumArms; ++arm) {
				arm_states[arm].ui_torques =
					graphics->getUITorques(kArmConfigs[arm].robot_name);
				display_states[arm] = arm_states[arm];
			}
		}

		for (size_t arm = 0; arm < kNumArms; ++arm) {
			const ArmConfig& config = kArmConfigs[arm];
			graphics->updateObjectGraphics(
				config.marker_name,
				stiffnessMarkerPose(
					config.table_center,
					display_states[arm].true_environment_stiffness));
			graphics->updateOverlayLabel(
				config.overlay_label_name,
				overlayLabel(config, display_states[arm]),
				config.use_aob_gain_correction ? 0.65 : 0.88,
				config.use_aob_gain_correction ? 0.82 : 0.88,
				config.use_aob_gain_correction ? 1.0 : 0.88);
		}

		graphics->renderGraphicsWorld();
	}

	fSimulationRunning = false;
	sim_thread.join();
	ctrl_thread.join();

	return 0;
}

void control(
	array<shared_ptr<SaiModel::SaiModel>, kNumArms> robots,
	shared_ptr<SaiSimulation::SaiSimulation> sim) {

	vector<ArmControlRuntime> runtimes;
	runtimes.reserve(kNumArms);
	for (size_t arm = 0; arm < kNumArms; ++arm) {
		robots[arm]->updateModel();
		runtimes.emplace_back(
			robots[arm],
			kArmConfigs[arm].use_aob_gain_correction);
	}

	SaiCommon::LoopTimer timer(kControlFreq, 1e6);

	while (fSimulationRunning) {
		timer.waitForNextLoop();

		for (size_t arm = 0; arm < kNumArms; ++arm) {
			robots[arm]->setQ(
				sim->getJointPositions(kArmConfigs[arm].robot_name));
			robots[arm]->setDq(
				sim->getJointVelocities(kArmConfigs[arm].robot_name));
			robots[arm]->updateModel();
			runtimes[arm].robot_controller->updateControllerTaskModels();
		}

		array<Vector3d, kNumArms> sensed_force_sensor_snapshots;
		array<double, kNumArms> true_stiffness_snapshots;
		array<double, kNumArms> penetration_snapshots;
		{
			lock_guard<mutex> guard(mutex_torques);
			for (size_t arm = 0; arm < kNumArms; ++arm) {
				sensed_force_sensor_snapshots[arm] =
					arm_states[arm].sensed_force;
				true_stiffness_snapshots[arm] =
					arm_states[arm].true_environment_stiffness;
				penetration_snapshots[arm] =
					arm_states[arm].environment_penetration;
			}
		}

		const double time = timer.elapsedSimTime();
		for (size_t arm = 0; arm < kNumArms; ++arm) {
			const ArmConfig& config = kArmConfigs[arm];
			ArmControlRuntime& runtime = runtimes[arm];
			auto motion_force_task = runtime.motion_force_task;

			motion_force_task->updateSensedForceAndMoment(
				sensed_force_sensor_snapshots[arm],
				Vector3d::Zero());

			const double sensed_force_z =
				motion_force_task->getSensedForceControlWorldFrame()(2);
			const double measured_velocity_z =
				motion_force_task->getCurrentLinearVelocity()(2);

			if (!runtime.force_control_enabled) {
				runtime.goal_position(2) -= 1.5e-4;
				motion_force_task->setGoalPosition(runtime.goal_position);

				if (sensed_force_z <= -1.0) {
					runtime.force_control_enabled = true;
					if (runtime.stiffness_observer) {
						runtime.stiffness_observer->reInitialize();
						runtime.stiffness_observer->setPlantStateEstimate(
							VectorXd::Constant(1, sensed_force_z));
						runtime.stiffness_observer->setPreviousReference(
							VectorXd::Constant(1, measured_velocity_z));
					}

					motion_force_task->parametrizeForceMotionSpaces(
						1, Vector3d::UnitZ());
					motion_force_task->setClosedLoopForceControl();
					motion_force_task->enablePassivity();

					cout << config.display_name
						 << " contact detected. Force control enabled."
						 << endl;
				}
			}

			if (runtime.force_control_enabled) {
				const double desired_force_z =
					kMeanGoalForceZ -
					kGoalForceAmplitude *
						std::sin(2.0 * M_PI * kGoalForceFrequency * time);
				motion_force_task->setGoalForce(
					Vector3d(0.0, 0.0, desired_force_z));

				if (runtime.stiffness_observer) {
					const VectorXd observer_input =
						VectorXd::Constant(1, measured_velocity_z);
					const VectorXd observer_measurement =
						VectorXd::Constant(1, sensed_force_z);
					runtime.stiffness_observer->update(
						observer_input,
						observer_measurement);

					const double estimated_force_z =
						runtime.stiffness_observer
							->getPlantStateEstimate()(0);
					const double equivalent_velocity_disturbance =
						runtime.stiffness_observer
							->getActiveStateEstimate()(0);
					bool stiffness_measurement_valid = false;
					double raw_stiffness = runtime.stiffness_estimate;
					if (penetration_snapshots[arm] >
							kMinimumEstimatorPenetration &&
						std::abs(estimated_force_z) >
							kMinimumEstimatorForce) {
						raw_stiffness =
							std::abs(estimated_force_z) /
							penetration_snapshots[arm];
						stiffness_measurement_valid = true;
					} else if (std::abs(measured_velocity_z) >
								   kMinimumEstimatorVelocity &&
							   std::abs(sensed_force_z) >
								   kMinimumEstimatorForce) {
						raw_stiffness =
							std::abs(kNominalEnvironmentStiffness *
									 (measured_velocity_z +
									  equivalent_velocity_disturbance) /
									 measured_velocity_z);
						stiffness_measurement_valid = true;
					}

					if (stiffness_measurement_valid) {
						const double bounded_stiffness = std::clamp(
							raw_stiffness,
							kMinEnvironmentStiffness,
							kMaxEnvironmentStiffness);
						runtime.stiffness_estimate =
							(1.0 - kStiffnessLowPass) *
								runtime.stiffness_estimate +
							kStiffnessLowPass * bounded_stiffness;
					}

					runtime.force_gains =
						gainsFromStiffness(runtime.stiffness_estimate);
					motion_force_task->setForceControlGains(
						runtime.force_gains.kp,
						runtime.force_gains.kv,
						runtime.force_gains.ki);
				}
			}
		}

		if (timer.elapsedCycles() % 1000 == 999) {
			cout << fixed << setprecision(3);
			for (size_t arm = 0; arm < kNumArms; ++arm) {
				const ArmConfig& config = kArmConfigs[arm];
				const ArmControlRuntime& runtime = runtimes[arm];
				const auto motion_force_task = runtime.motion_force_task;
				const double goal_force_z =
					motion_force_task->getGoalForce()(2);
				const double sensed_force_z =
					motion_force_task
						->getSensedForceControlWorldFrame()(2);
				cout << config.display_name
					 << " | goal force z: "
					 << goal_force_z
					 << " N | sensed force z: "
					 << sensed_force_z
					 << " N | error z: "
					 << goal_force_z - sensed_force_z
					 << " N | true stiffness: "
					 << true_stiffness_snapshots[arm]
					 << " N/m | penetration: "
					 << penetration_snapshots[arm]
					 << " m";
				if (runtime.stiffness_observer) {
					cout << " | AOB force: "
						 << runtime.stiffness_observer
								->getPlantStateEstimate()(0)
						 << " N | AOB stiffness: "
						 << runtime.stiffness_estimate
						 << " N/m";
				} else {
					cout << " | fixed nominal force gains";
				}
				cout << " | gains kp/kv/ki: "
					 << runtime.force_gains.kp << "/"
					 << runtime.force_gains.kv << "/"
					 << runtime.force_gains.ki << endl;
			}
		}

		{
			lock_guard<mutex> guard(mutex_torques);
			for (size_t arm = 0; arm < kNumArms; ++arm) {
				arm_states[arm].control_torques =
					runtimes[arm].robot_controller
						->computeControlTorques();
				arm_states[arm].stiffness_estimate =
					runtimes[arm].stiffness_estimate;
				arm_states[arm].force_gains =
					runtimes[arm].force_gains;
				arm_states[arm].goal_force_z =
					runtimes[arm].motion_force_task->getGoalForce()(2);
				arm_states[arm].sensed_force_z =
					runtimes[arm].motion_force_task
						->getSensedForceControlWorldFrame()(2);
				arm_states[arm].force_error_z =
					arm_states[arm].goal_force_z -
					arm_states[arm].sensed_force_z;
			}
		}
	}

	timer.stop();
	cout << "\nControl loop timer stats:\n";
	timer.printInfoPostRun();
}

void simulation(shared_ptr<SaiSimulation::SaiSimulation> sim) {
	fSimulationRunning = true;

	double sim_freq = 2000.0;
	SaiCommon::LoopTimer timer(sim_freq);
	sim->setTimestep(1.0 / sim_freq);

	array<shared_ptr<SaiModel::SaiModel>, kNumArms> sim_robots;
	for (size_t arm = 0; arm < kNumArms; ++arm) {
		sim_robots[arm] =
			make_shared<SaiModel::SaiModel>(robot_file, false);
		sim_robots[arm]->setTRobotBase(
			sim->getRobotBaseTransform(kArmConfigs[arm].robot_name));
	}

	while (fSimulationRunning) {
		timer.waitForNextLoop();

		array<VectorXd, kNumArms> environment_torques;
		array<Vector3d, kNumArms> sensed_force_sensors;
		array<double, kNumArms> stiffnesses;
		array<double, kNumArms> penetrations;

		for (size_t arm = 0; arm < kNumArms; ++arm) {
			const ArmConfig& config = kArmConfigs[arm];
			auto sim_robot = sim_robots[arm];
			sim_robot->setQ(sim->getJointPositions(config.robot_name));
			sim_robot->setDq(sim->getJointVelocities(config.robot_name));
			sim_robot->updateModel();

			const Vector3d current_position =
				sim_robot->positionInWorld(link_name);
			const Vector3d current_velocity =
				sim_robot->linearVelocityInWorld(link_name);
			const double stiffness = trueEnvironmentStiffness(sim->time());
			const double penetration =
				std::max(0.0, kEnvironmentSurfaceZ - current_position(2));

			Vector3d environment_force_on_robot = Vector3d::Zero();
			if (penetration > 0.0) {
				environment_force_on_robot(2) =
					stiffness * penetration -
					kEnvironmentDamping * current_velocity(2);
				if (environment_force_on_robot(2) < 0.0) {
					environment_force_on_robot.setZero();
				}
			}

			const MatrixXd Jv = sim_robot->JvWorldFrame(link_name);
			environment_torques[arm] =
				Jv.transpose() * environment_force_on_robot;
			const Vector3d sensed_force_world =
				-environment_force_on_robot;
			sensed_force_sensors[arm] =
				sim_robot->rotationInWorld(link_name).transpose() *
				sensed_force_world;
			stiffnesses[arm] = stiffness;
			penetrations[arm] = penetration;
		}

		{
			lock_guard<mutex> guard(mutex_torques);
			for (size_t arm = 0; arm < kNumArms; ++arm) {
				const ArmConfig& config = kArmConfigs[arm];
				sim->setJointTorques(
					config.robot_name,
					arm_states[arm].control_torques +
						arm_states[arm].ui_torques +
						environment_torques[arm]);
				arm_states[arm].sensed_force =
					sensed_force_sensors[arm];
				arm_states[arm].true_environment_stiffness =
					stiffnesses[arm];
				arm_states[arm].environment_penetration =
					penetrations[arm];
			}
		}
		sim->integrate();
	}

	timer.stop();
	cout << "\nSimulation loop timer stats:\n";
	timer.printInfoPostRun();
}
