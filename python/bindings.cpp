#include <pybind11/eigen.h>
#include <pybind11/pybind11.h>
#include <pybind11/stl.h>

#include <string>

#include "helper_modules/SaiPrimitivesCommonDefinitions.h"
#include "HapticDeviceController.h"
#include "POPCBilateralTeleoperation.h"
#include "RobotController.h"
#include "tasks/ComMotionTask.h"
#include "tasks/CentroidalAngularMomentumTask.h"
#include "tasks/CentroidalLinearMomentumTask.h"
#include "tasks/JointTask.h"
#include "tasks/MotionForceTask.h"
#include "tasks/MomentumTask.h"
#include "tasks/TemplateTask.h"

namespace py = pybind11;
using namespace SaiPrimitives;

PYBIND11_MODULE(sai_primitives_py, m) {
	const std::string sai_model_path =
		"/Users/william/OpenSai/core/sai-model/build/python";
	py::module_::import("sys").attr("path").attr("insert")(0, sai_model_path);

	py::module_ sai_model_module;
	try {
		sai_model_module = py::module_::import("sai_model_py");
	} catch (const py::error_already_set&) {
		throw std::runtime_error(
			"sai_primitives_py failed to import 'sai_model_py' from " +
			sai_model_path + ".");
	}

	auto robot_from_py = [](const py::object& robot_obj)
		-> std::shared_ptr<SaiModel::SaiModel> {
		SaiModel::SaiModel* raw = robot_obj.cast<SaiModel::SaiModel*>();
		auto keep_alive = std::make_shared<py::object>(robot_obj);
		return std::shared_ptr<SaiModel::SaiModel>(
			raw, [keep_alive](SaiModel::SaiModel*) mutable { keep_alive.reset(); });
	};

	m.def(
		"create_robot",
		[sai_model_module](const std::string& urdf_file,
						  const bool verbose = false) {
			return sai_model_module.attr("SaiModel")(urdf_file, verbose);
		},
		py::arg("urdf_file"), py::arg("verbose") = false);

	py::enum_<DynamicDecouplingType>(m, "DynamicDecouplingType")
		.value("FULL_DYNAMIC_DECOUPLING",
			   DynamicDecouplingType::FULL_DYNAMIC_DECOUPLING)
		.value("BOUNDED_INERTIA_ESTIMATES",
			   DynamicDecouplingType::BOUNDED_INERTIA_ESTIMATES)
		.value("IMPEDANCE", DynamicDecouplingType::IMPEDANCE)
		.export_values();

	py::class_<PIDGains>(m, "PIDGains")
		.def(py::init<double, double, double>(), py::arg("kp"), py::arg("kv"),
			 py::arg("ki"))
		.def_readwrite("kp", &PIDGains::kp)
		.def_readwrite("kv", &PIDGains::kv)
		.def_readwrite("ki", &PIDGains::ki);

	py::enum_<TaskType>(m, "TaskType")
		.value("UNDEFINED", TaskType::UNDEFINED)
		.value("JOINT_LIMIT_AVOIDANCE_TASK", TaskType::JOINT_LIMIT_AVOIDANCE_TASK)
		.value("JOINT_TASK", TaskType::JOINT_TASK)
		.value("MOTION_FORCE_TASK", TaskType::MOTION_FORCE_TASK)
		.value("CENTROIDAL_ANGULAR_MOMENTUM_TASK",
			   TaskType::CENTROIDAL_ANGULAR_MOMENTUM_TASK)
		.value("CENTROIDAL_LINEAR_MOMENTUM_TASK",
			   TaskType::CENTROIDAL_LINEAR_MOMENTUM_TASK)
		.value("MOMENTUM_TASK", TaskType::MOMENTUM_TASK)
		.export_values();

	py::class_<TemplateTask, std::shared_ptr<TemplateTask>>(m, "TemplateTask")
		.def("updateTaskModel", &TemplateTask::updateTaskModel)
		.def("computeTorques",
			 py::overload_cast<>(&TemplateTask::computeTorques))
		.def("computeTorques",
			 py::overload_cast<const Eigen::VectorXd&>(
				 &TemplateTask::computeTorques),
			 py::arg("tau_prec"))
		.def("reInitializeTask", &TemplateTask::reInitializeTask)
		.def("getTaskNullspace", &TemplateTask::getTaskNullspace)
		.def("getPreviousTasksNullspace", &TemplateTask::getPreviousTasksNullspace)
		.def("getTaskAndPreviousNullspace",
			 &TemplateTask::getTaskAndPreviousNullspace)
		.def("getLoopTimestep", &TemplateTask::getLoopTimestep)
		.def("getTaskType", &TemplateTask::getTaskType)
		.def("getTaskName", &TemplateTask::getTaskName);

	py::class_<JointTask, TemplateTask, std::shared_ptr<JointTask>>(m,
																	"JointTask")
		.def(py::init([&](const py::object& robot_obj,
						 const std::string& task_name, double loop_timestep) {
				 auto robot = robot_from_py(robot_obj);
				 return std::make_shared<JointTask>(robot, task_name, loop_timestep);
			 }),
			 py::arg("robot"), py::arg("task_name") = "joint_task",
			 py::arg("loop_timestep") = 0.001)
		.def(py::init([&](const py::object& robot_obj,
						 const Eigen::MatrixXd& joint_selection_matrix,
						 const std::string& task_name, double loop_timestep) {
				 auto robot = robot_from_py(robot_obj);
				 return std::make_shared<JointTask>(
					 robot, joint_selection_matrix, task_name, loop_timestep);
			 }),
			 py::arg("robot"), py::arg("joint_selection_matrix"),
			 py::arg("task_name") = "partial_joint_task",
			 py::arg("loop_timestep") = 0.001)
		.def("updateTaskModel", &JointTask::updateTaskModel, py::arg("N_prec"))
		.def("computeTorques", py::overload_cast<>(&JointTask::computeTorques))
		.def("computeTorques",
			 py::overload_cast<const Eigen::VectorXd&>(&JointTask::computeTorques),
			 py::arg("tau_prec"))
		.def("reInitializeTask", &JointTask::reInitializeTask)
		.def("getJointSelectionMatrix", &JointTask::getJointSelectionMatrix)
		.def("getTaskDof", &JointTask::getTaskDof)
		.def("isFullJointTask", &JointTask::isFullJointTask)
		.def("getCurrentPosition", &JointTask::getCurrentPosition)
		.def("setGoalPosition", &JointTask::setGoalPosition, py::arg("goal_position"))
		.def("getGoalPosition", &JointTask::getGoalPosition)
		.def("getCurrentVelocity", &JointTask::getCurrentVelocity)
		.def("setGoalVelocity", &JointTask::setGoalVelocity, py::arg("goal_velocity"))
		.def("getGoalVelocity", &JointTask::getGoalVelocity)
		.def("setGoalAcceleration", &JointTask::setGoalAcceleration,
			 py::arg("goal_acceleration"))
		.def("getGoalAcceleration", &JointTask::getGoalAcceleration)
		.def("getDesiredPosition", &JointTask::getDesiredPosition)
		.def("getDesiredVelocity", &JointTask::getDesiredVelocity)
		.def("getDesiredAcceleration", &JointTask::getDesiredAcceleration)
		.def("getTaskNullspace", &JointTask::getTaskNullspace)
		.def("getPreviousTasksNullspace", &JointTask::getPreviousTasksNullspace)
		.def("getTaskAndPreviousNullspace", &JointTask::getTaskAndPreviousNullspace)
		.def("setGains",
			 py::overload_cast<const Eigen::VectorXd&, const Eigen::VectorXd&,
							   const Eigen::VectorXd&>(&JointTask::setGains),
			 py::arg("kp"), py::arg("kv"), py::arg("ki"))
		.def("setGains",
			 py::overload_cast<double, double, double>(&JointTask::setGains),
			 py::arg("kp"), py::arg("kv"), py::arg("ki") = 0.0)
		.def("setGainsUnsafe", &JointTask::setGainsUnsafe, py::arg("kp"),
			 py::arg("kv"), py::arg("ki"))
		.def("getGains", &JointTask::getGains)
		.def("enableInternalOtgAccelerationLimited",
			 py::overload_cast<const Eigen::VectorXd&, const Eigen::VectorXd&>(
				 &JointTask::enableInternalOtgAccelerationLimited),
			 py::arg("max_velocity"), py::arg("max_acceleration"))
		.def("enableInternalOtgAccelerationLimited",
			 py::overload_cast<double, double>(
				 &JointTask::enableInternalOtgAccelerationLimited),
			 py::arg("max_velocity"), py::arg("max_acceleration"))
		.def("enableInternalOtgJerkLimited",
			 py::overload_cast<const Eigen::VectorXd&, const Eigen::VectorXd&,
							   const Eigen::VectorXd&>(
				 &JointTask::enableInternalOtgJerkLimited),
			 py::arg("max_velocity"), py::arg("max_acceleration"),
			 py::arg("max_jerk"))
		.def("enableInternalOtgJerkLimited",
			 py::overload_cast<double, double, double>(
				 &JointTask::enableInternalOtgJerkLimited),
			 py::arg("max_velocity"), py::arg("max_acceleration"),
			 py::arg("max_jerk"))
		.def("disableInternalOtg", &JointTask::disableInternalOtg)
		.def("getInternalOtgEnabled", &JointTask::getInternalOtgEnabled)
		.def("enableVelocitySaturation",
			 py::overload_cast<const Eigen::VectorXd&>(
				 &JointTask::enableVelocitySaturation),
			 py::arg("saturation_velocity"))
		.def("enableVelocitySaturation",
			 py::overload_cast<double>(&JointTask::enableVelocitySaturation),
			 py::arg("saturation_velocity"))
		.def("disableVelocitySaturation", &JointTask::disableVelocitySaturation)
		.def("getVelocitySaturationEnabled",
			 &JointTask::getVelocitySaturationEnabled)
		.def("getVelocitySaturationMaxVelocity",
			 &JointTask::getVelocitySaturationMaxVelocity)
		.def("setDynamicDecouplingType", &JointTask::setDynamicDecouplingType,
			 py::arg("type"))
		.def("setBoundedInertiaEstimateThreshold",
			 &JointTask::setBoundedInertiaEstimateThreshold, py::arg("threshold"))
		.def("getBoundedInertiaEstimateThreshold",
			 &JointTask::getBoundedInertiaEstimateThreshold)
		.def("goalPositionReached", &JointTask::goalPositionReached,
			 py::arg("tol") = 1e-2)
		.def("resetIntegrators", &JointTask::resetIntegrators);

	py::class_<CentroidalAngularMomentumTask, TemplateTask,
			   std::shared_ptr<CentroidalAngularMomentumTask>>(
		m, "CentroidalAngularMomentumTask")
		.def(py::init([&](const py::object& robot_obj,
						 const std::string& task_name,
						 double loop_timestep) {
				 auto robot = robot_from_py(robot_obj);
				 return std::make_shared<CentroidalAngularMomentumTask>(
					 robot, task_name, loop_timestep);
			 }),
			 py::arg("robot"),
			 py::arg("task_name") = "centroidal_angular_momentum_task",
			 py::arg("loop_timestep") = 0.001)
		.def("updateTaskModel", &CentroidalAngularMomentumTask::updateTaskModel,
			 py::arg("N_prec"))
		.def("computeTorques",
			 py::overload_cast<>(&CentroidalAngularMomentumTask::computeTorques))
		.def("computeTorques",
			 py::overload_cast<const Eigen::VectorXd&>(
				 &CentroidalAngularMomentumTask::computeTorques),
			 py::arg("tau_prec"))
		.def("reInitializeTask",
			 &CentroidalAngularMomentumTask::reInitializeTask)
		.def("getTaskNullspace",
			 &CentroidalAngularMomentumTask::getTaskNullspace)
		.def("getPreviousTasksNullspace",
			 &CentroidalAngularMomentumTask::getPreviousTasksNullspace)
		.def("getTaskAndPreviousNullspace",
			 &CentroidalAngularMomentumTask::getTaskAndPreviousNullspace)
		.def("getCurrentMomentum",
			 &CentroidalAngularMomentumTask::getCurrentMomentum)
		.def("setGoalMomentum",
			 &CentroidalAngularMomentumTask::setGoalMomentum,
			 py::arg("goal_momentum"))
		.def("getGoalMomentum",
			 &CentroidalAngularMomentumTask::getGoalMomentum)
		.def("setGoalMomentumVelocity",
			 &CentroidalAngularMomentumTask::setGoalMomentumVelocity,
			 py::arg("goal_momentum_velocity"))
		.def("getGoalMomentumVelocity",
			 &CentroidalAngularMomentumTask::getGoalMomentumVelocity)
		.def("getMomentumError",
			 &CentroidalAngularMomentumTask::getMomentumError)
		.def("setGains",
			 py::overload_cast<double>(
				 &CentroidalAngularMomentumTask::setGains),
			 py::arg("kp"))
		.def("setGains",
			 py::overload_cast<const Eigen::Vector3d&>(
				 &CentroidalAngularMomentumTask::setGains),
			 py::arg("kp"))
		.def("getGains", &CentroidalAngularMomentumTask::getGains)
		.def("setDynamicDecouplingType",
			 &CentroidalAngularMomentumTask::setDynamicDecouplingType,
			 py::arg("type"))
		.def("setBoundedInertiaEstimateThreshold",
			 &CentroidalAngularMomentumTask::setBoundedInertiaEstimateThreshold,
			 py::arg("threshold"))
		.def("getBoundedInertiaEstimateThreshold",
			 &CentroidalAngularMomentumTask::getBoundedInertiaEstimateThreshold)
		.def("getJacobian", &CentroidalAngularMomentumTask::getJacobian);

	py::class_<CentroidalLinearMomentumTask, TemplateTask,
			   std::shared_ptr<CentroidalLinearMomentumTask>>(
		m, "CentroidalLinearMomentumTask")
		.def(py::init([&](const py::object& robot_obj,
						 const std::string& task_name,
						 double loop_timestep) {
				 auto robot = robot_from_py(robot_obj);
				 return std::make_shared<CentroidalLinearMomentumTask>(
					 robot, task_name, loop_timestep);
			 }),
			 py::arg("robot"),
			 py::arg("task_name") = "centroidal_linear_momentum_task",
			 py::arg("loop_timestep") = 0.001)
		.def("updateTaskModel", &CentroidalLinearMomentumTask::updateTaskModel,
			 py::arg("N_prec"))
		.def("computeTorques",
			 py::overload_cast<>(&CentroidalLinearMomentumTask::computeTorques))
		.def("computeTorques",
			 py::overload_cast<const Eigen::VectorXd&>(
				 &CentroidalLinearMomentumTask::computeTorques),
			 py::arg("tau_prec"))
		.def("reInitializeTask",
			 &CentroidalLinearMomentumTask::reInitializeTask)
		.def("getTaskNullspace",
			 &CentroidalLinearMomentumTask::getTaskNullspace)
		.def("getPreviousTasksNullspace",
			 &CentroidalLinearMomentumTask::getPreviousTasksNullspace)
		.def("getTaskAndPreviousNullspace",
			 &CentroidalLinearMomentumTask::getTaskAndPreviousNullspace)
		.def("getCurrentMomentum",
			 &CentroidalLinearMomentumTask::getCurrentMomentum)
		.def("setGoalMomentum",
			 &CentroidalLinearMomentumTask::setGoalMomentum,
			 py::arg("goal_momentum"))
		.def("getGoalMomentum",
			 &CentroidalLinearMomentumTask::getGoalMomentum)
		.def("setGoalMomentumVelocity",
			 &CentroidalLinearMomentumTask::setGoalMomentumVelocity,
			 py::arg("goal_momentum_velocity"))
		.def("getGoalMomentumVelocity",
			 &CentroidalLinearMomentumTask::getGoalMomentumVelocity)
		.def("getMomentumError",
			 &CentroidalLinearMomentumTask::getMomentumError)
		.def("setGains",
			 py::overload_cast<double>(
				 &CentroidalLinearMomentumTask::setGains),
			 py::arg("kp"))
		.def("setGains",
			 py::overload_cast<const Eigen::Vector3d&>(
				 &CentroidalLinearMomentumTask::setGains),
			 py::arg("kp"))
		.def("getGains", &CentroidalLinearMomentumTask::getGains)
		.def("setDynamicDecouplingType",
			 &CentroidalLinearMomentumTask::setDynamicDecouplingType,
			 py::arg("type"))
		.def("setBoundedInertiaEstimateThreshold",
			 &CentroidalLinearMomentumTask::setBoundedInertiaEstimateThreshold,
			 py::arg("threshold"))
		.def("getBoundedInertiaEstimateThreshold",
			 &CentroidalLinearMomentumTask::getBoundedInertiaEstimateThreshold)
		.def("getJacobian", &CentroidalLinearMomentumTask::getJacobian);

	py::class_<MomentumTask, TemplateTask, std::shared_ptr<MomentumTask>>(
		m, "MomentumTask")
		.def(py::init([&](const py::object& robot_obj,
						 const std::string& link_name,
						 const std::string& task_name,
						 double loop_timestep) {
				 auto robot = robot_from_py(robot_obj);
				 return std::make_shared<MomentumTask>(
					 robot, link_name, Eigen::Affine3d::Identity(), task_name,
					 loop_timestep);
			 }),
			 py::arg("robot"), py::arg("link_name"),
			 py::arg("task_name") = "momentum_task",
			 py::arg("loop_timestep") = 0.001)
		.def(py::init([&](const py::object& robot_obj,
						 const std::string& link_name,
						 const Eigen::Affine3d& compliant_frame,
						 const std::string& task_name,
						 double loop_timestep) {
				 auto robot = robot_from_py(robot_obj);
				 return std::make_shared<MomentumTask>(
					 robot, link_name, compliant_frame, task_name,
					 loop_timestep);
			 }),
			 py::arg("robot"), py::arg("link_name"),
			 py::arg("compliant_frame"),
			 py::arg("task_name") = "momentum_task",
			 py::arg("loop_timestep") = 0.001)
		.def("updateTaskModel", &MomentumTask::updateTaskModel,
			 py::arg("N_prec"))
		.def("computeTorques",
			 py::overload_cast<>(&MomentumTask::computeTorques))
		.def("computeTorques",
			 py::overload_cast<const Eigen::VectorXd&>(
				 &MomentumTask::computeTorques),
			 py::arg("tau_prec"))
		.def("reInitializeTask", &MomentumTask::reInitializeTask)
		.def("getTaskNullspace", &MomentumTask::getTaskNullspace)
		.def("getPreviousTasksNullspace",
			 &MomentumTask::getPreviousTasksNullspace)
		.def("getTaskAndPreviousNullspace",
			 &MomentumTask::getTaskAndPreviousNullspace)
		.def("getLinkName", &MomentumTask::getLinkName)
		.def("getCompliantFrame", &MomentumTask::getCompliantFrame)
		.def("getCurrentMomentum", &MomentumTask::getCurrentMomentum)
		.def("setGoalMomentum", &MomentumTask::setGoalMomentum,
			 py::arg("goal_momentum"))
		.def("getGoalMomentum", &MomentumTask::getGoalMomentum)
		.def("setGoalMomentumVelocity",
			 &MomentumTask::setGoalMomentumVelocity,
			 py::arg("goal_momentum_velocity"))
		.def("getGoalMomentumVelocity",
			 &MomentumTask::getGoalMomentumVelocity)
		.def("getDesiredMomentumVelocity",
			 &MomentumTask::getDesiredMomentumVelocity)
		.def("getMomentumError", &MomentumTask::getMomentumError)
		.def("setGains",
			 py::overload_cast<double>(&MomentumTask::setGains),
			 py::arg("kp"))
		.def("setGains",
			 py::overload_cast<const Eigen::VectorXd&>(&MomentumTask::setGains),
			 py::arg("kp"))
		.def("getGains", &MomentumTask::getGains)
		.def("setDynamicDecouplingType",
			 &MomentumTask::setDynamicDecouplingType, py::arg("type"))
		.def("setBoundedInertiaEstimateThreshold",
			 &MomentumTask::setBoundedInertiaEstimateThreshold,
			 py::arg("threshold"))
		.def("getBoundedInertiaEstimateThreshold",
			 &MomentumTask::getBoundedInertiaEstimateThreshold)
		.def("getJacobian", &MomentumTask::getJacobian)
		.def("getKineticEnergyGradient",
			 &MomentumTask::getKineticEnergyGradient)
		.def("computeKineticEnergyGradient",
			 &MomentumTask::computeKineticEnergyGradient);

	py::class_<MotionForceTask, TemplateTask, std::shared_ptr<MotionForceTask>>(
		m, "MotionForceTask")
		.def(py::init([&](const py::object& robot_obj,
						 const std::string& link_name,
						 const std::string& task_name,
						 bool is_force_motion_parametrization_in_compliant_frame,
						 double loop_timestep) {
				 auto robot = robot_from_py(robot_obj);
				 return std::make_shared<MotionForceTask>(
					 robot, link_name, Eigen::Affine3d::Identity(), task_name,
					 is_force_motion_parametrization_in_compliant_frame,
					 loop_timestep);
			 }),
			 py::arg("robot"), py::arg("link_name"),
			 py::arg("task_name") = "motion_force_task",
			 py::arg("is_force_motion_parametrization_in_compliant_frame") = false,
			 py::arg("loop_timestep") = 0.001)
		.def(py::init([&](const py::object& robot_obj,
						 const std::string& link_name,
						 const Eigen::Affine3d& compliant_frame,
						 const std::string& task_name,
						 bool is_force_motion_parametrization_in_compliant_frame,
						 double loop_timestep) {
				 auto robot = robot_from_py(robot_obj);
				 return std::make_shared<MotionForceTask>(
					 robot, link_name, compliant_frame, task_name,
					 is_force_motion_parametrization_in_compliant_frame,
					 loop_timestep);
			 }),
			 py::arg("robot"), py::arg("link_name"),
			 py::arg("compliant_frame"),
			 py::arg("task_name") = "motion_force_task",
			 py::arg("is_force_motion_parametrization_in_compliant_frame") = false,
			 py::arg("loop_timestep") = 0.001)
		.def(py::init([&](const py::object& robot_obj,
						 const std::string& link_name,
						 const std::vector<Eigen::Vector3d>&
							 controlled_directions_translation,
						 const std::vector<Eigen::Vector3d>&
							 controlled_directions_rotation,
						 const std::string& task_name,
						 bool is_force_motion_parametrization_in_compliant_frame,
						 double loop_timestep) {
				 auto robot = robot_from_py(robot_obj);
				 return std::make_shared<MotionForceTask>(
					 robot, link_name, controlled_directions_translation,
					 controlled_directions_rotation, Eigen::Affine3d::Identity(),
					 task_name,
					 is_force_motion_parametrization_in_compliant_frame,
					 loop_timestep);
			 }),
			 py::arg("robot"), py::arg("link_name"),
			 py::arg("controlled_directions_translation"),
			 py::arg("controlled_directions_rotation"),
			 py::arg("task_name") = "partial_motion_force_task",
			 py::arg("is_force_motion_parametrization_in_compliant_frame") = false,
			 py::arg("loop_timestep") = 0.001)
		.def(py::init([&](const py::object& robot_obj,
						 const std::string& link_name,
						 const std::vector<Eigen::Vector3d>&
							 controlled_directions_translation,
						 const std::vector<Eigen::Vector3d>&
							 controlled_directions_rotation,
						 const Eigen::Affine3d& compliant_frame,
						 const std::string& task_name,
						 bool is_force_motion_parametrization_in_compliant_frame,
						 double loop_timestep) {
				 auto robot = robot_from_py(robot_obj);
				 return std::make_shared<MotionForceTask>(
					 robot, link_name, controlled_directions_translation,
					 controlled_directions_rotation, compliant_frame, task_name,
					 is_force_motion_parametrization_in_compliant_frame,
					 loop_timestep);
			 }),
			 py::arg("robot"), py::arg("link_name"),
			 py::arg("controlled_directions_translation"),
			 py::arg("controlled_directions_rotation"),
			 py::arg("compliant_frame"),
			 py::arg("task_name") = "partial_motion_force_task",
			 py::arg("is_force_motion_parametrization_in_compliant_frame") = false,
			 py::arg("loop_timestep") = 0.001)
		.def("updateTaskModel", &MotionForceTask::updateTaskModel, py::arg("N_prec"))
		.def("computeTorques",
			 py::overload_cast<>(&MotionForceTask::computeTorques))
		.def("computeTorques",
			 py::overload_cast<const Eigen::VectorXd&>(
				 &MotionForceTask::computeTorques),
			 py::arg("tau_prec"))
		.def("reInitializeTask", &MotionForceTask::reInitializeTask)
		.def("getCurrentPosition", &MotionForceTask::getCurrentPosition)
		.def("getCurrentLinearVelocity", &MotionForceTask::getCurrentLinearVelocity)
		.def("getCurrentOrientation", &MotionForceTask::getCurrentOrientation)
		.def("getCurrentAngularVelocity", &MotionForceTask::getCurrentAngularVelocity)
		.def("getCurrentPose", &MotionForceTask::getCurrentPose)
		.def("getSensedForceControlWorldFrame",
			 &MotionForceTask::getSensedForceControlWorldFrame)
		.def("getSensedMomentControlWorldFrame",
			 &MotionForceTask::getSensedMomentControlWorldFrame)
		.def("getSensedForceSensor", &MotionForceTask::getSensedForceSensor)
		.def("getSensedMomentSensor", &MotionForceTask::getSensedMomentSensor)
		.def("getTaskNullspace", &MotionForceTask::getTaskNullspace)
		.def("getPreviousTasksNullspace",
			 &MotionForceTask::getPreviousTasksNullspace)
		.def("getTaskAndPreviousNullspace",
			 &MotionForceTask::getTaskAndPreviousNullspace)
		.def("setGoalPosition", &MotionForceTask::setGoalPosition,
			 py::arg("goal_position"))
		.def("getGoalPosition", &MotionForceTask::getGoalPosition)
		.def("setGoalOrientation", &MotionForceTask::setGoalOrientation,
			 py::arg("goal_orientation"))
		.def("getGoalOrientation", &MotionForceTask::getGoalOrientation)
		.def("setGoalLinearVelocity", &MotionForceTask::setGoalLinearVelocity,
			 py::arg("goal_linvel"))
		.def("getGoalLinearVelocity", &MotionForceTask::getGoalLinearVelocity)
		.def("setGoalAngularVelocity", &MotionForceTask::setGoalAngularVelocity,
			 py::arg("goal_angvel"))
		.def("getGoalAngularVelocity", &MotionForceTask::getGoalAngularVelocity)
		.def("setGoalLinearAcceleration",
			 &MotionForceTask::setGoalLinearAcceleration, py::arg("goal_linaccel"))
		.def("getGoalLinearAcceleration",
			 &MotionForceTask::getGoalLinearAcceleration)
		.def("setGoalAngularAcceleration",
			 &MotionForceTask::setGoalAngularAcceleration, py::arg("goal_angaccel"))
		.def("getGoalAngularAcceleration",
			 &MotionForceTask::getGoalAngularAcceleration)
		.def("getDesiredPosition", &MotionForceTask::getDesiredPosition)
		.def("getDesiredOrientation", &MotionForceTask::getDesiredOrientation)
		.def("getDesiredLinearVelocity", &MotionForceTask::getDesiredLinearVelocity)
		.def("getDesiredAngularVelocity",
			 &MotionForceTask::getDesiredAngularVelocity)
		.def("getDesiredLinearAcceleration",
			 &MotionForceTask::getDesiredLinearAcceleration)
		.def("getDesiredAngularAcceleration",
			 &MotionForceTask::getDesiredAngularAcceleration)
		.def("getUnitMassForce", &MotionForceTask::getUnitMassForce)
		.def("getPositionError", &MotionForceTask::getPositionError)
		.def("getOrientationError", &MotionForceTask::getOrientationError)
		.def("setPosControlGains",
			 py::overload_cast<double, double, double>(
				 &MotionForceTask::setPosControlGains),
			 py::arg("kp_pos"), py::arg("kv_pos"), py::arg("ki_pos") = 0.0)
		.def("setPosControlGains",
			 py::overload_cast<const Eigen::VectorXd&, const Eigen::VectorXd&,
							   const Eigen::VectorXd&>(
				 &MotionForceTask::setPosControlGains),
			 py::arg("kp_pos"), py::arg("kv_pos"), py::arg("ki_pos"))
		.def("setPosControlGainsUnsafe", &MotionForceTask::setPosControlGainsUnsafe,
			 py::arg("kp_pos"), py::arg("kv_pos"), py::arg("ki_pos"))
		.def("getPosControlGains", &MotionForceTask::getPosControlGains)
		.def("setOriControlGains",
			 py::overload_cast<double, double, double>(
				 &MotionForceTask::setOriControlGains),
			 py::arg("kp_ori"), py::arg("kv_ori"), py::arg("ki_ori") = 0.0)
		.def("setOriControlGains",
			 py::overload_cast<const Eigen::VectorXd&, const Eigen::VectorXd&,
							   const Eigen::VectorXd&>(
				 &MotionForceTask::setOriControlGains),
			 py::arg("kp_ori"), py::arg("kv_ori"), py::arg("ki_ori"))
		.def("setOriControlGainsUnsafe", &MotionForceTask::setOriControlGainsUnsafe,
			 py::arg("kp_ori"), py::arg("kv_ori"), py::arg("ki_ori"))
		.def("getOriControlGains", &MotionForceTask::getOriControlGains)
		.def("setForceControlGains", py::overload_cast<double, double, double>(
										 &MotionForceTask::setForceControlGains),
			 py::arg("kp_force"), py::arg("kv_force"), py::arg("ki_force"))
		.def("getForceControlGains", &MotionForceTask::getForceControlGains)
		.def("setMomentControlGains", py::overload_cast<double, double, double>(
										  &MotionForceTask::setMomentControlGains),
			 py::arg("kp_moment"), py::arg("kv_moment"), py::arg("ki_moment"))
		.def("getMomentControlGains", &MotionForceTask::getMomentControlGains)
		.def("setFeedforwardForceGain", &MotionForceTask::setFeedforwardForceGain,
			 py::arg("kff_force"))
		.def("getFeedforwardForceGain", &MotionForceTask::getFeedforwardForceGain)
		.def("setFeedforwardmomentGain", &MotionForceTask::setFeedforwardmomentGain,
			 py::arg("kff_moment"))
		.def("getFeedforwardmomentGain",
			 &MotionForceTask::getFeedforwardmomentGain)
		.def("setMaxForceControlFeedbackOutput",
			 &MotionForceTask::setMaxForceControlFeedbackOutput,
			 py::arg("max_force_control_feedback_output"))
		.def("getMaxForceControlFeedbackOutput",
			 &MotionForceTask::getMaxForceControlFeedbackOutput)
		.def("setMaxMomentControlFeedbackOutput",
			 &MotionForceTask::setMaxMomentControlFeedbackOutput,
			 py::arg("max_moment_control_feedback_output"))
		.def("getMaxMomentControlFeedbackOutput",
			 &MotionForceTask::getMaxMomentControlFeedbackOutput)
		.def("setGoalForce", &MotionForceTask::setGoalForce, py::arg("goal_force"))
		.def("getGoalForce", &MotionForceTask::getGoalForce)
		.def("setGoalMoment", &MotionForceTask::setGoalMoment,
			 py::arg("goal_moment"))
		.def("getGoalMoment", &MotionForceTask::getGoalMoment)
		.def("enableInternalOtgAccelerationLimited",
			 &MotionForceTask::enableInternalOtgAccelerationLimited,
			 py::arg("max_linear_velelocity"),
			 py::arg("max_linear_acceleration"), py::arg("max_angular_velocity"),
			 py::arg("max_angular_acceleration"))
		.def("enableInternalOtgJerkLimited",
			 &MotionForceTask::enableInternalOtgJerkLimited,
			 py::arg("max_linear_velelocity"),
			 py::arg("max_linear_acceleration"), py::arg("max_linear_jerk"),
			 py::arg("max_angular_velocity"),
			 py::arg("max_angular_acceleration"), py::arg("max_angular_jerk"))
		.def("disableInternalOtg", &MotionForceTask::disableInternalOtg)
		.def("getInternalOtgEnabled", &MotionForceTask::getInternalOtgEnabled)
		.def("enableVelocitySaturation", &MotionForceTask::enableVelocitySaturation,
			 py::arg("linear_vel_sat") = 0.3,
			 py::arg("angular_vel_sat") = M_PI / 3.0)
		.def("disableVelocitySaturation",
			 &MotionForceTask::disableVelocitySaturation)
		.def("getVelocitySaturationEnabled",
			 &MotionForceTask::getVelocitySaturationEnabled)
		.def("getLinearSaturationVelocity",
			 &MotionForceTask::getLinearSaturationVelocity)
		.def("getAngularSaturationVelocity",
			 &MotionForceTask::getAngularSaturationVelocity)
		.def("enableForceDampingDecoupling",
			 &MotionForceTask::enableForceDampingDecoupling)
		.def("disableForceDampingDecoupling",
			 &MotionForceTask::disableForceDampingDecoupling)
		.def("enableZeroForceCrossing", &MotionForceTask::enableZeroForceCrossing)
		.def("enableZeroMomentCrossing", &MotionForceTask::enableZeroMomentCrossing)
		.def("disableZeroForceCrossing", &MotionForceTask::disableZeroForceCrossing)
		.def("disableZeroMomentCrossing",
			 &MotionForceTask::disableZeroMomentCrossing)
		.def("enableZeroPositionCrossing",
			 &MotionForceTask::enableZeroPositionCrossing)
		.def("enableZeroOrientationCrossing",
			 &MotionForceTask::enableZeroOrientationCrossing)
		.def("disableZeroPositionCrossing",
			 &MotionForceTask::disableZeroPositionCrossing)
		.def("disableZeroOrientationCrossing",
			 &MotionForceTask::disableZeroOrientationCrossing)
		.def("goalPositionReached", &MotionForceTask::goalPositionReached,
			 py::arg("tolerance"), py::arg("verbose") = false)
		.def("goalOrientationReached", &MotionForceTask::goalOrientationReached,
			 py::arg("tolerance"), py::arg("verbose") = false)
		.def("goalPoseReached", &MotionForceTask::goalPoseReached,
			 py::arg("pos_tol"), py::arg("ori_tol"), py::arg("verbose") = false)
		.def("setForceSensorFrame", &MotionForceTask::setForceSensorFrame,
			 py::arg("link_name"), py::arg("transformation_in_link"))
		.def("updateSensedForceAndMoment",
			 &MotionForceTask::updateSensedForceAndMoment,
			 py::arg("sensed_force_sensor_frame"),
			 py::arg("sensed_moment_sensor_frame"))
		.def("parametrizeForceMotionSpaces",
			 &MotionForceTask::parametrizeForceMotionSpaces,
			 py::arg("force_space_dimension"),
			 py::arg("force_or_motion_single_axis"))
		.def("getForceSpaceDimension", &MotionForceTask::getForceSpaceDimension)
		.def("getForceMotionSingleAxis", &MotionForceTask::getForceMotionSingleAxis)
		.def("parametrizeMomentRotMotionSpaces",
			 &MotionForceTask::parametrizeMomentRotMotionSpaces,
			 py::arg("moment_space_dimension"),
			 py::arg("moment_or_rot_motion_single_axis") =
				 Eigen::Vector3d::Zero())
		.def("getMomentSpaceDimension", &MotionForceTask::getMomentSpaceDimension)
		.def("getMomentRotMotionSingleAxis",
			 &MotionForceTask::getMomentRotMotionSingleAxis)
		.def("sigmaForce", &MotionForceTask::sigmaForce)
		.def("sigmaPosition", &MotionForceTask::sigmaPosition)
		.def("sigmaMoment", &MotionForceTask::sigmaMoment)
		.def("sigmaOrientation", &MotionForceTask::sigmaOrientation)
		.def("setClosedLoopForceControl", &MotionForceTask::setClosedLoopForceControl,
			 py::arg("closed_loop_force_control") = true)
		.def("setClosedLoopMomentControl",
			 &MotionForceTask::setClosedLoopMomentControl,
			 py::arg("closed_loop_moment_control") = true)
		.def("enablePassivity", &MotionForceTask::enablePassivity)
		.def("disablePassivity", &MotionForceTask::disablePassivity)
		.def("resetIntegrators", &MotionForceTask::resetIntegrators)
		.def("resetIntegratorsLinear", &MotionForceTask::resetIntegratorsLinear)
		.def("resetIntegratorsAngular", &MotionForceTask::resetIntegratorsAngular)
		.def("posSelectionProjector", &MotionForceTask::posSelectionProjector)
		.def("oriSelectionProjector", &MotionForceTask::oriSelectionProjector)
		.def("setDynamicDecouplingType", &MotionForceTask::setDynamicDecouplingType,
			 py::arg("type"))
		.def("setBoundedInertiaEstimateThreshold",
			 py::overload_cast<double>(
				 &MotionForceTask::setBoundedInertiaEstimateThreshold),
			 py::arg("bie_threshold"))
		.def("setBoundedInertiaEstimateThreshold",
			 py::overload_cast<double, double>(
				 &MotionForceTask::setBoundedInertiaEstimateThreshold),
			 py::arg("bie_threshold"), py::arg("sjs_threshold"))
		.def("handleAllSingularitiesAsTypeOne",
			 &MotionForceTask::handleAllSingularitiesAsTypeOne, py::arg("flag"))
		.def("enableSingularityHandling",
			 &MotionForceTask::enableSingularityHandling)
		.def("disableSingularityHandling",
			 &MotionForceTask::disableSingularityHandling)
		.def("setSingularityHandlingBound",
			 &MotionForceTask::setSingularityHandlingBound, py::arg("s_max"))
		.def("setSingularityHandlingGains",
			 &MotionForceTask::setSingularityHandlingGains, py::arg("kv_type_1"),
			 py::arg("kv_type_2"))
		.def("isExitingSingularity", &MotionForceTask::isExitingSingularity)
		.def("setSingularityExitInterpolatorTol",
			 &MotionForceTask::setSingularityExitInterpolatorTol,
			 py::arg("pos_tol"), py::arg("ori_tol"))
		.def("setSingularityVelExitInterpolatorTol",
			 &MotionForceTask::setSingularityVelExitInterpolatorTol,
			 py::arg("linear_vel_tol"), py::arg("angular_vel_tol"))
		.def("setSingularityExitVelocity",
			 &MotionForceTask::setSingularityExitVelocity,
			 py::arg("linear_vel"), py::arg("angular_vel"))
		.def("enableSingularityExitInterpolationVelocityCheck",
			 &MotionForceTask::enableSingularityExitInterpolationVelocityCheck)
		.def("disableSingularityExitInterpolationVelocityCheck",
			 &MotionForceTask::disableSingularityExitInterpolationVelocityCheck);

	py::class_<ComMotionTask, TemplateTask, std::shared_ptr<ComMotionTask>>(
		m, "ComMotionTask")
		.def(py::init([&](const py::object& robot_obj,
						 const std::string& link_name,
						 const std::string& task_name,
						 bool is_force_motion_parametrization_in_compliant_frame,
						 double loop_timestep) {
				 auto robot = robot_from_py(robot_obj);
				 return std::make_shared<ComMotionTask>(
					 robot, link_name, Eigen::Affine3d::Identity(), task_name,
					 is_force_motion_parametrization_in_compliant_frame,
					 loop_timestep);
			 }),
			 py::arg("robot"), py::arg("link_name") = "",
			 py::arg("task_name") = "com_motion_task",
			 py::arg("is_force_motion_parametrization_in_compliant_frame") = false,
			 py::arg("loop_timestep") = 0.001)
		.def(py::init([&](const py::object& robot_obj,
						 const std::string& link_name,
						 const Eigen::Affine3d& compliant_frame,
						 const std::string& task_name,
						 bool is_force_motion_parametrization_in_compliant_frame,
						 double loop_timestep) {
				 auto robot = robot_from_py(robot_obj);
				 return std::make_shared<ComMotionTask>(
					 robot, link_name, compliant_frame, task_name,
					 is_force_motion_parametrization_in_compliant_frame,
					 loop_timestep);
			 }),
			 py::arg("robot"), py::arg("link_name") = "",
			 py::arg("compliant_frame"),
			 py::arg("task_name") = "com_motion_task",
			 py::arg("is_force_motion_parametrization_in_compliant_frame") = false,
			 py::arg("loop_timestep") = 0.001)
		.def(py::init([&](const py::object& robot_obj,
						 const std::string& link_name,
						 const std::vector<Eigen::Vector3d>&
							 controlled_directions_translation,
						 const std::vector<Eigen::Vector3d>&
							 controlled_directions_rotation,
						 const std::string& task_name,
						 bool is_force_motion_parametrization_in_compliant_frame,
						 double loop_timestep) {
				 auto robot = robot_from_py(robot_obj);
				 return std::make_shared<ComMotionTask>(
					 robot, link_name, controlled_directions_translation,
					 controlled_directions_rotation, Eigen::Affine3d::Identity(),
					 task_name,
					 is_force_motion_parametrization_in_compliant_frame,
					 loop_timestep);
			 }),
			 py::arg("robot"), py::arg("link_name"),
			 py::arg("controlled_directions_translation"),
			 py::arg("controlled_directions_rotation"),
			 py::arg("task_name") = "com_motion_task",
			 py::arg("is_force_motion_parametrization_in_compliant_frame") = false,
			 py::arg("loop_timestep") = 0.001)
		.def(py::init([&](const py::object& robot_obj,
						 const std::string& link_name,
						 const std::vector<Eigen::Vector3d>&
							 controlled_directions_translation,
						 const std::vector<Eigen::Vector3d>&
							 controlled_directions_rotation,
						 const Eigen::Affine3d& compliant_frame,
						 const std::string& task_name,
						 bool is_force_motion_parametrization_in_compliant_frame,
						 double loop_timestep) {
				 auto robot = robot_from_py(robot_obj);
				 return std::make_shared<ComMotionTask>(
					 robot, link_name, controlled_directions_translation,
					 controlled_directions_rotation, compliant_frame, task_name,
					 is_force_motion_parametrization_in_compliant_frame,
					 loop_timestep);
			 }),
			 py::arg("robot"), py::arg("link_name"),
			 py::arg("controlled_directions_translation"),
			 py::arg("controlled_directions_rotation"),
			 py::arg("compliant_frame"),
			 py::arg("task_name") = "com_motion_task",
			 py::arg("is_force_motion_parametrization_in_compliant_frame") = false,
			 py::arg("loop_timestep") = 0.001)
		.def("updateTaskModel", &ComMotionTask::updateTaskModel, py::arg("N_prec"))
		.def("computeTorques", py::overload_cast<>(&ComMotionTask::computeTorques))
		.def("computeTorques",
			 py::overload_cast<const Eigen::VectorXd&>(&ComMotionTask::computeTorques),
			 py::arg("tau_prec"))
		.def("reInitializeTask", &ComMotionTask::reInitializeTask)
		.def("getCurrentPosition", &ComMotionTask::getCurrentPosition)
		.def("getCurrentLinearVelocity", &ComMotionTask::getCurrentLinearVelocity)
		.def("getCurrentOrientation", &ComMotionTask::getCurrentOrientation)
		.def("getCurrentAngularVelocity", &ComMotionTask::getCurrentAngularVelocity)
		.def("getSensedForce", &ComMotionTask::getSensedForce)
		.def("getSensedMoment", &ComMotionTask::getSensedMoment)
		.def("getTaskNullspace", &ComMotionTask::getTaskNullspace)
		.def("getPreviousTasksNullspace", &ComMotionTask::getPreviousTasksNullspace)
		.def("getTaskAndPreviousNullspace",
			 &ComMotionTask::getTaskAndPreviousNullspace)
		.def("setGoalPosition", &ComMotionTask::setGoalPosition,
			 py::arg("goal_position"))
		.def("getGoalPosition", &ComMotionTask::getGoalPosition)
		.def("setGoalOrientation", &ComMotionTask::setGoalOrientation,
			 py::arg("goal_orientation"))
		.def("getGoalOrientation", &ComMotionTask::getGoalOrientation)
		.def("setGoalLinearVelocity", &ComMotionTask::setGoalLinearVelocity,
			 py::arg("goal_linvel"))
		.def("getGoalLinearVelocity", &ComMotionTask::getGoalLinearVelocity)
		.def("setGoalAngularVelocity", &ComMotionTask::setGoalAngularVelocity,
			 py::arg("goal_angvel"))
		.def("getGoalAngularVelocity", &ComMotionTask::getGoalAngularVelocity)
		.def("setGoalLinearAcceleration", &ComMotionTask::setGoalLinearAcceleration,
			 py::arg("goal_linaccel"))
		.def("getGoalLinearAcceleration", &ComMotionTask::getGoalLinearAcceleration)
		.def("setGoalAngularAcceleration",
			 &ComMotionTask::setGoalAngularAcceleration, py::arg("goal_angaccel"))
		.def("getGoalAngularAcceleration",
			 &ComMotionTask::getGoalAngularAcceleration)
		.def("getUnitMassForce", &ComMotionTask::getUnitMassForce)
		.def("getPositionError", &ComMotionTask::getPositionError)
		.def("getOrientationError", &ComMotionTask::getOrientationError)
		.def("getLinearVelocityError", &ComMotionTask::getLinearVelocityError)
		.def("getAngularVelocityError", &ComMotionTask::getAngularVelocityError)
		.def("setPosControlGains",
			 py::overload_cast<double, double, double>(
				 &ComMotionTask::setPosControlGains),
			 py::arg("kp_pos"), py::arg("kv_pos"), py::arg("ki_pos") = 0.0)
		.def("setPosControlGains",
			 py::overload_cast<const Eigen::Vector3d&, const Eigen::Vector3d&,
							   const Eigen::Vector3d&>(
				 &ComMotionTask::setPosControlGains),
			 py::arg("kp_pos"), py::arg("kv_pos"), py::arg("ki_pos"))
		.def("setPosControlGains",
			 py::overload_cast<const Eigen::VectorXd&, const Eigen::VectorXd&,
							   const Eigen::VectorXd&>(
				 &ComMotionTask::setPosControlGains),
			 py::arg("kp_pos"), py::arg("kv_pos"), py::arg("ki_pos"))
		.def("getPosControlGains", &ComMotionTask::getPosControlGains)
		.def("setOriControlGains",
			 py::overload_cast<double, double, double>(
				 &ComMotionTask::setOriControlGains),
			 py::arg("kp_ori"), py::arg("kv_ori"), py::arg("ki_ori") = 0.0)
		.def("setOriControlGains",
			 py::overload_cast<const Eigen::Vector3d&, const Eigen::Vector3d&,
							   const Eigen::Vector3d&>(
				 &ComMotionTask::setOriControlGains),
			 py::arg("kp_ori"), py::arg("kv_ori"), py::arg("ki_ori"))
		.def("setOriControlGains",
			 py::overload_cast<const Eigen::VectorXd&, const Eigen::VectorXd&,
							   const Eigen::VectorXd&>(
				 &ComMotionTask::setOriControlGains),
			 py::arg("kp_ori"), py::arg("kv_ori"), py::arg("ki_ori"))
		.def("getOriControlGains", &ComMotionTask::getOriControlGains)
		.def("setForceControlGains", py::overload_cast<double, double, double>(
										 &ComMotionTask::setForceControlGains),
			 py::arg("kp_force"), py::arg("kv_force"), py::arg("ki_force"))
		.def("getForceControlGains", &ComMotionTask::getForceControlGains)
		.def("setMomentControlGains", py::overload_cast<double, double, double>(
										  &ComMotionTask::setMomentControlGains),
			 py::arg("kp_moment"), py::arg("kv_moment"), py::arg("ki_moment"))
		.def("getMomentControlGains", &ComMotionTask::getMomentControlGains)
		.def("setGoalForce", &ComMotionTask::setGoalForce, py::arg("goal_force"))
		.def("getGoalForce", &ComMotionTask::getGoalForce)
		.def("setGoalMoment", &ComMotionTask::setGoalMoment,
			 py::arg("goal_moment"))
		.def("getGoalMoment", &ComMotionTask::getGoalMoment)
		.def("enableInternalOtgAccelerationLimited",
			 &ComMotionTask::enableInternalOtgAccelerationLimited,
			 py::arg("max_linear_velelocity"),
			 py::arg("max_linear_acceleration"), py::arg("max_angular_velocity"),
			 py::arg("max_angular_acceleration"))
		.def("enableInternalOtgJerkLimited",
			 &ComMotionTask::enableInternalOtgJerkLimited,
			 py::arg("max_linear_velelocity"),
			 py::arg("max_linear_acceleration"), py::arg("max_linear_jerk"),
			 py::arg("max_angular_velocity"),
			 py::arg("max_angular_acceleration"), py::arg("max_angular_jerk"))
		.def("disableInternalOtg", &ComMotionTask::disableInternalOtg)
		.def("getInternalOtgEnabled", &ComMotionTask::getInternalOtgEnabled)
		.def("enableVelocitySaturation", &ComMotionTask::enableVelocitySaturation,
			 py::arg("linear_vel_sat") = 0.3,
			 py::arg("angular_vel_sat") = M_PI / 3.0)
		.def("disableVelocitySaturation", &ComMotionTask::disableVelocitySaturation)
		.def("getVelocitySaturationEnabled",
			 &ComMotionTask::getVelocitySaturationEnabled)
		.def("getLinearSaturationVelocity",
			 &ComMotionTask::getLinearSaturationVelocity)
		.def("getAngularSaturationVelocity",
			 &ComMotionTask::getAngularSaturationVelocity)
		.def("goalPositionReached", &ComMotionTask::goalPositionReached,
			 py::arg("tolerance"), py::arg("verbose") = false)
		.def("goalOrientationReached", &ComMotionTask::goalOrientationReached,
			 py::arg("tolerance"), py::arg("verbose") = false)
		.def("setForceSensorFrame", &ComMotionTask::setForceSensorFrame,
			 py::arg("link_name"), py::arg("transformation_in_link"))
		.def("updateSensedForceAndMoment",
			 &ComMotionTask::updateSensedForceAndMoment,
			 py::arg("sensed_force_sensor_frame"),
			 py::arg("sensed_moment_sensor_frame"))
		.def("parametrizeForceMotionSpaces",
			 &ComMotionTask::parametrizeForceMotionSpaces,
			 py::arg("force_space_dimension"),
			 py::arg("force_or_motion_single_axis"))
		.def("getForceSpaceDimension", &ComMotionTask::getForceSpaceDimension)
		.def("getForceMotionSingleAxis", &ComMotionTask::getForceMotionSingleAxis)
		.def("parametrizeMomentRotMotionSpaces",
			 &ComMotionTask::parametrizeMomentRotMotionSpaces,
			 py::arg("moment_space_dimension"),
			 py::arg("moment_or_rot_motion_single_axis") =
				 Eigen::Vector3d::Zero())
		.def("getMomentSpaceDimension", &ComMotionTask::getMomentSpaceDimension)
		.def("getMomentRotMotionSingleAxis",
			 &ComMotionTask::getMomentRotMotionSingleAxis)
		.def("sigmaForce", &ComMotionTask::sigmaForce)
		.def("sigmaPosition", &ComMotionTask::sigmaPosition)
		.def("sigmaMoment", &ComMotionTask::sigmaMoment)
		.def("sigmaOrientation", &ComMotionTask::sigmaOrientation)
		.def("setClosedLoopForceControl", &ComMotionTask::setClosedLoopForceControl,
			 py::arg("closed_loop_force_control") = true)
		.def("setClosedLoopMomentControl",
			 &ComMotionTask::setClosedLoopMomentControl,
			 py::arg("closed_loop_moment_control") = true)
		.def("enablePassivity", &ComMotionTask::enablePassivity)
		.def("disablePassivity", &ComMotionTask::disablePassivity)
		.def("resetIntegrators", &ComMotionTask::resetIntegrators)
		.def("resetIntegratorsLinear", &ComMotionTask::resetIntegratorsLinear)
		.def("resetIntegratorsAngular", &ComMotionTask::resetIntegratorsAngular)
		.def("posSelectionProjector", &ComMotionTask::posSelectionProjector)
		.def("oriSelectionProjector", &ComMotionTask::oriSelectionProjector)
		.def("getUnitControlForces", &ComMotionTask::getUnitControlForces)
		.def("setDynamicDecouplingType", &ComMotionTask::setDynamicDecouplingType,
			 py::arg("type"))
		.def("setBieThreshold", &ComMotionTask::setBieThreshold, py::arg("val"))
		.def("setSingularityThreshold", &ComMotionTask::setSingularityThreshold,
			 py::arg("val"));

	py::class_<RobotController, std::shared_ptr<RobotController>>(
		m, "RobotController")
		.def(py::init([&](const py::object& robot_obj,
						 std::vector<std::shared_ptr<TemplateTask>> tasks) {
				 auto robot = robot_from_py(robot_obj);
				 return std::make_shared<RobotController>(robot, tasks);
			 }),
			 py::arg("robot"), py::arg("tasks"))
		.def("updateControllerTaskModels",
			 &RobotController::updateControllerTaskModels)
		.def("computeControlTorques", &RobotController::computeControlTorques)
		.def("enableGravityCompensation",
			 &RobotController::enableGravityCompensation,
			 py::arg("enable_gravity_compensation"))
		.def("enableJointLimitAvoidance",
			 &RobotController::enableJointLimitAvoidance,
			 py::arg("enable_joint_limit_avoidance"))
		.def("enableTorqueSaturation", &RobotController::enableTorqueSaturation,
			 py::arg("enable_torque_saturation"))
		.def("reinitializeTasks", &RobotController::reinitializeTasks)
		.def("getJointTaskByName", &RobotController::getJointTaskByName,
			 py::arg("task_name"))
		.def("getMotionForceTaskByName",
			 &RobotController::getMotionForceTaskByName, py::arg("task_name"))
		.def("getCentroidalAngularMomentumTaskByName",
			 &RobotController::getCentroidalAngularMomentumTaskByName,
			 py::arg("task_name"))
		.def("getCentroidalLinearMomentumTaskByName",
			 &RobotController::getCentroidalLinearMomentumTaskByName,
			 py::arg("task_name"))
		.def("getMomentumTaskByName", &RobotController::getMomentumTaskByName,
			 py::arg("task_name"))
		.def("getTaskNames", &RobotController::getTaskNames);

	py::enum_<HapticControlType>(m, "HapticControlType")
		.value("HOMING", HapticControlType::HOMING)
		.value("CLUTCH", HapticControlType::CLUTCH)
		.value("MOTION_MOTION", HapticControlType::MOTION_MOTION)
		.value("FORCE_MOTION", HapticControlType::FORCE_MOTION)
		.export_values();

	py::class_<HapticControllerInput>(m, "HapticControllerInput")
		.def(py::init<>())
		.def_readwrite("device_position", &HapticControllerInput::device_position)
		.def_readwrite("device_orientation",
					   &HapticControllerInput::device_orientation)
		.def_readwrite("device_linear_velocity",
					   &HapticControllerInput::device_linear_velocity)
		.def_readwrite("device_angular_velocity",
					   &HapticControllerInput::device_angular_velocity)
		.def_readwrite("robot_position", &HapticControllerInput::robot_position)
		.def_readwrite("robot_orientation", &HapticControllerInput::robot_orientation)
		.def_readwrite("robot_linear_velocity",
					   &HapticControllerInput::robot_linear_velocity)
		.def_readwrite("robot_angular_velocity",
					   &HapticControllerInput::robot_angular_velocity)
		.def_readwrite("robot_sensed_force",
					   &HapticControllerInput::robot_sensed_force)
		.def_readwrite("robot_sensed_moment",
					   &HapticControllerInput::robot_sensed_moment);

	py::class_<HapticControllerOutput>(m, "HapticControllerOutput")
		.def(py::init<>())
		.def_readwrite("robot_goal_position",
					   &HapticControllerOutput::robot_goal_position)
		.def_readwrite("robot_goal_orientation",
					   &HapticControllerOutput::robot_goal_orientation)
		.def_readwrite("device_command_force",
					   &HapticControllerOutput::device_command_force)
		.def_readwrite("device_command_moment",
					   &HapticControllerOutput::device_command_moment);

	py::class_<HapticDeviceController::DeviceLimits>(m, "DeviceLimits")
		.def(py::init<const Eigen::Vector3d&, const Eigen::Vector3d&,
					  const Eigen::Vector3d&>(),
			 py::arg("max_stiffness"), py::arg("max_damping"),
			 py::arg("max_force_torque"))
		.def_readwrite("max_linear_stiffness",
					   &HapticDeviceController::DeviceLimits::max_linear_stiffness)
		.def_readwrite("max_angular_stiffness",
					   &HapticDeviceController::DeviceLimits::max_angular_stiffness)
		.def_readwrite(
			"max_gripper_stiffness",
			&HapticDeviceController::DeviceLimits::max_gripper_stiffness)
		.def_readwrite("max_linear_damping",
					   &HapticDeviceController::DeviceLimits::max_linear_damping)
		.def_readwrite("max_angular_damping",
					   &HapticDeviceController::DeviceLimits::max_angular_damping)
		.def_readwrite("max_gripper_damping",
					   &HapticDeviceController::DeviceLimits::max_gripper_damping)
		.def_readwrite("max_force", &HapticDeviceController::DeviceLimits::max_force)
		.def_readwrite("max_torque",
					   &HapticDeviceController::DeviceLimits::max_torque)
		.def_readwrite("max_gripper_force",
					   &HapticDeviceController::DeviceLimits::max_gripper_force);

	py::class_<HapticDeviceController, std::shared_ptr<HapticDeviceController>>(
		m, "HapticDeviceController")
		.def(py::init<const HapticDeviceController::DeviceLimits&,
					  const Eigen::Affine3d&, const Eigen::Affine3d&,
					  const Eigen::Matrix3d&>(),
			 py::arg("device_limits"), py::arg("robot_initial_pose"),
			 py::arg("device_home_pose"),
			 py::arg("device_base_rotation_in_world") =
				 Eigen::Matrix3d::Identity())
		.def("computeHapticControl", &HapticDeviceController::computeHapticControl,
			 py::arg("input"), py::arg("verbose") = false)
		.def("getDeviceLimits", &HapticDeviceController::getDeviceLimits,
			 py::return_value_policy::reference_internal)
		.def("getLatestOutput", &HapticDeviceController::getLatestOutput,
			 py::return_value_policy::reference_internal)
		.def("getLatestInput", &HapticDeviceController::getLatestInput,
			 py::return_value_policy::reference_internal)
		.def("getRotationWorldToDeviceBase",
			 &HapticDeviceController::getRotationWorldToDeviceBase,
			 py::return_value_policy::reference_internal)
		.def("setHapticControlType", &HapticDeviceController::setHapticControlType,
			 py::arg("haptic_control_type"))
		.def("getHapticControlType", &HapticDeviceController::getHapticControlType,
			 py::return_value_policy::reference_internal)
		.def("enableOrientationTeleop",
			 &HapticDeviceController::enableOrientationTeleop)
		.def("disableOrientationTeleop",
			 &HapticDeviceController::disableOrientationTeleop)
		.def("getOrientationTeleopEnabled",
			 &HapticDeviceController::getOrientationTeleopEnabled)
		.def("getHomed", &HapticDeviceController::getHomed)
		.def("parametrizeProxyForceFeedbackSpace",
			 &HapticDeviceController::parametrizeProxyForceFeedbackSpace,
			 py::arg("proxy_feedback_space_dimension"),
			 py::arg("proxy_or_direct_feedback_axis"))
		.def("parametrizeProxyForceFeedbackSpaceFromRobotForceSpace",
			 &HapticDeviceController::
				 parametrizeProxyForceFeedbackSpaceFromRobotForceSpace,
			 py::arg("robot_sigma_force"))
		.def("parametrizeProxyMomentFeedbackSpace",
			 &HapticDeviceController::parametrizeProxyMomentFeedbackSpace,
			 py::arg("proxy_feedback_space_dimension"),
			 py::arg("proxy_or_direct_feedback_axis"))
		.def("parametrizeProxyMomentFeedbackSpaceFromRobotForceSpace",
			 &HapticDeviceController::
				 parametrizeProxyMomentFeedbackSpaceFromRobotForceSpace,
			 py::arg("robot_sigma_moment"))
		.def("getSigmaProxyForce", &HapticDeviceController::getSigmaProxyForce,
			 py::return_value_policy::reference_internal)
		.def("getSigmaDirectForceFeedback",
			 &HapticDeviceController::getSigmaDirectForceFeedback)
		.def("getSigmaProxyMoment", &HapticDeviceController::getSigmaProxyMoment,
			 py::return_value_policy::reference_internal)
		.def("getSigmaDirectMomentFeedback",
			 &HapticDeviceController::getSigmaDirectMomentFeedback)
		.def("setScalingFactors", &HapticDeviceController::setScalingFactors,
			 py::arg("scaling_factor_pos"), py::arg("scaling_factor_ori") = 1.0)
		.def("getScalingFactorPos", &HapticDeviceController::getScalingFactorPos)
		.def("getScalingFactorOri", &HapticDeviceController::getScalingFactorOri)
		.def("setReductionFactorForce",
			 &HapticDeviceController::setReductionFactorForce,
			 py::arg("reduction_factor_force"))
		.def("setReductionFactorMoment",
			 &HapticDeviceController::setReductionFactorMoment,
			 py::arg("reduction_factor_moment"))
		.def("setDeviceControlGains",
			 py::overload_cast<const double, const double>(
				 &HapticDeviceController::setDeviceControlGains),
			 py::arg("kp_pos"), py::arg("kv_pos"))
		.def("setDeviceControlGains",
			 py::overload_cast<const double, const double, const double,
							   const double>(
				 &HapticDeviceController::setDeviceControlGains),
			 py::arg("kp_pos"), py::arg("kv_pos"), py::arg("kp_ori"),
			 py::arg("kv_ori"))
		.def("setHapticGuidanceGains",
			 py::overload_cast<const double, const double>(
				 &HapticDeviceController::setHapticGuidanceGains),
			 py::arg("kp_guidance_pos"), py::arg("kv_guidance_pos"))
		.def("setHapticGuidanceGains",
			 py::overload_cast<const double, const double, const double,
							   const double>(
				 &HapticDeviceController::setHapticGuidanceGains),
			 py::arg("kp_guidance_pos"), py::arg("kv_guidance_pos"),
			 py::arg("kp_guidance_ori"), py::arg("kv_guidance_ori"))
		.def("enablePlaneGuidance",
			 py::overload_cast<const Eigen::Vector3d, const Eigen::Vector3d>(
				 &HapticDeviceController::enablePlaneGuidance),
			 py::arg("plane_origin_point"), py::arg("plane_normal_direction"))
		.def("enablePlaneGuidance",
			 py::overload_cast<>(&HapticDeviceController::enablePlaneGuidance))
		.def("disablePlaneGuidance", &HapticDeviceController::disablePlaneGuidance)
		.def("getPlaneGuidanceEnabled",
			 &HapticDeviceController::getPlaneGuidanceEnabled)
		.def("enableLineGuidance",
			 py::overload_cast<const Eigen::Vector3d, const Eigen::Vector3d>(
				 &HapticDeviceController::enableLineGuidance),
			 py::arg("line_origin_point"), py::arg("line_direction"))
		.def("enableLineGuidance",
			 py::overload_cast<>(&HapticDeviceController::enableLineGuidance))
		.def("disableLineGuidance", &HapticDeviceController::disableLineGuidance)
		.def("getLineGuidanceEnabled", &HapticDeviceController::getLineGuidanceEnabled)
		.def("enableHapticWorkspaceVirtualLimits",
			 py::overload_cast<double, double>(
				 &HapticDeviceController::enableHapticWorkspaceVirtualLimits),
			 py::arg("device_workspace_radius_limit"),
			 py::arg("device_workspace_angle_limit"))
		.def("enableHapticWorkspaceVirtualLimits",
			 py::overload_cast<>(
				 &HapticDeviceController::enableHapticWorkspaceVirtualLimits))
		.def("disableHapticWorkspaceVirtualLimits",
			 &HapticDeviceController::disableHapticWorkspaceVirtualLimits)
		.def("getHapticWorkspaceVirtualLimitsEnabled",
			 &HapticDeviceController::getHapticWorkspaceVirtualLimitsEnabled)
		.def("setVariableDampingGainsPos",
			 &HapticDeviceController::setVariableDampingGainsPos,
			 py::arg("velocity_thresholds"), py::arg("variable_damping_gains"))
		.def("setVariableDampingGainsOri",
			 &HapticDeviceController::setVariableDampingGainsOri,
			 py::arg("velocity_thresholds"), py::arg("variable_damping_gains"))
		.def("setAdmittanceFactors", &HapticDeviceController::setAdmittanceFactors,
			 py::arg("device_force_to_robot_delta_position"),
			 py::arg("device_moment_to_robot_delta_orientation"))
		.def("setHomingMaxVelocity", &HapticDeviceController::setHomingMaxVelocity,
			 py::arg("homing_max_linvel"), py::arg("homing_max_angvel"))
		.def("setForceDeadbandForceMotionController",
			 &HapticDeviceController::setForceDeadbandForceMotionController,
			 py::arg("force_deadband"))
		.def("setMomentDeadbandForceMotionController",
			 &HapticDeviceController::setMomentDeadbandForceMotionController,
			 py::arg("force_deadband"));

	py::class_<POPCBilateralTeleoperation,
			   std::shared_ptr<POPCBilateralTeleoperation>>(
		m, "POPCBilateralTeleoperation")
		.def(py::init<const std::shared_ptr<MotionForceTask>&,
					  const std::shared_ptr<HapticDeviceController>&, const double>(),
			 py::arg("motion_force_task"), py::arg("haptic_controller"),
			 py::arg("loop_dt"))
		.def("reInitialize", &POPCBilateralTeleoperation::reInitialize)
		.def("computeAdditionalHapticDampingForce",
			 &POPCBilateralTeleoperation::computeAdditionalHapticDampingForce);
}
