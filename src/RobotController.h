/**
 * RobotController.h
 *
 *	A controller class for a single robot that takes a list of ordered tasks and
 *compute the control torques for the robot
 *
 * Author: Mikael Jorda
 * Created: September 2023
 */

#ifndef SAI_PRIMITIVES_ROBOT_CONTROLLER_H_
#define SAI_PRIMITIVES_ROBOT_CONTROLLER_H_

#include <memory>
#include <vector>

#include "tasks/JointLimitAvoidanceTask.h"
#include "tasks/JointTask.h"
#include "tasks/MotionForceTask.h"
#include "tasks/TemplateTask.h"

namespace SaiPrimitives {

/**
 * @brief This calss implements a controller composed of a list of tasks in a
 * hierarchical order. It has options to compute the gravity compensation, joint
 * limit avoidance and torque saturation if required.
 *
 */
class RobotController {
public:
	struct DefaultParameters {
		static constexpr bool enable_gravity_compensation = false;
		static constexpr bool enable_joint_limit_avoidance = false;
		static constexpr bool enable_torque_saturation = false;
	};

	/**
	 * @brief Construct a new Robot Controller object
	 *
	 * @param robot A pointer to a SaiModel object for the robot that is to be
	 * controlled
	 * @param tasks A vector of shared pointers to the tasks that are to be
	 * controlled. The order of the tasks in the vector represents the
	 * hierarchical order.
	 */
	RobotController(std::shared_ptr<SaiModel::SaiModel>& robot,
					std::vector<std::shared_ptr<TemplateTask>>& tasks);

	/**
	 * @brief Updates the task models for all tasks in the controller (the robot
	 * model needs to be updated externally)
	 *
	 */
	void updateControllerTaskModels();

	/**
	 * @brief Computes the control torques for that robot controller (the robot
	 * model needs to be updated externally)
	 *
	 * @return Eigen::VectorXd the control torques
	 */
	Eigen::VectorXd computeControlTorques();

	/// @brief enable or disable gravity compensation
	void enableGravityCompensation(const bool enable_gravity_compensation) {
		_enable_gravity_compensation = enable_gravity_compensation;
	}
	/// @brief enable or disable the joint limit avoidance task
	void enableJointLimitAvoidance(const bool enable_joint_limit_avoidance) {
		_enable_joint_limit_avoidance = enable_joint_limit_avoidance;
	}
	/// @brief enable or disable the torque saturation
	void enableTorqueSaturation(const bool enable_torque_saturation) {
		_enable_torque_saturation = enable_torque_saturation;
	}

	/// @brief re initialize all the tasks to the current robot state
	void reinitializeTasks();

	/// @brief get a joint task by name, throws an error if no joint task with
	/// that name exists
	std::shared_ptr<JointTask> getJointTaskByName(const std::string& task_name);
	/// @brief get a motion force task by name, throws an error if no motion
	/// force task with that name exists
	std::shared_ptr<MotionForceTask> getMotionForceTaskByName(
		const std::string& task_name);

	/// @brief get the task names in the order they are stored in the controller
	const std::vector<std::string>& getTaskNames() const { return _task_names; }

private:
	/// @brief pointer to the robot model
	std::shared_ptr<SaiModel::SaiModel> _robot;
	/// @brief vector of tasks in the controller
	std::vector<std::shared_ptr<TemplateTask>> _tasks;
	/// @brief vector of task names in the order they are stored in the controller
	std::vector<std::string> _task_names;
	/// @brief flag to enable gravity compensation
	bool _enable_gravity_compensation;
	/// @brief flag to enable joint limit avoidance
	bool _enable_joint_limit_avoidance;
	/// @brief flag to enable torque saturation
	bool _enable_torque_saturation;
	/// @brief pointer to the joint limit avoidance task
	std::unique_ptr<JointLimitAvoidanceTask> _joint_limit_avoidance_task;
	/// @brief nullspace of the joint limit avoidance task if active
	MatrixXd _N_constraints;
	/// @brief vector of torque limits for each joint (assumes the torque limts are symmetric)
	VectorXd _torque_limits;
};

} /* namespace SaiPrimitives */

#endif /* SAI_PRIMITIVES_ROBOT_CONTROLLER_H_ */