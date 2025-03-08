/*
 * Example of singularity handling by smoothing Lambda in/out of singularities.
 */

#include <math.h>
#include <signal.h>

#include <iostream>
#include <mutex>
#include <string>
#include <thread>

#include "Sai2Graphics.h"
#include "Sai2Model.h"
#include "Sai2Simulation.h"
#include "tasks/JointTask.h"
#include "tasks/MotionForceTask.h"
#include "tasks/JointHandler.h"
#include "timer/LoopTimer.h"
#include "logger/Logger.h"
bool fSimulationRunning = false;
void sighandler(int) { fSimulationRunning = false; }

using namespace std;
using namespace Eigen;
using namespace chai3d;

// config file names and object names
const string world_file = "${EXAMPLE_98_FOLDER}/world.urdf";
const string robot_file = "${EXAMPLE_98_FOLDER}/ppbot.urdf";
const string robot_name = "PPBOT";

// ui torques and control torques
VectorXd ui_torques;
VectorXd control_torques;

// mutex for global variables between different threads
mutex mutex_torques;
mutex mutex_robot;

// ellipsoid information 
Vector2d ellipsoid_center = Vector2d(1, 1);
Vector2d ellipsoid_radii = Vector2d(0.5, 0.25);

chai3d::cShapeLine* force_line;
chai3d::cShapeLine* nullspace_force_line;

// simulation and control loop
void control(shared_ptr<Sai2Model::Sai2Model> robot,
			 shared_ptr<Sai2Simulation::Sai2Simulation> sim);
void simulation(shared_ptr<Sai2Model::Sai2Model> robot,
				shared_ptr<Sai2Simulation::Sai2Simulation> sim);

// Function to calculate the 2D distance from point P to the capsule
// and return the direction vector to the closest point
std::pair<double, Eigen::Vector2d> distanceToCapsule(const Eigen::Vector2d& P, const Eigen::Vector2d& A, const Eigen::Vector2d& B, double r) {
    // Vector AB and AP
    Eigen::Vector2d AB = B - A;
    Eigen::Vector2d AP = P - A;

    // Compute the projection scalar t of AP onto AB
    double AB_dot_AB = AB.dot(AB);
    double AP_dot_AB = AP.dot(AB);
    double t = AP_dot_AB / AB_dot_AB;

    // Variables to store the closest point and direction vector
    Eigen::Vector2d closestPoint;
    Eigen::Vector2d direction;

    // If t is between 0 and 1, the perpendicular projection is within the segment
    if (t >= 0 && t <= 1) {
        // Perpendicular distance from P to the line AB
        double crossProduct = std::abs(AP.x() * AB.y() - AP.y() * AB.x());
        double distance = crossProduct / AB.norm();
        closestPoint = A + t * AB;  // Closest point on the line segment
        direction = (P - closestPoint).normalized();  // Direction vector from P to closest point
        return {std::max(distance - r, 0.0), direction};
    } else {
        // If t is outside [0, 1], compute the distance to the nearest endpoint (A or B)
        if (t < 0) {
            closestPoint = A;
        } else {
            closestPoint = B;
        }
        direction = (P - closestPoint).normalized();  // Direction vector from P to closest point
        return {(P - closestPoint).norm() - r, direction};
    }
}

// Function to compute the closest distance and direction to an ellipse centered at (cx, cy)
std::pair<double, Eigen::Vector2d> distanceToEllipse(double x0, double y0, double cx, double cy, double a, double b, int max_iter = 100, double tol = 1e-6) {
    // Transform point to ellipse's local frame
    double px = x0 - cx;
    double py = y0 - cy;

    // Initial guess for theta
    double theta = atan2(py * a, px * b);

    for (int i = 0; i < max_iter; ++i) {
        double cosT = cos(theta), sinT = sin(theta);
        double x = a * cosT, y = b * sinT;

        // Compute function and its derivative
        double fx = (x - px) * (-a * sinT) + (y - py) * (b * cosT);
        double dfx = (-a * cosT) * (-a * sinT) + (b * sinT) * (b * cosT);

        // Newton update
        double theta_new = theta - fx / dfx;

        // Convergence check
        if (std::abs(theta_new - theta) < tol) {
            theta = theta_new;
            break;
        }
        theta = theta_new;
    }

    // Closest point on the ellipse (convert back to world frame)
    double x_closest = cx + a * cos(theta);
    double y_closest = cy + b * sin(theta);

    // Compute distance
    double distance = std::sqrt((x_closest - x0) * (x_closest - x0) + (y_closest - y0) * (y_closest - y0));

    // Compute direction (unit vector)
    Eigen::Vector2d direction(x_closest - x0, y_closest - y0);
    if (distance > 1e-10) {  // Avoid division by zero
        direction.normalize();
    }

    return {distance, direction};
}

//------------ main function
int main(int argc, char** argv) {
	Sai2Model::URDF_FOLDERS["EXAMPLE_98_FOLDER"] =
		string(EXAMPLES_FOLDER) + "/98-nullspace";
	cout << "Loading URDF world model file: " << world_file << endl;

	// set up signal handler
	signal(SIGABRT, &sighandler);
	signal(SIGTERM, &sighandler);
	signal(SIGINT, &sighandler);

	// load graphics scene
	auto graphics = make_shared<Sai2Graphics::Sai2Graphics>(world_file);
	graphics->addUIForceInteraction(robot_name);
	// graphics->showTransparency(true, robot_name, 0.5);

	// load simulation world
	auto sim = make_shared<Sai2Simulation::Sai2Simulation>(world_file);

	// load robots
	auto robot = make_shared<Sai2Model::Sai2Model>(robot_file, false);
	robot->setQ(sim->getJointPositions(robot_name));
	robot->updateModel();

	// sim->setJointPositions(robot_name, 0 * robot->q());

	// intitialize global torques variables
	ui_torques = VectorXd::Zero(robot->dof());
	control_torques = VectorXd::Zero(robot->dof());

	// Vector2d capsule_a = ellipsoid_center - Vector2d(ellipsoid_radii(0), 0) - Vector2d(ellipsoid_radii(1), 0);
    // Vector2d capsule_b = ellipsoid_center + Vector2d(ellipsoid_radii(0), 0) - Vector2d(ellipsoid_radii(1), 0);
    // double capsule_radius = ellipsoid_radii(1);

	// add capsule graphics 
	chai3d::cShapeBox* box = new chai3d::cShapeBox(ellipsoid_radii(0) * 2, ellipsoid_radii(1) * 2, 0.001);
	box->setLocalPos(chai3d::cVector3d(ellipsoid_center(0) - ellipsoid_radii(0) / 2, ellipsoid_center(1), ellipsoid_center(2)));
	box->m_material->setBlue();
	box->setTransparencyLevel(0.2);
	graphics->_world->addChild(box);

	chai3d::cShapeSphere* right_sphere = new chai3d::cShapeSphere(ellipsoid_radii(1));
	right_sphere->setLocalPos(chai3d::cVector3d(ellipsoid_center(0) - ellipsoid_radii(0) / 2 + ellipsoid_radii(0), ellipsoid_center(1), ellipsoid_center(2)));
	right_sphere->m_material->setBlue();
	right_sphere->setTransparencyLevel(0.2);
	graphics->_world->addChild(right_sphere);

	chai3d::cShapeSphere* left_sphere = new chai3d::cShapeSphere(ellipsoid_radii(1));
	left_sphere->setLocalPos(chai3d::cVector3d(ellipsoid_center(0) - ellipsoid_radii(0) / 2 - ellipsoid_radii(0), ellipsoid_center(1), ellipsoid_center(2)));
	left_sphere->m_material->setBlue();
	left_sphere->setTransparencyLevel(0.2);
	graphics->_world->addChild(left_sphere);

	// outer
	chai3d::cShapeBox* outer_box = new chai3d::cShapeBox(ellipsoid_radii(0) * 2 + 0.6, ellipsoid_radii(1) * 2 + 0.6, 0.001);
	outer_box->setLocalPos(chai3d::cVector3d(ellipsoid_center(0) - ellipsoid_radii(0) / 2, ellipsoid_center(1), ellipsoid_center(2)));
	outer_box->m_material->setGreen();
	outer_box->setTransparencyLevel(0.2);
	graphics->_world->addChild(outer_box);

	chai3d::cShapeSphere* outer_right_sphere = new chai3d::cShapeSphere(ellipsoid_radii(1) + 0.3);
	outer_right_sphere->setLocalPos(chai3d::cVector3d(ellipsoid_center(0) - ellipsoid_radii(0) / 2 + 0.3 + ellipsoid_radii(0), ellipsoid_center(1), ellipsoid_center(2)));
	outer_right_sphere->m_material->setGreen();
	outer_right_sphere->setTransparencyLevel(0.2);
	graphics->_world->addChild(outer_right_sphere);

	chai3d::cShapeSphere* outer_left_sphere = new chai3d::cShapeSphere(ellipsoid_radii(1) + 0.3);
	outer_left_sphere->setLocalPos(chai3d::cVector3d(ellipsoid_center(0) - ellipsoid_radii(0) / 2 - 0.3 - ellipsoid_radii(0), ellipsoid_center(1), ellipsoid_center(2)));
	outer_left_sphere->m_material->setGreen();
	outer_left_sphere->setTransparencyLevel(0.2);
	graphics->_world->addChild(outer_left_sphere);

	// add apf force line

	// add task force line 
	force_line = new chai3d::cShapeLine(cVector3d(0, 0, 0), cVector3d(0, 0, 0));
	force_line->m_colorPointA.set(0, 0, 1);  // force line blue 
	force_line->setLineWidth(5);
	graphics->_world->addChild(force_line);

	// add nullspace line 
	nullspace_force_line = new chai3d::cShapeLine(cVector3d(0, 0, 0), cVector3d(0, 0, 0));
	nullspace_force_line->m_colorPointA.set(0, 1, 0);  // nullspace line green 
	nullspace_force_line->setLineWidth(5);
	graphics->_world->addChild(nullspace_force_line);

	// add goal sphere
	chai3d::cShapeSphere* goal_sphere = new chai3d::cShapeSphere(0.1);
	goal_sphere->setLocalPos(chai3d::cVector3d(3, 2, 0));
	goal_sphere->m_material->setRed();
	// outer_left_sphere->setTransparencyLevel(0.2);
	graphics->_world->addChild(goal_sphere);

	// // add ellipsoid graphics 
	// chai3d::cShapeEllipsoid* obstacle = new chai3d::cShapeEllipsoid(ellipsoid_radii(0), ellipsoid_radii(1), 0.001);
	// obstacle->setLocalPos(chai3d::cVector3d(ellipsoid_center(0), ellipsoid_center(1), ellipsoid_center(2)));
	// obstacle->m_material->setBlue();
	// obstacle->setTransparencyLevel(0.2);
	// graphics->_world->addChild(obstacle);

	// chai3d::cShapeEllipsoid* outer_obstacle = new chai3d::cShapeEllipsoid(ellipsoid_radii(0) + 0.3, ellipsoid_radii(1) + 0.3, 0.001);
	// outer_obstacle->setLocalPos(chai3d::cVector3d(ellipsoid_center(0), ellipsoid_center(1), ellipsoid_center(2)));
	// outer_obstacle->m_material->setGreen();
	// outer_obstacle->setTransparencyLevel(0.2);
	// graphics->_world->addChild(outer_obstacle);

	// change background
	graphics->_world->setBackgroundColor(1, 1, 1);

	// start the simulation thread first
	fSimulationRunning = true;
	thread sim_thread(simulation, robot, sim);

	// next start the control thread
	thread ctrl_thread(control, robot, sim);

	// while window is open:
	while (graphics->isWindowOpen()) {
		{
			lock_guard<mutex> lock(mutex_robot);
			graphics->updateRobotGraphics(robot_name, robot->q());
		}
		graphics->renderGraphicsWorld();
		{
			lock_guard<mutex> lock(mutex_torques);
			ui_torques = graphics->getUITorques(robot_name);
		}
	}

	// stop simulation
	fSimulationRunning = false;
	sim_thread.join();
	ctrl_thread.join();

	return 0;
}

//------------------------------------------------------------------------------
void control(shared_ptr<Sai2Model::Sai2Model> robot,
			 shared_ptr<Sai2Simulation::Sai2Simulation> sim) {

	// update robot model and initialize control vectors
	robot->updateModel();
	int dof = robot->dof();
	MatrixXd N_prec = MatrixXd::Identity(dof, dof);

    // Capsule collision
    Vector2d capsule_a = ellipsoid_center - Vector2d(ellipsoid_radii(0), 0) - Vector2d(ellipsoid_radii(1), 0);
    Vector2d capsule_b = ellipsoid_center + Vector2d(ellipsoid_radii(0), 0) - Vector2d(ellipsoid_radii(1), 0);
    double capsule_radius = ellipsoid_radii(1);

	// Position plus orientation task
	string link_name = "link2";
	Vector3d pos_in_link = Vector3d(0, 0, 0);
	// Affine3d compliant_frame = Affine3d(Translation3d(pos_in_link));

	// // Full motion force task
	// auto motion_force_task = make_unique<Sai2Primitives::MotionForceTask>(
	// 	robot, link_name, compliant_frame);

	// Joint task
	auto joint_task = std::make_shared<Sai2Primitives::JointTask>(robot);
	joint_task->disableInternalOtg();
	joint_task->enableVelocitySaturation(0.2);
	joint_task->setDynamicDecouplingType(Sai2Primitives::FULL_DYNAMIC_DECOUPLING);
	VectorXd joint_task_torques = VectorXd::Zero(robot->dof());

	// // Partial motion force task
	// vector<Vector3d> controlled_directions_translation = {
	// 	Vector3d::UnitX(), Vector3d::UnitY()};
	// vector<Vector3d> controlled_directions_rotation = {};
	// auto motion_force_task = make_shared<Sai2Primitives::MotionForceTask>(
	// 	robot, link_name, controlled_directions_translation,
	// 	controlled_directions_rotation, compliant_frame);

    // motion_force_task->disableInternalOtg();
    // motion_force_task->disableSingularityHandling();
    // motion_force_task->enableVelocitySaturation(0.2);
    // motion_force_task->setPosControlGains(100, 20);
    // motion_force_task->setDynamicDecouplingType(Sai2Primitives::FULL_DYNAMIC_DECOUPLING);
	// VectorXd motion_force_task_torques = VectorXd::Zero(dof);

	// // no gains setting here, using the default task values
	// const Matrix3d initial_orientation = robot->rotation(link_name);
	// const Vector3d initial_position = robot->position(link_name, pos_in_link);

	// joint task to control the redundancy
	// using default gains and interpolation settings
	// auto joint_task = make_unique<Sai2Primitives::JointTask>(robot);
    // joint_task->disableInternalOtg();
    // joint_task->enableVelocitySaturation(M_PI);
    // joint_task->setGains(100, 20);
	// VectorXd joint_task_torques = VectorXd::Zero(dof);

	// VectorXd initial_q = robot->q();
    // joint_task->setGoalPosition(Vector3d::Zero());

    // // apf joint limits 
    // auto joint_limits = robot->jointLimits();
    // VectorXd q_min(robot->dof()), q_max(robot->dof());
    // int cnt = 0;
    // for (auto limit : joint_limits) {
    //     q_min(cnt) = limit.position_lower + 0.5;
    //     q_max(cnt) = limit.position_upper - 0.5;
    //     cnt++;
    // }
    // double eta = 0.5;
    // VectorXd q_init = robot->q();
    // VectorXd q_delta = VectorXd::Zero(robot->dof());
    // q_delta(5) = 5;

    // Vector3d x_init = motion_force_task->getCurrentPosition();
	Vector2d x_init = joint_task->getCurrentPosition();
    Vector2d x_delta = Vector2d(1, 2);
	bool first_loop = true;
	bool first_entry = true;
	double zone_width = 0.3 + 0.1;
	double outer_zone_width = zone_width;

	joint_task->setGoalPosition(joint_task->getCurrentPosition() + x_delta);

    // // desired position offsets 
    // vector<Vector3d> desired_offsets {Vector3d(2, 0, 0), Vector3d(0, 0, 0), 
    //                                   Vector3d(0, 2, 0), Vector3d(0, 0, 0), 
    //                                   Vector3d(0, -2, 0), Vector3d(0, 0, 0),
    //                                   Vector3d(0, 0, 2), Vector3d(0, 0, 0)};
    // // vector<Vector3d> desired_offsets {Vector3d(2, 0, 0)};
	// double t_initial = 2;
	// vector<double> t_wait {5, 5};
    // // double t_wait = 10;  // wait between switching desired positions 
	// // double t_reset_wait = 5;  // wait when resetting position 
    // double prev_time = 0;
    // // int cnt = 6 * 1;
	// cnt = 0;
    // int max_cnt = desired_offsets.size();

	// create logger
	Sai2Common::Logger logger("apf", false);
	// VectorXd svalues = VectorXd::Zero(6);
    // VectorXd robot_q = robot->q();
    Vector3d ee_pos = robot->position(link_name, pos_in_link);
    // Vector3d x_curr;
    // Vector3d control_forces = Vector3d::Zero();
    Vector3d constraint_direction = Vector3d::Zero();
    int constraint_flag = 0;
	// // logger.addToLog(svalues, "svalues");
    // logger.addToLog(ee_pos, "ee_pos");
    // logger.addToLog(constraint_flag, "constraint_flag");
    // logger.addToLog(control_forces, "control_forces");
    // logger.addToLog(constraint_direction, "constraint_direction");
	// logger.start();

	// create a loop timer
	double control_freq = 1000;
	Sai2Common::LoopTimer timer(control_freq, 1e6);

	while (fSimulationRunning) {
		timer.waitForNextLoop();
		const double time = timer.elapsedSimTime();

		// update robot 
        {
			lock_guard<mutex> lock(mutex_robot);
			// read joint positions, velocities, update model 
			robot->setQ(sim->getJointPositions(robot_name));
			robot->setDq(sim->getJointVelocities(robot_name));
			robot->updateModel();

        	// robot_q = robot->q();
            // x_curr = robot_q;

			if (first_loop) {
				x_init = robot->q();
				first_loop = false;
			}

			// debug 
			ee_pos = robot->position(link_name, pos_in_link);
	        // ee_pos = x_curr;
			// std::cout << "Initial position: " << x_init.transpose() << "\n";
			// std::cout << "Current position: " << x_curr.transpose() << "\n";
			// std::cout << "Motion task current position: " << motion_force_task->getCurrentPosition().transpose() << "\n";
			// std::cout << "Goal position: " << motion_force_task->getGoalPosition().transpose() << "\n";
        }

		// joint_task->setGoalPosition((x_init + x_delta).head(2));
		// motion_force_task->setGoalPosition(x_init + x_delta);

        // auto [distance, con_dir] = distanceToCapsule(ee_pos.head(2), capsule_a, capsule_b, capsule_radius);
        auto [distance, con_dir] = distanceToCapsule(robot->q(), capsule_a, capsule_b, capsule_radius);
		// auto [distance, con_dir] = distanceToEllipse(ee_pos(0), ee_pos(1), ellipsoid_center(0), ellipsoid_center(1), ellipsoid_radii(0), ellipsoid_radii(1));
        constraint_direction = - Vector3d(con_dir(0), con_dir(1), 0).normalized();

		if (distance < 0) {
			std::cout << "Inside ellipsoid: setting distance to minimum value\n";
			distance = 1e-6;
		}

		std::cout << "constraint direction: " << constraint_direction.transpose() << "\n";
		std::cout << "distance: " << distance << "\n";

		// compute apf forces in direction
		Vector3d apf_force = Vector3d::Zero();
		double eta = 1.0;
		if (distance < zone_width) {
			apf_force = - eta * std::abs(((1 / distance) - (1 / zone_width))) * (1 / (distance * distance)) * constraint_direction;
			if (first_entry) {
				// zone_width *= 2;
				outer_zone_width += 0.01;
				first_entry = false;
			}
		}
		std::cout << "apf force: " << apf_force.transpose() << "\n";

		// update tasks model. Order is important to define the hierarchy
		N_prec.setIdentity();
        // constraint_flag = 0;

		// compute task without nullspace
		// motion_force_task->updateTaskModel(N_prec);
		joint_task->updateTaskModel(N_prec);
		// Vector3d task_forces_without_nullspace = motion_force_task->computeTorques();
		VectorXd task_forces_without_nullspace = joint_task->computeTorques();
		std::cout << "Task forces before nullspace: " << task_forces_without_nullspace.transpose() << "\n";

        // add constraint direction if in zone of influence 
		MatrixXd Jv_con = MatrixXd::Zero(1, robot->dof());
		// VectorXd apf_torques = VectorXd::Zero(robot->dof());

		// if inside zone or past first entry 
		if (distance > outer_zone_width && !first_entry) {

			std::cout << "outside zone width\n";
			// compute control torques with nullspace
			// motion_force_task->updateTaskModel(N_prec);
			N_prec.setIdentity();
			joint_task->updateTaskModel(N_prec);
			// Vector3d task_forces = motion_force_task->computeTorques();
			joint_task_torques = joint_task->computeTorques();
			// transfer torques
			// motion_force_task_torques = task_forces;

			// draw lines (blue = force without nullspace, green is force in nullspace)
			force_line->m_pointA = chai3d::cVector3d(ee_pos);
			force_line->m_pointB = chai3d::cVector3d(ee_pos) + 0.5 * chai3d::cVector3d(task_forces_without_nullspace(0), \
																							task_forces_without_nullspace(1), 0);

			nullspace_force_line->m_pointA = chai3d::cVector3d(ee_pos);
			auto task_forces_normalized = joint_task_torques.normalized();
			nullspace_force_line->m_pointB = chai3d::cVector3d(ee_pos) + 0.5 * chai3d::cVector3d(task_forces_normalized(0), task_forces_normalized(1), 0);
		} else if (!first_entry) {
        // if (distance < zone_width) {
		// if (!first_entry) {
            // constraint_flag = 1;
            {
			    lock_guard<mutex> lock(mutex_robot);
				
				// if (constraint_direction.dot(task_forces_without_nullspace) > 0) {
	                // Jv_con = constraint_direction.transpose() * robot->Jv(link_name, pos_in_link);
					// N_prec = robot->nullspaceMatrix(Jv_con);
				if (!first_entry) {
					Jv_con = constraint_direction.transpose() * robot->Jv(link_name, pos_in_link);
					// std::cout << "Jv_con: " << Jv_con << "\n";
					// std::cout << robot->Jv(link_name, pos_in_link) << "\n";
					// Jv_con = constraint_direction.head(2).transpose();
					N_prec = robot->nullspaceMatrix(Jv_con);
					// MatrixXd J_task = robot->Jv(link_name, pos_in_link);
					// MatrixXd J_bar = robot->MInv() * J_task.transpose() * robot->taskInertiaMatrix(J_task);
					// N_prec = MatrixXd::Identity(2, 2) - J_bar * J_task;
					// std::cout << "Nullspace: \n" << N_prec << "\n";

					// std::cout << "projected task jacobian: \n" << robot->Jv(link_name, pos_in_link) * N_prec << "\n";

				// 	// check if distance is outside new distance; if it is, then reset first entry
				// 	if (distance > outer_zone_width) {
				// 		first_entry = true;
				// 		// zone_width += 0.1;
				// 		// outer_zone_width += 0.1;
				// 	}
				} else {
					N_prec.setIdentity();
				}

				// Jv_con = constraint_direction.head(2).transpose();
				// if (!first_entry) {
                	// N_prec = robot->nullspaceMatrix(Jv_con);  /// enable/disable nullspace projections 
				// }
				// apf_torques = Jv_con.transpose() * apf_force;
				// apf_torques = apf_force;
				// std::cout << "Nprec: " << N_prec << "\n";
            }

			// compute control torques with nullspace
			// motion_force_task->updateTaskModel(N_prec);
			joint_task->updateTaskModel(N_prec);
			// Vector3d task_forces = motion_force_task->computeTorques();
			joint_task_torques = joint_task->computeTorques();
			// transfer torques
			// motion_force_task_torques = task_forces;
			std::cout << "Task forces after nullspace: " << joint_task_torques.transpose() << "\n";

			// draw lines (blue = force without nullspace, green is force in nullspace)
			force_line->m_pointA = chai3d::cVector3d(ee_pos);
			force_line->m_pointB = chai3d::cVector3d(ee_pos) + 0.5 * chai3d::cVector3d(task_forces_without_nullspace(0), \
																							task_forces_without_nullspace(1), 0);

			nullspace_force_line->m_pointA = chai3d::cVector3d(ee_pos);
			auto task_forces_normalized = joint_task_torques.normalized();
			nullspace_force_line->m_pointB = chai3d::cVector3d(ee_pos) + 0.5 * chai3d::cVector3d(task_forces_normalized(0), task_forces_normalized(1), 0);

            // constraint debug
            // std::cout << "distance: " << distance << "\n";
            // std::cout << "constraint direction: " << constraint_direction.transpose() << "\n";
            // std::cout << "nullspace matrix: \n" << N_prec << "\n";
        } else {
			
			std::cout << "Not first entry\n";

			// compute control torques with nullspace
			// motion_force_task->updateTaskModel(N_prec);
			N_prec.setIdentity();
			joint_task->updateTaskModel(N_prec);
			// Vector3d task_forces = motion_force_task->computeTorques();
			joint_task_torques = joint_task->computeTorques();
			// transfer torques
			// motion_force_task_torques = task_forces;

			// draw lines (blue = force without nullspace, green is force in nullspace)
			force_line->m_pointA = chai3d::cVector3d(ee_pos);
			force_line->m_pointB = chai3d::cVector3d(ee_pos) + 0.5 * chai3d::cVector3d(task_forces_without_nullspace(0), \
																							task_forces_without_nullspace(1), 0);

			nullspace_force_line->m_pointA = chai3d::cVector3d(ee_pos);
			auto task_forces_normalized = joint_task_torques.normalized();
			nullspace_force_line->m_pointB = chai3d::cVector3d(ee_pos) + 0.5 * chai3d::cVector3d(task_forces_normalized(0), task_forces_normalized(1), 0);
		}

		// motion_force_task->updateTaskModel(N_prec);
		// N_prec = motion_force_task->getTaskAndPreviousNullspace();
		// after each task, need to update the nullspace
		// of the previous tasks in order to garantee
		// the dynamic consistency

		// joint_task->updateTaskModel(N_prec);

		// -------- set task goals and compute control torques
		// position: move to workspace extents 
        // if (time - prev_time > t_wait[cnt % 2]) {
            // motion_force_task->setGoalPosition(initial_position + desired_offsets[cnt]);
            // cnt++;
            // prev_time = time;
            // if (cnt == max_cnt) cnt = max_cnt - 1;
        // }
        // motion_force_task->setGoalLinearVelocity(Vector3d::Zero());
        // motion_force_task->setGoalLinearAcceleration(Vector3d::Zero());

        // motion_force_task->setGoalPosition(x_init + x_delta);

		// compute torques for the different tasks
		// motion_force_task_torques = motion_force_task->computeTorques();
		// joint_task_torques = joint_task->computeTorques();

        // // compute apf torques 
        // VectorXd apf_force = VectorXd::Zero(robot->dof());
        // for (int i = 0; i < robot->dof(); ++i) {
        //     // if (i == 3) {
        //         double rho_lower = robot->q()(i) - q_min(i);
        //         double rho_upper = q_max(i) - robot->q()(i);
        //         if (rho_lower < 0.08) {
        //             apf_force(i) = eta * std::abs(((1 / rho_lower) - (1 / 0.08))) * (1 / (rho_lower * rho_lower));
        //         } else if (rho_upper < -0.08) {
        //             apf_force(i) = - eta * std::abs(((1 / rho_upper) - (1 / (-0.08)))) * (1 / (rho_upper * rho_upper));
        //         }
        //     // }
        // }
        // std::cout << "apf force: " << apf_force.transpose() << "\n";

		//------ compute the final torques
		{
			lock_guard<mutex> lock(mutex_torques);
			// control_torques = joint_handler->computeTorques(motion_force_task_torques + joint_task_torques);
			// control_torques = motion_force_task_torques + joint_task_torques + 1 * apf_torques;
			// control_torques = motion_force_task_torques;
			control_torques = joint_task_torques + 1 * apf_force.head(2) - 5 * robot->dq();
			std::cout << "control torques: " << control_torques.transpose() << "\n";
		}

        // control_forces = motion_force_task->getNonSingularLambda() * motion_force_task->getUnitControlForces();


		// MatrixXd Jc = MatrixXd::Zero(1, robot->dof());
		// Jc(0) = 1;
		// MatrixXd force_projection = Jc * robot->dynConsistentInverseJacobian(Jc);
		// std::cout << force_projection.transpose() << "\n";

		// // -------------------------------------------
		// if (timer.elapsedCycles() % 500 == 0) {
		// 	cout << "time: " << time << endl;
		// 	cout << "position error : "
		// 		 << (motion_force_task->getGoalPosition() -
		// 			 motion_force_task->getCurrentPosition())
		// 				.norm()
		// 		 << endl;
		// 	cout << endl;
		// }
	}
	logger.stop();
	timer.stop();
	cout << "\nControl loop timer stats:\n";
	timer.printInfoPostRun();
}

//------------------------------------------------------------------------------
void simulation(shared_ptr<Sai2Model::Sai2Model> robot,
				shared_ptr<Sai2Simulation::Sai2Simulation> sim) {
	fSimulationRunning = true;

	sim->disableJointLimits(robot_name);

	// create a timer
	double sim_freq = 2000;
	Sai2Common::LoopTimer timer(sim_freq);

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

