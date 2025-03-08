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
const string robot_file = "${EXAMPLE_98_FOLDER}/rrrbot.urdf";
const string robot_name = "RRRBOT";

// ui torques and control torques
VectorXd ui_torques;
VectorXd control_torques;

// mutex for global variables between different threads
mutex mutex_torques;
mutex mutex_robot;

// simulation and control loop
void control(shared_ptr<Sai2Model::Sai2Model> robot,
			 shared_ptr<Sai2Simulation::Sai2Simulation> sim);
void simulation(shared_ptr<Sai2Model::Sai2Model> robot,
				shared_ptr<Sai2Simulation::Sai2Simulation> sim);

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

	// Joint task
	auto joint_task = std::make_shared<Sai2Primitives::JointTask>(robot);
	// joint_task->disableInternalOtg();
	joint_task->enableVelocitySaturation(M_PI);
	joint_task->setDynamicDecouplingType(Sai2Primitives::FULL_DYNAMIC_DECOUPLING);
	VectorXd joint_task_torques = VectorXd::Zero(robot->dof());
    VectorXd q_init = robot->q();

    // apf joint limits 
    VectorXd q_min(robot->dof()), q_max(robot->dof());
    // q_min << -100, -100, -M_PI / 2;
    // q_max << 100, 100, M_PI / 2;
    q_min << -100, - M_PI / 2, -100;
    q_max << 100, M_PI / 2, 100;
    // double zone_width = 0.08;
    double zone_width = 0.15;
    bool first_loop = false;
    double eta = 1.0;
    VectorXd q_delta = VectorXd::Zero(robot->dof());
    double sign_switcher = 1;
    double gamma_max = 10;

	// create logger
	Sai2Common::Logger logger("apf", false);
	// VectorXd svalues = VectorXd::Zero(6);
    // VectorXd robot_q = robot->q();
    // Vector3d ee_pos = robot->position(link_name, pos_in_link);
    // Vector3d x_curr;
    // Vector3d control_forces = Vector3d::Zero();
    // Vector3d constraint_direction = Vector3d::Zero();
    // int constraint_flag = 0;
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
				// x_init = robot->q();
				first_loop = false;
			}

			// debug 
			// ee_pos = robot->position(link_name, pos_in_link);
	        // ee_pos = x_curr;
			// std::cout << "Initial position: " << x_init.transpose() << "\n";
			// std::cout << "Current position: " << x_curr.transpose() << "\n";
			// std::cout << "Motion task current position: " << motion_force_task->getCurrentPosition().transpose() << "\n";
			// std::cout << "Goal position: " << motion_force_task->getGoalPosition().transpose() << "\n";
        }

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

        N_prec.setIdentity();
        joint_task->updateTaskModel(N_prec);
        VectorXd full_joint_task_torques = joint_task->computeTorques();

        // compute joint limit potential field and nullspace
        // MatrixXd N_prec = MatrixXd::Identity(dof, dof);
        VectorXd apf_force = VectorXd::Zero(robot->dof());
        MatrixXd Jbar = MatrixXd::Zero(1, robot->dof());
        MatrixXd J_c = MatrixXd::Zero(1, robot->dof());
        VectorXd gamma_apf = VectorXd::Zero(robot->dof());
        double alpha = -1;
        for (int i = 0; i < robot->dof(); ++i) {
            // if (i == 1) {
                double rho_lower = robot->q()(i) - q_min(i);
                double rho_upper = q_max(i) - robot->q()(i);
                if (rho_lower < zone_width) {
                    // std::cout << "q min\n";
                    apf_force(i) = eta * std::abs(((1 / rho_lower) - (1 / zone_width))) * (1 / (rho_lower * rho_lower));
                    alpha = std::clamp((robot->q()(i) - q_min(i)) / zone_width, 0.0, 1.0);
                    gamma_apf(i) = gamma_max;
                    J_c = MatrixXd::Zero(1, robot->dof());
                    J_c(0, i) = 1;
                    N_prec = robot->nullspaceMatrix(J_c);
                    Jbar = robot->dynConsistentInverseJacobian(J_c);
                } else if (rho_upper < zone_width) {
                    // std::cout << "q max\n";
                    apf_force(i) = - eta * std::abs(((1 / rho_upper) - (1 / (- zone_width)))) * (1 / (rho_upper * rho_upper));
                    alpha = std::clamp((q_max(i) - robot->q()(i)) / zone_width, 0.0, 1.0);
                    gamma_apf(i) = - gamma_max;
                    J_c = MatrixXd::Zero(1, robot->dof());
                    J_c(0, i) = 1;
                    N_prec = robot->nullspaceMatrix(J_c);
                    Jbar = robot->dynConsistentInverseJacobian(J_c);
                }
            // }
            // std::cout << "N_prec: \n" << N_prec << "\n";
        }
        // std::cout << "apf force: " << apf_force.transpose() << "\n";

        // update goal 
        // -------------------------------------------
		if (timer.elapsedCycles() % 10000 == 0) {
			cout << "time: " << time << endl;
            sign_switcher *= -1;
            // q_delta = sign_switcher * Vector3d(0, 0, 10);
            q_delta = sign_switcher * Vector3d(0, M_PI, 0);
            joint_task->setGoalPosition(q_init + q_delta);
            cout << "changing goal position to: " << sign_switcher * (q_init + q_delta).transpose() << "\n";
		}

        // N_prec.setIdentity();
        joint_task->updateTaskModel(N_prec);
        joint_task_torques = joint_task->computeTorques();
        // if (!apf_force.isZero()) {
            // std::cout << "joint task torques during apf: " << joint_task_torques.transpose() << "\n";
        // }

        // other implementation
        VectorXd leak_torque = VectorXd::Zero(robot->dof());
        if (alpha != -1) {
            leak_torque = std::pow(alpha, 2) * J_c.transpose() * (Jbar.transpose() * full_joint_task_torques) + gamma_apf * (1 - alpha) * (1 - alpha);
            // std::cout << "gamma apf: " << (gamma_apf * (1 - alpha) * (1 - alpha)).transpose() << "\n";
            // std::cout << "leak torque: " << leak_torque.transpose() << "\n";
        }

		// // joint_task->setGoalPosition((x_init + x_delta).head(2));
		// // motion_force_task->setGoalPosition(x_init + x_delta);

        // // auto [distance, con_dir] = distanceToCapsule(ee_pos.head(2), capsule_a, capsule_b, capsule_radius);
        // auto [distance, con_dir] = distanceToCapsule(robot->q(), capsule_a, capsule_b, capsule_radius);
		// // auto [distance, con_dir] = distanceToEllipse(ee_pos(0), ee_pos(1), ellipsoid_center(0), ellipsoid_center(1), ellipsoid_radii(0), ellipsoid_radii(1));
        // constraint_direction = - Vector3d(con_dir(0), con_dir(1), 0).normalized();

		// if (distance < 0) {
		// 	std::cout << "Inside ellipsoid: setting distance to minimum value\n";
		// 	distance = 1e-6;
		// }

		// std::cout << "constraint direction: " << constraint_direction.transpose() << "\n";
		// std::cout << "distance: " << distance << "\n";

		// // compute apf forces in direction
		// Vector3d apf_force = Vector3d::Zero();
		// double eta = 1.0;
		// if (distance < zone_width) {
		// 	apf_force = - eta * std::abs(((1 / distance) - (1 / zone_width))) * (1 / (distance * distance)) * constraint_direction;
		// 	if (first_entry) {
		// 		// zone_width *= 2;
		// 		outer_zone_width += 0.01;
		// 		first_entry = false;
		// 	}
		// }
		// std::cout << "apf force: " << apf_force.transpose() << "\n";

		// // update tasks model. Order is important to define the hierarchy
		// N_prec.setIdentity();
        // // constraint_flag = 0;

		// // compute task without nullspace
		// // motion_force_task->updateTaskModel(N_prec);
		// joint_task->updateTaskModel(N_prec);
		// // Vector3d task_forces_without_nullspace = motion_force_task->computeTorques();
		// VectorXd task_forces_without_nullspace = joint_task->computeTorques();
		// std::cout << "Task forces before nullspace: " << task_forces_without_nullspace.transpose() << "\n";

        // // add constraint direction if in zone of influence 
		// MatrixXd Jv_con = MatrixXd::Zero(1, robot->dof());
		// // VectorXd apf_torques = VectorXd::Zero(robot->dof());

		// // if inside zone or past first entry 
		// if (distance > outer_zone_width && !first_entry) {

		// 	std::cout << "outside zone width\n";
		// 	// compute control torques with nullspace
		// 	// motion_force_task->updateTaskModel(N_prec);
		// 	N_prec.setIdentity();
		// 	joint_task->updateTaskModel(N_prec);
		// 	// Vector3d task_forces = motion_force_task->computeTorques();
		// 	joint_task_torques = joint_task->computeTorques();
		// 	// transfer torques
		// 	// motion_force_task_torques = task_forces;

		// 	// draw lines (blue = force without nullspace, green is force in nullspace)
		// 	force_line->m_pointA = chai3d::cVector3d(ee_pos);
		// 	force_line->m_pointB = chai3d::cVector3d(ee_pos) + 0.5 * chai3d::cVector3d(task_forces_without_nullspace(0), \
		// 																					task_forces_without_nullspace(1), 0);

		// 	nullspace_force_line->m_pointA = chai3d::cVector3d(ee_pos);
		// 	auto task_forces_normalized = joint_task_torques.normalized();
		// 	nullspace_force_line->m_pointB = chai3d::cVector3d(ee_pos) + 0.5 * chai3d::cVector3d(task_forces_normalized(0), task_forces_normalized(1), 0);
		// } else if (!first_entry) {
        // // if (distance < zone_width) {
		// // if (!first_entry) {
        //     // constraint_flag = 1;
        //     {
		// 	    lock_guard<mutex> lock(mutex_robot);
				
		// 		// if (constraint_direction.dot(task_forces_without_nullspace) > 0) {
	    //             // Jv_con = constraint_direction.transpose() * robot->Jv(link_name, pos_in_link);
		// 			// N_prec = robot->nullspaceMatrix(Jv_con);
		// 		if (!first_entry) {
		// 			Jv_con = constraint_direction.transpose() * robot->Jv(link_name, pos_in_link);
		// 			// std::cout << "Jv_con: " << Jv_con << "\n";
		// 			// std::cout << robot->Jv(link_name, pos_in_link) << "\n";
		// 			// Jv_con = constraint_direction.head(2).transpose();
		// 			N_prec = robot->nullspaceMatrix(Jv_con);
		// 			// MatrixXd J_task = robot->Jv(link_name, pos_in_link);
		// 			// MatrixXd J_bar = robot->MInv() * J_task.transpose() * robot->taskInertiaMatrix(J_task);
		// 			// N_prec = MatrixXd::Identity(2, 2) - J_bar * J_task;
		// 			// std::cout << "Nullspace: \n" << N_prec << "\n";

		// 			// std::cout << "projected task jacobian: \n" << robot->Jv(link_name, pos_in_link) * N_prec << "\n";

		// 		// 	// check if distance is outside new distance; if it is, then reset first entry
		// 		// 	if (distance > outer_zone_width) {
		// 		// 		first_entry = true;
		// 		// 		// zone_width += 0.1;
		// 		// 		// outer_zone_width += 0.1;
		// 		// 	}
		// 		} else {
		// 			N_prec.setIdentity();
		// 		}

		// 		// Jv_con = constraint_direction.head(2).transpose();
		// 		// if (!first_entry) {
        //         	// N_prec = robot->nullspaceMatrix(Jv_con);  /// enable/disable nullspace projections 
		// 		// }
		// 		// apf_torques = Jv_con.transpose() * apf_force;
		// 		// apf_torques = apf_force;
		// 		// std::cout << "Nprec: " << N_prec << "\n";
        //     }

		// 	// compute control torques with nullspace
		// 	// motion_force_task->updateTaskModel(N_prec);
		// 	joint_task->updateTaskModel(N_prec);
		// 	// Vector3d task_forces = motion_force_task->computeTorques();
		// 	joint_task_torques = joint_task->computeTorques();
		// 	// transfer torques
		// 	// motion_force_task_torques = task_forces;
		// 	std::cout << "Task forces after nullspace: " << joint_task_torques.transpose() << "\n";

		// 	// draw lines (blue = force without nullspace, green is force in nullspace)
		// 	force_line->m_pointA = chai3d::cVector3d(ee_pos);
		// 	force_line->m_pointB = chai3d::cVector3d(ee_pos) + 0.5 * chai3d::cVector3d(task_forces_without_nullspace(0), \
		// 																					task_forces_without_nullspace(1), 0);

		// 	nullspace_force_line->m_pointA = chai3d::cVector3d(ee_pos);
		// 	auto task_forces_normalized = joint_task_torques.normalized();
		// 	nullspace_force_line->m_pointB = chai3d::cVector3d(ee_pos) + 0.5 * chai3d::cVector3d(task_forces_normalized(0), task_forces_normalized(1), 0);

        //     // constraint debug
        //     // std::cout << "distance: " << distance << "\n";
        //     // std::cout << "constraint direction: " << constraint_direction.transpose() << "\n";
        //     // std::cout << "nullspace matrix: \n" << N_prec << "\n";
        // } else {
			
		// 	std::cout << "Not first entry\n";

		// 	// compute control torques with nullspace
		// 	// motion_force_task->updateTaskModel(N_prec);
		// 	N_prec.setIdentity();
		// 	joint_task->updateTaskModel(N_prec);
		// 	// Vector3d task_forces = motion_force_task->computeTorques();
		// 	joint_task_torques = joint_task->computeTorques();
		// 	// transfer torques
		// 	// motion_force_task_torques = task_forces;

		// 	// draw lines (blue = force without nullspace, green is force in nullspace)
		// 	force_line->m_pointA = chai3d::cVector3d(ee_pos);
		// 	force_line->m_pointB = chai3d::cVector3d(ee_pos) + 0.5 * chai3d::cVector3d(task_forces_without_nullspace(0), \
		// 																					task_forces_without_nullspace(1), 0);

		// 	nullspace_force_line->m_pointA = chai3d::cVector3d(ee_pos);
		// 	auto task_forces_normalized = joint_task_torques.normalized();
		// 	nullspace_force_line->m_pointB = chai3d::cVector3d(ee_pos) + 0.5 * chai3d::cVector3d(task_forces_normalized(0), task_forces_normalized(1), 0);
		// }

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
			// control_torques = joint_task_torques + 1 * apf_force.head(2) - 5 * robot->dq();
			// control_torques = joint_task_torques + 1 * robot->M() * apf_force - 5 * robot->M() * robot->dq();
			control_torques = joint_task_torques + leak_torque - 5 * robot->M() * robot->dq();
			// std::cout << "control torques: " << control_torques.transpose() << "\n";
            // std::cout << "joint task error: " << (robot->q() - joint_task->getGoalPosition()).transpose() << "\n";
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

