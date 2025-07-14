/**
 * @file 999-visualization.cpp 
 * @author William Chong (wmchong@stanford.edu)
 * @brief 
 * @version 0.1
 * @date 2025-03-08
 * 
 * @copyright Copyright (c) 2025
 * 
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
#include <yaml-cpp/yaml.h>
bool fSimulationRunning = false;
void sighandler(int) { fSimulationRunning = false; }

using namespace std;
using namespace Eigen;
using namespace chai3d;

// config file names and object names
const string world_file = "${EXAMPLE_998_FOLDER}/world.urdf";
// const string robot_file =
//     "${SAI2_MODEL_URDF_FOLDER}/panda/panda_arm_gripper_fixed.urdf";
// const string robot_file =
//     "${SAI2_MODEL_URDF_FOLDER}/mmp_panda/mmp_panda.urdf";
const string robot_file = "${EXAMPLE_998_FOLDER}/ppbot.urdf";
const string robot_name = "ppbot";
const string yaml_fname = "../../../examples/998-apf-comparison/params.yaml";

// ui torques and control torques
VectorXd ui_torques;
VectorXd control_torques;

// mutex for global variables between different threads
mutex mutex_torques;
mutex mutex_robot;

/*
	GLOBALS
*/
Vector2d START, END;
double ETA, RHO_0, RHO_1;
double APF_A, APF_B, SF;
double VEL, MIN_VEL, VEL_SF;
double KV, T_WAIT;
double DISTANCE = 100;
Vector2d APF_CENTER = Vector2d::Zero();
Vector2d CLOSEST_POINT = Vector2d::Zero();
double MASS = 1;
bool NULLSPACE_FLAG = false;
bool EXIT_FLAG = false;
bool SLOWDOWN_FLAG = false;
Vector2d CONSTRAINT_VECTOR = Vector2d::Zero();
bool FIRST_ENTRY = false;
bool INSIDE_APF = false;
bool ENABLE_ARROW = false;
bool ENABLE_POSITION_CHANGE = false;
double T_POSITION_CHANGE = 0;
Vector3d GOAL_POSITION = Vector3d::Zero();
double THICKNESS = 0;
bool SECOND_OBSTACLE = false;

// simulation and control loop
void control(shared_ptr<Sai2Model::Sai2Model> robot,
			 shared_ptr<Sai2Simulation::Sai2Simulation> sim);
void simulation(shared_ptr<Sai2Model::Sai2Model> robot,
				shared_ptr<Sai2Simulation::Sai2Simulation> sim);

// Function to get mesh by name
cMesh* getMeshByName(cMultiMesh* multiMesh, const std::string& name) {
	for (size_t i = 0; i < multiMesh->getNumMeshes(); i++) {
		cMesh* mesh = multiMesh->getMesh(i);
		std::cout << mesh->m_name << "\n";
		if (mesh->m_name == name) { // Compare name
			return mesh;
		}
	}
	return nullptr; // Not found
}

/*
***********************************
	Zone of influence drawing
***********************************
*/
// Helper: Sign function
double sgn(double x) { return (x >= 0) ? 1.0 : -1.0; }

// Evaluate superellipse point and normal
void generateSuperellipseWithOffset(
    double a, double b,
    double radius,
    int numPoints,
    std::vector<cVector3d>& originalPoints,
    std::vector<cVector3d>& offsetPoints)
{
    originalPoints.clear();
    offsetPoints.clear();

    const int n = 8;
    for (int i = 0; i < numPoints; ++i)
    {
        double theta = 2.0 * M_PI * i / numPoints;

        // Superellipse parametric equation
        double cos_t = cos(theta);
        double sin_t = sin(theta);
        double x = a * sgn(cos_t) * pow(fabs(cos_t), 2.0 / n);
        double y = b * sgn(sin_t) * pow(fabs(sin_t), 2.0 / n);

        originalPoints.push_back(cVector3d(x, y, 0));

        // Gradient for normal
        double dx = 8 * pow(x / a, 7) / a;
        double dy = 8 * pow(y / b, 7) / b;
        Eigen::Vector2d grad(dx, dy);
        Eigen::Vector2d normal = grad.normalized();

        // Offset point outward
        Eigen::Vector2d offset = Eigen::Vector2d(x, y) + radius * normal;
        offsetPoints.push_back(cVector3d(offset.x(), offset.y(), 0));
    }
}

void drawBoundaryLines(cWorld* world,
                       const std::vector<cVector3d>& points,
                       const cColorf& color,
                       double lineWidth = 1.5)
{
    for (size_t i = 0; i < points.size(); ++i)
    {
        size_t next = (i + 1) % points.size();

        auto line = new cShapeLine(points[i], points[next]);
        line->m_colorPointA = color;
        line->m_colorPointB = color;
        line->setLineWidth(lineWidth);

        world->addChild(line);
    }
}

// void fillSuperellipseBand(cWorld* world,
//                           const std::vector<cVector3d>& inner,
//                           const std::vector<cVector3d>& outer,
//                           cColorf& color,
// 						  const double& transparency = 1.0,
// 						  const double& z_offset = 0)
// {
//     if (inner.size() != outer.size() || inner.size() < 3)
//         return;

//     auto mesh = new cMesh();
//     size_t N = inner.size();

//     for (size_t i = 0; i < N; ++i)
//     {
//         size_t j = (i + 1) % N;

//         // Vertices: inner[i], outer[i], outer[j], inner[j]
//         int v0 = mesh->newVertex(inner[i] - cVector3d(0, 0, z_offset));
//         int v1 = mesh->newVertex(outer[i] - cVector3d(0, 0, z_offset));
//         int v2 = mesh->newVertex(outer[j] - cVector3d(0, 0, z_offset));
//         int v3 = mesh->newVertex(inner[j] - cVector3d(0, 0, z_offset));

//         // First triangle
//         mesh->newTriangle(v0, v1, v2);

//         // Second triangle
//         mesh->newTriangle(v0, v2, v3);
//     }

//     // Properly set material colors
// 	mesh->setUseTransparency(true);
//     mesh->m_material->m_ambient  = color;
//     mesh->m_material->m_diffuse  = color;
//     mesh->m_material->m_specular = cColorf(1.0, 1.0, 1.0);
//     mesh->m_material->setShininess(100);
// 	// mesh->m_material->setUseTransparency(true);
// 	mesh->m_material->setTransparencyLevel(transparency);
//     mesh->setUseVertexColors(false);

// 	// mesh->m_material->m_ambient = cColorf(0.4f, 0.3f, 0.2f);
// 	// mesh->m_material->m_diffuse = cColorf(0.9f, 0.6f, 0.3f);
// 	// mesh->m_material->m_specular = cColorf(0.6f, 0.4f, 0.3f);
// 	// mesh->m_material->setShininess(80);

//     world->addChild(mesh);
// }

void fillSuperellipseBand(cWorld* world,
                          const std::vector<cVector3d>& inner,
                          const std::vector<cVector3d>& outer,
                          cColorf& color,
                          const double& transparency = 1.0,
                          const double& z_offset = 0.0,
                          const double& thickness = 0.05)
{
    if (inner.size() != outer.size() || inner.size() < 3)
        return;

    auto mesh = new chai3d::cMesh();
    size_t N = inner.size();

    double z_top = z_offset + thickness / 2.0;
    double z_bot = z_offset - thickness / 2.0;

    std::vector<int> innerTopIndices, outerTopIndices;
    std::vector<int> innerBotIndices, outerBotIndices;

    for (size_t i = 0; i < N; ++i)
    {
        // Create top and bottom ring vertices
        int v_inner_top = mesh->newVertex(inner[i] + cVector3d(0, 0, z_top));
        int v_outer_top = mesh->newVertex(outer[i] + cVector3d(0, 0, z_top));
        int v_inner_bot = mesh->newVertex(inner[i] + cVector3d(0, 0, z_bot));
        int v_outer_bot = mesh->newVertex(outer[i] + cVector3d(0, 0, z_bot));

        innerTopIndices.push_back(v_inner_top);
        outerTopIndices.push_back(v_outer_top);
        innerBotIndices.push_back(v_inner_bot);
        outerBotIndices.push_back(v_outer_bot);
    }

    // Top band triangles
    for (size_t i = 0; i < N; ++i)
    {
        size_t j = (i + 1) % N;

        int v0 = innerTopIndices[i];
        int v1 = outerTopIndices[i];
        int v2 = outerTopIndices[j];
        int v3 = innerTopIndices[j];

        mesh->newTriangle(v0, v1, v2);
        mesh->newTriangle(v0, v2, v3);
    }

    // Bottom band triangles (reverse winding)
    for (size_t i = 0; i < N; ++i)
    {
        size_t j = (i + 1) % N;

        int v0 = innerBotIndices[i];
        int v1 = outerBotIndices[i];
        int v2 = outerBotIndices[j];
        int v3 = innerBotIndices[j];

        mesh->newTriangle(v0, v2, v1);
        mesh->newTriangle(v0, v3, v2);
    }

    // Optional: Side walls (outer ring)
    for (size_t i = 0; i < N; ++i)
    {
        size_t j = (i + 1) % N;

        int top1 = outerTopIndices[i];
        int top2 = outerTopIndices[j];
        int bot1 = outerBotIndices[i];
        int bot2 = outerBotIndices[j];

        mesh->newTriangle(top1, bot1, bot2);
        mesh->newTriangle(top1, bot2, top2);
    }

    // Optional: Side walls (inner ring, reverse winding)
    for (size_t i = 0; i < N; ++i)
    {
        size_t j = (i + 1) % N;

        int top1 = innerTopIndices[i];
        int top2 = innerTopIndices[j];
        int bot1 = innerBotIndices[i];
        int bot2 = innerBotIndices[j];

        mesh->newTriangle(top2, bot2, bot1);
        mesh->newTriangle(top2, bot1, top1);
    }

    // Set material
    mesh->setUseTransparency(true);
    mesh->m_material->m_ambient  = color;
    mesh->m_material->m_diffuse  = color;
    mesh->m_material->m_specular = chai3d::cColorf(1.0, 1.0, 1.0);
    mesh->m_material->setShininess(100);
    mesh->m_material->setTransparencyLevel(transparency);
    mesh->setUseVertexColors(false);

    world->addChild(mesh);
}

/*
************************************
	APF Gradient Computation
************************************
*/
// Solve (x/a)^8 + (y/b)^8 = 1 for closest (x, y) to point (px, py)
Eigen::Vector3d closestPointSuperEllipse(double a, double b, const Eigen::Vector3d& p) {
    double px = p(0), py = p(1), pz = p(2);

    // Handle origin case
    if (px == 0 && py == 0) {
        // Any point on surface is equally close — pick x = a, y = 0
        return Eigen::Vector3d(a, 0, 0);
    }

    // Transform to first quadrant
    double sx = (px >= 0) ? 1 : -1;
    double sy = (py >= 0) ? 1 : -1;
    double ux = std::abs(px), uy = std::abs(py);

    // Parameterize the superellipse using angle t
    auto surface = [&](double t) -> Eigen::Vector2d {
        return Eigen::Vector2d(
            a * std::pow(std::cos(t), 1.0 / 8),
            b * std::pow(std::sin(t), 1.0 / 8)
        );
    };

    auto distanceSquared = [&](double t) -> double {
        Eigen::Vector2d pt = surface(t);
        double dx = pt(0) - ux;
        double dy = pt(1) - uy;
        return dx * dx + dy * dy;
    };

    // Golden-section search to minimize distance on [0, pi/2]
    const double phi = (1 + std::sqrt(5)) / 2;
    double tol = 1e-6;
    double a0 = 1e-4, b0 = M_PI_2 - 1e-4; // avoid endpoints

    double c = b0 - (b0 - a0) / phi;
    double d = a0 + (b0 - a0) / phi;

    while (std::abs(b0 - a0) > tol) {
        if (distanceSquared(c) < distanceSquared(d)) {
            b0 = d;
        } else {
            a0 = c;
        }
        c = b0 - (b0 - a0) / phi;
        d = a0 + (b0 - a0) / phi;
    }

    double t_opt = 0.5 * (a0 + b0);
    Eigen::Vector2d closest2d = surface(t_opt);
    closest2d(0) *= sx;
    closest2d(1) *= sy;

    // Closest point on z=0 plane
    Eigen::Vector3d closest(closest2d(0), closest2d(1), 0.0);
    return closest;
}

double sign(double x) {
    return (x >= 0.0) ? 1.0 : -1.0;
}

// Parametric point on superellipse for angle theta
Eigen::Vector2d superellipsePoint(double theta, double a, double b, double n) {
    double cos_t = std::cos(theta);
    double sin_t = std::sin(theta);

    double x = a * sign(cos_t) * std::pow(std::abs(cos_t), 2.0 / n);
    double y = b * sign(sin_t) * std::pow(std::abs(sin_t), 2.0 / n);

    return Eigen::Vector2d(x, y);
}

// Returns distance and closest point on superellipse
std::pair<double, Eigen::Vector2d> closestDistanceToSuperellipse(
    const Eigen::Vector2d& point,
    double a, double b,
    double n = 8.0,
    int num_samples = 10000
) {
    double min_dist = std::numeric_limits<double>::infinity();
    Eigen::Vector2d closest_point;

    for (int i = 0; i < num_samples; ++i) {
        double theta = (2.0 * M_PI * i) / num_samples;
        Eigen::Vector2d p = superellipsePoint(theta, a, b, n);
        double dist = (p - point).norm();
        if (dist < min_dist) {
            min_dist = dist;
            closest_point = p;
        }
    }

    return {min_dist, closest_point};
}

// Projects a point onto the superellipse (x/a)^n + (y/b)^n = 1
Eigen::Vector2d projectOntoSuperellipse(const Eigen::Vector2d& pt, double a, double b, double n, int max_iter = 100, double tol = 1e-6)
{
    Eigen::Vector2d x = pt;

    // Normalize initial guess onto superellipse
    double xn = std::pow(std::abs(x[0] / a), n);
    double yn = std::pow(std::abs(x[1] / b), n);
    if (xn + yn > 1.0) {
        double scale = std::pow(xn + yn, -1.0 / n);
        x[0] *= scale;
        x[1] *= scale;
    }

    for (int i = 0; i < max_iter; ++i) {
        // Constraint value and gradient
        double gx = n * std::pow(std::abs(x[0] / a), n - 1) * ((x[0] >= 0) ? 1 : -1) / a;
        double gy = n * std::pow(std::abs(x[1] / b), n - 1) * ((x[1] >= 0) ? 1 : -1) / b;
        double constraint = std::pow(std::abs(x[0] / a), n) + std::pow(std::abs(x[1] / b), n) - 1.0;

        Eigen::Vector2d grad_g(gx, gy);
        double lambda = 2.0 * (x - pt).dot(grad_g) / grad_g.squaredNorm();

        // Newton update (with line search fallback if needed)
        Eigen::Vector2d x_new = x - 0.1 * ((x - pt) - 0.5 * lambda * grad_g);  // step size = 0.1

        double error = std::abs(std::pow(std::abs(x_new[0] / a), n) + std::pow(std::abs(x_new[1] / b), n) - 1.0);
        x = x_new;
        if (error < tol) break;
    }

    return x;
}

// Finds closest point on superellipse and returns distance
std::pair<Eigen::Vector2d, double> closestPointToSuperellipseRefined(
    const Eigen::Vector2d& pt, double a, double b, double n)
{
    Eigen::Vector2d projected = projectOntoSuperellipse(pt, a, b, n);
    double dist = (projected - pt).norm();
    return {projected, dist};
}

// 2D ONLY
Eigen::Vector3d distanceVectorToSuperEllipse(double a, double b, const Eigen::Vector3d& p) {
    Eigen::Vector3d closest = closestPointSuperEllipse(a, b, p);
    return p - closest;
}

// Project a point onto the superellipse curve using iterative method
Eigen::Vector2d projectToSuperellipse(double x0, double y0, double a, double b, int max_iters = 100, double tol = 1e-6) {
    Eigen::Vector2d p(x0, y0);

    // Initial guess: scale original point onto the curve
    double norm = std::pow(std::pow(std::abs(x0/a), 8) + std::pow(std::abs(y0/b), 8), 1.0/8.0);
    if (norm < 1e-6) norm = 1.0;
    Eigen::Vector2d q = p / norm;

    for (int i = 0; i < max_iters; ++i) {
        double x = q.x(), y = q.y();
        double fx = std::pow(x/a, 8) + std::pow(y/b, 8) - 1.0;

        if (std::abs(fx) < tol) break;

        // Gradient of the constraint
        double dfx_dx = 8 * std::pow(std::abs(x)/a, 7) * (x >= 0 ? 1 : -1) / a;
        double dfx_dy = 8 * std::pow(std::abs(y)/b, 7) * (y >= 0 ? 1 : -1) / b;
        Eigen::Vector2d gradF(dfx_dx, dfx_dy);

        // Gradient of distance squared
        Eigen::Vector2d gradD = 2.0 * (q - p);

        // Lagrange multiplier
        double lambda = gradF.dot(gradD) / gradF.squaredNorm();

        // Projected gradient descent step
        Eigen::Vector2d step = gradD - lambda * gradF;
        double alpha = 0.05;  // step size
        q -= alpha * step;

        if (step.norm() < tol) break;
    }

    return q;
}

// Distance to superellipse
double distanceToSuperellipse(double x0, double y0, double a, double b) {
    Eigen::Vector2d closest = projectToSuperellipse(x0, y0, a, b);
    return (closest - Eigen::Vector2d(x0, y0)).norm();
}

bool isInsideSuperellipse(double x0, double y0, double a, double b) {
    double val = std::pow(std::abs(x0 / a), 8) + std::pow(std::abs(y0 / b), 8);
    return val <= 1.0;
}

// Project a point onto the 8th-power superellipse: (x/a)^8 + (y/b)^8 = 1
Eigen::Vector2d projectOntoSuperellipseN8(const Eigen::Vector2d& pt, double a, double b, int max_iter = 100, double tol = 1e-6)
{
    Eigen::Vector2d x = pt;

    // Normalize if outside the superellipse
    double xn = std::pow(std::abs(x[0] / a), 8);
    double yn = std::pow(std::abs(x[1] / b), 8);
    if (xn + yn > 1.0) {
        double scale = std::pow(xn + yn, -1.0 / 8.0);
        x[0] *= scale;
        x[1] *= scale;
    }

    for (int i = 0; i < max_iter; ++i) {
        // Constraint gradient for n=8
        double gx = 8.0 * std::pow(std::abs(x[0] / a), 7) * ((x[0] >= 0) ? 1 : -1) / a;
        double gy = 8.0 * std::pow(std::abs(x[1] / b), 7) * ((x[1] >= 0) ? 1 : -1) / b;
        Eigen::Vector2d grad_g(gx, gy);

        // Constraint value
        double constraint = std::pow(std::abs(x[0] / a), 8) + std::pow(std::abs(x[1] / b), 8) - 1.0;

        // Lagrange multiplier for projection step
        double lambda = 2.0 * (x - pt).dot(grad_g) / grad_g.squaredNorm();

        // Gradient step projected onto constraint manifold
        Eigen::Vector2d x_new = x - 0.1 * ((x - pt) - 0.5 * lambda * grad_g);

        // Check convergence
        double err = std::abs(std::pow(std::abs(x_new[0] / a), 8) + std::pow(std::abs(x_new[1] / b), 8) - 1.0);
        x = x_new;
        if (err < tol) break;
    }

    return x;
}

Eigen::Vector2d closestPointToSuperellipse(const Eigen::Vector2d& query, double a, double b, int n = 8, int max_iter = 100, double tol = 1e-6) {
    using namespace Eigen;
    using std::pow;

    Vector3d vars;  // [x, y, lambda]
    vars.head<2>() = query.normalized(); // initialize close to query point
    vars(2) = 0.0;  // lambda

    for (int iter = 0; iter < max_iter; ++iter) {
        double x = vars(0), y = vars(1), lambda = vars(2);
        double xn1 = pow(std::abs(x), n - 1) * ((x >= 0) ? 1 : -1);
        double yn1 = pow(std::abs(y), n - 1) * ((y >= 0) ? 1 : -1);
        double xn = xn1 * x;
        double yn = yn1 * y;

        Vector3d F;
        F(0) = x - query(0) + lambda * n / pow(a, n) * xn1;
        F(1) = y - query(1) + lambda * n / pow(b, n) * yn1;
        F(2) = pow(x / a, n) + pow(y / b, n) - 1;

        // Jacobian
        Matrix3d J;
        J.setZero();
        J(0, 0) = 1 + lambda * n * (n - 1) / pow(a, n) * pow(std::abs(x), n - 2);
        J(0, 2) = n / pow(a, n) * xn1;

        J(1, 1) = 1 + lambda * n * (n - 1) / pow(b, n) * pow(std::abs(y), n - 2);
        J(1, 2) = n / pow(b, n) * yn1;

        J(2, 0) = n * pow(x / a, n - 1) / a;
        J(2, 1) = n * pow(y / b, n - 1) / b;

        Vector3d delta = J.fullPivLu().solve(-F);
        vars += delta;

        if (delta.norm() < tol) break;
    }

    return vars.head<2>();
}

// Wrapper to return closest point and distance
std::pair<Eigen::Vector2d, double> closestPointToSuperellipseN8(const Eigen::Vector2d& pt, double a, double b)
{
    Eigen::Vector2d closest = projectOntoSuperellipseN8(pt, a, b);
    double dist = (closest - pt).norm();
    return {closest, dist};
}

// Compute apf force from n-ellipsoid (n = 4)
Vector2d computeApfForce(const double a, 
						 const double b,
						 const VectorXd& p, 
						 const VectorXd& center,
						 const double eta,
						 const double rho_0,
						 const bool flag_project = false) {
	// Vector3d closest_point = closestPointSuperEllipse(a, b, p - center);
	// auto closest_point = projectToSuperellipse(p(0), p(1), a, b);
	// auto [distance, closest_point] = closestDistanceToSuperellipse(p, a, b);
	// auto [closest_point, distance] = closestPointToSuperellipseEigen(p, a, b, 100000);	// closestPointSuperEllipse(p, a, b);
	// auto [closest_point, distance] = closestPointToSuperellipseN8(p, a, b);
	auto closest_point = closestPointToSuperellipse(p, a, b);

	CLOSEST_POINT = closest_point;
	// std::cout << "closest point: " << closest_point.transpose() << "\n";
	// std::cout << "current robot: " << p.transpose() << "\n";
	CONSTRAINT_VECTOR = (p - closest_point.head(2)).normalized();

	if (flag_project) {
		// CONSTRAINT_VECTOR(0) = 0;
		// CONSTRAINT_VECTOR.normalize();
	}

	double distance = (p - closest_point.head(2)).norm();
	DISTANCE = distance;

	// std::cout << "current distance: " << distance << "\n";

	if (distance > rho_0) {
		// CONSTRAINT_VECTOR.setZero();
		// if (FIRST_ENTRY) {
			// FIRST_ENTRY = false;
		// }
		INSIDE_APF = false;
		return Vector2d::Zero();
	} else {
		FIRST_ENTRY = true;
		INSIDE_APF = true;

		Vector2d force = eta * ((1 / distance) - (1 / rho_0)) * (1 / (distance * distance)) * CONSTRAINT_VECTOR.normalized();

		// if (flag_project) {
			// force(0) = 0;
			// force(0) *= 0.3;
		// }

		return force.head(2);
	}
}

/*
	GRAPHICS
*/
chai3d::cMesh* createFilled2DSuperellipse_n4(
    double a, double b,
    int resolution = 100,
    double z_offset = 0,
    chai3d::cColorf color = chai3d::cColorf(),
    double transparency = 1.0,
    double thickness = 0.05)
{
    using namespace chai3d;

    cMesh* mesh = new cMesh();

    // Set material properties
    mesh->m_material->m_ambient  = color;
    mesh->m_material->m_diffuse  = color;
    mesh->m_material->m_specular = cColorf(1.0, 1.0, 1.0);
    mesh->setTransparencyLevel(transparency);
    mesh->m_material->setShininess(50);

    double n = 8;
    double half_thickness = 0.5 * thickness;

    // Centered top and bottom Z planes
    // double z_top = z_offset + half_thickness;
    // double z_bot = z_offset - half_thickness;
    double z_top = z_offset + thickness;
    double z_bot = z_offset;

    // Create top and bottom center vertices
    int centerTopIdx = mesh->newVertex(cVector3d(0.0, 0.0, z_top));
    int centerBotIdx = mesh->newVertex(cVector3d(0.0, 0.0, z_bot));
    mesh->m_vertices->setNormal(centerTopIdx, cVector3d(0, 0, 1));
    mesh->m_vertices->setNormal(centerBotIdx, cVector3d(0, 0, -1));

    std::vector<int> topIndices;
    std::vector<int> botIndices;

    for (int i = 0; i < resolution; ++i)
    {
        double theta = 2.0 * M_PI * i / resolution;

        double cos_t = cos(theta);
        double sin_t = sin(theta);

        double x = a * pow(fabs(cos_t), 2.0 / n) * ((cos_t >= 0) ? 1 : -1);
        double y = b * pow(fabs(sin_t), 2.0 / n) * ((sin_t >= 0) ? 1 : -1);

        int idxTop = mesh->newVertex(cVector3d(x, y, z_top));
        int idxBot = mesh->newVertex(cVector3d(x, y, z_bot));

        mesh->m_vertices->setNormal(idxTop, cVector3d(0, 0, 1));
        mesh->m_vertices->setNormal(idxBot, cVector3d(0, 0, -1));

        topIndices.push_back(idxTop);
        botIndices.push_back(idxBot);
    }

    // Top face
    for (int i = 0; i < resolution; ++i)
    {
        int idx1 = topIndices[i];
        int idx2 = topIndices[(i + 1) % resolution];
        mesh->newTriangle(centerTopIdx, idx1, idx2);
    }

    // Bottom face (reverse winding)
    for (int i = 0; i < resolution; ++i)
    {
        int idx1 = botIndices[i];
        int idx2 = botIndices[(i + 1) % resolution];
        mesh->newTriangle(centerBotIdx, idx2, idx1);
    }

    // Side walls
    for (int i = 0; i < resolution; ++i)
    {
        int top1 = topIndices[i];
        int top2 = topIndices[(i + 1) % resolution];
        int bot1 = botIndices[i];
        int bot2 = botIndices[(i + 1) % resolution];

        // First triangle of the quad
        mesh->newTriangle(top1, bot1, bot2);
        // Second triangle of the quad
        mesh->newTriangle(top1, bot2, top2);
    }

    return mesh;
}
// chai3d::cMesh* createFilled2DSuperellipse_n4(
//     double a, double b, int resolution = 100, double z_offset = 0,
//     chai3d::cColorf color = chai3d::cColorf(), double transparency = 1)
// {
//     using namespace chai3d;

//     cMesh* mesh = new cMesh();

//     // Properly set material colors
//     mesh->m_material->m_ambient  = color;
//     mesh->m_material->m_diffuse  = color;
//     mesh->m_material->m_specular = cColorf(1.0, 1.0, 1.0);
// 	mesh->setTransparencyLevel(transparency);
//     mesh->m_material->setShininess(50);

//     // Center vertex
//     int centerIdx = mesh->newVertex(cVector3d(0.0, 0.0, 0.0 + z_offset));
//     mesh->m_vertices->setNormal(centerIdx, cVector3d(0, 0, 1));

//     std::vector<int> boundaryIndices;
// 	double n = 8;

//     for (int i = 0; i < resolution; ++i)
//     {
//         double theta = 2.0 * M_PI * i / resolution;

//         double cos_t = cos(theta);
//         double sin_t = sin(theta);

//         double sign_x = (cos_t >= 0.0) ? 1.0 : -1.0;
//         double sign_y = (sin_t >= 0.0) ? 1.0 : -1.0;

//         // double x = a * sign_x * pow(fabs(cos_t), 0.5); // n = 4 → 2/n = 0.5
//         // double y = b * sign_y * pow(fabs(sin_t), 0.5);

// 		double x = a * pow(fabs(cos_t), 2.0 / n) * ((cos_t >= 0) ? 1 : -1);
//         double y = b * pow(fabs(sin_t), 2.0 / n) * ((sin_t >= 0) ? 1 : -1);

//         int idx = mesh->newVertex(cVector3d(x, y, 0.0 + z_offset));
//         mesh->m_vertices->setNormal(idx, cVector3d(0, 0, 1));
//         boundaryIndices.push_back(idx);
//     }

//     // Triangulate with fan from center
//     for (int i = 0; i < resolution; ++i)
//     {
//         int idx0 = centerIdx;
//         int idx1 = boundaryIndices[i];
//         int idx2 = boundaryIndices[(i + 1) % resolution];
//         mesh->newTriangle(idx0, idx1, idx2);
//     }

//     return mesh;
// }

int getSign(double value) {
    if (value > 0) {
        return 1;
    } else if (value < 0) {
        return -1;
    } else {
        return 0;
    }
}

// sigmoid function for velocity saturation
// alpha from 1 -> 0 as approaching constraint 
double getMaxVelFunction(const double alpha, const double entry_vel, const double exit_vel) {
    // std::cout << "alpha: " << alpha << "\n";
    return exit_vel + (entry_vel - exit_vel) * ((1 - cos(M_PI * (1 - alpha))) / 2);
}

/*
	Point Segment
*/
// Global or persistent trail object
cMultiSegment* g_trail = nullptr;

// Optional: store previous point
cVector3d g_lastPoint;
bool g_firstPoint = true;

void addTrailPoint(const cVector3d& newPoint)
{
    if (!g_trail)
    {
        std::cerr << "Error: Trail not initialized!" << std::endl;
        return;
    }

    if (!g_firstPoint)
    {
        // Add a new segment from last point to this new point
        g_trail->newSegment(g_lastPoint, newPoint);

    }

    g_lastPoint = newPoint;
    g_firstPoint = false;
	// chai3d::cColorf color;
	// color.setWhite();
	// g_trail->setLineColor(color);

    // // Optional: limit number of segments to avoid overflow
    // const int maxSegments = 200;
    // if (g_trail->getNumSegments() > maxSegments)
    // {
    //     g_trail->deleteSegment(0); // delete oldest
    // }
}

/*
	Point Trail
*/
cMultiPoint* pointTrail;

// Add a new point
void updatePointTrail(const cVector3d& newPoint) {
    pointTrail->newPoint(newPoint);
}

/*
	Force arrows
*/
chai3d::cMesh* force_arrow;
chai3d::cMesh* projected_force_arrow;
chai3d::cShapeCylinder* force_arrow_shaft;
chai3d::cShapeCylinder* projected_force_arrow_shaft;

Eigen::Matrix3d computeRotationAboutZ(const Eigen::Vector2d& v1, const Eigen::Vector2d& v2) {
    // Normalize the input vectors
    Eigen::Vector2d v1_norm = v1.normalized();
    Eigen::Vector2d v2_norm = v2.normalized();

    // Compute angle between vectors using atan2
    double angle1 = std::atan2(v1_norm.y(), v1_norm.x());
    double angle2 = std::atan2(v2_norm.y(), v2_norm.x());
    double theta = angle2 - angle1;

    // Create 3x3 rotation matrix about Z
    Eigen::Matrix3d R = Eigen::Matrix3d::Identity();
    R(0, 0) = std::cos(theta);
    R(0, 1) = -std::sin(theta);
    R(1, 0) = std::sin(theta);
    R(1, 1) = std::cos(theta);
    // R(2, 2) = 1 is already set

    return R;
}

Eigen::VectorXd removeComponentInDirection(const Eigen::VectorXd& v, const Eigen::VectorXd& d) {
    // Projection of v onto d
    Eigen::VectorXd proj = (v.dot(d) / d.dot(d)) * d;
    // Remove the component in direction d
    return v - proj;
}


void createArrow(cMesh* a_mesh, 
				 const double& a_length,
				 const double& a_radiusShaft,
				 const double& a_lengthTip,
				 const double& a_radiusTip,
				 const bool a_includeTipsAtBothExtremities,
				 const unsigned int a_numSides,
				 const cVector3d& a_direction,
				 const cVector3d& a_pos,
				 const cColorf& a_color,
				 const bool& cone_only = false)
{
    // sanity check
    if ((a_direction.length() == 0.0)   ||
        (a_length < 0)                  ||
        (a_radiusShaft < 0)             ||
        (a_lengthTip < 0)               ||
        (a_radiusTip > 360)) 
    { return; }
    
    // create rotation frame from direction vector
    cVector3d vx,vy,vz,t0,t1, t;;
    t0.set(1,0,0);
    t1.set(0,1,0);
    vz = cNormalize(a_direction);
    double ang0 = cAngle(vz, t0);
    double ang1 = cAngle(vz, t1);
    if (ang0>ang1)
    {
        t = cCross(vz, t0);  
    }
    else
    {
        t = cCross(vz, t1);
    }
    vy = cNormalize(t);
    vx = cNormalize(cCross(vy, vz));

    cMatrix3d rot;
    rot.setCol(vx, vy, vz);

    cVector3d offset0, offset1, offset2;
    double length;
    if (a_includeTipsAtBothExtremities)
    {
        length = a_length - 2.0 * a_lengthTip;
        offset0.set(0, 0, length + a_lengthTip);
        offset1.set(0, 0, a_lengthTip);
    }
    else
    {
        length = a_length - 1.0 * a_lengthTip;
        offset0.set(0, 0, length);
        offset1.set(0, 0, 0);
    }
    
    // create first tip
    cVector3d pos0 = cAdd(a_pos, cMul(rot, offset0));
    cMatrix3d rot0 = rot;

	// cMesh* mesh1 = a_mesh->newMesh();

	if (cone_only) {
		cCreateCone(a_mesh, 
					a_lengthTip,  
					a_radiusTip,
					0.0,
					a_numSides,
					1,
					1,
					true,
					false,
					a_pos,
					rot0,
					a_color);
	} else {
		cCreateCone(a_mesh, 
					a_lengthTip,  
					a_radiusTip,
					0.0,
					a_numSides,
					1,
					1,
					true,
					false,
					pos0,
					rot0,
					a_color);
	}

    // create arrow shaft
    cVector3d pos1 = cAdd(a_pos, cMul(rot, offset1));
    cMatrix3d rot1 = rot;

    if (length > 0)
    {
		if (!cone_only) {
			// cMesh* mesh2 = a_mesh->newMesh();
			cCreateCylinder(a_mesh, 
							length,  
							a_radiusShaft,
							a_numSides,
							1,
							1,
							true,
							true,
							pos1,
							rot1,
							a_color);
		}
    }

    // create possibly second tip
    if (a_includeTipsAtBothExtremities)
    {
        offset2.set(0, 0, a_lengthTip);
        cVector3d pos2 = cAdd(a_pos, cMul(rot, offset2));  
        cMatrix3d rot2p;
        rot2p.identity();
        rot2p.rotateAboutGlobalAxisDeg(cVector3d(1,0,0), 180);
        cMatrix3d rot2 = cMul(rot,rot2p);

		// cMesh* mesh3 = a_mesh->newMesh();
        cCreateCone(a_mesh, 
                    a_lengthTip,  
                    a_radiusTip,
                    0.0,
                    a_numSides,
                    1,
                    1,
                    true,
                    false,
                    pos2,
                    rot2,
                    a_color);
    }
}

double LENGTH;
double RADIUS;
double TIP_LENGTH;
double TIP_RADIUS;

/*
	Label
*/
chai3d::cLabel* label;

/*
	Closest distance line
*/
chai3d::cShapeLine* closest_distance_line;

//------------ main function
int main(int argc, char** argv) {
	Sai2Model::URDF_FOLDERS["EXAMPLE_998_FOLDER"] =
		string(EXAMPLES_FOLDER) + "/998-apf-comparison";
	cout << "Loading URDF world model file: " << world_file << endl;

	std::cout << yaml_fname << "\n";

	// set up signal handler
	signal(SIGABRT, &sighandler);
	signal(SIGTERM, &sighandler);
	signal(SIGINT, &sighandler);

	// load graphics scene
	auto graphics = make_shared<Sai2Graphics::Sai2Graphics>(world_file);
	graphics->addUIForceInteraction(robot_name);
    // graphics->setBackgroundColor(.678, .847, .902);
    graphics->setBackgroundColor(1, 1, 1);
	// graphics->showTransparency(true, robot_name, 0.5);

	graphics->getCamera("camera")->setFieldViewAngleDeg(45);

	/*
		YAML config
	*/
	YAML::Node config = YAML::LoadFile(yaml_fname);
	std::vector<double> tmp;

	tmp = config["object_config"]["start"].as<std::vector<double>>();
	START = Vector2d(tmp[0], tmp[1]);

	tmp = config["object_config"]["end"].as<std::vector<double>>();
	END = Vector2d(tmp[0], tmp[1]);

	tmp = config["object_config"]["goal"].as<std::vector<double>>();
	GOAL_POSITION = Vector3d(tmp[0], tmp[1], 0);
	
	ETA = config["object_config"]["eta"].as<double>();	
	RHO_0 = config["object_config"]["rho"].as<double>();
	RHO_1 = config["object_config"]["rho_1"].as<double>();
	VEL = config["object_config"]["vel"].as<double>();
	MIN_VEL = config["object_config"]["min_vel"].as<double>();
	MASS = config["object_config"]["mass"].as<double>();
	NULLSPACE_FLAG = config["object_config"]["nullspace"].as<bool>();
	EXIT_FLAG = config["object_config"]["exit"].as<bool>();
	SLOWDOWN_FLAG = config["object_config"]["vel_sat"].as<bool>();
	SF = config["object_config"]["sf"].as<double>();
	VEL_SF = config["object_config"]["vel_sf"].as<double>();
	KV = config["object_config"]["kv"].as<double>();
	T_WAIT = config["object_config"]["wait"].as<double>();

	// arrow dimensions
	LENGTH = config["object_config"]["length"].as<double>();
	RADIUS = config["object_config"]["radius"].as<double>();
	TIP_LENGTH = config["object_config"]["tip_length"].as<double>();
	TIP_RADIUS = config["object_config"]["tip_radius"].as<double>();
	ENABLE_ARROW = config["object_config"]["enable_arrow"].as<bool>();
	ENABLE_POSITION_CHANGE = config["object_config"]["enable_position_change"].as<bool>();
	T_POSITION_CHANGE = config["object_config"]["t_position_change"].as<double>();
	THICKNESS = config["object_config"]["thickness"].as<double>();
	
	// add super-ellipsoid
	// Create superellipse with a=0.2, b=0.1, n=4
	APF_A = config["object_config"]["a"].as<double>();
	APF_B = config["object_config"]["b"].as<double>();
	// SF = 4;
	// VEL_SF = 6;
	int resolution = 10000;

	auto color = chai3d::cColorf();
	// color.setGrayLight();
	color.setRedLightCoral();
	// color.setWhite();
	auto obstacle = createFilled2DSuperellipse_n4(APF_A, APF_B, resolution, 0 * (0.002 - 0.003), color, 1.0, THICKNESS);
	// obstacle->setLineWidth(2.0);

	int numSamples = 1000;

	std::vector<cVector3d> superellipse, outerBoundary, whiteOuterBoundary, velBoundary, whiteVelBoundary;
	generateSuperellipseWithOffset(APF_A, APF_B, RHO_0 + 0.001 * 0, numSamples, superellipse, outerBoundary);
	// generateSuperellipseWithOffset(APF_A, APF_B, RHO_0 + 0.001, numSamples, superellipse, whiteOuterBoundary);

	// Draw both
	// color.setGrayLight();
	// drawBoundaryLines(graphics->_world, superellipse, color);  // Green inner curve
	// color.setRed();
	// drawBoundaryLines(graphics->_world, outerBoundary, color); // Red outer boundary

	// color.setRed();
	// color.setBlueNavy();
	// color.setRed();
	color.setRedLightCoral();
	fillSuperellipseBand(graphics->_world,
						 superellipse,
						 outerBoundary,
						 color, 0.2, 0.002 * 0, THICKNESS * 0);  // light red fill

	// color.setWhite();
	// fillSuperellipseBand(graphics->_world,
	// 					 outerBoundary,
	// 					 whiteOuterBoundary,
	// 					 color, 0.2, 0.002 * 0, THICKNESS * 0);  // light red fill

	// color.setRed();
	// auto apf_zone = createFilled2DSuperellipse_n4(SF * 0.5 * APF_A, SF * APF_B, resolution, 0.001 - 0.003, color, 1.0);
	// apf_zone->setLineWidth(2.0);

	// color.setOrange();
	// auto vel_zone = createFilled2DSuperellipse_n4(VEL_SF * APF_A, VEL_SF * APF_B, resolution, 0 - 0.003, color, 0.3);

	graphics->_world->addChild(obstacle);
	// graphics->_world->addChild(apf_zone);

	if (SLOWDOWN_FLAG) {
		// graphics->_world->addChild(vel_zone);
		// color.setOrange();
		// color.setBlue();
		color = chai3d::cColorf(0.5020, 0.6667, 0.6902, 1);
		// color.setWhite();
		// color = cColorf(1.0f, 0.7f, 0.4f); // Light orange
		generateSuperellipseWithOffset(APF_A, APF_B, RHO_1, numSamples, outerBoundary, velBoundary);
		// drawBoundaryLines(graphics->_world, velBoundary, color); // Red outer boundary
		fillSuperellipseBand(graphics->_world,
							 outerBoundary,
							 velBoundary,
							 color,
							 0.1, 0.003 * 0, THICKNESS * 0);  // light red fill

		// color.setWhite();
		// generateSuperellipseWithOffset(APF_A, APF_B, RHO_1 + 0.001, numSamples, superellipse, whiteVelBoundary);
		// fillSuperellipseBand(graphics->_world,
		// 					 velBoundary,
		// 					 whiteVelBoundary,
		// 					 color,
		// 					 0.1, 0.003 * 0, THICKNESS * 0);  // light red fill	
	}

	cColorf light_green(0, 0, 0);
	cColorf force_color(0, 1, 1); // light blue

	// create cylinders for force arrows
	// force_arrow_shaft = new chai3d::cShapeCylinder(RADIUS, RADIUS, LENGTH);
	projected_force_arrow_shaft = new chai3d::cShapeCylinder(RADIUS, RADIUS, LENGTH);  // change scaling 
	projected_force_arrow_shaft->m_material->setColor(force_color);

	// force_arrow_shaft->setUseTransparency(true);
	projected_force_arrow_shaft->setUseTransparency(true);

	// force_arrow_shaft->setTransparencyLevel(0.0);
	projected_force_arrow_shaft->setTransparencyLevel(0.0);

	// projected_force_arrow_shaft->setTopRadius(0);
	// projected_force_arrow_shaft->setBaseRadius(0);

	// graphics->_world->addChild(force_arrow_shaft);
	// graphics->_world->addChild(projected_force_arrow_shaft);

	// create arrow if nullspace (to show the control forces, projected control force)
	force_arrow = new chai3d::cMesh();
	projected_force_arrow = new chai3d::cMesh();
	cVector3d direction = cVector3d(1, 0, 0);
	color.setBlack();
	createArrow(force_arrow, LENGTH, RADIUS, TIP_LENGTH, TIP_RADIUS, false, 100, direction, cVector3d(0, 0, 0), color);
	color.setBlue();
	createArrow(projected_force_arrow, LENGTH, RADIUS, TIP_LENGTH, TIP_RADIUS, false, 100, direction, cVector3d(0, 0, 0), force_color, true);
	force_arrow->setUseTransparency(true);
	projected_force_arrow->setUseTransparency(true);

	force_arrow->m_material->setColor(light_green);
	projected_force_arrow->m_material->setColor(force_color);
	// force_arrow->scaleXYZ(1, 1, 0);
	// projected_force_arrow->scaleXYZ(1, 1, 0);

	if (NULLSPACE_FLAG && ENABLE_ARROW) {
		graphics->_world->addChild(force_arrow);
		graphics->_world->addChild(projected_force_arrow);
		graphics->_world->addChild(projected_force_arrow_shaft);
	}

	// create line for closest distance 
	closest_distance_line = new chai3d::cShapeLine();
	// color.setGreenMediumSpring();
	// color.setWhite();
	// color = chai3d::cColorf(0.62, 0.50, 0.62);  // purple
	color = chai3d::cColorf(0.46, 0.82, 0.44);
	closest_distance_line->m_colorPointA = color;
	closest_distance_line->m_colorPointB = color;
	closest_distance_line->m_pointA = chai3d::cVector3d(100, 0, 0);
	closest_distance_line->m_pointB = chai3d::cVector3d(100, 0, 0);
	closest_distance_line->setLineWidth(10);
	graphics->_world->addChild(closest_distance_line);

	// load simulation world
	auto sim = make_shared<Sai2Simulation::Sai2Simulation>(world_file);
	sim->setJointPositions(robot_name, START);

	// load robots
	auto robot = make_shared<Sai2Model::Sai2Model>(robot_file, false);
	robot->setQ(sim->getJointPositions(robot_name));
	robot->updateModel();

    // std::cout << "ee pos: " << robot->position("link7", Vector3d(0, 0, 0)).transpose();
	// std::cout << "link 2 transform: " << robot->transform("link2").linear() << "\n" << robot->transform("link2").translation().transpose() << "\n";

	// intitialize global torques variables
	ui_torques = VectorXd::Zero(robot->dof());
	control_torques = VectorXd::Zero(robot->dof());

	g_trail = new cMultiSegment();
	// g_trail->m_colorLine.set(0.0, 0.0, 0.0); // green line
	g_trail->setLineWidth(10.0);
	color.setWhite();
	g_trail->setLineColor(color);
	// g_trail->setUseDisplayList(true);
	graphics->_world->addChild(g_trail);

	pointTrail = new cMultiPoint();
	// pointTrail->m_pointSize = 2.0;
	pointTrail->setPointSize(8.0);

	pointTrail->setShowEnabled(true);
	color.setOrange();
	pointTrail->setPointColor(color);
	graphics->_world->addChild(pointTrail);

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
			Affine3d T_object = Affine3d::Identity();
			T_object.translation() = Vector3d(CLOSEST_POINT(0), CLOSEST_POINT(1), 0);
			graphics->updateObjectGraphics("closest_point", T_object);
			Affine3d T_goal = Affine3d::Identity();
			T_goal.translation() = GOAL_POSITION;
			// graphics->updateObjectGraphics("goal", T_goal);

			closest_distance_line->m_pointA = chai3d::cVector3d(robot->q()(0), robot->q()(1), 0);
			closest_distance_line->m_pointB = chai3d::cVector3d(CLOSEST_POINT(0), CLOSEST_POINT(1), 0);
			
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
	// ctrl_thread.join();

	return 0;
}

//------------------------------------------------------------------------------
void control(shared_ptr<Sai2Model::Sai2Model> robot,
			 shared_ptr<Sai2Simulation::Sai2Simulation> sim) {
	// update robot model and initialize control vectors
	robot->updateModel();
	int dof = robot->dof();
	MatrixXd N_prec = MatrixXd::Identity(dof, dof);

	// motion task to control robot
	std::vector<Vector3d> controlled_directions_translation = {Vector3d::UnitX(), Vector3d::UnitY()};
	std::vector<Vector3d> controlled_directions_rotation = {};
	auto motion_task = make_unique<Sai2Primitives::MotionForceTask>(robot, "link1", controlled_directions_translation, controlled_directions_rotation);
	motion_task->enableVelocitySaturation(VEL);
	motion_task->setDynamicDecouplingType(Sai2Primitives::FULL_DYNAMIC_DECOUPLING);
	motion_task->disableSingularityHandling();
	motion_task->disableInternalOtg();
	motion_task->setPosControlGains(100, 20, 0);
	VectorXd motion_task_torques = VectorXd::Zero(dof);

	motion_task->setGoalPosition(Vector3d(END(0), END(1), 0));

	// // joint task to control the redundancy
	// // using default gains and interpolation settings
	// auto joint_task = make_unique<Sai2Primitives::JointTask>(robot);
    // joint_task->disableInternalOtg();
    // // joint_task->enableVelocitySaturation(1.5 * M_PI);
    // joint_task->enableVelocitySaturation(VEL);
    // // joint_task->enableVelocitySaturation(0.5);
    // // joint_task->enableVelocitySaturation(0.01);
    // joint_task->setGains(100, 20);
	// joint_task->setDynamicDecouplingType(Sai2Primitives::FULL_DYNAMIC_DECOUPLING);
	// VectorXd joint_task_torques = VectorXd::Zero(dof);

	// // VectorXd initial_q = robot->q();
    // joint_task->setGoalPosition(END);

	double t_initial = 2;
	vector<double> t_wait {5, 5};
    double prev_time = 0;
	int cnt = 0;

	// create logger
	Sai2Common::Logger logger("joints", false);
	VectorXd svalues = VectorXd::Zero(6);
    VectorXd robot_q = robot->q();
	VectorXd robot_dq = robot->dq();
	VectorXd robot_torque = VectorXd::Zero(robot->dof());
	VectorXi joint_pos_state = VectorXi::Zero(robot->dof());
	VectorXi joint_vel_state = VectorXi::Zero(robot->dof());
	// VectorXd joint_task_torques = VectorXd::Zero(robot->dof());
	int constraint_flag = 0;
	// logger.addToLog(svalues, "svalues");
    logger.addToLog(robot_q, "robot_q");
	logger.addToLog(robot_dq, "robot_dq");
	logger.addToLog(robot_torque, "robot_torque");
	// logger.addToLog(joint_pos_state, "joint_pos_state");
	// logger.addToLog(joint_vel_state, "joint_vel_state");
	// logger.start(1000);

	// create a loop timer
	double control_freq = 1000;
	Sai2Common::LoopTimer timer(control_freq, 1e6);
	double start_time = timer.elapsedSimTime();

	while (fSimulationRunning) {
		timer.waitForNextLoop();
		const double time = timer.elapsedSimTime();

		// read joint positions, velocities, update model
		robot->setQ(sim->getJointPositions(robot_name));
		robot->setDq(sim->getJointVelocities(robot_name));
		robot->updateModel();
		// MatrixXd M = robot->M();
		// M.bottomRightCorner(3, 3) += 0.15 * Matrix3d::Identity();
		// robot->updateModel(M);

        robot_q = robot->q();
		robot_dq = robot->dq();

		// updatePointTrail(Vector3d(robot_q(0), robot_q(1), 0));
		addTrailPoint(Vector3d(robot_q(0), robot_q(1), 0));

		// timed position change
		if (ENABLE_POSITION_CHANGE) {
			if (time - start_time > T_POSITION_CHANGE) {
				motion_task->setGoalPosition(Vector3d(robot->q()(0), robot->q()(1) - 0.5, 0));
				GOAL_POSITION = Vector3d(robot->q()(0), robot->q()(1) - 0.5, 0);
				ENABLE_POSITION_CHANGE = false;
			}
		}

		// velocity saturation check
		if (SLOWDOWN_FLAG) {
			double distance = distanceToSuperellipse(robot->q()(0), robot->q()(1), SF * APF_A, SF * APF_B);
			double distance_to_outer = distanceToSuperellipse(robot->q()(0), robot->q()(1), VEL_SF * APF_A, VEL_SF * APF_B);

			// if (distance < RHO_1 && isInsideSuperellipse(robot->q()(0), robot->q()(1), VEL_SF * APF_A, VEL_SF * APF_B)) {
			// if (distance < RHO_0 || isInsideSuperellipse(robot->q()(0), robot->q()(1), SF * APF_A, SF * APF_B)) {
			// if (isInsideSuperellipse(robot->q()(0), robot->q()(1), SF * APF_A, SF * APF_B)) {
			if (DISTANCE <= RHO_0) {
			// if (isInsideSuperellipse(robot->q()(0), robot->q()(1), SF * APF_A, SF * APF_B)) {
				motion_task->enableVelocitySaturation(MIN_VEL);
			// } else if (distance < RHO_1 || isInsideSuperellipse(robot->q()(0), robot->q()(1), VEL_SF * APF_A, VEL_SF * APF_B)) {
			// } else if (isInsideSuperellipse(robot->q()(0), robot->q()(1), VEL_SF * APF_A, VEL_SF * APF_B)) {
			} else if (DISTANCE <= RHO_1) {
				// std::cout << (distance - RHO_0) / (RHO_1 - RHO_0) << "\n";
				// std::cout << RHO_0 << ", " << RHO_1 << "\n";
				// double alpha = std::clamp(std::abs((distance - RHO_0) / (RHO_1 - RHO_0)), 0.0, 1.0);
				// double alpha = std::clamp(std::abs((distance_to_outer) / (distance_to_outer + distance)), 0.0, 1.0);
				double alpha = 1 - std::clamp((DISTANCE - RHO_0) / (RHO_1 - RHO_0), 0.0, 1.0);
				// alpha = 0;
				double max_vel = getMaxVelFunction(alpha, VEL, MIN_VEL);
				// std::cout << "alpha: " << alpha << "\n";
				// std::cout << "max vel: " << max_vel << "\n";
				motion_task->enableVelocitySaturation(max_vel);
			} else {
				motion_task->enableVelocitySaturation(VEL);
			}
		}

		// update tasks model. Order is important to define the hierarchy
		N_prec = MatrixXd::Identity(dof, dof);
		motion_task->updateTaskModel(N_prec);
		auto motion_task_torques_without_nullspace = motion_task->computeTorques();

		/*
			If nullspace option, update nullspace
		*/
	    VectorXd damping_torques = VectorXd::Zero(2);
		MatrixXd J_con = MatrixXd::Zero(1, 2);
		if (NULLSPACE_FLAG) {
			// if (FIRST_ENTRY && isInsideSuperellipse(robot->q()(0), robot->q()(1), SF * APF_A, SF * APF_B)) {
			// if (isInsideSuperellipse(robot->q()(0), robot->q()(1), SF * APF_A, SF * APF_B) && DISTANCE <= RHO_0) {
			// std::cout << "distance: " << DISTANCE << "\n";
			if (DISTANCE < RHO_0) {
				// std::cout << "constraint vector: " << CONSTRAINT_VECTOR.transpose() << "\n";
				// J_con = Vector3d(CONSTRAINT_VECTOR(0), CONSTRAINT_VECTOR(1), 0).transpose() * robot->Jv("link1");
				J_con = CONSTRAINT_VECTOR.transpose() * robot->Jv("link1");  // 1 x 2

				// project down
				MatrixXd U = Sai2Model::matrixRangeBasis(J_con);
				J_con = U.transpose() * J_con;

				N_prec = robot->nullspaceMatrix(J_con);
				// std::cout << "N_prec: " << N_prec << "\n";

				if (isnan(N_prec(0, 0))) {
					throw runtime_error("nan");
				}

				Vector2d force_bias = 0 * Vector2d(0, 1);
				damping_torques = - KV * J_con * robot->dq();	

				/*
					Update arrows
				*/		
				projected_force_arrow_shaft->setTransparencyLevel(1.0);
				projected_force_arrow_shaft->setLocalPos(robot->q()(0), robot->q()(1), 0);
				// force_arrow_shaft->setTransparencyLevel(1.0);
				// force_arrow_shaft->setLocalPos(robot->q()(0), robot->q()(1), 0);

				// arrows
				if (ENABLE_ARROW) {
					projected_force_arrow->setTransparencyLevel(1.0);
					force_arrow->setTransparencyLevel(1.0);
				} else {
					projected_force_arrow->setTransparencyLevel(0.0);
					force_arrow->setTransparencyLevel(0.0);
				}
				// rotation matrix about z axis 
				// Vector2d force_vector = (Vector2d(END(0), END(1)) - Vector2d(robot->q()(0), robot->q()(1)));
				Vector2d force_vector = motion_task_torques_without_nullspace;

				// project force vector onto orthogonal 
				auto projected_force_vector = removeComponentInDirection(force_vector, CONSTRAINT_VECTOR);

				Matrix3d force_rot = computeRotationAboutZ(Vector2d(1, 0), projected_force_vector);
				double force_scaling = projected_force_vector.norm() / force_vector.norm();
				// std::cout << force_scaling << "\n";
				// projected_force_arrow->scale(LENGTH * force_scaling);  // resize projected force arrow based on the projected force vector ratio 

				// get revised arrow head position based on the scaled length
				Vector3d local_pos = Vector3d(robot->q()(0), robot->q()(1), 0) + force_scaling * LENGTH * Vector3d(projected_force_vector(0), projected_force_vector(1), 0).normalized();

				// Resize shaft
				// projected_force_arrow->getMesh(0)->scale(force_scaling * LENGTH);

				// Resize tip
				// projected_force_arrow->setLocalRot(cMatrix3d(Matrix3d::Identity()));
				// projected_force_arrow->getMesh(0)->scaleXYZ(1, 1, 0);
				// projected_force_arrow->getMesh(1)->scaleXYZ(0, 0, 0);
				// projected_force_arrow->setLocalPos(robot->q()(0), robot->q()(1), 0);
				projected_force_arrow->setLocalPos(local_pos(0), local_pos(1), local_pos(2));

				// Move tip to the end of shaft
				// projected_force_arrow->getMesh(1)->setLocalPos(0, 0, LENGTH);

				projected_force_arrow->setLocalRot(cMatrix3d(force_rot));

				Matrix3d rot_y = AngleAxisd(M_PI / 2, Vector3d::UnitY()).toRotationMatrix();
				projected_force_arrow_shaft->setLocalRot(cMatrix3d(force_rot * rot_y));
				projected_force_arrow_shaft->setHeight(LENGTH * force_scaling);

				force_arrow->setLocalPos(robot->q()(0), robot->q()(1), 0);
				force_rot = computeRotationAboutZ(Vector2d(1, 0), force_vector);
				force_arrow->setLocalRot(cMatrix3d(force_rot));

			} else {
				// force_arrow->setLocalPos(cVector3d(100, 0, 0));

				force_arrow->setLocalPos(robot->q()(0), robot->q()(1), 0);
				Vector2d force_vector = Vector2d(END(0), END(1)) - Vector2d(robot->q()(0), robot->q()(1));
				Matrix3d force_rot = computeRotationAboutZ(Vector2d(1, 0), force_vector);
				force_arrow->setLocalRot(cMatrix3d(force_rot));

				projected_force_arrow_shaft->setTransparencyLevel(0.0);
				projected_force_arrow->setTransparencyLevel(0.0);
				force_arrow->setTransparencyLevel(0.0);
			}
		}

		if (DISTANCE < RHO_0) {
			J_con = CONSTRAINT_VECTOR.transpose() * robot->Jv("link1");  // 1 x 2
			damping_torques = - KV * J_con * robot->dq();
		}

		// check exit flag 
		if (EXIT_FLAG) {
			Vector2d control_force = GOAL_POSITION.head(2) - robot->q().head(2);
			// std::cout << "control force: " << control_force.transpose() << "\n";
			// std::cout << "constraint vector: " << CONSTRAINT_VECTOR.transpose() << "\n";
			if (CONSTRAINT_VECTOR.dot(-control_force.normalized()) < 0 && DISTANCE < RHO_0) {
				N_prec = MatrixXd::Identity(dof, dof);
				damping_torques.setZero();
				// if (!isInsideSuperellipse(robot->q()(0), robot->q()(1), SF * APF_A, SF * APF_B)) {
					// damping_torques.setZero();
				// }
				// projected_force_arrow->setTransparencyLevel(0.0);
				// projected_force_arrow_shaft->setTransparencyLevel(0.0);
			} 
		}

		// try with original and nullspace projection only 
		{
			// lock_guard<mutex> lock(mutex_robot);
			motion_task->updateTaskModel(N_prec);
			// motion_force_task->updateTaskModel(N_prec);
		}

		if (DISTANCE > RHO_0) {
			projected_force_arrow->setTransparencyLevel(0.0);
		}

		// joint_task->setGoalPosition(END);
		motion_task_torques = motion_task->computeTorques();     

		// special case for normal apf (KEEP PROJECTION IN THE (0, 1) DIRECTION)
		bool flag_project = false;
		if (robot->q()(0) < 0.4) {
			flag_project = true;
		}

		std::cout << robot->q().transpose() << "\n";

		// compute apf torques 
		auto apf_torque = computeApfForce(APF_A, APF_B, robot->q(), APF_CENTER, ETA, RHO_0, flag_project);  // 2 x 1 force vector 
		std::cout << "apf torque: " << apf_torque.transpose() << "\n";

		// J_con = CONSTRAINT_VECTOR.transpose();

		// // project down
		// MatrixXd U = Sai2Model::matrixRangeBasis(J_con);
		// J_con = U.transpose() * J_con;

		// std::cout << "J_con: " << J_con << "\n";

		// if (!isInsideSuperellipse(robot->q()(0), robot->q()(1), SF * APF_A, SF * APF_B)) {
		// 	apf_torque.setZero();
		// }
		// std::cout << "apf torque: " << apf_torque.transpose() << "\n";

		Vector2d force_bias = 3 * Vector2d(0, 1);
		if (DISTANCE > RHO_0) {
			force_bias.setZero();
		}

		//------ compute the final torques
		{
			lock_guard<mutex> lock(mutex_torques);
			if (time - start_time > T_WAIT) {
				control_torques = motion_task_torques + MASS * apf_torque + J_con.transpose() * MASS * damping_torques;
				// control_torques = motion_task_torques + J_con.transpose() * MASS * apf_torque;
				// control_torques = N_prec.transpose() * motion_task_torques + MASS * apf_torque + J_con.transpose() * MASS * damping_torques;  // not baseline 
				// control_torques = motion_task_torques + MASS * apf_torque + MASS * damping_torques;
				// std::cout << "motion task: " << motion_task_torques.transpose() << "\n";
				// std::cout << "apf torque: " << apf_torque.transpose() << "\n";
				// std::cout << control_torques.transpose() << "\n";
				// force_arrow->setTransparencyLevel(1.0);
				if (motion_task->goalPositionReached(5e-2)) {
					force_arrow->setTransparencyLevel(0.0);					
				}
			} else {
				force_arrow->setTransparencyLevel(0.0);
			}
		}
		robot_torque = control_torques;

		// // log joint state 
		// auto joint_states = joint_handler->getJointState();
		// joint_pos_state = joint_states.first;
		// joint_vel_state = joint_states.second;

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
	double sim_freq = 1000;
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