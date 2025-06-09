#ifndef SAI_PRIMITIVES_COMMON_DEFINITIONS_H_
#define SAI_PRIMITIVES_COMMON_DEFINITIONS_H_

#include <Eigen/Dense>
#include <vector>

namespace SaiPrimitives {

/**
 * @brief Enum to define the type of dynamic decoupling to be used in the
 * impedance controller
 *
 */
enum DynamicDecouplingType {
	FULL_DYNAMIC_DECOUPLING,	// use the real Mass matrix
	BOUNDED_INERTIA_ESTIMATES,	// use a Mass matrix computed from
								// saturating the minimal values of the Mass
								// Matrix
	IMPEDANCE,					// use Identity for the Mass matrix
};

/**
 * @brief structure to store the gains of a PID controller
 * 
 */
// struct PIDGains {
// 	double kp;
// 	double kv;
// 	double ki;

// 	PIDGains(double kp, double kv, double ki) : kp(kp), kv(kv), ki(ki) {}
// };
struct PIDGains {
	Eigen::Vector3d kp;
	Eigen::Vector3d kv;
	Eigen::Vector3d ki;

	PIDGains(const Eigen::Vector3d& kp, const Eigen::Vector3d& kv, const Eigen::Vector3d& ki)
		: kp(kp), kv(kv), ki(ki) {}

	PIDGains(double kp_scalar, double kv_scalar, double ki_scalar)
		: kp(Eigen::Vector3d::Constant(kp_scalar)),
		  kv(Eigen::Vector3d::Constant(kv_scalar)),
		  ki(Eigen::Vector3d::Constant(ki_scalar)) {}
};
/// @brief get a vector of P gains from a vector of PIDGains
Eigen::VectorXd extractKpFromGainVector(const std::vector<PIDGains>& gains);
/// @brief get a vector of D gains from a vector of PIDGains
Eigen::VectorXd extractKvFromGainVector(const std::vector<PIDGains>& gains);
/// @brief get a vector of I gains from a vector of PIDGains
Eigen::VectorXd extractKiFromGainVector(const std::vector<PIDGains>& gains);

}  // namespace SaiPrimitives

#endif	// SAI_PRIMITIVES_COMMON_DEFINITIONS_H_
