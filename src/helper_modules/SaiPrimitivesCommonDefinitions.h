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
struct PIDGains {
	double kp;
	double kv;
	double ki;

	PIDGains(double kp, double kv, double ki) : kp(kp), kv(kv), ki(ki) {}
};

/// @brief get a vector of P gains from a vector of PIDGains
Eigen::VectorXd extractKpFromGainVector(const std::vector<PIDGains>& gains);
/// @brief get a vector of D gains from a vector of PIDGains
Eigen::VectorXd extractKvFromGainVector(const std::vector<PIDGains>& gains);
/// @brief get a vector of I gains from a vector of PIDGains
Eigen::VectorXd extractKiFromGainVector(const std::vector<PIDGains>& gains);

}  // namespace SaiPrimitives

#endif	// SAI_PRIMITIVES_COMMON_DEFINITIONS_H_
