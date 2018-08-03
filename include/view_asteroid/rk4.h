

#ifndef RK4_H_
#define RK4_H_

#include <Eigen/Dense>

// ROS message types
#include "geometry_msgs/Vector3.h"
#include "view_asteroid/helper.h"

class rk4 {  // Runge-kutta for time-invariant systems
 public:

 	// Constructors
  rk4();
 	rk4(const Eigen::Vector3d &omega0,
        const double &quat_scalar,
        const Eigen::Vector3d &quat_vec,
        const Eigen::Matrix3d &J);

 	// Methods
	void DifferentialEquation(const Eigen::Vector3d &torque,
                            const Eigen::Vector3d &omega,
                            const Eigen::Vector4d &quat,
		  				              Eigen::Vector3d *omega_dot,
                            Eigen::Vector4d *quat_dot);
 	void UpdateStates(const double &dt, const Eigen::Vector3d &torque);
  void GetStates(Eigen::Vector3d *omega,
                 Eigen::Quaterniond *quat);
  void SetAngularVelocity(Eigen::Vector3d &omega);
    // void GetStates(geometry_msgs::Vector3 *state);

    Eigen::Vector3d omega_;
    Eigen::Vector4d quat_;
    Eigen::Matrix3d J_;
    Eigen::Matrix3d J_inv_;
};


#endif  // ML_CLASS_H_