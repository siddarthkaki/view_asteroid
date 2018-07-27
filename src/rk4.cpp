#include "view_asteroid/rk4.h"

// Runge-kutta class ----------------------------------------------
rk4::rk4() {

}

rk4::rk4(const Eigen::Vector3d &omega0,
         const double &quat_scalar,
         const Eigen::Vector3d &quat_vec,
         const Eigen::Matrix3d &J) {
	omega_ = omega0;
	quat_[0] = quat_scalar;
	quat_[1] = quat_vec[0];
	quat_[2] = quat_vec[1];
	quat_[3] = quat_vec[2];
	quat_.normalize();
	J_ = J;
}

void rk4::DifferentialEquation(const Eigen::Vector3d &omega,
                               const Eigen::Vector4d &quat,
		  				       Eigen::Vector3d *omega_dot,
                               Eigen::Vector4d *quat_dot) {

	*omega_dot = -helper::skew(omega)*J_*omega;

	Eigen::Matrix4d M;
	M <<      0.0, -omega[0], -omega[1], -omega[2],
	     omega[0],       0.0,  omega[2], -omega[1],
	     omega[1], -omega[2],       0.0,  omega[0],
	     omega[2],  omega[1], -omega[0],       0.0;
	*quat_dot = 0.5*M*quat;
	// *quat_dot = quat;
}

void rk4::UpdateStates(const double &dt) {
	const double dt_half = dt/2.0;
	Eigen::Vector3d kw1, kw2, kw3, kw4;
	Eigen::Vector4d kq1, kq2, kq3, kq4;

	this->DifferentialEquation(omega_,               quat_,               &kw1, &kq1);
	this->DifferentialEquation(omega_ + dt_half*kw1, quat_ + dt_half*kq1, &kw2, &kq2);
	this->DifferentialEquation(omega_ + dt_half*kw2, quat_ + dt_half*kq2, &kw3, &kq3);
	this->DifferentialEquation(omega_ + dt*kw3,      quat_ + dt*kq3,      &kw4, &kq4);

	Eigen::Vector3d deltaW = dt*(kw1 + 2.0*kw2 + 2.0*kw3 + kw4)/6.0;
	Eigen::Vector4d deltaQ = dt*(kq1 + 2.0*kq2 + 2.0*kq3 + kq4)/6.0;
	omega_ = omega_ + deltaW;
	quat_ = (quat_ + deltaQ).normalized();
}

void rk4::GetStates(Eigen::Vector3d *omega,
                    Eigen::Quaterniond *quat) {
	*omega = omega_;
	*quat = Eigen::Quaterniond(quat_[0], quat_[1], quat_[2], quat_[3]);
}

// void rk4::GetState(geometry_msgs::Vector3 *state) {
// 	Eigen::Vector3d eigen_state;
// 	this->GetState(&eigen_state);
// 	state->x = eigen_state[0];
// 	state->y = eigen_state[1];
// 	state->z = eigen_state[2];
// }