#include "plane.hpp"

Plane::Plane(double phi, double theta, double psi) {
  vel_body << 60 * 6076 / 3600.0f, 0, 0;

  w = get_angular_vel(0);
  euler_dot = get_rate_of_change_of_euler(0, 0, w.x(), w.y(), w.z());
  euler << 0, 0, 0;
  vel_NED = vel_body;
  pos_NED << 0, 0, 0;

  DCM << 1, 0, 0, 0, 1, 0, 0, 0, 1;

  ws.push_back(get_angular_vel(time));
  euler_dots.push_back(euler_dot);
  eulers.push_back(euler);
  vel_NEDs.push_back(vel_NED);
  pos_NEDs.push_back(pos_NED);
}

Plane::~Plane() {}

Eigen::Vector3d Plane::get_angular_vel(double time) {
  Eigen::Vector3d w_temp;
  w_temp << M_PI / 6, cos(6 / M_PI * time), 3 * sin(30 / M_PI * time);
  return w_temp;
}

Eigen::Vector3d Plane::get_rate_of_change_of_euler(double phi, double theta, double p, double q, double r) {
  Eigen::Vector3d wdot;
  
  Eigen::Matrix3d gimbal_eqn_matrix;
  gimbal_eqn_matrix << 1, tan(theta) * sin(phi), tan(theta) * cos(phi),
                        0, cos(phi), -sin(phi),
                        0, sin(phi) / cos(theta), cos(phi) / cos(theta);

  Eigen::Vector3d w;
  w << p, q, r;

  wdot <<  gimbal_eqn_matrix * w;
  return wdot;
}

std::tuple<Eigen::Vector3d, Eigen::Vector3d, Eigen::Matrix3d> Plane::get_euler_dot_and_c_dot(
  Eigen::Vector3d euler, 
  Eigen::Matrix3d DCM, 
  Eigen::Vector3d vel_body, double time) {
  Eigen::Vector3d w_temp = get_angular_vel(time);
  
  double p = w_temp.x();
  double q = w_temp.y();
  double r = w_temp.z();

  Eigen::Vector3d euler_dot_temp = get_rate_of_change_of_euler(
    euler.x(), euler.y(),
    p, q, r
  );

  Eigen::Matrix3d DCM_update;
  DCM_update << 0, -r, q, r, 0, -p, -q, p, 0;

  Eigen::Matrix3d DCM_dot = DCM * DCM_update;

  Eigen::Vector3d vel_ned = DCM * vel_body;

  return std::make_tuple(euler_dot_temp, vel_ned, DCM_dot);
}

std::tuple<Eigen::Vector3d, Eigen::Vector3d, Eigen::Matrix3d> operator+(
  const std::tuple<Eigen::Vector3d, Eigen::Vector3d, Eigen::Matrix3d> &lhs,
  const std::tuple<Eigen::Vector3d, Eigen::Vector3d, Eigen::Matrix3d> &rhs
) {
  Eigen::Vector3d euler_dot1, euler_dot2;
  Eigen::Vector3d vel_ned1, vel_ned2;
  Eigen::Matrix3d DCM_dot1, DCM_dot2;

  std::tie(euler_dot1, vel_ned1, DCM_dot1) = lhs;
  std::tie(euler_dot2, vel_ned2, DCM_dot2) = rhs;

  return std::make_tuple(euler_dot1 + euler_dot2, 
    vel_ned1 + vel_ned2, 
    DCM_dot1 + DCM_dot2);
}

std::tuple<Eigen::Vector3d, Eigen::Vector3d, Eigen::Matrix3d> operator*(
  const double &m,
  std::tuple<Eigen::Vector3d, Eigen::Vector3d, Eigen::Matrix3d> &rhs
) {
  Eigen::Vector3d euler_dot;
  Eigen::Vector3d vel_ned;
  Eigen::Matrix3d DCM_dot;

  std::tie(euler_dot, vel_ned, DCM_dot) = rhs;

  return std::make_tuple(m * euler_dot, 
    m * vel_ned, 
    m * DCM_dot);
}

void Plane::fly(double dt) {
  Eigen::Matrix3d DCM_dot;

  std::tuple<Eigen::Vector3d, Eigen::Vector3d, Eigen::Matrix3d> dot1 = get_euler_dot_and_c_dot(
    euler, DCM, vel_body, time
  );
  std::tie(euler_dot, vel_NED, DCM_dot) = dot1;
  std::tuple<Eigen::Vector3d, Eigen::Vector3d, Eigen::Matrix3d> dot2 = get_euler_dot_and_c_dot(
    euler + euler_dot * dt / 2, DCM + DCM_dot * dt / 2, vel_body, time + dt / 2
  );
  std::tie(euler_dot, vel_NED, DCM_dot) = dot2;
  std::tuple<Eigen::Vector3d, Eigen::Vector3d, Eigen::Matrix3d> dot3 = get_euler_dot_and_c_dot(
    euler + euler_dot * dt / 2, DCM + DCM_dot * dt / 2, vel_body, time + dt / 2
  );
  std::tie(euler_dot, vel_NED, DCM_dot) = dot3;
  std::tuple<Eigen::Vector3d, Eigen::Vector3d, Eigen::Matrix3d> dot4 = get_euler_dot_and_c_dot(
    euler + euler_dot * dt, DCM + DCM_dot * dt, vel_body, time + dt
  );

  double m = (1.0f / 6.0f);
  std::tuple<Eigen::Vector3d, Eigen::Vector3d, Eigen::Matrix3d> dot =
    (dot1 + (2.0f * dot2) + (2.0f * dot3) + dot4);
  dot = m * dot;

  std::tie(euler_dot, vel_NED, DCM_dot) = dot;

  pos_NED += vel_NED * dt;
  DCM += DCM_dot * dt;
  euler += euler_dot * dt;
  time += dt;

  euler.x() = fmod(euler.x(), 2 * M_PI);
  euler.y() = fmod(euler.y(), 2 * M_PI);
  euler.z() = fmod(euler.z(), 2 * M_PI);

  ws.push_back(get_angular_vel(time));
  euler_dots.push_back(euler_dot);
  eulers.push_back(euler);
  vel_NEDs.push_back(vel_NED);
  pos_NEDs.push_back(pos_NED);

  times.push_back(time);
}