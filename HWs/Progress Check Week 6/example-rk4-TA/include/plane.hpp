#ifndef PLANE_HPP
#define PLANE_HPP

#include <cmath>
#include <vector>

#include <Eigen/Core>

class Plane {
public:
  Plane(double phi, double theta, double psi);

  ~Plane();

  /**
   * @brief  Simulate the flying using numerical integration
   * @note   
   * @param  dt: time interval
   * @retval None
   */
  void fly(double dt);

private:
  /**
   * @brief  Get angular velcity vector
   * @note   
   * @param  time: current flight time
   * @retval angular velocity
   */
  Eigen::Vector3d get_angular_vel(double time);

  /**
   * @brief  Calculate the rate of change of the aircraft Euler angles using gimbal equation
   * @note   
   * @param  phi: euler angle PHI of current plane orientaion
   * @param  theta: euler angle THETA of current plane orientaion
   * @retval the rate of change of the aircraft Euler angles
   */
  Eigen::Vector3d get_rate_of_change_of_euler(double phi, double theta, double p, double q, double r);

  /**
   * @brief  This method is used to return the value of 
   * @note   
   * @param  w: angular velocity
   * @param  euler: eular angle 
   * @param  DCM: direction cosine matrix
   * @param  vel_body: velocity under body frame which is constant
   * @param  time: total flight time
   * @retval tuple of state dots and DCM dot
   */
  std::tuple<Eigen::Vector3d, Eigen::Vector3d, Eigen::Matrix3d> get_euler_dot_and_c_dot(
    Eigen::Vector3d euler, 
    Eigen::Matrix3d DCM, 
    Eigen::Vector3d vel_body, double time);

public:
  /**
   * States:
   * p, q, r
   * phi_dot, theta_dot, psi _dot
   * phi, theta, psi 
   * vel_x, vel_y, vel_z,
   * pos_x, pos_y, pos_z
   * 
   * vel and pos are expressed under NED frame
   */
  Eigen::Vector3d w;
  Eigen::Vector3d euler_dot;
  Eigen::Vector3d euler;
  Eigen::Vector3d vel_NED;
  Eigen::Vector3d pos_NED;

  Eigen::Vector3d vel_body;

  Eigen::Matrix3d DCM;

  double time;

  std::vector<Eigen::Vector3d> ws, euler_dots, eulers, vel_NEDs, pos_NEDs;

  std::vector<double> times;
};

#endif /* PLANE_HPP */
