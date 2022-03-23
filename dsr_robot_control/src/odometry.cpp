#include "../include/odometry.h"
#include <boost/bind.hpp>

namespace dsr_controller
{
  namespace bacc = boost::accumulators;

  Odometry::Odometry(size_t velocity_rolling_window_size)
  : timestamp_(0.0)
  , x_(0.0)
  , y_(0.0)
  , heading_(0.0)
  , linear_(0.0)
  , angular_(0.0)
  , velocity_rolling_window_size_(velocity_rolling_window_size)
  , linear_acc_(RollingWindow::window_size = velocity_rolling_window_size)
  , angular_acc_(RollingWindow::window_size = velocity_rolling_window_size)
  {
  }

  void Odometry::init(const ros::Time& time)
  {
    // Reset accumulators and timestamp:
    resetAccumulators();
    timestamp_ = time;
  }

  bool Odometry::update(double steer_front_pos, double steer_rear_pos, double drive_front_pos, double drive_rear_pos, const ros::Time &time)
  {
    /// Get current wheel joint positions:
    const double steer_front_wheel_cur_pos  = steer_front_pos;
    const double steer_rear_wheel_cur_pos   = steer_rear_pos;
    const double drive_front_wheel_cur_pos  = drive_front_pos  * wheel_diameter_/2.0;
    const double drive_rear_wheel_cur_pos   = drive_rear_pos * wheel_diameter_/2.0;

    /// Estimate velocity of wheels using old and current position:
    const double steer_front_wheel_est_vel  = steer_front_wheel_cur_pos  - steer_front_wheel_old_pos_;
    const double steer_rear_wheel_est_vel   = steer_rear_wheel_cur_pos - steer_rear_wheel_old_pos_;
    const double drive_front_wheel_est_vel  = drive_front_wheel_cur_pos  - drive_front_wheel_old_pos_;
    const double drive_rear_wheel_est_vel   = drive_rear_wheel_cur_pos - drive_rear_wheel_old_pos_;

    /// Integrate odometry:
    integrateXY(steer_front_wheel_est_vel, steer_rear_wheel_est_vel, drive_front_wheel_est_vel, drive_rear_wheel_est_vel);

    /// Update old position with current:
    steer_front_wheel_old_pos_  = steer_front_wheel_cur_pos;
    steer_rear_wheel_old_pos_   = steer_rear_wheel_cur_pos;
    drive_front_wheel_old_pos_  = drive_front_wheel_cur_pos;
    drive_rear_wheel_old_pos_   = drive_rear_wheel_cur_pos;
    

    // /// We cannot estimate the speed with very small time intervals:
    // const double dt = (time - timestamp_).toSec();
    // if (dt < 0.0001)
    //   return false; // Interval too small to integrate with

    // timestamp_ = time;

    // /// Estimate speeds using a rolling mean to filter them out:
    // linear_acc_(linear/dt);
    // angular_acc_(angular/dt);

    // linear_ = bacc::rolling_mean(linear_acc_);
    // angular_ = bacc::rolling_mean(angular_acc_);

    return true;
  }

  void Odometry::updateOpenLoop(double linear, double angular, const ros::Time &time)
  {
    /// Save last linear and angular velocity:
    linear_ = linear;
    angular_ = angular;

    /// Integrate odometry:
    const double dt = (time - timestamp_).toSec();
    timestamp_ = time;
    integrate_fun_(linear * dt, angular * dt);
  }

  void Odometry::setParams(double wheel_diameter, double h)
  {
    wheel_diameter_ = wheel_diameter;
    h_ = h;
  }

  void Odometry::setVelocityRollingWindowSize(size_t velocity_rolling_window_size)
  {
    velocity_rolling_window_size_ = velocity_rolling_window_size;

    resetAccumulators();
  }

  void Odometry::integrateRungeKutta2(double linear, double angular)
  {
    const double direction = heading_ + angular * 0.5;

    /// Runge-Kutta 2nd order integration:
    x_       += linear * cos(direction);
    y_       += linear * sin(direction);
    heading_ += angular;
  }

  /**
   * \brief Other possible integration method provided by the class
   * \param linear
   * \param angular
   */
  void Odometry::integrateExact(double linear, double angular)
  {
    if (fabs(angular) < 1e-6)
      integrateRungeKutta2(linear, angular);
    else
    {
      /// Exact integration (should solve problems when angular is zero):
      const double heading_old = heading_;
      const double r = linear/angular;
      heading_ += angular;
      x_       +=  r * (sin(heading_) - sin(heading_old));
      y_       += -r * (cos(heading_) - cos(heading_old));
    }
  }

  void Odometry::integrateXY(double steer_front_diff_pos, double steer_rear_diff_pos, double drive_front_diff_pos, double drive_rear_diff_pos)
  {
    const double dx_front_wheel_pos = drive_front_diff_pos * cos(steer_front_wheel_old_pos_ + steer_front_diff_pos/2);
    const double dy_front_wheel_pos = drive_front_diff_pos * sin(steer_front_wheel_old_pos_ + steer_front_diff_pos/2);
    const double dx_rear_wheel_pos = drive_rear_diff_pos * cos(steer_rear_wheel_old_pos_ + steer_rear_diff_pos/2);
    const double dy_rear_wheel_pos = drive_rear_diff_pos * sin(steer_rear_wheel_old_pos_ + steer_rear_diff_pos/2);

    double dx_pos    = (dx_front_wheel_pos + dx_rear_wheel_pos)/2.0;
    double dy_pos    = (dy_front_wheel_pos + dy_rear_wheel_pos)/2.0;
    double dheading_pos  = (dy_front_wheel_pos - dy_rear_wheel_pos)*h_;

    heading_ += dheading_pos;
    x_ += dx_pos*cos(heading_) - dy_pos*sin(heading_);
    y_ += dx_pos*sin(heading_) + dy_pos*cos(heading_);
  }

  void Odometry::resetAccumulators()
  {
    linear_acc_ = RollingMeanAcc(RollingWindow::window_size = velocity_rolling_window_size_);
    angular_acc_ = RollingMeanAcc(RollingWindow::window_size = velocity_rolling_window_size_);
  }

} // namespace diff_drive_controller
