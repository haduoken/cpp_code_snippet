#include "eigenpose.h"

EigenPose::EigenPose() {
  R_ << 1.0, .0, .0, .0, 1.0, .0, .0, .0, 1.0;
  t_ << .0, .0, .0;
}

EigenPose::EigenPose(double x, double y, double z, double roll, double pitch, double yaw) {
  t_ << x, y, z;
  Eigen::Quaterniond q = Eigen::Quaterniond(Eigen::AngleAxisd(yaw, Eigen::Vector3d::UnitZ())) *
                         Eigen::Quaterniond(Eigen::AngleAxisd(pitch, Eigen::Vector3d::UnitY())) *
                         Eigen::Quaterniond(Eigen::AngleAxisd(roll, Eigen::Vector3d::UnitX()));
  R_ = q.toRotationMatrix();
}

EigenPose::EigenPose(const Eigen::Matrix3d& R, const Eigen::Vector3d& t) : R_(R), t_(t) {}

EigenPose::EigenPose(const Eigen::Quaterniond& q, const Eigen::Vector3d& t) {
  R_ = q.toRotationMatrix();
  t_ = t;
}

EigenPose::EigenPose(const EigenPose& e_pose) {
  R_ = e_pose.R_;
  t_ = e_pose.t_;
}

EigenPose& EigenPose::operator=(const EigenPose& e_pose) {
  R_ = e_pose.R_;
  t_ = e_pose.t_;
}

EigenPose EigenPose::operator*(const EigenPose& T_rel) { return EigenPose(R_ * T_rel.R_, R_ * T_rel.t_ + t_); }

Eigen::Vector3d EigenPose::operator*(const Eigen::Vector3d& point_in) { return R_ * point_in + t_; }

Eigen::Vector3d EigenPose::GetRPY() const {
  Eigen::Vector3d RPY;
  RPY(1) = asin(-R_(2, 0));
  double c_pitch = cos(RPY(1));
  RPY(0) = atan2(R_(2, 1) / c_pitch, R_(2, 2) / c_pitch);
  RPY(2) = atan2(R_(1, 0) / c_pitch, R_(0, 0) / c_pitch);
  return RPY;
}

EigenPose EigenPose::inverse() { return EigenPose(R_.inverse(), -R_.inverse() * t_); }

void EigenPose::Reset(const double& x, const double& y, const double& z, const double& roll, const double& pitch, const double& yaw) {
  t_ << x, y, z;
  Eigen::Quaterniond q = Eigen::Quaterniond(Eigen::AngleAxisd(yaw, Eigen::Vector3d::UnitZ())) *
                         Eigen::Quaterniond(Eigen::AngleAxisd(pitch, Eigen::Vector3d::UnitY())) *
                         Eigen::Quaterniond(Eigen::AngleAxisd(roll, Eigen::Vector3d::UnitX()));
  R_ = q.toRotationMatrix();
}

EigenPose EigenPose::CalRelativeEigenPose(const EigenPose& pose_target) { return this->inverse() * pose_target; }
