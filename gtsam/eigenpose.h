#pragma once

#include <geometry_msgs/Pose.h>
#include <gtsam/geometry/Pose3.h>
#include <gtsam/geometry/Rot3.h>
#include <pcl_common.h>
#include <tf/transform_datatypes.h>
#include <Eigen/Dense>
#include <iostream>

class EigenPose {
 public:
  EigenPose();

  EigenPose(double x, double y, double z, double roll, double pitch, double yaw);

  EigenPose(const Eigen::Matrix3d& R, const Eigen::Vector3d& t);

  EigenPose(const Eigen::Quaterniond& q, const Eigen::Vector3d& t);

  EigenPose(const EigenPose& e_pose);

  EigenPose(const geometry_msgs::Pose& msg) {
    Eigen::Quaterniond q(msg.orientation.w, msg.orientation.x, msg.orientation.y, msg.orientation.z);
    Eigen::Vector3d t;
    t << msg.position.x, msg.position.y, msg.position.z;
    R_ = q.toRotationMatrix();
    t_ = t;
  }
  EigenPose(Eigen::Affine3f pose) {
    float x, y, z, roll, pitch, yaw;
    pcl::getTranslationAndEulerAngles(pose, x, y, z, roll, pitch, yaw);
    t_ << x, y, z;
    Eigen::Quaterniond q = Eigen::Quaterniond(Eigen::AngleAxisd(yaw, Eigen::Vector3d::UnitZ())) *
                           Eigen::Quaterniond(Eigen::AngleAxisd(pitch, Eigen::Vector3d::UnitY())) *
                           Eigen::Quaterniond(Eigen::AngleAxisd(roll, Eigen::Vector3d::UnitX()));
    R_ = q.toRotationMatrix();
  }
  EigenPose(const gtsam::Pose3& pose) {
    t_ << pose.x(), pose.y(), pose.z();
    Eigen::Quaterniond q = Eigen::Quaterniond(Eigen::AngleAxisd(pose.rotation().yaw(), Eigen::Vector3d::UnitZ())) *
                           Eigen::Quaterniond(Eigen::AngleAxisd(pose.rotation().pitch(), Eigen::Vector3d::UnitY())) *
                           Eigen::Quaterniond(Eigen::AngleAxisd(pose.rotation().roll(), Eigen::Vector3d::UnitX()));
    R_ = q.toRotationMatrix();
  }

  EigenPose& operator=(const EigenPose& e_pose);

  EigenPose operator*(const EigenPose& T_rel);

  Eigen::Vector3d operator*(const Eigen::Vector3d& point_in);

  //  Eigen::Vector3d GetRPY() const
  //  {
  //    Eigen::Vector3d YPR = R_.eulerAngles(2, 1, 0);
  //    Eigen::Vector3d RPY;
  //    RPY << YPR(2), YPR(1), YPR(0);
  //    return RPY;
  //  }

  Eigen::Vector3d GetRPY() const;

  EigenPose inverse();

  EigenPose CalRelativeEigenPose(const EigenPose& pose_target);

  geometry_msgs::Quaternion GetGeoQ() {
    geometry_msgs::Quaternion q;
    Eigen::Quaterniond eigen_q(R_);
    q.w = eigen_q.w();
    q.x = eigen_q.x();
    q.y = eigen_q.y();
    q.z = eigen_q.z();
    return q;
  }

  tf::Quaternion GetTfQ() {
    tf::Quaternion q;
    return tf::createQuaternionFromRPY(GetRPY()[0], GetRPY()[1], GetRPY()[2]);
  }

  Eigen::Quaterniond GetQ() {
    Eigen::Quaterniond eigen_q(R_);
    return eigen_q;
  }

  void GetEuler(float* data) {
    data[0] = GetRPY()[0];
    data[1] = GetRPY()[1];
    data[2] = GetRPY()[2];
    data[3] = t_[0];
    data[4] = t_[1];
    data[5] = t_[2];
  }

  gtsam::Pose3 GetPose3() { return gtsam::Pose3(gtsam::Rot3::RzRyRx(GetRPY()[0], GetRPY()[1], GetRPY()[2]), gtsam::Point3(t_[0], t_[1], t_[2])); }

  void Reset(const double& x = .0, const double& y = .0, const double& z = .0, const double& roll = .0, const double& pitch = .0, const double& yaw = .0);
  void Reset(const Eigen::Vector3d& tran, const Eigen::Vector3d& rot) { Reset(tran.x(), tran.y(), tran.z(), rot.x(), rot.y(), rot.z()); }

  Eigen::Matrix3d R_;
  Eigen::Vector3d t_;

  friend std::ostream& operator<<(std::ostream& os, EigenPose& pose) {
    os << "x:" << pose.t_[0] << " y:" << pose.t_[1] << " z:" << pose.t_[2] << " roll:" << pose.GetRPY()[0] << " pitch:" << pose.GetRPY()[1]
       << " yaw:" << pose.GetRPY()[2] << std::endl;
    return os;
  }
  void GetPrint(char* buff) { sprintf(buff, "x:%f y:%f z:%f roll:%f pitch:%f yaw:%f \n", t_[0], t_[1], t_[2], GetRPY()[0], GetRPY()[1], GetRPY()[2]); }
};
