#pragma once

#include <geometry_msgs/Pose.h>
#include <gtsam/geometry/Pose3.h>
#include <gtsam/geometry/Rot3.h>
#include "pcl_common.h"
#include <tf/transform_datatypes.h>
#include <Eigen/Dense>
#include <iostream>
#include <string>
using namespace std;

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
  void Save(string file_name) {
    ofstream outfile;
    outfile.open(file_name, ios::out | ios::trunc);
    outfile << GetRPY()[0] << " " << GetRPY()[1] << " " << GetRPY()[2] << " " << t_[0] << " " << t_[1] << " " << t_[2] << "\n";
    outfile.close();
  }
  EigenPose Load(string file_name) {
    ifstream infile;
    infile.open(file_name);
    double x, y, z, roll, pitch, yaw;
    infile >> roll;
    infile >> pitch;
    infile >> yaw;
    infile >> x;
    infile >> y;
    infile >> z;
    infile.close();
    t_ << x, y, z;
    Eigen::Quaterniond q = Eigen::Quaterniond(Eigen::AngleAxisd(yaw, Eigen::Vector3d::UnitZ())) *
                           Eigen::Quaterniond(Eigen::AngleAxisd(pitch, Eigen::Vector3d::UnitY())) *
                           Eigen::Quaterniond(Eigen::AngleAxisd(roll, Eigen::Vector3d::UnitX()));
    R_ = q.toRotationMatrix();
  }

  void SetFloat6(float pose_in[]) {
    t_[0] = pose_in[3];
    t_[1] = pose_in[4];
    t_[2] = pose_in[5];
    Eigen::Quaterniond q = Eigen::Quaterniond(Eigen::AngleAxisd(pose_in[2], Eigen::Vector3d::UnitZ())) *
                           Eigen::Quaterniond(Eigen::AngleAxisd(pose_in[1], Eigen::Vector3d::UnitY())) *
                           Eigen::Quaterniond(Eigen::AngleAxisd(pose_in[0], Eigen::Vector3d::UnitX()));
    R_ = q.toRotationMatrix();
  }
  void GetFloat6(float pose_out[]) {
    pose_out[0] = GetRPY()[0];
    pose_out[1] = GetRPY()[1];
    pose_out[2] = GetRPY()[2];
    pose_out[3] = t_[0];
    pose_out[4] = t_[1];
    pose_out[5] = t_[2];
  }

  void Restrict2D() {
    //    Eigen::Quaterniond q = Eigen::Quaterniond(Eigen::AngleAxisd(pose_in[0], Eigen::Vector3d::UnitX()));
    //    R_ = q.toRotationMatrix();
  }
  bool IsIdentity() {
    double dis = t_.norm() + GetRPY().norm();
    return dis < 0.00001;
  }

  EigenPose Get2DPose() { return EigenPose(t_[0], t_[1], 0, 0, 0, GetRPY()[2]); }

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

  double DisTo(EigenPose& other) {
    Eigen::Vector3d pt(1.0, 1.0, 1.0);
    Eigen::Vector3d dt = this->inverse() * other * pt - pt;
    return dt.norm();
  }

  bool IsStill() {
    double dis = t_.norm() + GetRPY().norm();
//    cout << "dis is " << dis << endl;
    return dis < 0.0001;
  }

  void GetEuler(float* data) {
    data[0] = GetRPY()[0];
    data[1] = GetRPY()[1];
    data[2] = GetRPY()[2];
    data[3] = t_[0];
    data[4] = t_[1];
    data[5] = t_[2];
  }

  EigenPose GetRatioPose(double ratio) {
    return EigenPose(ratio * t_[0], ratio * t_[1], ratio * t_[2], ratio * GetRPY()[0], ratio * GetRPY()[1], ratio * GetRPY()[2]);
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
  string GetPrint() {
    char buff[200] = {};
    sprintf(buff, "x:%f y:%f z:%f roll:%f pitch:%f yaw:%f \n", t_[0], t_[1], t_[2], GetRPY()[0], GetRPY()[1], GetRPY()[2]);
    return string(buff);
  }
};
