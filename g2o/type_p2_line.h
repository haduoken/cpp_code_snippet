//
// Created by kilox on 20-7-14.
//

#ifndef TEST_APP_TYPE_P2_LINE_H
#define TEST_APP_TYPE_P2_LINE_H

#include <core/base_unary_edge.h>
#include <core/eigen_types.h>
#include <sophus/se3.h>
#include <types/slam3d/vertex_se3.h>
#include <iostream>

using namespace std;
using namespace g2o;

class TransformVertex : public g2o::BaseVertex<6, Sophus::SE3> {
 public:
  EIGEN_MAKE_ALIGNED_OPERATOR_NEW;
  TransformVertex() {}
  virtual bool read(istream &is) { return false; }
  virtual bool write(ostream &os) const { return false; }
  virtual void setToOriginImpl() { _estimate = Sophus::SE3(); }
  virtual void oplusImpl(const double *update) override {
    Sophus::SE3 up(Sophus::SO3(update[3], update[4], update[5]), Eigen::Vector3d(update[0], update[1], update[2]));
    auto old_estimate = _estimate;
    _estimate = up * _estimate;
    //    std::cout << "update is" << up << " estimate from " << old_estimate << " to " << _estimate << std::endl;
    //    g2o::Vector6d v(update);
    //    Sophus::SE3 update_se3 = Sophus::SE3::exp(v);
    //    std::cout << "update is " << v(0, 0) << v(0, 1) << v(0, 2) << v(0, 3) << v(0, 4) << v(0, 5) << std::endl;
    //    _estimate = Sophus::SE3::exp(v) * _estimate;
  }
};
class Point2PlaneEdge : public g2o::BaseUnaryEdge<1, double, TransformVertex> {
 public:
  EIGEN_MAKE_ALIGNED_OPERATOR_NEW;

  Point2PlaneEdge(Eigen::Vector3d &p0, Eigen::Vector3d &p1, Eigen::Vector3d &p2, Eigen::Vector3d &p3) : p0_(p0), p1_(p1), p2_(p2), p3_(p3) {
    v12_ = p2 - p1;
    v13_ = p3 - p1;
    v12_13_ = v12_.cross(v13_);
    v12_13_sq_norm_ = v12_13_.squaredNorm();
  }

  virtual bool read(std::istream & /*is*/) {
    cerr << __PRETTY_FUNCTION__ << " not implemented yet" << endl;
    return false;
  }
  virtual bool write(std::ostream & /*os*/) const {
    cerr << __PRETTY_FUNCTION__ << " not implemented yet" << endl;
    return false;
  }

  void computeError() {
    const TransformVertex *v_trans = static_cast<const TransformVertex *>(_vertices[0]);
    Eigen::Vector3d current_p0 = v_trans->estimate() * p0_;
    Eigen::Vector3d v10 = current_p0 - p1_;

    // 计算出距离的平方
    double dot_product = v10.transpose() * v12_13_;
    double distance = dot_product * dot_product / v12_13_sq_norm_;
    _error(0, 0) = distance;
    std::cout << "current loss is " << distance << std::endl;
  }

  // 计算雅各比
  virtual void linearizeOplus() {
    const TransformVertex *v_trans = static_cast<const TransformVertex *>(_vertices[0]);
    Eigen::Vector3d current_p0 = v_trans->estimate() * p0_;
    Eigen::Vector3d v10 = current_p0 - p1_;

    double c = 2 / v12_13_sq_norm_;
    Eigen::RowVector4d coeff_matrix;
    coeff_matrix.block<1, 3>(0, 0) = v12_13_;
    coeff_matrix.block<1, 3>(0, 0) *= c;
    coeff_matrix(0, 3) = 1;

    Eigen::Matrix<double, 4, 6> lie_algebra_jacobin = Eigen::MatrixXd::Zero(4, 6);
    lie_algebra_jacobin.block<3, 3>(0, 0) = Eigen::MatrixXd::Identity(3, 3);
    lie_algebra_jacobin.block<3, 3>(0, 3) = -Sophus::SO3::hat(p0_);

    Eigen::Matrix<double, 1, 6> line_jacobin = coeff_matrix * lie_algebra_jacobin;
    // 先t后r
    _jacobianOplusXi(0, 0) = line_jacobin(0, 0);
    _jacobianOplusXi(0, 1) = line_jacobin(0, 1);
    _jacobianOplusXi(0, 2) = line_jacobin(0, 2);
    _jacobianOplusXi(0, 3) = line_jacobin(0, 3);
    _jacobianOplusXi(0, 4) = line_jacobin(0, 4);
    _jacobianOplusXi(0, 5) = line_jacobin(0, 5);
  }

  Eigen::Vector3d p0_, p1_, p2_, p3_, v12_, v13_, v12_13_;
  double v12_13_sq_norm_;
};
class Point2LineEdge : public g2o::BaseUnaryEdge<1, double, TransformVertex> {
 public:
  EIGEN_MAKE_ALIGNED_OPERATOR_NEW;

  Point2LineEdge(Eigen::Vector3d &p0, Eigen::Vector3d &p1, Eigen::Vector3d &p2) : p0_(p0), p1_(p1), p2_(p2) {
    v12_ = p2_ - p1_;
    // 向量的2范数
    v12_sq_norm_ = v12_.squaredNorm();
  }

  virtual bool read(std::istream & /*is*/) {
    cerr << __PRETTY_FUNCTION__ << " not implemented yet" << endl;
    return false;
  }
  virtual bool write(std::ostream & /*os*/) const {
    cerr << __PRETTY_FUNCTION__ << " not implemented yet" << endl;
    return false;
  }

  void computeError() {
    const TransformVertex *v_trans = static_cast<const TransformVertex *>(_vertices[0]);
    Eigen::Vector3d current_p0 = v_trans->estimate() * p0_;
    Eigen::Vector3d v10 = current_p0 - p1_;

    // 计算出距离的平方
    Eigen::Vector3d v12_10 = v12_.cross(v10);
    double distance = v12_10.squaredNorm() / v12_sq_norm_;
    _error(0, 0) = distance;
    std::cout << "current loss is " << distance << std::endl;
  }

  // 计算雅各比
  virtual void linearizeOplus() {
    const TransformVertex *v_trans = static_cast<const TransformVertex *>(_vertices[0]);
    Eigen::Vector3d current_p0 = v_trans->estimate() * p0_;
    Eigen::Vector3d v10 = current_p0 - p1_;

    Eigen::Vector3d v12_10 = v12_.cross(v10);
    double c = 2 / v12_sq_norm_;
    Eigen::RowVector4d coeff_matrix;
    coeff_matrix.block<1, 3>(0, 0) = v12_10.transpose() * Sophus::SO3::hat(v12_);
    coeff_matrix.block<1, 3>(0, 0) *= c;
    coeff_matrix(0, 3) = 1;

    Eigen::Matrix<double, 4, 6> lie_algebra_jacobin = Eigen::MatrixXd::Zero(4, 6);
    lie_algebra_jacobin.block<3, 3>(0, 0) = Eigen::MatrixXd::Identity(3, 3);
    lie_algebra_jacobin.block<3, 3>(0, 3) = -Sophus::SO3::hat(p0_);

    Eigen::Matrix<double, 1, 6> line_jacobin = coeff_matrix * lie_algebra_jacobin;
    _jacobianOplusXi(0, 0) = line_jacobin(0, 0);
    _jacobianOplusXi(0, 1) = line_jacobin(0, 1);
    _jacobianOplusXi(0, 2) = line_jacobin(0, 2);
    _jacobianOplusXi(0, 3) = line_jacobin(0, 3);
    _jacobianOplusXi(0, 4) = line_jacobin(0, 4);
    _jacobianOplusXi(0, 5) = line_jacobin(0, 5);
  }

  Eigen::Vector3d p0_, p1_, p2_, v12_;
  double v12_sq_norm_;
};

#endif  // TEST_APP_TYPE_P2_LINE_H
