#include <Eigen/Core>
#include <Eigen/Dense>
#include <iostream>

using namespace std;

template <typename Derived>
static Eigen::Matrix<typename Derived::Scalar, 3, 3> skewSymmetric(const Eigen::MatrixBase<Derived> &q) {
  Eigen::Matrix<typename Derived::Scalar, 3, 3> ans;
  ans << typename Derived::Scalar(0), -q(2), q(1), q(2), typename Derived::Scalar(0), -q(0), -q(1), q(0), typename Derived::Scalar(0);
  return ans;
}

template <typename Derived>
static Eigen::Quaternion<typename Derived::Scalar> positify(const Eigen::QuaternionBase<Derived> &q) {
  // printf("a: %f %f %f %f", q.w(), q.x(), q.y(), q.z());
  // Eigen::Quaternion<typename Derived::Scalar> p(-q.w(), -q.x(), -q.y(), -q.z());
  // printf("b: %f %f %f %f", p.w(), p.x(), p.y(), p.z());
  // return q.template w() >= (typename Derived::Scalar)(0.0) ? q : Eigen::Quaternion<typename Derived::Scalar>(-q.w(), -q.x(), -q.y(), -q.z());
  return q;
}
template <typename Derived>
static Eigen::Matrix<typename Derived::Scalar, 4, 4> Qleft(const Eigen::QuaternionBase<Derived> &q) {
  Eigen::Quaternion<typename Derived::Scalar> qq = positify(q);
  Eigen::Matrix<typename Derived::Scalar, 4, 4> ans;
  ans(0, 0) = qq.w(), ans.template block<1, 3>(0, 1) = -qq.vec().transpose();
  ans.template block<3, 1>(1, 0) = qq.vec(),
                              ans.template block<3, 3>(1, 1) = qq.w() * Eigen::Matrix<typename Derived::Scalar, 3, 3>::Identity() + skewSymmetric(qq.vec());
  return ans;
}

template <typename Derived>
static Eigen::Matrix<typename Derived::Scalar, 4, 4> Qright(const Eigen::QuaternionBase<Derived> &p) {
  Eigen::Quaternion<typename Derived::Scalar> pp = positify(p);
  Eigen::Matrix<typename Derived::Scalar, 4, 4> ans;
  ans(0, 0) = pp.w(), ans.template block<1, 3>(0, 1) = -pp.vec().transpose();
  ans.template block<3, 1>(1, 0) = pp.vec(),
                              ans.template block<3, 3>(1, 1) = pp.w() * Eigen::Matrix<typename Derived::Scalar, 3, 3>::Identity() - skewSymmetric(pp.vec());
  return ans;
}

int main() {
  //
  Eigen::Quaterniond qij = Eigen::Quaterniond(Eigen::AngleAxisd(M_PI / 4, Eigen::Vector3d::UnitZ()));
  Eigen::Quaterniond qi = Eigen::Quaterniond(Eigen::AngleAxisd(0.1, Eigen::Vector3d::UnitX()));
  Eigen::Quaterniond qj = Eigen::Quaterniond(Eigen::AngleAxisd(0.2, Eigen::Vector3d::UnitY()));

  Eigen::Vector3d trans(1, 1, 0);

  Eigen::Vector3d aft_trans = qij * trans;

  cout << "aft_trans is " << aft_trans << endl;

  Eigen::Quaterniond qx(0, 0.5, 0.5, 0.5);

  Eigen::Matrix3d result1 = (Qleft(qj.inverse() * qi) * Qright(qij)).bottomRightCorner<3, 3>();

  cout << "result 1 is " << result1 << endl;

  Eigen::Matrix3d result2 = (Qleft(qij.inverse()) * Qright(qi.inverse() * qj)).bottomRightCorner<3, 3>();
  cout << " result 2 is " << result2 << endl;

  //  Eigen::Matrix3d R1;
  //  R1 << 0, -3, 2, 3, 0, -1, -2, 1, 0;
  //
  //  cout << "R1 is " << R1 << endl;
  //
  //  cout << "Rw^ is " << q * R1 << endl;
  //
  //  cout << "w^R is " << R1 * q << endl;
}