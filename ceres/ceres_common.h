#include <ceres/ceres.h>
#include <ceres/rotation.h>
#include <iostream>
using namespace std;

template <typename Derived>
Eigen::Quaternion<typename Derived::Scalar> deltaQ(const Eigen::MatrixBase<Derived> &theta) {
  typedef typename Derived::Scalar Scalar_t;

  Eigen::Quaternion<Scalar_t> dq;
  Eigen::Matrix<Scalar_t, 3, 1> half_theta = theta;
  half_theta /= static_cast<Scalar_t>(2.0);
  dq.w() = static_cast<Scalar_t>(1.0);
  dq.x() = half_theta.x();
  dq.y() = half_theta.y();
  dq.z() = half_theta.z();
  return dq;
}

class PoseLocalParameterization : public ceres::LocalParameterization {
  virtual bool Plus(const double *x, const double *delta, double *x_plus_delta) const {
    Eigen::Map<const Eigen::Vector3d> _p(x);
    Eigen::Map<const Eigen::Quaterniond> _q(x + 3);

    Eigen::Map<const Eigen::Vector3d> dp(delta);

    Eigen::Quaterniond dq = deltaQ(Eigen::Map<const Eigen::Vector3d>(delta + 3));

    Eigen::Map<Eigen::Vector3d> p(x_plus_delta);
    Eigen::Map<Eigen::Quaterniond> q(x_plus_delta + 3);

    p = _p + dp;
    // q --> q * [0,0.5*delta]
    q = (_q * dq).normalized();

    return true;
  };
  virtual bool ComputeJacobian(const double *x, double *jacobian) const {
    // 这里的jacobian设置为Identity, 对残差的雅克比在cost_function里面进行计算
    Eigen::Map<Eigen::Matrix<double, 7, 6, Eigen::RowMajor>> j(jacobian);
    j.topRows<6>().setIdentity();
    j.bottomRows<1>().setZero();

    return true;
  }
  virtual int GlobalSize() const { return 7; };
  virtual int LocalSize() const { return 6; };
};

template <typename Derived>
Eigen::Matrix<typename Derived::Scalar, 3, 3> skewSymmetric(const Eigen::MatrixBase<Derived> &q) {
  Eigen::Matrix<typename Derived::Scalar, 3, 3> ans;
  ans << typename Derived::Scalar(0), -q(2), q(1), q(2), typename Derived::Scalar(0), -q(0), -q(1), q(0), typename Derived::Scalar(0);
  return ans;
}

class Point2PlaneFactor : public ceres::SizedCostFunction<1, 7> {
 public:
  Point2PlaneFactor() = delete;
  Point2PlaneFactor(const Eigen::Vector3d &pt, const Eigen::Vector4d &plane) : pt_(pt), plane_(plane){};

  virtual bool Evaluate(double const *const *parameters, double *residuals, double **jacobians) const {
    Eigen::Vector3d t(parameters[0][0], parameters[0][1], parameters[0][2]);
    Eigen::Quaterniond q(parameters[0][6], parameters[0][3], parameters[0][4], parameters[0][5]);

    //    cout << "input q is x: " << q.x() << " y: " << q.y() << " z: " << q.z() << " w: " << q.w() << endl;
    //    cout << "input t is " << t << endl;
    Eigen::Vector3d pt_trans = q * pt_ + t;

    Eigen::Vector3d norm = plane_.block<3, 1>(0, 0);
    double length = norm.norm();
    double d = plane_[3] / length;
    norm.normalize();

    residuals[0] = norm.transpose() * pt_trans + d;

    if (jacobians) {
      if (jacobians[0]) {
        Eigen::Map<Eigen::Matrix<double, 1, 7, Eigen::RowMajor>> jacobian(jacobians[0]);
        jacobian.setZero();

        jacobian.block<1, 3>(0, 0) = norm.transpose();

        jacobian.block<1, 3>(0, 3) = norm.transpose() * (-q.toRotationMatrix() * skewSymmetric(pt_));
      }
    }
    return true;
  }

  Eigen::Vector3d pt_;
  Eigen::Vector4d plane_;
};
template <typename T>
inline void QuaternionInverse(const T q[4], T q_inverse[4]) {
  q_inverse[0] = q[0];
  q_inverse[1] = -q[1];
  q_inverse[2] = -q[2];
  q_inverse[3] = -q[3];
};

// 位移约束
struct TError {
  TError(double t_x, double t_y, double t_z, double var) : t_x(t_x), t_y(t_y), t_z(t_z), var(var) {}

  template <typename T>
  bool operator()(const T *tj, T *residuals) const {
    residuals[0] = (tj[0] - T(t_x)) / T(var);
    residuals[1] = (tj[1] - T(t_y)) / T(var);
    residuals[2] = (tj[2] - T(t_z)) / T(var);

    return true;
  }

  static ceres::CostFunction *Create(const double t_x, const double t_y, const double t_z, const double var) {
    return (new ceres::AutoDiffCostFunction<TError, 3, 3>(new TError(t_x, t_y, t_z, var)));
  }

  double t_x, t_y, t_z, var;
};

// 变换约束
// 变换约束
struct RTError {
  RTError(double t_x, double t_y, double t_z, double q_w, double q_x, double q_y, double q_z, double t_var, double q_var)
      : t_x(t_x), t_y(t_y), t_z(t_z), q_w(q_w), q_x(q_x), q_y(q_y), q_z(q_z), t_var(t_var), q_var(q_var) {}

  template <typename T>
  bool operator()(const T *const param, T *residuals) const {
    T t_i[3];
    t_i[0] = param[0];
    t_i[1] = param[1];
    t_i[2] = param[2];

    T q_i[4];
    q_i[0] = param[6];
    q_i[1] = param[3];
    q_i[2] = param[4];
    q_i[3] = param[5];

    // 位移的残差为t_ij - t_
    residuals[0] = (t_i[0] - T(t_x)) / T(t_var);
    residuals[1] = (t_i[1] - T(t_y)) / T(t_var);
    residuals[2] = (t_i[2] - T(t_z)) / T(t_var);

    T measure_q[4];
    measure_q[0] = T(q_w);
    measure_q[1] = T(q_x);
    measure_q[2] = T(q_y);
    measure_q[3] = T(q_z);
    T measure_q_inv[4];
    QuaternionInverse(measure_q, measure_q_inv);

    // 观测值为 q , 计算  q_i 与 q 之间的残差 [q_i * q.inv]xyz
    T error_q[4];
    ceres::QuaternionProduct(measure_q_inv, q_i, error_q);

    residuals[3] = T(2) * error_q[1] / T(q_var);
    residuals[4] = T(2) * error_q[2] / T(q_var);
    residuals[5] = T(2) * error_q[3] / T(q_var);
    return true;
  }

  static ceres::CostFunction *Create(const double t_x, const double t_y, const double t_z, const double q_w, const double q_x, const double q_y,
                                     const double q_z, const double t_var, const double q_var) {
    return (new ceres::AutoDiffCostFunction<RTError, 6, 7>(new RTError(t_x, t_y, t_z, q_w, q_x, q_y, q_z, t_var, q_var)));
  }

  double t_x, t_y, t_z;
  double q_w, q_x, q_y, q_z;
  double t_var, q_var;
};

struct RError {
  RError(double w, double x, double y, double z) : w_(w), x_(x), y_(y), z_(z) {}

  template <typename T>
  bool operator()(const T *const param, T *residuals) const {
    T current_q[4];
    current_q[0] = param[0];
    current_q[1] = param[1];
    current_q[2] = param[2];
    current_q[3] = param[3];

    T target_q[4];
    target_q[0] = T(w_);
    target_q[1] = T(x_);
    target_q[2] = T(y_);
    target_q[3] = T(z_);

    T inv_target[4];
    QuaternionInverse(target_q, inv_target);

    T error_q[4];
    ceres::QuaternionProduct(inv_target, current_q, error_q);
    residuals[0] = T(2) * error_q[1];
    residuals[1] = T(2) * error_q[2];
    residuals[2] = T(2) * error_q[3];
    return true;
  }
  static ceres::CostFunction *Create(const double w, const double x, const double y, const double z) {
    return (new ceres::AutoDiffCostFunction<RError, 3, 4>(new RError(w, x, y, z)));
  }

  double w_, x_, y_, z_;
};

struct RLimitError {
  RLimitError(double w, double x, double y, double z) : w_(w), x_(x), y_(y), z_(z) {}

  template <typename T>
  bool operator()(const T *const param, T *residuals) const {
    T current_q[4];
    current_q[0] = param[0];
    current_q[1] = param[1];
    current_q[2] = param[2];
    current_q[3] = param[3];

    T target_q[4];
    target_q[0] = T(w_);
    target_q[1] = T(x_);
    target_q[2] = T(y_);
    target_q[3] = T(z_);

    T inv_target[4];
    QuaternionInverse(target_q, inv_target);

    T error_q[4];
    ceres::QuaternionProduct(inv_target, current_q, error_q);

    residuals[0] = T(2) * error_q[1];
    residuals[1] = T(2) * error_q[2];
    //    residuals[2] = T(2) * error_q[3];
    return true;
  }
  static ceres::CostFunction *Create(const double w, const double x, const double y, const double z) {
    return (new ceres::AutoDiffCostFunction<RLimitError, 2, 4>(new RLimitError(w, x, y, z)));
  }

  double w_, x_, y_, z_;
};

struct RLimitErrorV2 {
  RLimitErrorV2(double w, double x, double y, double z) : w_(w), x_(x), y_(y), z_(z) {}

  template <typename T>
  bool operator()(const T *const param, T *residuals) const {
    T current_q[4];
    current_q[0] = param[0];
    current_q[1] = param[1];
    current_q[2] = param[2];
    current_q[3] = param[3];

    T target_q[4];
    target_q[0] = T(w_);
    target_q[1] = T(x_);
    target_q[2] = T(y_);
    target_q[3] = T(z_);

    T inv_target[4];
    QuaternionInverse(target_q, inv_target);

    T error_q[4];
    ceres::QuaternionProduct(inv_target, current_q, error_q);
    T e3[3];
    e3[0] = T(0);
    e3[1] = T(0);
    e3[2] = T(1);

    T result_e[3];
    ceres::QuaternionRotatePoint(error_q, e3, result_e);

    residuals[0] = result_e[0];
    residuals[1] = result_e[1];
    residuals[2] = result_e[2] - T(1);
    return true;
  }
  static ceres::CostFunction *Create(const double w, const double x, const double y, const double z) {
    return (new ceres::AutoDiffCostFunction<RLimitErrorV2, 3, 4>(new RLimitErrorV2(w, x, y, z)));
  }

  double w_, x_, y_, z_;
};

class RotationICPFactor : public ceres::SizedCostFunction<3, 4> {
 public:
  RotationICPFactor(const Eigen::Vector3d &fi, const Eigen::Vector3d &fj) : fi_(fi), fj_(fj){};

  virtual bool Evaluate(double const *const *parameters, double *residuals, double **jacobians) const {
    Eigen::Quaterniond Q(parameters[0][0], parameters[0][1], parameters[0][2], parameters[0][3]);
    cout << "Q is w: " << Q.w() << " x: " << Q.x() << " y: " << Q.y() << " z: " << Q.z() << endl;
    Eigen::Matrix3d R = Q.toRotationMatrix();

    Eigen::Map<Eigen::Vector3d> residual(residuals);
    residual = Q * fi_ - fj_;

    if (jacobians) {
      if (jacobians[0]) {
        Eigen::Map<Eigen::Matrix<double, 3, 4, Eigen::RowMajor>> jacobian(jacobians[0]);

        jacobian.rightCols<3>() = -R * skewSymmetric(fi_);

        jacobian.leftCols<1>().setZero();
      }
    }
    return true;
  }

  static ceres::CostFunction *Create(const Eigen::Vector3d fi, const Eigen::Vector3d fj) { return new RotationICPFactor(fi, fj); }

  Eigen::Vector3d fi_, fj_;
};

// 约束之后, 参数与目标的旋转向量z轴是重合的
struct RTLimitRPError {
  RTLimitRPError(double t_x, double t_y, double t_z, double q_w, double q_x, double q_y, double q_z, double t_var, double q_var)
      : t_x(t_x), t_y(t_y), t_z(t_z), q_w(q_w), q_x(q_x), q_y(q_y), q_z(q_z), t_var(t_var), q_var(q_var) {}

  template <typename T>
  bool operator()(const T *const param, T *residuals) const {
    T t_i[3];
    t_i[0] = param[0];
    t_i[1] = param[1];
    t_i[2] = param[2];

    T q_i[4];
    q_i[0] = param[6];
    q_i[1] = param[3];
    q_i[2] = param[4];
    q_i[3] = param[5];

    // 位移的残差为t_ij - t_
    residuals[0] = (t_i[0] - T(t_x)) / T(t_var);
    residuals[1] = (t_i[1] - T(t_y)) / T(t_var);
    residuals[2] = (t_i[2] - T(t_z)) / T(t_var);

    T measure_q[4];
    measure_q[0] = T(q_w);
    measure_q[1] = T(q_x);
    measure_q[2] = T(q_y);
    measure_q[3] = T(q_z);
    T measure_q_inv[4];
    QuaternionInverse(measure_q, measure_q_inv);

    // 观测值为 q , 计算  q_i 与 q 之间的残差 [q_i * q.inv]xyz
    T error_q[4];
    ceres::QuaternionProduct(measure_q_inv, q_i, error_q);

    // error_q[1~3] 应该拟合到(0,0,1)
    residuals[3] = T(2) * error_q[1] / T(q_var);
    residuals[4] = T(2) * error_q[2] / T(q_var);
    //    residuals[5] = T(2) * error_q[3] / T(q_var);
    return true;
  }

  static ceres::CostFunction *Create(const double t_x, const double t_y, const double t_z, const double q_w, const double q_x, const double q_y,
                                     const double q_z, const double t_var, const double q_var) {
    return (new ceres::AutoDiffCostFunction<RTLimitRPError, 5, 7>(new RTLimitRPError(t_x, t_y, t_z, q_w, q_x, q_y, q_z, t_var, q_var)));
  }

  double t_x, t_y, t_z;
  double q_w, q_x, q_y, q_z;
  double t_var, q_var;
};

// 变换约束对Ti,Tj的约束
struct RelativeRTError {
  RelativeRTError(double t_x, double t_y, double t_z, double q_w, double q_x, double q_y, double q_z, double t_var, double q_var)
      : t_x(t_x), t_y(t_y), t_z(t_z), q_w(q_w), q_x(q_x), q_y(q_y), q_z(q_z), t_var(t_var), q_var(q_var) {}

  template <typename T>
  bool operator()(const T *const q_i, const T *t_i, const T *q_j, const T *t_j, T *residuals) const {
    T temp_t_ij[3];
    temp_t_ij[0] = t_j[0] - t_i[0];
    temp_t_ij[1] = t_j[1] - t_i[1];
    temp_t_ij[2] = t_j[2] - t_i[2];

    T q_i_inv[4];
    QuaternionInverse(q_i, q_i_inv);
    // 计算出当前的q_ij
    T q_i_j[4];
    ceres::QuaternionProduct(q_i_inv, q_j, q_i_j);

    // 计算出当前的t_ij
    T t_ij[3];
    ceres::QuaternionRotatePoint(q_i_inv, temp_t_ij, t_ij);

    // 位移的残差为t_ij - t_
    residuals[0] = (t_ij[0] - T(t_x)) / T(t_var);
    residuals[1] = (t_ij[1] - T(t_y)) / T(t_var);
    residuals[2] = (t_ij[2] - T(t_z)) / T(t_var);

    T relative_q[4];
    relative_q[0] = T(q_w);
    relative_q[1] = T(q_x);
    relative_q[2] = T(q_y);
    relative_q[3] = T(q_z);
    T relative_q_inv[4];
    QuaternionInverse(relative_q, relative_q_inv);

    // 观测值为relative_q_inv, 计算q_ij 与 relative_q_inv之间的残差 [q_ij * q.inv]xyz
    T error_q[4];
    ceres::QuaternionProduct(relative_q_inv, q_i_j, error_q);

    residuals[3] = T(2) * error_q[1] / T(q_var);
    residuals[4] = T(2) * error_q[2] / T(q_var);
    residuals[5] = T(2) * error_q[3] / T(q_var);

    return true;
  }

  static ceres::CostFunction *Create(const double t_x, const double t_y, const double t_z, const double q_w, const double q_x, const double q_y,
                                     const double q_z, const double t_var, const double q_var) {
    return (new ceres::AutoDiffCostFunction<RelativeRTError, 6, 4, 3, 4, 3>(new RelativeRTError(t_x, t_y, t_z, q_w, q_x, q_y, q_z, t_var, q_var)));
  }

  double t_x, t_y, t_z;
  double q_w, q_x, q_y, q_z;
  double t_var, q_var;
};