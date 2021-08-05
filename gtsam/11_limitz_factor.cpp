#include <Eigen/Core>
#include <cmath>
#include <iostream>
// Eigen 几何模块
#include <Eigen/Geometry>

//#include <sophus/se3.h>
//#include <sophus/so3.h>
#include <gtsam/geometry/Point3.h>
#include <gtsam/geometry/Pose2.h>
#include <gtsam/geometry/Pose3.h>
#include <gtsam/inference/Symbol.h>
#include <gtsam/nonlinear/GaussNewtonOptimizer.h>
#include <gtsam/nonlinear/ISAM2.h>
#include <gtsam/nonlinear/LevenbergMarquardtOptimizer.h>
#include <gtsam/nonlinear/Marginals.h>
#include <gtsam/slam/BetweenFactor.h>
#include <gtsam/slam/PoseTranslationPrior.h>
#include <gtsam/slam/PriorFactor.h>
#include <gtsam/slam/dataset.h>

#include <opencv2/core/core.hpp>

#include <boost/optional/optional_io.hpp>

#include <random>
using namespace gtsam;
using namespace std;

Pose3 GetPose3(double x = 0, double y = 0, double z = 0, double roll = 0, double pitch = 0, double yaw = 0) {
  return gtsam::Pose3(gtsam::Rot3::RzRyRx(roll, pitch, yaw), gtsam::Point3(x, y, z));
}

Eigen::MatrixXd skew(const Eigen::Vector3d& v) {
  Eigen::Matrix3d ret;
  ret << 0, -v[2], v[1], v[2], 0, -v[0], -v[1], v[0], 0;
  return ret;
}
Pose3 AddNoise(Pose3& pose_in) {
  std::random_device rd;
  std::default_random_engine generator_(rd());
  std::normal_distribution<double> noise(0, 0.1);
  return pose_in * GetPose3(noise(generator_), noise(generator_), noise(generator_), noise(generator_), noise(generator_), noise(generator_));
}

noiseModel::Diagonal::shared_ptr GetNoise(double noiseScore) {
  gtsam::Vector Vector6(6);
  Vector6 << noiseScore, noiseScore, noiseScore, noiseScore, noiseScore, noiseScore;
  noiseModel::Diagonal::shared_ptr constraintNoise = noiseModel::Diagonal::Variances(Vector6);
  return constraintNoise;
}

class CustomPoseTranslationPrior : public NoiseModelFactor1<Pose3> {
 public:
  typedef PoseTranslationPrior<Pose3> This;
  typedef NoiseModelFactor1<Pose3> Base;
  typedef Pose3 Pose;
  typedef typename Pose3::Translation Translation;
  typedef typename Pose3::Rotation Rotation;

  GTSAM_CONCEPT_POSE_TYPE(Pose)
  GTSAM_CONCEPT_GROUP_TYPE(Pose)
  GTSAM_CONCEPT_LIE_TYPE(Translation)

 protected:
  Translation measured_;

 public:
  /** default constructor - only use for serialization */
  CustomPoseTranslationPrior() {}

  /** standard constructor */
  CustomPoseTranslationPrior(Key key, const Translation& measured, const noiseModel::Base::shared_ptr& model) : Base(model, key), measured_(measured) {}

  /** Constructor that pulls the translation from an incoming POSE */
  CustomPoseTranslationPrior(Key key, const Pose3& pose_z, const noiseModel::Base::shared_ptr& model) : Base(model, key), measured_(pose_z.translation()) {}

  virtual ~CustomPoseTranslationPrior() {}

  const Translation& measured() const { return measured_; }

  /// @return a deep copy of this factor
  virtual gtsam::NonlinearFactor::shared_ptr clone() const {
    return boost::static_pointer_cast<gtsam::NonlinearFactor>(gtsam::NonlinearFactor::shared_ptr(new CustomPoseTranslationPrior(*this)));
  }

  /** h(x)-z */
  Vector evaluateError(const Pose& pose, boost::optional<Matrix&> H = boost::none) const {
    const Translation& newTrans = pose.translation();
    const Rotation& R = pose.rotation();
    const int tDim = traits<Translation>::GetDimension(newTrans);
    const int xDim = traits<Pose>::GetDimension(pose);
    if (H) {
      *H = Matrix::Zero(tDim, xDim);
      std::pair<size_t, size_t> transInterval = Pose3::translationInterval();
      (*H).middleCols(transInterval.first, tDim) = R.matrix();
      cout << "H is " << H << endl;
    }

    // Local的意思是newTrans - measure
    //    return traits<Translation>::Local(measured_, newTrans);
    return newTrans - measured_;
  }

 private:
  /** Serialization function */
  friend class boost::serialization::access;
  template <class ARCHIVE>
  void serialize(ARCHIVE& ar, const unsigned int /*version*/) {
    ar& boost::serialization::make_nvp("NoiseModelFactor1", boost::serialization::base_object<Base>(*this));
    ar& BOOST_SERIALIZATION_NVP(measured_);
  }
};

class LimitZFactor : public NoiseModelFactor1<Pose3> {
 private:
  Vector1 view_;
  static const int xDim = FixedDimension<Pose3>::value;

 public:
  LimitZFactor(Key i, double view, const SharedNoiseModel& model) : NoiseModelFactor1<Pose3>(model, i) { view_ << view; }
  Vector evaluateError(const Pose3& q, boost::optional<Matrix&> H = boost::none) const {
    Vector4 x1;
    x1 << 0, 0, 0, 1;
    Vector4 x2;
    x2 << 0, 0, 1, 0;

    Vector1 current = x2.transpose() * q.matrix() * x1;

    if (H) {
      cout << "x dim is " << xDim << endl;
      Eigen::Matrix<double, 4, 6> tp_dot;
      tp_dot.block<3, 3>(0, 0) = Eigen::Matrix3d::Zero();
      tp_dot.block<1, 4>(3, 0) = Eigen::Matrix<double, 1, 4>().setZero();
      tp_dot.block<3, 3>(0, 3) = q.rotation().matrix();

      Eigen::RowVectorXd jacobian = x2.transpose() * tp_dot;
      cout << "q is " << q << endl;
      cout << "jacobian is " << jacobian << endl;
      (*H) = (Matrix(1, 6) << jacobian[0], jacobian[1], jacobian[2], jacobian[3], jacobian[4], jacobian[5]).finished();
    }
    Vector1 error = current - view_;
    cout << "error is " << error << endl;
    return error;
  }
  virtual gtsam::NonlinearFactor::shared_ptr clone() const {
    return boost::static_pointer_cast<gtsam::NonlinearFactor>(gtsam::NonlinearFactor::shared_ptr(new LimitZFactor(*this)));
  }
};

int main(int argc, char** argv) {
  NonlinearFactorGraph graph;
  Values initialEstimate;
  Pose3 T_i = GetPose3(0.3, 0.4, 0.5, 0.3, 0.3, 0.3);
  //  Matrix6 jacobian;
  //  Vector6 T_lie_i = Pose3::Logmap(T_i, jacobian);
  //  cout << "T_lie_i is " << T_lie_i << endl;
  //  cout << "jacobian is " << jacobian << endl;
  initialEstimate.insert(0, AddNoise(T_i));

  auto prior_noise = GetNoise(1e-1);
  graph.add(PriorFactor<Pose3>(0, GetPose3(0.3, 0.4, 1.5, 0.1, 0.1, 0.1), prior_noise));

  gtsam::Vector1 limit_z_noise;
  limit_z_noise << 1e-1;
  noiseModel::Diagonal::shared_ptr limit_z_var = noiseModel::Diagonal::Variances(limit_z_noise);
  graph.add(LimitZFactor(0, 0, limit_z_var));

  //  gtsam::Vector3 limit_xyz_noise;
  //  limit_xyz_noise << 1e-1, 1e-1, 1e-1;
  //  noiseModel::Diagonal::shared_ptr limit_xyz_var = noiseModel::Diagonal::Variances(limit_xyz_noise);
  //  CustomPoseTranslationPrior t_factor(0, gtsam::Point3(0, 0, 0), limit_xyz_var);
  //
  //  graph.add(t_factor);

  ISAM2Params parameters;
  parameters.relinearizeThreshold = 0.01;
  parameters.relinearizeSkip = 1;
  ISAM2 isam(parameters);

  isam.update(graph, initialEstimate);
  isam.update();
  isam.update();
  isam.update();
  isam.update();
  isam.update();
  isam.update();

  Values currentEstimate = isam.calculateEstimate();
  cout << "****************************************************" << endl;
  currentEstimate.print("Current estimate: ");

  return 0;
}