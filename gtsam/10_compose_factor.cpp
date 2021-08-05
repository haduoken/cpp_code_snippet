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
#include <gtsam/slam/PriorFactor.h>
#include <gtsam/slam/dataset.h>

#include <opencv2/core/core.hpp>

#include <random>
using namespace gtsam;
using namespace std;

Pose3 GetPose3(double x = 0, double y = 0, double z = 0, double roll = 0, double pitch = 0, double yaw = 0) {
  return gtsam::Pose3(gtsam::Rot3::RzRyRx(roll, pitch, yaw), gtsam::Point3(x, y, z));
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

class ComposeFactor : public NoiseModelFactor2<Pose3, Pose3> {
 private:
  Pose3 T_lidar_;

 public:
  ComposeFactor(Key i, Key j, Pose3& T_lidar, const SharedNoiseModel& model) : NoiseModelFactor2<Pose3, Pose3>(model, i, j), T_lidar_(T_lidar) {}
  virtual ~ComposeFactor() {}
  Vector evaluateError(const Pose3& q1, const Pose3& q2, boost::optional<Matrix&> H1 = boost::none, boost::optional<Matrix&> H2 = boost::none) const {
    Pose3 hx = traits<Pose3>::Compose(q1, q2, H1, H2);  // h(x)
    return traits<Pose3>::Local(T_lidar_, hx);

    //    Pose3 result = T_lidar_.inverse() * q1 * q2;
    //
    //    if (H1) (*H1) = (Matrix(6, 6) << -_mx * exp(q[0] * _mx + q[1]), -exp(q[0] * _mx + q[1])).finished();
    //    if (H2) (*H2) = (Matrix(6, 6) << -_mx * exp(q[0] * _mx + q[1]), -exp(q[0] * _mx + q[1])).finished();
    //
    //    return (Vector(1) << _my - exp(q[0] * _mx + q[1])).finished();
    //    if (H) (*H) = (Matrix(2, 2) << -_mx * exp(q[0] * _mx + q[1]), -exp(q[0] * _mx + q[1]), 0, 0).finished();
    //    return (Vector(2) << _my - exp(q[0] * _mx + q[1]), 0).finished();
  }
  virtual gtsam::NonlinearFactor::shared_ptr clone() const {
    return boost::static_pointer_cast<gtsam::NonlinearFactor>(gtsam::NonlinearFactor::shared_ptr(new ComposeFactor(*this)));
  }
};

int main(int argc, char** argv) {
  NonlinearFactorGraph graph;
  Values initialEstimate;

  Pose3 T_body_lidar_real = GetPose3(0.3, 0.4, 0.5, 0.1, 0.1, 0.1);
  initialEstimate.insert(Symbol('p', 0), AddNoise(T_body_lidar_real));
  vector<Pose3> v_T_body_real;
  vector<Pose3> v_T_lidar_real;

  for (int i = 0; i < 100; i++) {
    Pose3 tmp_b = GetPose3(0, 0.5 * i, 0, 0, 0, 0);
    initialEstimate.insert(Symbol('b', i), AddNoise(tmp_b));
    Pose3 tmp_l = tmp_b * T_body_lidar_real;
    v_T_body_real.push_back(tmp_b);
    v_T_lidar_real.push_back(tmp_l);
  }

  auto prior_noise = GetNoise(1e-10);
  graph.add(PriorFactor<Pose3>(Symbol('b', 0), GetPose3(), prior_noise));

  auto odom_noise = GetNoise(0.1);
  for (int i = 0; i < v_T_body_real.size() - 1; i++) {
    int j = i + 1;
    Pose3 T_odom = v_T_body_real[i].inverse() * v_T_body_real[j];
    Pose3 T_noise_odom = AddNoise(T_odom);
    graph.add(BetweenFactor<Pose3>(Symbol('b', i), Symbol('b', j), T_noise_odom, odom_noise));
  }

  auto lidar_noise = GetNoise(0.1);
  for (int i = 0; i < v_T_lidar_real.size(); i++) {
    Pose3 T_lidar = v_T_lidar_real[i];
    Pose3 T_noise_lidar = AddNoise(T_lidar);
    graph.add(ComposeFactor(Symbol('b', i), Symbol('p', 0), T_noise_lidar, lidar_noise));
  }
  cout << "hello" << endl;

  ISAM2Params parameters;
  parameters.relinearizeThreshold = 0.01;
  parameters.relinearizeSkip = 1;
  ISAM2 isam(parameters);

  isam.update(graph, initialEstimate);
  isam.update();
  isam.update();
  isam.update();
  isam.update();

  Values currentEstimate = isam.calculateEstimate();
  cout << "****************************************************" << endl;
  currentEstimate.print("Current estimate: ");

  return 0;
}