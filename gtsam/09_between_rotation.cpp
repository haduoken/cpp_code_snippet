/**
 * A simple 2D pose slam example
 *  - The robot moves in a 2 meter square
 *  - The robot moves 2 meters each step, turning 90 degrees after each step
 *  - The robot initially faces along the X axis (horizontal, to the right in 2D)
 *  - We have full odometry between pose
 *  - We have a loop closure constraint when the robot returns to the first position
 *                    x5 -- x4
 *                    |     |
 *              x1 -- x2 -- x3
 */

// Each variable in the system (poses and landmarks) must be identified with a unique key.
// We can either use simple integer keys (1, 2, 3, ...) or symbols (X1, X2, L1).
// Here we will use Symbols
#include <gtsam/inference/Symbol.h>

// We want to use iSAM2 to solve the pose optimazation problem incrementally, so
// include iSAM2 here
#include <gtsam/nonlinear/ISAM2.h>

// iSAM2 requires as input a set set of new factors to be added stored in a factor graph,
// and initial guesses for any new variables used in the added factors
#include <gtsam/nonlinear/NonlinearFactor.h>
#include <gtsam/nonlinear/NonlinearFactorGraph.h>

#include <vector>

// In planar SLAM example we use Pose2 variables (x, y, theta) to represent the robot poses
#include <gtsam/geometry/Pose3.h>
#include <gtsam/geometry/Rot3.h>

// We will use simple integer Keys to refer to the robot poses.
#include <gtsam/inference/Key.h>

// In GTSAM, measurement functions are represented as 'factors'. Several common factors
// have been provided with the library for solving robotics/SLAM/Bundle Adjustment problems.
// Here we will use Between factors for the relative motion described by odometry measurements.
// We will also use a Between Factor to encode the loop closure constraint
// Also, we will initialize the robot at the origin using a Prior factor.
#include <gtsam/slam/BetweenFactor.h>
#include <gtsam/slam/PriorFactor.h>

// The nonlinear solvers within GTSAM are iterative solvers, meaning they linearize the
// nonlinear functions around an initial linearization point, then solve the linear system
// to update the linearization point. This happens repeatedly until the solver converges
// to a consistent set of variable values. This requires us to specify an initial guess
// for each variable, held in a Values container.
#include <gtsam/nonlinear/Values.h>

using namespace std;
using namespace gtsam;

class RotCompose : public NoiseModelFactor2<Vector1, Vector1> {
 private:
  Rot3 Rc_;

 public:
  RotCompose(Key i, Key j, const SharedNoiseModel& model, Rot3& Rc) : NoiseModelFactor2<Vector1, Vector1>(model, i, j), Rc_(Rc) {}
  virtual ~RotCompose() {}
  Vector evaluateError(const Rot3& q1, const Rot3& q2, boost::optional<Matrix&> H1 = boost::none, boost::optional<Matrix&> H2 = boost::none) const {
    // fx = e^(mx + c)

    if (H1) (*H1) = (Matrix(1, 1) << -_mx * exp(q1[0] * _mx + q2[0])).finished();
    if (H2) (*H2) = (Matrix(1, 1) << -exp(q1[0] * _mx + q2[0])).finished();
    return (Vector(1) << _my - exp(q1[0] * _mx + q2[0])).finished();
    //    if (H) (*H) = (Matrix(2, 2) << -_mx * exp(q[0] * _mx + q[1]), -exp(q[0] * _mx + q[1]), 0, 0).finished();
    //    return (Vector(2) << _my - exp(q[0] * _mx + q[1]), 0).finished();
  }
  virtual gtsam::NonlinearFactor::shared_ptr clone() const {
    return boost::static_pointer_cast<gtsam::NonlinearFactor>(gtsam::NonlinearFactor::shared_ptr(new RotCompose(*this)));
  }
};

Pose3 GetPose3(double x = 0, double y = 0, double z = 0, double roll = 0, double pitch = 0, double yaw = 0) {
  return gtsam::Pose3(gtsam::Rot3::RzRyRx(roll, pitch, yaw), gtsam::Point3(x, y, z));
}

noiseModel::Diagonal::shared_ptr GetNoise(double noiseScore) {
  gtsam::Vector Vector6(6);
  Vector6 << noiseScore, noiseScore, noiseScore, noiseScore, noiseScore, noiseScore;
  noiseModel::Diagonal::shared_ptr constraintNoise = noiseModel::Diagonal::Variances(Vector6);
  return constraintNoise;
}

int main() {
  //　向量保存好模拟的位姿和测量，到时候一个个往isam2里填加
  std::vector<BetweenFactor<Pose3> > gra;
  std::vector<Pose3> initPose, realPose;

  // 定义noise

  NonlinearFactorGraph graph;
  Values initialEstimate;

  Rot3 R_b_i = gtsam::Rot3::RzRyRx(0.1, 0.2, 0.3);
  Pose3 Tb_last = GetPose3();
  for (int i = 0; i < 100; i++) {
    Pose3 Tb = GetPose3(0.1 * i, 0.2 * i, 0.3 * i, 0.1 * i, 0.2 * i, 0.3 * i);
    Rot3 Ri = Tb.rotation() * R_b_i;

    auto odom_noise = GetNoise(0.1);
    graph.push_back(BetweenFactor<Pose3>(Symbol('p', i), Symbol('p', i + 1), Tb_last.inverse() * Tb, odom_noise));
    graph.push_back(BetweenFactor<Rot3>(Symbol('r', 0), Symbol('p', i + 1), )) Tb_last = Tb;
  }

  // 添加odom
  Symbol('x', 1);
  graph.push_back(BetweenFactor<Pose3>(Symbol('p', i), GetPose3(1.2), odom_noise));
  graph.push_back(BetweenFactor<Pose3>(2, 3, GetPose3(1.2), odom_noise));
  graph.push_back(BetweenFactor<Pose3>(3, 4, GetPose3(1.2), odom_noise));

  // 添加初始值
  initialEstimate.insert(0, GetPose3(0, 100));
  initialEstimate.insert(1, GetPose3(1.2, 100));
  initialEstimate.insert(2, GetPose3(2.4, 0));
  initialEstimate.insert(3, GetPose3(3.6, 100));
  initialEstimate.insert(4, GetPose3(4.8, 100));

  // 添加prior
  auto prior_noise = GetNoise(0.1);
  graph.push_back(PriorFactor<Pose3>(0, GetPose3(0), prior_noise));
  graph.push_back(PriorFactor<Pose3>(4, GetPose3(4), prior_noise));

  ISAM2Params parameters;
  parameters.relinearizeThreshold = 0.01;
  parameters.relinearizeSkip = 1;
  ISAM2 isam(parameters);

  // 再多一次计算
  isam.update(graph, initialEstimate);
  isam.update();
  isam.update();
  isam.update();
  isam.update();

  // 计算完之后清空graph
  //  graph.resize(0);

  Values currentEstimate = isam.calculateEstimate();
  currentEstimate.print("Current estimate: ");

  return 0;
}
