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
#include <gtsam/nonlinear/NonlinearFactorGraph.h>

#include <vector>

// In planar SLAM example we use Pose2 variables (x, y, theta) to represent the robot poses
#include <gtsam/geometry/Pose3.h>

// We will use simple integer Keys to refer to the robot poses.
#include <gtsam/inference/Key.h>

// In GTSAM, measurement functions are represented as 'factors'. Several common factors
// have been provided with the library for solving robotics/SLAM/Bundle Adjustment problems.
// Here we will use Between factors for the relative motion described by odometry measurements.
// We will also use a Between Factor to encode the loop closure constraint
// Also, we will initialize the robot at the origin using a Prior factor.
#include <gtsam/navigation/GPSFactor.h>
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

Pose3 GetPose3(double x = 0, double y = 0, double z = 0, double roll = 0, double pitch = 0, double yaw = 0) {
  return gtsam::Pose3(gtsam::Rot3::RzRyRx(roll, pitch, yaw), gtsam::Point3(x, y, z));
}

noiseModel::Diagonal::shared_ptr GetNoise(double noiseScore) {
  gtsam::Vector Vector6(6);
  Vector6 << noiseScore, noiseScore, noiseScore, noiseScore, noiseScore, noiseScore;
  noiseModel::Diagonal::shared_ptr constraintNoise = noiseModel::Diagonal::Variances(Vector6);
  return constraintNoise;
}

void testRPY() {
  auto pose = gtsam::Pose3(gtsam::Rot3::RzRyRx(1, 2, 3), gtsam::Point3(1, 1, 1));

  cout << "pose is " << pose << endl;

  auto roll = pose.rotation().rpy()[0];
  auto pitch = pose.rotation().rpy()[1];
  auto yaw = pose.rotation().rpy()[2];

  //  cout << "rpy is " << roll << " " << pitch << " " << yaw << endl;
  cout << "rpy is " << pose.rotation().rpy() << endl;
  cout << "ypr is " << pose.rotation().ypr() << endl;

  // 解析出来是 Yaw(z) * Pitch(y) * Roll(x) 的矩阵, 这个就是123....
  Eigen::Quaterniond try_q = Eigen::Quaterniond(Eigen::AngleAxisd(-0.141593, Eigen::Vector3d::UnitZ())) *
                             Eigen::Quaterniond(Eigen::AngleAxisd(1.14159, Eigen::Vector3d::UnitY())) *
                             Eigen::Quaterniond(Eigen::AngleAxisd(-2.14159, Eigen::Vector3d::UnitX()));

  Eigen::Matrix3d try_R = try_q.toRotationMatrix();
  cout << "try_R is " << try_R << endl;

  // 定轴RPY, yaw=1 pitch=2, roll=3
  Eigen::Quaterniond q = Eigen::Quaterniond(Eigen::AngleAxisd(1, Eigen::Vector3d::UnitZ())) *
                         Eigen::Quaterniond(Eigen::AngleAxisd(2, Eigen::Vector3d::UnitY())) *
                         Eigen::Quaterniond(Eigen::AngleAxisd(3, Eigen::Vector3d::UnitX()));
  Eigen::Matrix3d R = q.toRotationMatrix();
  cout << " R is " << R << endl;
}

int main() {
  //　向量保存好模拟的位姿和测量，到时候一个个往isam2里填加
  std::vector<BetweenFactor<Pose3> > gra;
  std::vector<Pose3> initPose, realPose;

  // 定义noise

  NonlinearFactorGraph graph;
  Values initialEstimate;

  // 先构造一个graph但是没有先验factor

  // 添加初始值
  initialEstimate.insert(0, GetPose3(0));
  initialEstimate.insert(1, GetPose3(0.8));
  initialEstimate.insert(2, GetPose3(2.2));
  initialEstimate.insert(3, GetPose3(3.3));
  initialEstimate.insert(4, GetPose3(4.2));

  // 添加prior
  gtsam::Vector v_tmp_1(6), v_tmp_2(6), v_tmp_3(3);
  v_tmp_1 << 1e-1, 1e-1, 1e-1, 1e-1, 1e-1, 1e-1;
  v_tmp_2 << 1e10, 1e10, 1e10, 1e10, 1e10, 1e10;
  v_tmp_3 << 0.1, 0.1, 10;
  auto odom_noise = noiseModel::Diagonal::Variances(v_tmp_1);
  auto prior_noise = noiseModel::Diagonal::Variances(v_tmp_2);
  auto gps_noise = noiseModel::Diagonal::Variances(v_tmp_3);

  graph.push_back(PriorFactor<Pose3>(0, GetPose3(0), prior_noise));
  graph.push_back(BetweenFactor<Pose3>(0, 1, GetPose3(1), odom_noise));
  graph.push_back(BetweenFactor<Pose3>(1, 2, GetPose3(1), odom_noise));
  graph.push_back(BetweenFactor<Pose3>(2, 3, GetPose3(1), odom_noise));
  graph.push_back(BetweenFactor<Pose3>(3, 4, GetPose3(1), odom_noise));

  // 需要至少3个gps数据进行优化
  graph.push_back(GPSFactor(0, gtsam::Point3(0, 0, 0), gps_noise));
  graph.push_back(GPSFactor(1, gtsam::Point3(0, 1, 0), gps_noise));
  graph.push_back(GPSFactor(2, gtsam::Point3(0, 2, 0), gps_noise));

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
