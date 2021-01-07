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
#include <gtsam/geometry/Pose2.h>

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

#include <gtsam/nonlinear/LevenbergMarquardtOptimizer.h>

#include <gtsam/nonlinear/Marginals.h>

using namespace std;
using namespace gtsam;

class UnaryFactor : public NoiseModelFactor1<Pose2> {
  double mx_, my_;

 public:
  UnaryFactor(Key j, double x, double y, const SharedNoiseModel &model) : NoiseModelFactor1<Pose2>(model, j), mx_(x), my_(y){};

  Vector evaluateError(const Pose2 &q, boost::optional<Matrix &> H = boost::none) const {
    if (H) {
      *H = (Matrix(2, 3) << 1.0, 0.0, 0.0, 0.0, 1.0, 0.0).finished();
    }
    return (Vector(2) << q.x() - mx_, q.y() - my_).finished();
  }
};

int main() {
  NonlinearFactorGraph graph;

  Pose2 priorX1(0, 0, 0);
  noiseModel::Diagonal::shared_ptr priorNoise = noiseModel::Diagonal::Sigmas(Vector3(0.3, 0.3, 0.1));
  graph.add(PriorFactor<Pose2>(1, priorX1, priorNoise));

  // 添加odom的观测
  Pose2 t_X1_X2(2, 0, 0);
  noiseModel::Diagonal::shared_ptr odometryNoise = noiseModel::Diagonal::Sigmas(Vector3(0.2, 0.2, 0.1));

  graph.add(BetweenFactor<Pose2>(1, 2, t_X1_X2, odometryNoise));

  Pose2 t_X2_X3(2, 0, 0);
  graph.add(BetweenFactor<Pose2>(2, 3, t_X2_X3, odometryNoise));

  graph.print();

  // 传入values
  Values initial;
  initial.insert(1, Pose2(0.5, 0, 0.2));
  initial.insert(2, Pose2(10, 0.1, -0.2));
  initial.insert(3, Pose2(4.1, 0.1, 0.1));

  Values result = LevenbergMarquardtOptimizer(graph, initial).optimize();
  result.print();

  Marginals marginals(graph, result);
  // 输出xi的协方差矩阵
  cout << " x1 coveriance: \n" << marginals.marginalCovariance(1) << endl;
  cout << " x2 coveriance: \n" << marginals.marginalCovariance(2) << endl;
  cout << " x3 coveriance: \n" << marginals.marginalCovariance(3) << endl;

  // add unary measurement factors, like GPS, on all three poses
  noiseModel::Diagonal::shared_ptr unaryNoise = noiseModel::Diagonal::Sigmas(Vector2(0.1, 0.1));  // 10cm std on x,y
  graph.add(boost::make_shared<UnaryFactor>(1, 0.0, 0.0, unaryNoise));
  graph.add(boost::make_shared<UnaryFactor>(2, 2.1, 0.0, unaryNoise));
  graph.add(boost::make_shared<UnaryFactor>(3, 4.2, 0.0, unaryNoise));
  result = LevenbergMarquardtOptimizer(graph, initial).optimize();
  result.print();

  return 0;
}
