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

Pose3 GetPose3(double x, double y, double z, double roll, double pitch, double yaw) {
  return gtsam::Pose3(gtsam::Rot3::RzRyRx(roll, pitch, yaw), gtsam::Point3(x, y, z));
}

int main() {
  //　向量保存好模拟的位姿和测量，到时候一个个往isam2里填加
  std::vector<BetweenFactor<Pose3> > gra;
  std::vector<Pose3> initPose, realPose;

  // For simplicity, we will use the same noise model for odometry and loop closures
  double noiseScore = 0.2;
  gtsam::Vector Vector6(6);
  Vector6 << noiseScore, noiseScore, noiseScore, noiseScore, noiseScore, noiseScore;
  noiseModel::Diagonal::shared_ptr constraintNoise = noiseModel::Diagonal::Variances(Vector6);

  //  realPose.push_back(GetPose3(0, 0, 0, 0, 0, 0));
  //  realPose.push_back(GetPose3(0, 0, 0, 0, 0, 0));
  //  realPose.push_back(GetPose3(2, 0, 0, 0, 0, 0));
  //  realPose.push_back(GetPose3(4, 0, 0, 0, 0, 0));
  //  realPose.push_back(GetPose3(4, 2, 0, 0, 0, M_PI_2));
  //  realPose.push_back(GetPose3(2, 2, 0, 0, 0, M_PI));
  //
  //  cout<<" 1 to 2 "<<realPose[1].inverse() * realPose[2]<<endl;
  //  cout<<" 2 to 3 "<<realPose[2].inverse() * realPose[3]<<endl;
  //  cout<<" 3 to 4 "<<realPose[3].inverse() * realPose[4]<<endl;
  //  cout<<" 4 to 5 "<<realPose[4].inverse() * realPose[5]<<endl;
  //  cout<<" 5 to 2 "<<realPose[5].inverse() * realPose[2]<<endl;

  gra.push_back(BetweenFactor<Pose3>(1, 2, GetPose3(2.5, 0, 0, 0, 0, 0), constraintNoise));
  gra.push_back(BetweenFactor<Pose3>(2, 3, GetPose3(2, 0, 0, 0, 0, 0), constraintNoise));
  gra.push_back(BetweenFactor<Pose3>(3, 4, GetPose3(0, 2, 0, 0, 0, M_PI_2), constraintNoise));
  gra.push_back(BetweenFactor<Pose3>(4, 5, GetPose3(0, 2, 0, 0, 0, M_PI_2), constraintNoise));
  gra.push_back(BetweenFactor<Pose3>(5, 2, GetPose3(0, 2, 0, 0, 0, M_PI), constraintNoise));

  initPose.push_back(GetPose3(0, 0.0, 0, 0, 0, 0));
  initPose.push_back(GetPose3(1.8, 0, 0, 0, 0, 0));
  initPose.push_back(GetPose3(4.2, 0, 0, 0, 0, 0));
  initPose.push_back(GetPose3(4.1, 2.1, 0, 0, 0, M_PI_2));
  initPose.push_back(GetPose3(2.1, 2.2, 0, 0, 0, M_PI));

  // Create an iSAM2 object. Unlike iSAM1, which performs periodic batch steps to maintain proper linearization
  // and efficient variable ordering, iSAM2 performs partial relinearization/reordering at each step. A parameter
  // structure is available that allows the user to set various properties, such as the relinearization threshold
  // and type of linear solver. For this example, we we set the relinearization threshold small so the iSAM2 result
  // will approach the batch result.
  ISAM2Params parameters;
  parameters.relinearizeThreshold = 0.01;
  parameters.relinearizeSkip = 1;
  ISAM2 isam(parameters);

  // Create a Factor Graph and Values to hold the new data
  // 注意isam2的graph里只添加isam2更新状态以后新测量到的约束
  NonlinearFactorGraph graph;
  Values initialEstimate;

  // the first pose don't need to update
  for (int i = 0; i < 5; i++) {
    // Add an initial guess for the current pose
    initialEstimate.insert(i + 1, initPose[i]);

    if (i == 0) {
      //  Add a prior on the first pose, setting it to the origin
      // A prior factor consists of a mean and a noise model (covariance matrix)
      double noiseScore = 0.00002;
      gtsam::Vector Vector6(6);
      Vector6 << noiseScore, noiseScore, noiseScore, noiseScore, noiseScore, noiseScore;
      noiseModel::Diagonal::shared_ptr prior_noise = noiseModel::Diagonal::Variances(Vector6);
      graph.push_back(PriorFactor<Pose3>(1, GetPose3(0, 0, 0, 0, 0, 0), prior_noise));

    } else {
      graph.push_back(gra[i - 1]);  // ie: when i = 1 , robot at pos2,  there is a edge gra[0] between pos1 and pos2
      if (i == 2) {
        double noiseScore = 0.2;
        gtsam::Vector Vector6(6);
        Vector6 << noiseScore, noiseScore, noiseScore, noiseScore, noiseScore, noiseScore;
        noiseModel::Diagonal::shared_ptr prior_noise = noiseModel::Diagonal::Variances(Vector6);
        graph.push_back(PriorFactor<Pose3>(2, GetPose3(1.5, 0, 0, 0, 0, 0), prior_noise));
        // 不能仅添加部分约束
        //        graph.push_back(PriorFactor<Point3>(2, Point3(1.7, 0, 0), prior_noise));
      }
      if (i == 4) {
        graph.push_back(gra[4]);  //  when robot at pos5, there two edge, one is pos4 ->pos5, another is pos5->pos2  (grad[4])
      }

      isam.update(graph, initialEstimate);
      isam.update();

      Values currentEstimate = isam.calculateEstimate();
      cout << "****************************************************" << endl;
      cout << "Frame " << i << ": " << endl;
      currentEstimate.print("Current estimate: ");

      // Clear the factor graph and values for the next iteration
      // 特别重要，update以后，清空原来的约束。已经加入到isam2的那些会用bayes tree保管，你不用操心了。
      graph.resize(0);
      initialEstimate.clear();

      // 再多一次计算
      isam.update(graph);
      isam.update();

      currentEstimate = isam.calculateEstimate();
      cout << "****************************************************" << endl;
      cout << "Frame " << i << ": " << endl;
      currentEstimate.print("Current estimate: ");

      // Clear the factor graph and values for the next iteration
      // 特别重要，update以后，清空原来的约束。已经加入到isam2的那些会用bayes tree保管，你不用操心了。
      graph.resize(0);
      initialEstimate.clear();
    }
  }
  return 0;
}
