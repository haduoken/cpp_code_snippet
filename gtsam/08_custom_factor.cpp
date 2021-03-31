#include <gtsam/nonlinear/ISAM2.h>
#include <gtsam/nonlinear/NonlinearFactor.h>
#include <gtsam/nonlinear/NonlinearFactorGraph.h>

using namespace std;
using namespace gtsam;

class CurveFitFactor : public NoiseModelFactor1<Vector2> {
 private:
  double _mx, _my;

 public:
  CurveFitFactor(Key j, const SharedNoiseModel& model, double x, double y) : NoiseModelFactor1<Vector2>(model, j), _mx(x), _my(y) {}
  virtual ~CurveFitFactor() {}
  Vector evaluateError(const Vector2& q, boost::optional<Matrix&> H = boost::none) const {
    // fx = e^(mx + c)

    if (H) (*H) = (Matrix(1, 2) << -_mx * exp(q[0] * _mx + q[1]), -exp(q[0] * _mx + q[1])).finished();
    return (Vector(1) << _my - exp(q[0] * _mx + q[1])).finished();
    //    if (H) (*H) = (Matrix(2, 2) << -_mx * exp(q[0] * _mx + q[1]), -exp(q[0] * _mx + q[1]), 0, 0).finished();
    //    return (Vector(2) << _my - exp(q[0] * _mx + q[1]), 0).finished();
  }
  virtual gtsam::NonlinearFactor::shared_ptr clone() const {
    return boost::static_pointer_cast<gtsam::NonlinearFactor>(gtsam::NonlinearFactor::shared_ptr(new CurveFitFactor(*this)));
  }
};

class CurveFitFactor2 : public NoiseModelFactor1<Vector3> {
 private:
  double _mx, _my;

 public:
  CurveFitFactor2(Key j, const SharedNoiseModel& model, double x, double y) : NoiseModelFactor1<Vector3>(model, j), _mx(x), _my(y) {}
  virtual ~CurveFitFactor2() {}
  Vector evaluateError(const Vector3& q, boost::optional<Matrix&> H = boost::none) const {
    // fx = e^(mx + c)

    if (H) (*H) = (Matrix(1, 3) << -_mx * _mx, -_mx, -1).finished();
    return (Vector(1) << _my - (q[0] * _mx * _mx + q[1] * _mx + q[2])).finished();
    //    if (H) (*H) = (Matrix(2, 2) << -_mx * exp(q[0] * _mx + q[1]), -exp(q[0] * _mx + q[1]), 0, 0).finished();
    //    return (Vector(2) << _my - exp(q[0] * _mx + q[1]), 0).finished();
  }
  virtual gtsam::NonlinearFactor::shared_ptr clone() const {
    return boost::static_pointer_cast<gtsam::NonlinearFactor>(gtsam::NonlinearFactor::shared_ptr(new CurveFitFactor2(*this)));
  }
};

class CurveFitFactor3 : public NoiseModelFactor2<Vector1, Vector1> {
 private:
  double _mx, _my;

 public:
  CurveFitFactor3(Key i, Key j, const SharedNoiseModel& model, double x, double y) : NoiseModelFactor2<Vector1, Vector1>(model, i, j), _mx(x), _my(y) {}
  virtual ~CurveFitFactor3() {}
  Vector evaluateError(const Vector1& q1, const Vector1& q2, boost::optional<Matrix&> H1 = boost::none, boost::optional<Matrix&> H2 = boost::none) const {
    // fx = e^(mx + c)

    if (H1) (*H1) = (Matrix(1, 1) << -_mx * exp(q1[0] * _mx + q2[0])).finished();
    if (H2) (*H2) = (Matrix(1, 1) << -exp(q1[0] * _mx + q2[0])).finished();
    return (Vector(1) << _my - exp(q1[0] * _mx + q2[0])).finished();
    //    if (H) (*H) = (Matrix(2, 2) << -_mx * exp(q[0] * _mx + q[1]), -exp(q[0] * _mx + q[1]), 0, 0).finished();
    //    return (Vector(2) << _my - exp(q[0] * _mx + q[1]), 0).finished();
  }
  virtual gtsam::NonlinearFactor::shared_ptr clone() const {
    return boost::static_pointer_cast<gtsam::NonlinearFactor>(gtsam::NonlinearFactor::shared_ptr(new CurveFitFactor3(*this)));
  }
};

void test1() {
  NonlinearFactorGraph graph;
  // noise model 是residual的noise model!!
  SharedNoiseModel sharedNoiseModel = noiseModel::Diagonal::Sigmas(Vector1(0.2));

  double a = 0.5, b = 1.5;

  std::random_device rd;
  std::default_random_engine generator_(rd());
  std::normal_distribution<double> noise(0, 1);

  for (int i = 0; i < 100; i++) {
    double x = i * 0.1 + noise(generator_);
    double y = exp(a * x + b) + noise(generator_);
    CurveFitFactor cf(0, sharedNoiseModel, x, y);
    graph.add(cf);
  }

  Values initial;
  initial.insert(0, Vector2(0.5, 0));

  ISAM2Params parameters;
  parameters.relinearizeThreshold = 0.01;
  parameters.relinearizeSkip = 1;
  ISAM2 isam(parameters);

  isam.update(graph, initial);
  isam.update();
  isam.update();
  isam.update();
  isam.update();

  Values currentEstimate = isam.calculateEstimate();
  cout << "****************************************************" << endl;
  currentEstimate.print("Current estimate: ");
}
void test2() {
  NonlinearFactorGraph graph;
  // noise model 是residual的noise model?
  SharedNoiseModel sharedNoiseModel = noiseModel::Diagonal::Sigmas(Vector1(0.2));

  double a = 0.5, b = 1.5, c = 3;

  std::random_device rd;
  std::default_random_engine generator_(rd());
  std::normal_distribution<double> noise(0, 1);

  for (int i = 0; i < 100; i++) {
    double x = i * 0.1 + noise(generator_);
    double y = a * x * x + b * x + c + noise(generator_);
    CurveFitFactor2 cf(0, sharedNoiseModel, x, y);
    graph.add(cf);
  }

  Values initial;
  initial.insert(0, Vector3(0.5, 0, 0));

  ISAM2Params parameters;
  parameters.relinearizeThreshold = 0.01;
  parameters.relinearizeSkip = 1;
  ISAM2 isam(parameters);

  isam.update(graph, initial);
  isam.update();
  isam.update();
  isam.update();
  isam.update();

  Values currentEstimate = isam.calculateEstimate();
  cout << "****************************************************" << endl;
  currentEstimate.print("Current estimate: ");
}
void test3() {
  NonlinearFactorGraph graph;
  // noise model 是residual的noise model!!
  SharedNoiseModel sharedNoiseModel = noiseModel::Diagonal::Sigmas(Vector1(0.2));

  double a = 0.5, b = 1.5;

  std::random_device rd;
  std::default_random_engine generator_(rd());
  std::normal_distribution<double> noise(0, 1);

  for (int i = 0; i < 100; i++) {
    double x = i * 0.1 + noise(generator_);
    double y = exp(a * x + b) + noise(generator_);
    CurveFitFactor3 cf(0, 1, sharedNoiseModel, x, y);
    graph.add(cf);
  }

  Values initial;
  initial.insert(0, Vector1(0.5));
  initial.insert(1, Vector1(0));

  ISAM2Params parameters;
  parameters.relinearizeThreshold = 0.01;
  parameters.relinearizeSkip = 1;
  ISAM2 isam(parameters);

  isam.update(graph, initial);
  isam.update();
  isam.update();
  isam.update();
  isam.update();

  Values currentEstimate = isam.calculateEstimate();
  cout << "****************************************************" << endl;
  currentEstimate.print("Current estimate: ");
}

int main() { test3(); }
