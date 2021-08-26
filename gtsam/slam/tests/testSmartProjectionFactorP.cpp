/* ----------------------------------------------------------------------------

 * GTSAM Copyright 2010, Georgia Tech Research Corporation,
 * Atlanta, Georgia 30332-0415
 * All Rights Reserved
 * Authors: Frank Dellaert, et al. (see THANKS for the full author list)

 * See LICENSE for the license information

 * -------------------------------------------------------------------------- */

/**
 *  @file  testSmartProjectionFactorP.cpp
 *  @brief Unit tests for SmartProjectionFactorP Class
 *  @author Chris Beall
 *  @author Luca Carlone
 *  @author Zsolt Kira
 *  @author Frank Dellaert
 *  @date   August 2021
 */

#include "smartFactorScenarios.h"
#include <gtsam/slam/PoseTranslationPrior.h>
#include <gtsam/nonlinear/LevenbergMarquardtOptimizer.h>
#include <gtsam/base/numericalDerivative.h>
#include <gtsam/base/serializationTestHelpers.h>
#include <CppUnitLite/TestHarness.h>
#include <boost/assign/std/map.hpp>
#include <iostream>

using namespace boost::assign;
using namespace std::placeholders;

static const double rankTol = 1.0;
// Create a noise model for the pixel error
static const double sigma = 0.1;
static SharedIsotropic model(noiseModel::Isotropic::Sigma(2, sigma));

// Convenience for named keys
using symbol_shorthand::X;
using symbol_shorthand::L;

// tests data
static Symbol x1('X', 1);
static Symbol x2('X', 2);
static Symbol x3('X', 3);

static Point2 measurement1(323.0, 240.0);

LevenbergMarquardtParams lmParams;
// Make more verbose like so (in tests):
// params.verbosityLM = LevenbergMarquardtParams::SUMMARY;

/* ************************************************************************* */
TEST( SmartProjectionFactorP, Constructor) {
  using namespace vanillaPose;
  SmartFactorP::shared_ptr factor1(new SmartFactorP(model));
}

/* ************************************************************************* */
TEST( SmartProjectionFactorP, Constructor2) {
  using namespace vanillaPose;
  SmartProjectionParams params;
  params.setRankTolerance(rankTol);
  SmartFactorP factor1(model, params);
}

/* ************************************************************************* */
TEST( SmartProjectionFactorP, Constructor3) {
  using namespace vanillaPose;
  SmartFactorP::shared_ptr factor1(new SmartFactorP(model));
  factor1->add(measurement1, x1, sharedK);
}

/* ************************************************************************* */
TEST( SmartProjectionFactorP, Constructor4) {
  using namespace vanillaPose;
  SmartProjectionParams params;
  params.setRankTolerance(rankTol);
  SmartFactorP factor1(model, params);
  factor1.add(measurement1, x1, sharedK);
}

/* ************************************************************************* */
TEST( SmartProjectionFactorP, Equals ) {
  using namespace vanillaPose;
  SmartFactorP::shared_ptr factor1(new SmartFactorP(model));
  factor1->add(measurement1, x1, sharedK);

  SmartFactorP::shared_ptr factor2(new SmartFactorP(model));
  factor2->add(measurement1, x1, sharedK);

  CHECK(assert_equal(*factor1, *factor2));
}

/* *************************************************************************/
TEST( SmartProjectionFactorP, noiseless ) {

  using namespace vanillaPose;

  // Project two landmarks into two cameras
  Point2 level_uv = level_camera.project(landmark1);
  Point2 level_uv_right = level_camera_right.project(landmark1);

  SmartFactorP factor(model);
  factor.add(level_uv, x1, sharedK);
  factor.add(level_uv_right, x2, sharedK);

  Values values; // it's a pose factor, hence these are poses
  values.insert(x1, cam1.pose());
  values.insert(x2, cam2.pose());

  double actualError = factor.error(values);
  double expectedError = 0.0;
  EXPECT_DOUBLES_EQUAL(expectedError, actualError, 1e-7);

  SmartFactorP::Cameras cameras = factor.cameras(values);
  double actualError2 = factor.totalReprojectionError(cameras);
  EXPECT_DOUBLES_EQUAL(expectedError, actualError2, 1e-7);

  // Calculate expected derivative for point (easiest to check)
  std::function<Vector(Point3)> f = //
      std::bind(&SmartFactorP::whitenedError<Point3>, factor, cameras, std::placeholders::_1);

  // Calculate using computeEP
  Matrix actualE;
  factor.triangulateAndComputeE(actualE, values);

  // get point
  boost::optional<Point3> point = factor.point();
  CHECK(point);

  // calculate numerical derivative with triangulated point
  Matrix expectedE = sigma * numericalDerivative11<Vector, Point3>(f, *point);
  EXPECT(assert_equal(expectedE, actualE, 1e-7));

  // Calculate using reprojectionError
  SmartFactorP::Cameras::FBlocks F;
  Matrix E;
  Vector actualErrors = factor.unwhitenedError(cameras, *point, F, E);
  EXPECT(assert_equal(expectedE, E, 1e-7));

  EXPECT(assert_equal(Z_4x1, actualErrors, 1e-7));

  // Calculate using computeJacobians
  Vector b;
  SmartFactorP::FBlocks Fs;
  factor.computeJacobians(Fs, E, b, cameras, *point);
  double actualError3 = b.squaredNorm();
  EXPECT(assert_equal(expectedE, E, 1e-7));
  EXPECT_DOUBLES_EQUAL(expectedError, actualError3, 1e-6);
}

/* *************************************************************************/
TEST( SmartProjectionFactorP, noisy ) {

  using namespace vanillaPose;

  // Project two landmarks into two cameras
  Point2 pixelError(0.2, 0.2);
  Point2 level_uv = level_camera.project(landmark1) + pixelError;
  Point2 level_uv_right = level_camera_right.project(landmark1);

  Values values;
  values.insert(x1, cam1.pose());
  Pose3 noise_pose = Pose3(Rot3::Ypr(-M_PI / 10, 0., -M_PI / 10),
      Point3(0.5, 0.1, 0.3));
  values.insert(x2, pose_right.compose(noise_pose));

  SmartFactorP::shared_ptr factor(new SmartFactorP(model));
  factor->add(level_uv, x1, sharedK);
  factor->add(level_uv_right, x2, sharedK);

  double actualError1 = factor->error(values);

  SmartFactorP::shared_ptr factor2(new SmartFactorP(model));
  Point2Vector measurements;
  measurements.push_back(level_uv);
  measurements.push_back(level_uv_right);

  std::vector<boost::shared_ptr<Cal3_S2>> sharedKs;
  sharedKs.push_back(sharedK);
  sharedKs.push_back(sharedK);

  KeyVector views {x1, x2};

  factor2->add(measurements, views, sharedKs);
  double actualError2 = factor2->error(values);
  DOUBLES_EQUAL(actualError1, actualError2, 1e-7);
}

/* *************************************************************************/
TEST(SmartProjectionFactorP, smartFactorWithSensorBodyTransform) {
  using namespace vanillaPose;

  // create arbitrary body_T_sensor (transforms from sensor to body)
  Pose3 body_T_sensor = Pose3(Rot3::Ypr(-M_PI / 2, 0., -M_PI / 2), Point3(1, 1, 1));

  // These are the poses we want to estimate, from camera measurements
  const Pose3 sensor_T_body = body_T_sensor.inverse();
  Pose3 wTb1 = cam1.pose() * sensor_T_body;
  Pose3 wTb2 = cam2.pose() * sensor_T_body;
  Pose3 wTb3 = cam3.pose() * sensor_T_body;

  // three landmarks ~5 meters infront of camera
  Point3 landmark1(5, 0.5, 1.2), landmark2(5, -0.5, 1.2), landmark3(5, 0, 3.0);

  Point2Vector measurements_cam1, measurements_cam2, measurements_cam3;

  // Project three landmarks into three cameras
  projectToMultipleCameras(cam1, cam2, cam3, landmark1, measurements_cam1);
  projectToMultipleCameras(cam1, cam2, cam3, landmark2, measurements_cam2);
  projectToMultipleCameras(cam1, cam2, cam3, landmark3, measurements_cam3);

  // Create smart factors
  KeyVector views {x1, x2, x3};

  SmartProjectionParams params;
  params.setRankTolerance(1.0);
  params.setDegeneracyMode(IGNORE_DEGENERACY);
  params.setEnableEPI(false);

  std::vector<boost::shared_ptr<Cal3_S2>> sharedKs;
  sharedKs.push_back(sharedK);
  sharedKs.push_back(sharedK);
  sharedKs.push_back(sharedK);

  std::vector<Pose3> body_T_sensors;
  body_T_sensors.push_back(body_T_sensor);
  body_T_sensors.push_back(body_T_sensor);
  body_T_sensors.push_back(body_T_sensor);

  SmartFactorP smartFactor1(model, params);
  smartFactor1.add(measurements_cam1, views, sharedKs, body_T_sensors);

  SmartFactorP smartFactor2(model, params);
  smartFactor2.add(measurements_cam2, views, sharedKs, body_T_sensors);

  SmartFactorP smartFactor3(model, params);
  smartFactor3.add(measurements_cam3, views, sharedKs, body_T_sensors);;

  const SharedDiagonal noisePrior = noiseModel::Isotropic::Sigma(6, 0.10);

  // Put all factors in factor graph, adding priors
  NonlinearFactorGraph graph;
  graph.push_back(smartFactor1);
  graph.push_back(smartFactor2);
  graph.push_back(smartFactor3);
  graph.addPrior(x1, wTb1, noisePrior);
  graph.addPrior(x2, wTb2, noisePrior);

  // Check errors at ground truth poses
  Values gtValues;
  gtValues.insert(x1, wTb1);
  gtValues.insert(x2, wTb2);
  gtValues.insert(x3, wTb3);
  double actualError = graph.error(gtValues);
  double expectedError = 0.0;
  DOUBLES_EQUAL(expectedError, actualError, 1e-7)

  Pose3 noise_pose = Pose3(Rot3::Ypr(-M_PI / 100, 0., -M_PI / 100),
                           Point3(0.1, 0.1, 0.1));
  Values values;
  values.insert(x1, wTb1);
  values.insert(x2, wTb2);
  // initialize third pose with some noise, we expect it to move back to
  // original pose3
  values.insert(x3, wTb3 * noise_pose);

  LevenbergMarquardtParams lmParams;
  Values result;
  LevenbergMarquardtOptimizer optimizer(graph, values, lmParams);
  result = optimizer.optimize();
//  graph.print("graph\n");
  EXPECT(assert_equal(wTb3, result.at<Pose3>(x3)));
}

///* *************************************************************************/
//TEST( SmartProjectionFactorP, 3poses_smart_projection_factor ) {
//
//  using namespace vanillaPose2;
//  Point2Vector measurements_cam1, measurements_cam2, measurements_cam3;
//
//  // Project three landmarks into three cameras
//  projectToMultipleCameras(cam1, cam2, cam3, landmark1, measurements_cam1);
//  projectToMultipleCameras(cam1, cam2, cam3, landmark2, measurements_cam2);
//  projectToMultipleCameras(cam1, cam2, cam3, landmark3, measurements_cam3);
//
//  KeyVector views;
//  views.push_back(x1);
//  views.push_back(x2);
//  views.push_back(x3);
//
//  SmartFactorP::shared_ptr smartFactor1(new SmartFactorP(model, sharedK2));
//  smartFactor1->add(measurements_cam1, views);
//
//  SmartFactorP::shared_ptr smartFactor2(new SmartFactorP(model, sharedK2));
//  smartFactor2->add(measurements_cam2, views);
//
//  SmartFactorP::shared_ptr smartFactor3(new SmartFactorP(model, sharedK2));
//  smartFactor3->add(measurements_cam3, views);
//
//  const SharedDiagonal noisePrior = noiseModel::Isotropic::Sigma(6, 0.10);
//
//  NonlinearFactorGraph graph;
//  graph.push_back(smartFactor1);
//  graph.push_back(smartFactor2);
//  graph.push_back(smartFactor3);
//  graph.addPrior(x1, cam1.pose(), noisePrior);
//  graph.addPrior(x2, cam2.pose(), noisePrior);
//
//  Values groundTruth;
//  groundTruth.insert(x1, cam1.pose());
//  groundTruth.insert(x2, cam2.pose());
//  groundTruth.insert(x3, cam3.pose());
//  DOUBLES_EQUAL(0, graph.error(groundTruth), 1e-9);
//
//  //  Pose3 noise_pose = Pose3(Rot3::Ypr(-M_PI/10, 0., -M_PI/10), Point3(0.5,0.1,0.3)); // noise from regular projection factor test below
//  Pose3 noise_pose = Pose3(Rot3::Ypr(-M_PI / 100, 0., -M_PI / 100),
//      Point3(0.1, 0.1, 0.1)); // smaller noise
//  Values values;
//  values.insert(x1, cam1.pose());
//  values.insert(x2, cam2.pose());
//  // initialize third pose with some noise, we expect it to move back to original pose_above
//  values.insert(x3, pose_above * noise_pose);
//  EXPECT(
//      assert_equal(
//          Pose3(
//              Rot3(0, -0.0314107591, 0.99950656, -0.99950656, -0.0313952598,
//                  -0.000986635786, 0.0314107591, -0.999013364, -0.0313952598),
//              Point3(0.1, -0.1, 1.9)), values.at<Pose3>(x3)));
//
//  Values result;
//  LevenbergMarquardtOptimizer optimizer(graph, values, lmParams);
//  result = optimizer.optimize();
//  EXPECT(assert_equal(pose_above, result.at<Pose3>(x3), 1e-6));
//}
//
///* *************************************************************************/
//TEST( SmartProjectionFactorP, Factors ) {
//
//  using namespace vanillaPose;
//
//  // Default cameras for simple derivatives
//  Rot3 R;
//  static Cal3_S2::shared_ptr sharedK(new Cal3_S2(100, 100, 0, 0, 0));
//  Camera cam1(Pose3(R, Point3(0, 0, 0)), sharedK), cam2(
//      Pose3(R, Point3(1, 0, 0)), sharedK);
//
//  // one landmarks 1m in front of camera
//  Point3 landmark1(0, 0, 10);
//
//  Point2Vector measurements_cam1;
//
//  // Project 2 landmarks into 2 cameras
//  measurements_cam1.push_back(cam1.project(landmark1));
//  measurements_cam1.push_back(cam2.project(landmark1));
//
//  // Create smart factors
//  KeyVector views {x1, x2};
//
//  SmartFactorP::shared_ptr smartFactor1 = boost::make_shared<SmartFactorP>(model, sharedK);
//  smartFactor1->add(measurements_cam1, views);
//
//  SmartFactorP::Cameras cameras;
//  cameras.push_back(cam1);
//  cameras.push_back(cam2);
//
//  // Make sure triangulation works
//  CHECK(smartFactor1->triangulateSafe(cameras));
//  CHECK(!smartFactor1->isDegenerate());
//  CHECK(!smartFactor1->isPointBehindCamera());
//  boost::optional<Point3> p = smartFactor1->point();
//  CHECK(p);
//  EXPECT(assert_equal(landmark1, *p));
//
//  VectorValues zeroDelta;
//  Vector6 delta;
//  delta.setZero();
//  zeroDelta.insert(x1, delta);
//  zeroDelta.insert(x2, delta);
//
//  VectorValues perturbedDelta;
//  delta.setOnes();
//  perturbedDelta.insert(x1, delta);
//  perturbedDelta.insert(x2, delta);
//  double expectedError = 2500;
//
//  // After eliminating the point, A1 and A2 contain 2-rank information on cameras:
//  Matrix16 A1, A2;
//  A1 << -10, 0, 0, 0, 1, 0;
//  A2 << 10, 0, 1, 0, -1, 0;
//  A1 *= 10. / sigma;
//  A2 *= 10. / sigma;
//  Matrix expectedInformation; // filled below
//  {
//    // createHessianFactor
//    Matrix66 G11 = 0.5 * A1.transpose() * A1;
//    Matrix66 G12 = 0.5 * A1.transpose() * A2;
//    Matrix66 G22 = 0.5 * A2.transpose() * A2;
//
//    Vector6 g1;
//    g1.setZero();
//    Vector6 g2;
//    g2.setZero();
//
//    double f = 0;
//
//    RegularHessianFactor<6> expected(x1, x2, G11, G12, g1, G22, g2, f);
//    expectedInformation = expected.information();
//
//    boost::shared_ptr<RegularHessianFactor<6> > actual =
//        smartFactor1->createHessianFactor(cameras, 0.0);
//    EXPECT(assert_equal(expectedInformation, actual->information(), 1e-6));
//    EXPECT(assert_equal(expected, *actual, 1e-6));
//    EXPECT_DOUBLES_EQUAL(0, actual->error(zeroDelta), 1e-6);
//    EXPECT_DOUBLES_EQUAL(expectedError, actual->error(perturbedDelta), 1e-6);
//  }
//
//  {
//    Matrix26 F1;
//    F1.setZero();
//    F1(0, 1) = -100;
//    F1(0, 3) = -10;
//    F1(1, 0) = 100;
//    F1(1, 4) = -10;
//    Matrix26 F2;
//    F2.setZero();
//    F2(0, 1) = -101;
//    F2(0, 3) = -10;
//    F2(0, 5) = -1;
//    F2(1, 0) = 100;
//    F2(1, 2) = 10;
//    F2(1, 4) = -10;
//    Matrix E(4, 3);
//    E.setZero();
//    E(0, 0) = 10;
//    E(1, 1) = 10;
//    E(2, 0) = 10;
//    E(2, 2) = 1;
//    E(3, 1) = 10;
//    SmartFactorP::FBlocks Fs = list_of<Matrix>(F1)(F2);
//    Vector b(4);
//    b.setZero();
//
//    // Create smart factors
//    KeyVector keys;
//    keys.push_back(x1);
//    keys.push_back(x2);
//
//    // createJacobianQFactor
//    SharedIsotropic n = noiseModel::Isotropic::Sigma(4, sigma);
//    Matrix3 P = (E.transpose() * E).inverse();
//    JacobianFactorQ<6, 2> expectedQ(keys, Fs, E, P, b, n);
//    EXPECT(assert_equal(expectedInformation, expectedQ.information(), 1e-6));
//
//    boost::shared_ptr<JacobianFactorQ<6, 2> > actualQ =
//        smartFactor1->createJacobianQFactor(cameras, 0.0);
//    CHECK(actualQ);
//    EXPECT(assert_equal(expectedInformation, actualQ->information(), 1e-6));
//    EXPECT(assert_equal(expectedQ, *actualQ));
//    EXPECT_DOUBLES_EQUAL(0, actualQ->error(zeroDelta), 1e-6);
//    EXPECT_DOUBLES_EQUAL(expectedError, actualQ->error(perturbedDelta), 1e-6);
//
//    // Whiten for RegularImplicitSchurFactor (does not have noise model)
//    model->WhitenSystem(E, b);
//    Matrix3 whiteP = (E.transpose() * E).inverse();
//    Fs[0] = model->Whiten(Fs[0]);
//    Fs[1] = model->Whiten(Fs[1]);
//
//    // createRegularImplicitSchurFactor
//    RegularImplicitSchurFactor<Camera> expected(keys, Fs, E, whiteP, b);
//
//    boost::shared_ptr<RegularImplicitSchurFactor<Camera> > actual =
//        smartFactor1->createRegularImplicitSchurFactor(cameras, 0.0);
//    CHECK(actual);
//    EXPECT(assert_equal(expectedInformation, expected.information(), 1e-6));
//    EXPECT(assert_equal(expectedInformation, actual->information(), 1e-6));
//    EXPECT(assert_equal(expected, *actual));
//    EXPECT_DOUBLES_EQUAL(0, actual->error(zeroDelta), 1e-6);
//    EXPECT_DOUBLES_EQUAL(expectedError, actual->error(perturbedDelta), 1e-6);
//  }
//
//  {
//    // createJacobianSVDFactor
//    Vector1 b;
//    b.setZero();
//    double s = sigma * sin(M_PI_4);
//    SharedIsotropic n = noiseModel::Isotropic::Sigma(4 - 3, sigma);
//    JacobianFactor expected(x1, s * A1, x2, s * A2, b, n);
//    EXPECT(assert_equal(expectedInformation, expected.information(), 1e-6));
//
//    boost::shared_ptr<JacobianFactor> actual =
//        smartFactor1->createJacobianSVDFactor(cameras, 0.0);
//    CHECK(actual);
//    EXPECT(assert_equal(expectedInformation, actual->information(), 1e-6));
//    EXPECT(assert_equal(expected, *actual));
//    EXPECT_DOUBLES_EQUAL(0, actual->error(zeroDelta), 1e-6);
//    EXPECT_DOUBLES_EQUAL(expectedError, actual->error(perturbedDelta), 1e-6);
//  }
//}
//
///* *************************************************************************/
//TEST( SmartProjectionFactorP, 3poses_iterative_smart_projection_factor ) {
//
//  using namespace vanillaPose;
//
//  KeyVector views {x1, x2, x3};
//
//  Point2Vector measurements_cam1, measurements_cam2, measurements_cam3;
//
//  // Project three landmarks into three cameras
//  projectToMultipleCameras(cam1, cam2, cam3, landmark1, measurements_cam1);
//  projectToMultipleCameras(cam1, cam2, cam3, landmark2, measurements_cam2);
//  projectToMultipleCameras(cam1, cam2, cam3, landmark3, measurements_cam3);
//
//  SmartFactorP::shared_ptr smartFactor1(new SmartFactorP(model, sharedK));
//  smartFactor1->add(measurements_cam1, views);
//
//  SmartFactorP::shared_ptr smartFactor2(new SmartFactorP(model, sharedK));
//  smartFactor2->add(measurements_cam2, views);
//
//  SmartFactorP::shared_ptr smartFactor3(new SmartFactorP(model, sharedK));
//  smartFactor3->add(measurements_cam3, views);
//
//  const SharedDiagonal noisePrior = noiseModel::Isotropic::Sigma(6, 0.10);
//
//  NonlinearFactorGraph graph;
//  graph.push_back(smartFactor1);
//  graph.push_back(smartFactor2);
//  graph.push_back(smartFactor3);
//  graph.addPrior(x1, cam1.pose(), noisePrior);
//  graph.addPrior(x2, cam2.pose(), noisePrior);
//
//  //  Pose3 noise_pose = Pose3(Rot3::Ypr(-M_PI/10, 0., -M_PI/10), Point3(0.5,0.1,0.3)); // noise from regular projection factor test below
//  Pose3 noise_pose = Pose3(Rot3::Ypr(-M_PI / 100, 0., -M_PI / 100),
//      Point3(0.1, 0.1, 0.1)); // smaller noise
//  Values values;
//  values.insert(x1, cam1.pose());
//  values.insert(x2, cam2.pose());
//  // initialize third pose with some noise, we expect it to move back to original pose_above
//  values.insert(x3, pose_above * noise_pose);
//  EXPECT(
//      assert_equal(
//          Pose3(
//              Rot3(1.11022302e-16, -0.0314107591, 0.99950656, -0.99950656,
//                  -0.0313952598, -0.000986635786, 0.0314107591, -0.999013364,
//                  -0.0313952598), Point3(0.1, -0.1, 1.9)),
//          values.at<Pose3>(x3)));
//
//  Values result;
//  LevenbergMarquardtOptimizer optimizer(graph, values, lmParams);
//  result = optimizer.optimize();
//  EXPECT(assert_equal(pose_above, result.at<Pose3>(x3), 1e-7));
//}
//
///* *************************************************************************/
//TEST( SmartProjectionFactorP, jacobianSVD ) {
//
//  using namespace vanillaPose;
//
//  KeyVector views {x1, x2, x3};
//
//  Point2Vector measurements_cam1, measurements_cam2, measurements_cam3;
//
//  // Project three landmarks into three cameras
//  projectToMultipleCameras(cam1, cam2, cam3, landmark1, measurements_cam1);
//  projectToMultipleCameras(cam1, cam2, cam3, landmark2, measurements_cam2);
//  projectToMultipleCameras(cam1, cam2, cam3, landmark3, measurements_cam3);
//
//  SmartProjectionParams params;
//  params.setRankTolerance(1.0);
//  params.setLinearizationMode(gtsam::JACOBIAN_SVD);
//  params.setDegeneracyMode(gtsam::IGNORE_DEGENERACY);
//  params.setEnableEPI(false);
//  SmartFactorP factor1(model, sharedK, params);
//
//  SmartFactorP::shared_ptr smartFactor1(
//      new SmartFactorP(model, sharedK, params));
//  smartFactor1->add(measurements_cam1, views);
//
//  SmartFactorP::shared_ptr smartFactor2(
//      new SmartFactorP(model, sharedK, params));
//  smartFactor2->add(measurements_cam2, views);
//
//  SmartFactorP::shared_ptr smartFactor3(
//      new SmartFactorP(model, sharedK, params));
//  smartFactor3->add(measurements_cam3, views);
//
//  const SharedDiagonal noisePrior = noiseModel::Isotropic::Sigma(6, 0.10);
//
//  NonlinearFactorGraph graph;
//  graph.push_back(smartFactor1);
//  graph.push_back(smartFactor2);
//  graph.push_back(smartFactor3);
//  graph.addPrior(x1, cam1.pose(), noisePrior);
//  graph.addPrior(x2, cam2.pose(), noisePrior);
//
//  //  Pose3 noise_pose = Pose3(Rot3::Ypr(-M_PI/10, 0., -M_PI/10), Point3(0.5,0.1,0.3)); // noise from regular projection factor test below
//  Pose3 noise_pose = Pose3(Rot3::Ypr(-M_PI / 100, 0., -M_PI / 100),
//      Point3(0.1, 0.1, 0.1)); // smaller noise
//  Values values;
//  values.insert(x1, cam1.pose());
//  values.insert(x2, cam2.pose());
//  values.insert(x3, pose_above * noise_pose);
//
//  Values result;
//  LevenbergMarquardtOptimizer optimizer(graph, values, lmParams);
//  result = optimizer.optimize();
//  EXPECT(assert_equal(pose_above, result.at<Pose3>(x3), 1e-6));
//}
//
///* *************************************************************************/
//TEST( SmartProjectionFactorP, landmarkDistance ) {
//
//  using namespace vanillaPose;
//
//  double excludeLandmarksFutherThanDist = 2;
//
//  KeyVector views {x1, x2, x3};
//
//  Point2Vector measurements_cam1, measurements_cam2, measurements_cam3;
//
//  // Project three landmarks into three cameras
//  projectToMultipleCameras(cam1, cam2, cam3, landmark1, measurements_cam1);
//  projectToMultipleCameras(cam1, cam2, cam3, landmark2, measurements_cam2);
//  projectToMultipleCameras(cam1, cam2, cam3, landmark3, measurements_cam3);
//
//  SmartProjectionParams params;
//  params.setRankTolerance(1.0);
//  params.setLinearizationMode(gtsam::JACOBIAN_SVD);
//  params.setDegeneracyMode(gtsam::IGNORE_DEGENERACY);
//  params.setLandmarkDistanceThreshold(excludeLandmarksFutherThanDist);
//  params.setEnableEPI(false);
//
//  SmartFactorP::shared_ptr smartFactor1(
//      new SmartFactorP(model, sharedK, params));
//  smartFactor1->add(measurements_cam1, views);
//
//  SmartFactorP::shared_ptr smartFactor2(
//      new SmartFactorP(model, sharedK, params));
//  smartFactor2->add(measurements_cam2, views);
//
//  SmartFactorP::shared_ptr smartFactor3(
//      new SmartFactorP(model, sharedK, params));
//  smartFactor3->add(measurements_cam3, views);
//
//  const SharedDiagonal noisePrior = noiseModel::Isotropic::Sigma(6, 0.10);
//
//  NonlinearFactorGraph graph;
//  graph.push_back(smartFactor1);
//  graph.push_back(smartFactor2);
//  graph.push_back(smartFactor3);
//  graph.addPrior(x1, cam1.pose(), noisePrior);
//  graph.addPrior(x2, cam2.pose(), noisePrior);
//
//  //  Pose3 noise_pose = Pose3(Rot3::Ypr(-M_PI/10, 0., -M_PI/10), Point3(0.5,0.1,0.3)); // noise from regular projection factor test below
//  Pose3 noise_pose = Pose3(Rot3::Ypr(-M_PI / 100, 0., -M_PI / 100),
//      Point3(0.1, 0.1, 0.1)); // smaller noise
//  Values values;
//  values.insert(x1, cam1.pose());
//  values.insert(x2, cam2.pose());
//  values.insert(x3, pose_above * noise_pose);
//
//  // All factors are disabled and pose should remain where it is
//  Values result;
//  LevenbergMarquardtOptimizer optimizer(graph, values, lmParams);
//  result = optimizer.optimize();
//  EXPECT(assert_equal(values.at<Pose3>(x3), result.at<Pose3>(x3)));
//}
//
///* *************************************************************************/
//TEST( SmartProjectionFactorP, dynamicOutlierRejection ) {
//
//  using namespace vanillaPose;
//
//  double excludeLandmarksFutherThanDist = 1e10;
//  double dynamicOutlierRejectionThreshold = 1; // max 1 pixel of average reprojection error
//
//  KeyVector views {x1, x2, x3};
//
//  // add fourth landmark
//  Point3 landmark4(5, -0.5, 1);
//
//  Point2Vector measurements_cam1, measurements_cam2, measurements_cam3,
//      measurements_cam4;
//
//  // Project 4 landmarks into three cameras
//  projectToMultipleCameras(cam1, cam2, cam3, landmark1, measurements_cam1);
//  projectToMultipleCameras(cam1, cam2, cam3, landmark2, measurements_cam2);
//  projectToMultipleCameras(cam1, cam2, cam3, landmark3, measurements_cam3);
//  projectToMultipleCameras(cam1, cam2, cam3, landmark4, measurements_cam4);
//  measurements_cam4.at(0) = measurements_cam4.at(0) + Point2(10, 10); // add outlier
//
//  SmartProjectionParams params;
//  params.setLinearizationMode(gtsam::JACOBIAN_SVD);
//  params.setLandmarkDistanceThreshold(excludeLandmarksFutherThanDist);
//  params.setDynamicOutlierRejectionThreshold(dynamicOutlierRejectionThreshold);
//
//  SmartFactorP::shared_ptr smartFactor1(
//      new SmartFactorP(model, sharedK, params));
//  smartFactor1->add(measurements_cam1, views);
//
//  SmartFactorP::shared_ptr smartFactor2(
//      new SmartFactorP(model, sharedK, params));
//  smartFactor2->add(measurements_cam2, views);
//
//  SmartFactorP::shared_ptr smartFactor3(
//      new SmartFactorP(model, sharedK, params));
//  smartFactor3->add(measurements_cam3, views);
//
//  SmartFactorP::shared_ptr smartFactor4(
//      new SmartFactorP(model, sharedK, params));
//  smartFactor4->add(measurements_cam4, views);
//
//  const SharedDiagonal noisePrior = noiseModel::Isotropic::Sigma(6, 0.10);
//
//  NonlinearFactorGraph graph;
//  graph.push_back(smartFactor1);
//  graph.push_back(smartFactor2);
//  graph.push_back(smartFactor3);
//  graph.push_back(smartFactor4);
//  graph.addPrior(x1, cam1.pose(), noisePrior);
//  graph.addPrior(x2, cam2.pose(), noisePrior);
//
//  Values values;
//  values.insert(x1, cam1.pose());
//  values.insert(x2, cam2.pose());
//  values.insert(x3, cam3.pose());
//
//  // All factors are disabled and pose should remain where it is
//  Values result;
//  LevenbergMarquardtOptimizer optimizer(graph, values, lmParams);
//  result = optimizer.optimize();
//  EXPECT(assert_equal(cam3.pose(), result.at<Pose3>(x3)));
//}
//
///* *************************************************************************/
//TEST( SmartProjectionFactorP, jacobianQ ) {
//
//  using namespace vanillaPose;
//
//  KeyVector views {x1, x2, x3};
//
//  Point2Vector measurements_cam1, measurements_cam2, measurements_cam3;
//
//  // Project three landmarks into three cameras
//  projectToMultipleCameras(cam1, cam2, cam3, landmark1, measurements_cam1);
//  projectToMultipleCameras(cam1, cam2, cam3, landmark2, measurements_cam2);
//  projectToMultipleCameras(cam1, cam2, cam3, landmark3, measurements_cam3);
//
//  SmartProjectionParams params;
//  params.setLinearizationMode(gtsam::JACOBIAN_Q);
//
//  SmartFactorP::shared_ptr smartFactor1(
//      new SmartFactorP(model, sharedK, params));
//  smartFactor1->add(measurements_cam1, views);
//
//  SmartFactorP::shared_ptr smartFactor2(
//      new SmartFactorP(model, sharedK, params));
//  smartFactor2->add(measurements_cam2, views);
//
//  SmartFactorP::shared_ptr smartFactor3(
//      new SmartFactorP(model, sharedK, params));
//  smartFactor3->add(measurements_cam3, views);
//
//  const SharedDiagonal noisePrior = noiseModel::Isotropic::Sigma(6, 0.10);
//
//  NonlinearFactorGraph graph;
//  graph.push_back(smartFactor1);
//  graph.push_back(smartFactor2);
//  graph.push_back(smartFactor3);
//  graph.addPrior(x1, cam1.pose(), noisePrior);
//  graph.addPrior(x2, cam2.pose(), noisePrior);
//
//  Pose3 noise_pose = Pose3(Rot3::Ypr(-M_PI / 100, 0., -M_PI / 100),
//      Point3(0.1, 0.1, 0.1)); // smaller noise
//  Values values;
//  values.insert(x1, cam1.pose());
//  values.insert(x2, cam2.pose());
//  values.insert(x3, pose_above * noise_pose);
//
//  Values result;
//  LevenbergMarquardtOptimizer optimizer(graph, values, lmParams);
//  result = optimizer.optimize();
//  EXPECT(assert_equal(pose_above, result.at<Pose3>(x3), 1e-6));
//}
//
///* *************************************************************************/
//TEST( SmartProjectionFactorP, 3poses_projection_factor ) {
//
//  using namespace vanillaPose2;
//
//  KeyVector views {x1, x2, x3};
//
//  typedef GenericProjectionFactor<Pose3, Point3> ProjectionFactor;
//  NonlinearFactorGraph graph;
//
//  // Project three landmarks into three cameras
//  graph.emplace_shared<ProjectionFactor>(cam1.project(landmark1), model, x1, L(1), sharedK2);
//  graph.emplace_shared<ProjectionFactor>(cam2.project(landmark1), model, x2, L(1), sharedK2);
//  graph.emplace_shared<ProjectionFactor>(cam3.project(landmark1), model, x3, L(1), sharedK2);
//
//  graph.emplace_shared<ProjectionFactor>(cam1.project(landmark2), model, x1, L(2), sharedK2);
//  graph.emplace_shared<ProjectionFactor>(cam2.project(landmark2), model, x2, L(2), sharedK2);
//  graph.emplace_shared<ProjectionFactor>(cam3.project(landmark2), model, x3, L(2), sharedK2);
//
//  graph.emplace_shared<ProjectionFactor>(cam1.project(landmark3), model, x1, L(3), sharedK2);
//  graph.emplace_shared<ProjectionFactor>(cam2.project(landmark3), model, x2, L(3), sharedK2);
//  graph.emplace_shared<ProjectionFactor>(cam3.project(landmark3), model, x3, L(3), sharedK2);
//
//  const SharedDiagonal noisePrior = noiseModel::Isotropic::Sigma(6, 0.10);
//  graph.addPrior(x1, level_pose, noisePrior);
//  graph.addPrior(x2, pose_right, noisePrior);
//
//  Pose3 noise_pose = Pose3(Rot3::Ypr(-M_PI / 10, 0., -M_PI / 10),
//      Point3(0.5, 0.1, 0.3));
//  Values values;
//  values.insert(x1, level_pose);
//  values.insert(x2, pose_right);
//  values.insert(x3, pose_above * noise_pose);
//  values.insert(L(1), landmark1);
//  values.insert(L(2), landmark2);
//  values.insert(L(3), landmark3);
//
//  DOUBLES_EQUAL(48406055, graph.error(values), 1);
//
//  LevenbergMarquardtOptimizer optimizer(graph, values, lmParams);
//  Values result = optimizer.optimize();
//
//  DOUBLES_EQUAL(0, graph.error(result), 1e-9);
//
//  EXPECT(assert_equal(pose_above, result.at<Pose3>(x3), 1e-7));
//}
//
///* *************************************************************************/
//TEST( SmartProjectionFactorP, CheckHessian) {
//
//  KeyVector views {x1, x2, x3};
//
//  using namespace vanillaPose;
//
//  // Two slightly different cameras
//  Pose3 pose2 = level_pose
//      * Pose3(Rot3::RzRyRx(-0.05, 0.0, -0.05), Point3(0, 0, 0));
//  Pose3 pose3 = pose2 * Pose3(Rot3::RzRyRx(-0.05, 0.0, -0.05), Point3(0, 0, 0));
//  Camera cam2(pose2, sharedK);
//  Camera cam3(pose3, sharedK);
//
//  Point2Vector measurements_cam1, measurements_cam2, measurements_cam3;
//
//  // Project three landmarks into three cameras
//  projectToMultipleCameras(cam1, cam2, cam3, landmark1, measurements_cam1);
//  projectToMultipleCameras(cam1, cam2, cam3, landmark2, measurements_cam2);
//  projectToMultipleCameras(cam1, cam2, cam3, landmark3, measurements_cam3);
//
//  SmartProjectionParams params;
//  params.setRankTolerance(10);
//
//  SmartFactorP::shared_ptr smartFactor1(
//      new SmartFactorP(model, sharedK, params)); // HESSIAN, by default
//  smartFactor1->add(measurements_cam1, views);
//
//  SmartFactorP::shared_ptr smartFactor2(
//      new SmartFactorP(model, sharedK, params)); // HESSIAN, by default
//  smartFactor2->add(measurements_cam2, views);
//
//  SmartFactorP::shared_ptr smartFactor3(
//      new SmartFactorP(model, sharedK, params)); // HESSIAN, by default
//  smartFactor3->add(measurements_cam3, views);
//
//  NonlinearFactorGraph graph;
//  graph.push_back(smartFactor1);
//  graph.push_back(smartFactor2);
//  graph.push_back(smartFactor3);
//
//  //  Pose3 noise_pose = Pose3(Rot3::Ypr(-M_PI/10, 0., -M_PI/10), Point3(0.5,0.1,0.3)); // noise from regular projection factor test below
//  Pose3 noise_pose = Pose3(Rot3::Ypr(-M_PI / 100, 0., -M_PI / 100),
//      Point3(0.1, 0.1, 0.1)); // smaller noise
//  Values values;
//  values.insert(x1, cam1.pose());
//  values.insert(x2, cam2.pose());
//  // initialize third pose with some noise, we expect it to move back to original pose_above
//  values.insert(x3, pose3 * noise_pose);
//  EXPECT(
//      assert_equal(
//          Pose3(
//              Rot3(0.00563056869, -0.130848107, 0.991386438, -0.991390265,
//                  -0.130426831, -0.0115837907, 0.130819108, -0.98278564,
//                  -0.130455917),
//              Point3(0.0897734171, -0.110201006, 0.901022872)),
//          values.at<Pose3>(x3)));
//
//  boost::shared_ptr<GaussianFactor> factor1 = smartFactor1->linearize(values);
//  boost::shared_ptr<GaussianFactor> factor2 = smartFactor2->linearize(values);
//  boost::shared_ptr<GaussianFactor> factor3 = smartFactor3->linearize(values);
//
//  Matrix CumulativeInformation = factor1->information() + factor2->information()
//      + factor3->information();
//
//  boost::shared_ptr<GaussianFactorGraph> GaussianGraph = graph.linearize(
//      values);
//  Matrix GraphInformation = GaussianGraph->hessian().first;
//
//  // Check Hessian
//  EXPECT(assert_equal(GraphInformation, CumulativeInformation, 1e-6));
//
//  Matrix AugInformationMatrix = factor1->augmentedInformation()
//      + factor2->augmentedInformation() + factor3->augmentedInformation();
//
//  // Check Information vector
//  Vector InfoVector = AugInformationMatrix.block(0, 18, 18, 1); // 18x18 Hessian + information vector
//
//  // Check Hessian
//  EXPECT(assert_equal(InfoVector, GaussianGraph->hessian().second, 1e-6));
//}
//
///* *************************************************************************/
//TEST( SmartProjectionFactorP, 3poses_2land_rotation_only_smart_projection_factor ) {
//  using namespace vanillaPose2;
//
//  KeyVector views {x1, x2, x3};
//
//  // Two different cameras, at the same position, but different rotations
//  Pose3 pose2 = level_pose * Pose3(Rot3::RzRyRx(-0.05, 0.0, -0.05), Point3(0,0,0));
//  Pose3 pose3 = pose2 * Pose3(Rot3::RzRyRx(-0.05, 0.0, -0.05), Point3(0,0,0));
//  Camera cam2(pose2, sharedK2);
//  Camera cam3(pose3, sharedK2);
//
//  Point2Vector measurements_cam1, measurements_cam2;
//
//  // Project three landmarks into three cameras
//  projectToMultipleCameras(cam1, cam2, cam3, landmark1, measurements_cam1);
//  projectToMultipleCameras(cam1, cam2, cam3, landmark2, measurements_cam2);
//
//  SmartProjectionParams params;
//  params.setRankTolerance(50);
//  params.setDegeneracyMode(gtsam::HANDLE_INFINITY);
//
//  SmartFactorP::shared_ptr smartFactor1(
//      new SmartFactorP(model, sharedK2, params));
//  smartFactor1->add(measurements_cam1, views);
//
//  SmartFactorP::shared_ptr smartFactor2(
//      new SmartFactorP(model, sharedK2, params));
//  smartFactor2->add(measurements_cam2, views);
//
//  const SharedDiagonal noisePrior = noiseModel::Isotropic::Sigma(6, 0.10);
//  const SharedDiagonal noisePriorTranslation = noiseModel::Isotropic::Sigma(3, 0.10);
//  Point3 positionPrior = Point3(0, 0, 1);
//
//  NonlinearFactorGraph graph;
//  graph.push_back(smartFactor1);
//  graph.push_back(smartFactor2);
//  graph.addPrior(x1, cam1.pose(), noisePrior);
//  graph.emplace_shared<PoseTranslationPrior<Pose3> >(x2, positionPrior, noisePriorTranslation);
//  graph.emplace_shared<PoseTranslationPrior<Pose3> >(x3, positionPrior, noisePriorTranslation);
//
//  Pose3 noise_pose = Pose3(Rot3::Ypr(-M_PI / 100, 0., -M_PI / 100),
//      Point3(0.1, 0.1, 0.1)); // smaller noise
//  Values values;
//  values.insert(x1, cam1.pose());
//  values.insert(x2, pose2 * noise_pose);
//  values.insert(x3, pose3 * noise_pose);
//
//  // params.verbosityLM = LevenbergMarquardtParams::SUMMARY;
//  LevenbergMarquardtOptimizer optimizer(graph, values, lmParams);
//  Values result = optimizer.optimize();
//  EXPECT(assert_equal(pose3, result.at<Pose3>(x3)));
//}
//
///* *************************************************************************/
//TEST( SmartProjectionFactorP, 3poses_rotation_only_smart_projection_factor ) {
//
//  // this test considers a condition in which the cheirality constraint is triggered
//  using namespace vanillaPose;
//
//  KeyVector views {x1, x2, x3};
//
//  // Two different cameras, at the same position, but different rotations
//  Pose3 pose2 = level_pose
//      * Pose3(Rot3::RzRyRx(-0.05, 0.0, -0.05), Point3(0, 0, 0));
//  Pose3 pose3 = pose2 * Pose3(Rot3::RzRyRx(-0.05, 0.0, -0.05), Point3(0, 0, 0));
//  Camera cam2(pose2, sharedK);
//  Camera cam3(pose3, sharedK);
//
//  Point2Vector measurements_cam1, measurements_cam2, measurements_cam3;
//
//  // Project three landmarks into three cameras
//  projectToMultipleCameras(cam1, cam2, cam3, landmark1, measurements_cam1);
//  projectToMultipleCameras(cam1, cam2, cam3, landmark2, measurements_cam2);
//  projectToMultipleCameras(cam1, cam2, cam3, landmark3, measurements_cam3);
//
//  SmartProjectionParams params;
//  params.setRankTolerance(10);
//  params.setDegeneracyMode(gtsam::ZERO_ON_DEGENERACY);
//
//  SmartFactorP::shared_ptr smartFactor1(
//      new SmartFactorP(model, sharedK, params));
//  smartFactor1->add(measurements_cam1, views);
//
//  SmartFactorP::shared_ptr smartFactor2(
//      new SmartFactorP(model, sharedK, params));
//  smartFactor2->add(measurements_cam2, views);
//
//  SmartFactorP::shared_ptr smartFactor3(
//      new SmartFactorP(model, sharedK, params));
//  smartFactor3->add(measurements_cam3, views);
//
//  const SharedDiagonal noisePrior = noiseModel::Isotropic::Sigma(6, 0.10);
//  const SharedDiagonal noisePriorTranslation = noiseModel::Isotropic::Sigma(3,
//      0.10);
//  Point3 positionPrior = Point3(0, 0, 1);
//
//  NonlinearFactorGraph graph;
//  graph.push_back(smartFactor1);
//  graph.push_back(smartFactor2);
//  graph.push_back(smartFactor3);
//  graph.addPrior(x1, cam1.pose(), noisePrior);
//  graph.emplace_shared<PoseTranslationPrior<Pose3> >(x2, positionPrior, noisePriorTranslation);
//  graph.emplace_shared<PoseTranslationPrior<Pose3> >(x3, positionPrior, noisePriorTranslation);
//
//  //  Pose3 noise_pose = Pose3(Rot3::Ypr(-M_PI/10, 0., -M_PI/10), Point3(0.5,0.1,0.3)); // noise from regular projection factor test below
//  Pose3 noise_pose = Pose3(Rot3::Ypr(-M_PI / 100, 0., -M_PI / 100),
//      Point3(0.1, 0.1, 0.1)); // smaller noise
//  Values values;
//  values.insert(x1, cam1.pose());
//  values.insert(x2, cam2.pose());
//  values.insert(x3, pose3 * noise_pose);
//  EXPECT(
//      assert_equal(
//          Pose3(
//              Rot3(0.00563056869, -0.130848107, 0.991386438, -0.991390265,
//                  -0.130426831, -0.0115837907, 0.130819108, -0.98278564,
//                  -0.130455917),
//              Point3(0.0897734171, -0.110201006, 0.901022872)),
//          values.at<Pose3>(x3)));
//
//  Values result;
//  LevenbergMarquardtOptimizer optimizer(graph, values, lmParams);
//  result = optimizer.optimize();
//
//  // Since we do not do anything on degenerate instances (ZERO_ON_DEGENERACY)
//  // rotation remains the same as the initial guess, but position is fixed by PoseTranslationPrior
//#ifdef GTSAM_THROW_CHEIRALITY_EXCEPTION
//  EXPECT(assert_equal(Pose3(values.at<Pose3>(x3).rotation(),
//      Point3(0,0,1)), result.at<Pose3>(x3)));
//#else
//  // if the check is disabled, no cheirality exception if thrown and the pose converges to the right rotation
//  // with modest accuracy since the configuration is essentially degenerate without the translation due to noise (noise_pose)
//  EXPECT(assert_equal(pose3, result.at<Pose3>(x3),1e-3));
//#endif
//}
//
///* *************************************************************************/
//TEST( SmartProjectionFactorP, Hessian ) {
//
//  using namespace vanillaPose2;
//
//  KeyVector views {x1, x2};
//
//  // Project three landmarks into 2 cameras
//  Point2 cam1_uv1 = cam1.project(landmark1);
//  Point2 cam2_uv1 = cam2.project(landmark1);
//  Point2Vector measurements_cam1;
//  measurements_cam1.push_back(cam1_uv1);
//  measurements_cam1.push_back(cam2_uv1);
//
//  SmartFactorP::shared_ptr smartFactor1(new SmartFactorP(model, sharedK2));
//  smartFactor1->add(measurements_cam1, views);
//
//  Pose3 noise_pose = Pose3(Rot3::Ypr(-M_PI / 10, 0., -M_PI / 10),
//      Point3(0.5, 0.1, 0.3));
//  Values values;
//  values.insert(x1, cam1.pose());
//  values.insert(x2, cam2.pose());
//
//  boost::shared_ptr<GaussianFactor> factor = smartFactor1->linearize(values);
//
//  // compute triangulation from linearization point
//  // compute reprojection errors (sum squared)
//  // compare with factor.info(): the bottom right element is the squared sum of the reprojection errors (normalized by the covariance)
//  // check that it is correctly scaled when using noiseProjection = [1/4  0; 0 1/4]
//}
//
///* *************************************************************************/
//TEST( SmartProjectionFactorP, HessianWithRotation ) {
//  // cout << " ************************ SmartProjectionFactorP: rotated Hessian **********************" << endl;
//
//  using namespace vanillaPose;
//
//  KeyVector views {x1, x2, x3};
//
//  Point2Vector measurements_cam1, measurements_cam2, measurements_cam3;
//
//  projectToMultipleCameras(cam1, cam2, cam3, landmark1, measurements_cam1);
//
//  SmartFactorP::shared_ptr smartFactorInstance(new SmartFactorP(model, sharedK));
//  smartFactorInstance->add(measurements_cam1, views);
//
//  Values values;
//  values.insert(x1, cam1.pose());
//  values.insert(x2, cam2.pose());
//  values.insert(x3, cam3.pose());
//
//  boost::shared_ptr<GaussianFactor> factor = smartFactorInstance->linearize(
//      values);
//
//  Pose3 poseDrift = Pose3(Rot3::Ypr(-M_PI / 2, 0., -M_PI / 2), Point3(0, 0, 0));
//
//  Values rotValues;
//  rotValues.insert(x1, poseDrift.compose(level_pose));
//  rotValues.insert(x2, poseDrift.compose(pose_right));
//  rotValues.insert(x3, poseDrift.compose(pose_above));
//
//  boost::shared_ptr<GaussianFactor> factorRot = smartFactorInstance->linearize(
//      rotValues);
//
//  // Hessian is invariant to rotations in the nondegenerate case
//  EXPECT(assert_equal(factor->information(), factorRot->information(), 1e-7));
//
//  Pose3 poseDrift2 = Pose3(Rot3::Ypr(-M_PI / 2, -M_PI / 3, -M_PI / 2),
//      Point3(10, -4, 5));
//
//  Values tranValues;
//  tranValues.insert(x1, poseDrift2.compose(level_pose));
//  tranValues.insert(x2, poseDrift2.compose(pose_right));
//  tranValues.insert(x3, poseDrift2.compose(pose_above));
//
//  boost::shared_ptr<GaussianFactor> factorRotTran =
//      smartFactorInstance->linearize(tranValues);
//
//  // Hessian is invariant to rotations and translations in the nondegenerate case
//  EXPECT(assert_equal(factor->information(), factorRotTran->information(), 1e-7));
//}
//
///* *************************************************************************/
//TEST( SmartProjectionFactorP, HessianWithRotationDegenerate ) {
//
//  using namespace vanillaPose2;
//
//  KeyVector views {x1, x2, x3};
//
//  // All cameras have the same pose so will be degenerate !
//  Camera cam2(level_pose, sharedK2);
//  Camera cam3(level_pose, sharedK2);
//
//  Point2Vector measurements_cam1;
//  projectToMultipleCameras(cam1, cam2, cam3, landmark1, measurements_cam1);
//
//  SmartFactorP::shared_ptr smartFactor(new SmartFactorP(model, sharedK2));
//  smartFactor->add(measurements_cam1, views);
//
//  Values values;
//  values.insert(x1, cam1.pose());
//  values.insert(x2, cam2.pose());
//  values.insert(x3, cam3.pose());
//
//  boost::shared_ptr<GaussianFactor> factor = smartFactor->linearize(values);
//
//  Pose3 poseDrift = Pose3(Rot3::Ypr(-M_PI / 2, 0., -M_PI / 2), Point3(0, 0, 0));
//
//  Values rotValues;
//  rotValues.insert(x1, poseDrift.compose(level_pose));
//  rotValues.insert(x2, poseDrift.compose(level_pose));
//  rotValues.insert(x3, poseDrift.compose(level_pose));
//
//  boost::shared_ptr<GaussianFactor> factorRot = //
//      smartFactor->linearize(rotValues);
//
//  // Hessian is invariant to rotations in the nondegenerate case
//  EXPECT(assert_equal(factor->information(), factorRot->information(), 1e-7));
//
//  Pose3 poseDrift2 = Pose3(Rot3::Ypr(-M_PI / 2, -M_PI / 3, -M_PI / 2),
//      Point3(10, -4, 5));
//
//  Values tranValues;
//  tranValues.insert(x1, poseDrift2.compose(level_pose));
//  tranValues.insert(x2, poseDrift2.compose(level_pose));
//  tranValues.insert(x3, poseDrift2.compose(level_pose));
//
//  boost::shared_ptr<GaussianFactor> factorRotTran = smartFactor->linearize(
//      tranValues);
//
//  // Hessian is invariant to rotations and translations in the nondegenerate case
//  EXPECT(assert_equal(factor->information(), factorRotTran->information(), 1e-7));
//}
//
///* ************************************************************************* */
//TEST( SmartProjectionFactorP, ConstructorWithCal3Bundler) {
//  using namespace bundlerPose;
//  SmartProjectionParams params;
//  params.setDegeneracyMode(gtsam::ZERO_ON_DEGENERACY);
//  SmartFactorP factor(model, sharedBundlerK, params);
//  factor.add(measurement1, x1);
//}
//
///* *************************************************************************/
//TEST( SmartProjectionFactorP, Cal3Bundler ) {
//
//  using namespace bundlerPose;
//
//  // three landmarks ~5 meters in front of camera
//  Point3 landmark3(3, 0, 3.0);
//
//  Point2Vector measurements_cam1, measurements_cam2, measurements_cam3;
//
//  // Project three landmarks into three cameras
//  projectToMultipleCameras(cam1, cam2, cam3, landmark1, measurements_cam1);
//  projectToMultipleCameras(cam1, cam2, cam3, landmark2, measurements_cam2);
//  projectToMultipleCameras(cam1, cam2, cam3, landmark3, measurements_cam3);
//
//  KeyVector views {x1, x2, x3};
//
//  SmartFactorP::shared_ptr smartFactor1(new SmartFactorP(model, sharedBundlerK));
//  smartFactor1->add(measurements_cam1, views);
//
//  SmartFactorP::shared_ptr smartFactor2(new SmartFactorP(model, sharedBundlerK));
//  smartFactor2->add(measurements_cam2, views);
//
//  SmartFactorP::shared_ptr smartFactor3(new SmartFactorP(model, sharedBundlerK));
//  smartFactor3->add(measurements_cam3, views);
//
//  const SharedDiagonal noisePrior = noiseModel::Isotropic::Sigma(6, 0.10);
//
//  NonlinearFactorGraph graph;
//  graph.push_back(smartFactor1);
//  graph.push_back(smartFactor2);
//  graph.push_back(smartFactor3);
//  graph.addPrior(x1, cam1.pose(), noisePrior);
//  graph.addPrior(x2, cam2.pose(), noisePrior);
//
//  //  Pose3 noise_pose = Pose3(Rot3::Ypr(-M_PI/10, 0., -M_PI/10), Point3(0.5,0.1,0.3)); // noise from regular projection factor test below
//  Pose3 noise_pose = Pose3(Rot3::Ypr(-M_PI / 100, 0., -M_PI / 100),
//      Point3(0.1, 0.1, 0.1)); // smaller noise
//  Values values;
//  values.insert(x1, cam1.pose());
//  values.insert(x2, cam2.pose());
//  // initialize third pose with some noise, we expect it to move back to original pose_above
//  values.insert(x3, pose_above * noise_pose);
//  EXPECT(
//      assert_equal(
//          Pose3(
//              Rot3(0, -0.0314107591, 0.99950656, -0.99950656, -0.0313952598,
//                  -0.000986635786, 0.0314107591, -0.999013364, -0.0313952598),
//              Point3(0.1, -0.1, 1.9)), values.at<Pose3>(x3)));
//
//  Values result;
//  LevenbergMarquardtOptimizer optimizer(graph, values, lmParams);
//  result = optimizer.optimize();
//  EXPECT(assert_equal(cam3.pose(), result.at<Pose3>(x3), 1e-6));
//}
//
///* *************************************************************************/
//TEST( SmartProjectionFactorP, Cal3BundlerRotationOnly ) {
//
//  using namespace bundlerPose;
//
//  KeyVector views {x1, x2, x3};
//
//  // Two different cameras
//  Pose3 pose2 = level_pose
//      * Pose3(Rot3::RzRyRx(-0.05, 0.0, -0.05), Point3(0, 0, 0));
//  Pose3 pose3 = pose2 * Pose3(Rot3::RzRyRx(-0.05, 0.0, -0.05), Point3(0, 0, 0));
//  Camera cam2(pose2, sharedBundlerK);
//  Camera cam3(pose3, sharedBundlerK);
//
//  // landmark3 at 3 meters now
//  Point3 landmark3(3, 0, 3.0);
//
//  Point2Vector measurements_cam1, measurements_cam2, measurements_cam3;
//
//  // Project three landmarks into three cameras
//  projectToMultipleCameras(cam1, cam2, cam3, landmark1, measurements_cam1);
//  projectToMultipleCameras(cam1, cam2, cam3, landmark2, measurements_cam2);
//  projectToMultipleCameras(cam1, cam2, cam3, landmark3, measurements_cam3);
//
//  SmartProjectionParams params;
//  params.setRankTolerance(10);
//  params.setDegeneracyMode(gtsam::ZERO_ON_DEGENERACY);
//
//  SmartFactorP::shared_ptr smartFactor1(
//      new SmartFactorP(model, sharedBundlerK, params));
//  smartFactor1->add(measurements_cam1, views);
//
//  SmartFactorP::shared_ptr smartFactor2(
//      new SmartFactorP(model, sharedBundlerK, params));
//  smartFactor2->add(measurements_cam2, views);
//
//  SmartFactorP::shared_ptr smartFactor3(
//      new SmartFactorP(model, sharedBundlerK, params));
//  smartFactor3->add(measurements_cam3, views);
//
//  const SharedDiagonal noisePrior = noiseModel::Isotropic::Sigma(6, 0.10);
//  const SharedDiagonal noisePriorTranslation = noiseModel::Isotropic::Sigma(3,
//      0.10);
//  Point3 positionPrior = Point3(0, 0, 1);
//
//  NonlinearFactorGraph graph;
//  graph.push_back(smartFactor1);
//  graph.push_back(smartFactor2);
//  graph.push_back(smartFactor3);
//  graph.addPrior(x1, cam1.pose(), noisePrior);
//  graph.emplace_shared<PoseTranslationPrior<Pose3> >(x2, positionPrior, noisePriorTranslation);
//  graph.emplace_shared<PoseTranslationPrior<Pose3> >(x3, positionPrior, noisePriorTranslation);
//
//  //  Pose3 noise_pose = Pose3(Rot3::Ypr(-M_PI/10, 0., -M_PI/10), Point3(0.5,0.1,0.3)); // noise from regular projection factor test below
//  Pose3 noise_pose = Pose3(Rot3::Ypr(-M_PI / 100, 0., -M_PI / 100),
//      Point3(0.1, 0.1, 0.1)); // smaller noise
//  Values values;
//  values.insert(x1, cam1.pose());
//  values.insert(x2, cam2.pose());
//  // initialize third pose with some noise, we expect it to move back to original pose_above
//  values.insert(x3, pose3 * noise_pose);
//  EXPECT(
//      assert_equal(
//          Pose3(
//              Rot3(0.00563056869, -0.130848107, 0.991386438, -0.991390265,
//                  -0.130426831, -0.0115837907, 0.130819108, -0.98278564,
//                  -0.130455917),
//              Point3(0.0897734171, -0.110201006, 0.901022872)),
//          values.at<Pose3>(x3)));
//
//  Values result;
//  LevenbergMarquardtOptimizer optimizer(graph, values, lmParams);
//  result = optimizer.optimize();
//
//  EXPECT(
//      assert_equal(
//          Pose3(
//              Rot3(0.00563056869, -0.130848107, 0.991386438, -0.991390265,
//                  -0.130426831, -0.0115837907, 0.130819108, -0.98278564,
//                  -0.130455917),
//              Point3(0.0897734171, -0.110201006, 0.901022872)),
//          values.at<Pose3>(x3)));
//}
//
///* ************************************************************************* */
//BOOST_CLASS_EXPORT_GUID(gtsam::noiseModel::Constrained, "gtsam_noiseModel_Constrained");
//BOOST_CLASS_EXPORT_GUID(gtsam::noiseModel::Diagonal, "gtsam_noiseModel_Diagonal");
//BOOST_CLASS_EXPORT_GUID(gtsam::noiseModel::Gaussian, "gtsam_noiseModel_Gaussian");
//BOOST_CLASS_EXPORT_GUID(gtsam::noiseModel::Unit, "gtsam_noiseModel_Unit");
//BOOST_CLASS_EXPORT_GUID(gtsam::noiseModel::Isotropic, "gtsam_noiseModel_Isotropic");
//BOOST_CLASS_EXPORT_GUID(gtsam::SharedNoiseModel, "gtsam_SharedNoiseModel");
//BOOST_CLASS_EXPORT_GUID(gtsam::SharedDiagonal, "gtsam_SharedDiagonal");
//
//TEST(SmartProjectionFactorP, serialize) {
//  using namespace vanillaPose;
//  using namespace gtsam::serializationTestHelpers;
//  SmartProjectionParams params;
//  params.setRankTolerance(rankTol);
//  SmartFactorP factor(model, sharedK, params);
//
//  EXPECT(equalsObj(factor));
//  EXPECT(equalsXML(factor));
//  EXPECT(equalsBinary(factor));
//}
//
//TEST(SmartProjectionFactorP, serialize2) {
//  using namespace vanillaPose;
//  using namespace gtsam::serializationTestHelpers;
//  SmartProjectionParams params;
//  params.setRankTolerance(rankTol);
//  Pose3 bts;
//  SmartFactorP factor(model, sharedK, bts, params);
//
//  // insert some measurments
//  KeyVector key_view;
//  Point2Vector meas_view;
//  key_view.push_back(Symbol('x', 1));
//  meas_view.push_back(Point2(10, 10));
//  factor.add(meas_view, key_view);
//
//  EXPECT(equalsObj(factor));
//  EXPECT(equalsXML(factor));
//  EXPECT(equalsBinary(factor));
//}

/* ************************************************************************* */
int main() {
  TestResult tr;
  return TestRegistry::runAllTests(tr);
}
/* ************************************************************************* */
