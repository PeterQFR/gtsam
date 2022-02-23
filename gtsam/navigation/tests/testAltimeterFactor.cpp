/* ----------------------------------------------------------------------------

 * GTSAM Copyright 2010, Georgia Tech Research Corporation,
 * Atlanta, Georgia 30332-0415
 * All Rights Reserved
 * Authors: Frank Dellaert, et al. (see THANKS for the full author list)

 * See LICENSE for the license information

 * -------------------------------------------------------------------------- */

/**
 * @file    testAltimeterFactor.cpp
 * @brief   Unit test for BarometricFactor
 * @author  Peter Milani
 * @date   23 Feb, 2022
 */

#include <CppUnitLite/TestHarness.h>
#include <gtsam/base/Testable.h>
#include <gtsam/base/numericalDerivative.h>
#include <gtsam/navigation/AltimeterFactor.h>

#include <boost/bind/bind.hpp>

using namespace std::placeholders;
using namespace std;
using namespace gtsam;

// *************************************************************************
namespace example {}


// *************************************************************************
TEST(AltimeterFactor, Constructor) {
    using namespace example;

    // meters to barometric.

    double altMeasurement = 10.;

    // Factor
    Key key(1);
    Key key2(2);
    SharedNoiseModel model = noiseModel::Isotropic::Sigma(1, 0.25);
    AltimeterFactor factor(key, key2, altMeasurement, model);

    // Create a linearization point at zero error
    Pose3 T(Rot3::RzRyRx(0., 0., 0.), Point3(0., 0., 10.));
    double altBias = 0.;
    Vector1 zero;
    zero << 0.;
    EXPECT(assert_equal(zero, factor.evaluateError(T, altBias), 1e-5));

    // Calculate numerical derivatives
    Matrix expectedH = numericalDerivative21<Vector, Pose3, double>(
        std::bind(&AltimeterFactor::evaluateError, &factor,
                  std::placeholders::_1, std::placeholders::_2, boost::none,
                  boost::none),
        T, altBias);

    Matrix expectedH2 = numericalDerivative22<Vector, Pose3, double>(
        std::bind(&AltimeterFactor::evaluateError, &factor,
                  std::placeholders::_1, std::placeholders::_2, boost::none,
                  boost::none),
        T, altBias);

    // Use the factor to calculate the derivative
    Matrix actualH, actualH2;
    factor.evaluateError(T, altBias, actualH, actualH2);

    // Verify we get the expected error
    EXPECT(assert_equal(expectedH, actualH, 1e-8));
    EXPECT(assert_equal(expectedH2, actualH2, 1e-8));
}

// *************************************************************************

//***************************************************************************
TEST(BarometricFactor, nonZero) {
    using namespace example;

    // meters to barometric.

    double altMeasurement = 10.;

    // Factor
    Key key(1);
    Key key2(2);
    SharedNoiseModel model = noiseModel::Isotropic::Sigma(1, 0.25);
    AltimeterFactor factor(key, key2, altMeasurement, model);

    Pose3 T(Rot3::RzRyRx(0.5, 1., 1.), Point3(20., 30., 1.));
    double altBias = 5.;

    // Calculate numerical derivatives
    Matrix expectedH = numericalDerivative21<Vector, Pose3, double>(
        std::bind(&AltimeterFactor::evaluateError, &factor,
                  std::placeholders::_1, std::placeholders::_2, boost::none,
                  boost::none),
        T, altBias);

    Matrix expectedH2 = numericalDerivative22<Vector, Pose3, double>(
        std::bind(&AltimeterFactor::evaluateError, &factor,
                  std::placeholders::_1, std::placeholders::_2, boost::none,
                  boost::none),
        T, altBias);

    // Use the factor to calculate the derivative and the error
    Matrix actualH, actualH2;
    Vector error = factor.evaluateError(T, altBias, actualH, actualH2);
    Vector actual = (Vector(1) << -4.0).finished();

    // Verify we get the expected error
    EXPECT(assert_equal(expectedH, actualH, 1e-8));
    EXPECT(assert_equal(expectedH2, actualH2, 1e-8));
    EXPECT(assert_equal(error, actual, 1e-8));
}

// *************************************************************************
int main() {
    TestResult tr;
    return TestRegistry::runAllTests(tr);
}
// *************************************************************************
