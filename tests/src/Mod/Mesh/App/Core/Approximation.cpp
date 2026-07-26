// SPDX-License-Identifier: LGPL-2.1-or-later

#include <gtest/gtest.h>

#include <algorithm>
#include <cmath>
#include <numbers>
#include <vector>

#include <Mod/Mesh/App/Core/Approximation.h>

// NOLINTBEGIN(cppcoreguidelines-*,readability-*)

namespace
{

// A 10x10 planar patch, tilted so that its normal is not axis aligned, translated so that its
// centre sits at (offset, offset, offset).
std::vector<Base::Vector3f> tiltedPatch(double offset, Base::Vector3d& expectedNormal)
{
    constexpr double slopeX = 0.3;
    constexpr double slopeY = -0.2;

    expectedNormal = Base::Vector3d(-slopeX, -slopeY, 1.0);
    expectedNormal.Normalize();

    std::vector<Base::Vector3f> points;
    constexpr int steps = 20;
    for (int i = 0; i < steps; ++i) {
        for (int j = 0; j < steps; ++j) {
            const double localX = -5.0 + 10.0 * i / (steps - 1);
            const double localY = -5.0 + 10.0 * j / (steps - 1);
            const double localZ = slopeX * localX + slopeY * localY;
            points.emplace_back(
                static_cast<float>(offset + localX),
                static_cast<float>(offset + localY),
                static_cast<float>(offset + localZ)
            );
        }
    }
    return points;
}

double angleBetweenDeg(const Base::Vector3d& lhs, const Base::Vector3d& rhs)
{
    double dot = std::fabs(lhs * rhs);
    dot = std::min(dot, 1.0);
    return std::acos(dot) * 180.0 / std::numbers::pi;
}

}  // namespace

// PlaneFit used to accumulate the raw second moments and subtract the squared mean afterwards,
// with the products formed in single precision. For a patch far away from the origin the two
// terms nearly cancel and the fitted normal degenerates completely.
TEST(PlaneFit, FitIsAccurateFarFromOrigin)
{
    for (double offset : {0.0, 100.0, 1000.0, 10000.0, 100000.0}) {
        Base::Vector3d expectedNormal;
        const std::vector<Base::Vector3f> points = tiltedPatch(offset, expectedNormal);

        MeshCore::PlaneFit fit;
        fit.AddPoints(points);
        fit.Fit();

        const Base::Vector3f fitted = fit.GetNormal();
        const Base::Vector3d normal(fitted.x, fitted.y, fitted.z);

        // The mesh stores points -- and PlaneFit reports its directions -- as floats, which puts
        // a floor of roughly 0.01 deg on what any implementation can achieve here; that floor is
        // already reached at offset 0. The point of the test is that moving the patch away from
        // the origin must not make things dramatically worse: before the fit was centred, the
        // very same patch came out 1.45 deg off at 10 m and 30.1 deg off at 100 m.
        EXPECT_LT(angleBetweenDeg(normal, expectedNormal), 0.05)
            << "plane normal is wrong for a patch at offset " << offset;
    }
}

TEST(PlaneFit, BaseIsTheCentreOfGravity)
{
    Base::Vector3d expectedNormal;
    const std::vector<Base::Vector3f> points = tiltedPatch(1000.0, expectedNormal);

    MeshCore::PlaneFit fit;
    fit.AddPoints(points);
    fit.Fit();

    const Base::Vector3f gravity = fit.GetGravity();
    const Base::Vector3f base = fit.GetBase();
    EXPECT_NEAR(base.x, gravity.x, 1.0e-2F);
    EXPECT_NEAR(base.y, gravity.y, 1.0e-2F);
    EXPECT_NEAR(base.z, gravity.z, 1.0e-2F);
}

TEST(PlaneFit, FittedPointsLieOnThePlane)
{
    Base::Vector3d expectedNormal;
    const std::vector<Base::Vector3f> points = tiltedPatch(10000.0, expectedNormal);

    MeshCore::PlaneFit fit;
    fit.AddPoints(points);
    fit.Fit();

    float worst = 0.0F;
    for (const auto& point : points) {
        worst = std::max(worst, std::fabs(fit.GetDistanceToPlane(point)));
    }

    // The input coordinates themselves are floats, so ~1e-3 mm of quantisation noise at this
    // distance from the origin is expected; anything beyond that comes from the fit.
    EXPECT_LT(worst, 0.01F);
}

// NOLINTEND(cppcoreguidelines-*,readability-*)
