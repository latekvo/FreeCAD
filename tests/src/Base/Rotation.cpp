#include <gtest/gtest.h>

#include <algorithm>
#include <cmath>
#include <cstdint>
#include <numbers>

#include <Base/Exception.h>
#include <Base/Matrix.h>
#include <Base/Rotation.h>


// NOLINTBEGIN(cppcoreguidelines-avoid-magic-numbers,readability-magic-numbers)
TEST(Rotation, TestNonUniformScaleLeft)
{
    Base::Rotation rot;
    rot.setYawPitchRoll(45.0, 0.0, 0.0);

    Base::Matrix4D mat;
    rot.getValue(mat);

    Base::Matrix4D scale;
    scale.scale(1.0, std::sqrt(3.0), 1.0);

    Base::Rotation scaled_rot(scale * mat);

    rot.setYawPitchRoll(60.0, 0.0, 0.0);
    EXPECT_EQ(scaled_rot.isSame(rot, 1.0e-7), true);
    EXPECT_EQ(rot.isSame(scaled_rot, 1.0e-7), true);
}

TEST(Rotation, TestNonUniformScaleRight)
{
    Base::Rotation rot;
    rot.setYawPitchRoll(20.0, 0.0, 0.0);

    Base::Matrix4D mat;
    rot.getValue(mat);

    Base::Matrix4D scale;
    scale.scale(2.0, 3.0, 4.0);

    Base::Rotation scaled_rot(mat * scale);

    EXPECT_EQ(scaled_rot.isSame(rot, 1.0e-7), true);
    EXPECT_EQ(rot.isSame(scaled_rot, 1.0e-7), true);
}

TEST(Rotation, TestUniformScaleGT1)
{
    Base::Rotation rot;
    rot.setYawPitchRoll(20.0, 0.0, 0.0);

    Base::Matrix4D mat;
    rot.getValue(mat);

    Base::Matrix4D scale;
    scale.scale(3.0, 3.0, 3.0);

    Base::Rotation scaled_rot(mat * scale);

    EXPECT_EQ(scaled_rot.isSame(rot, 1.0e-7), true);
    EXPECT_EQ(rot.isSame(scaled_rot, 1.0e-7), true);
}

TEST(Rotation, TestUniformScaleLT1)
{
    Base::Matrix4D mat;
    mat.scale(0.5);

    Base::Rotation scaled_rot(mat);

    EXPECT_EQ(scaled_rot.isSame(scaled_rot, 1.0e-7), true);
}

TEST(Rotation, TestRotationDecompose)
{
    Base::Matrix4D mat;
    mat.setCol(0, Base::Vector3d {1, 0, 0});
    mat.setCol(1, Base::Vector3d {1, 1, 0});
    mat.setCol(2, Base::Vector3d {0, 0, 1});

    // decompose rotation part
    EXPECT_TRUE(Base::Rotation {mat}.isIdentity());
}

namespace
{

// Deterministic pseudo random unit vectors, so the sweeps below are reproducible.
class UnitVectorSource
{
public:
    explicit UnitVectorSource(std::uint64_t seed)
        : state(seed)
    {}

    double uniform()
    {
        // xorshift64*
        state ^= state >> 12U;
        state ^= state << 25U;
        state ^= state >> 27U;
        auto val = static_cast<double>((state * 0x2545F4914F6CDD1DULL) >> 11U);
        return (val / static_cast<double>(1ULL << 53U)) * 2.0 - 1.0;
    }

    Base::Vector3d unitVector()
    {
        Base::Vector3d vec;
        do {
            vec = Base::Vector3d(uniform(), uniform(), uniform());
        } while (vec.Length() < 1.0e-3);
        return vec.Normalized();
    }

private:
    std::uint64_t state;
};

}  // namespace

// The axis and angle are recovered from the quaternion in evaluateVector(). Deriving the angle
// from acos(w) loses about half of the significant digits as |w| approaches 1, and once w rounds
// to exactly 1.0 -- which happens for every rotation below roughly 3e-8 rad -- the axis was
// thrown away altogether and replaced by +Z with an angle of zero.
TEST(Rotation, TestAxisAngleRoundTripForSmallAngles)
{
    UnitVectorSource src(20250728);

    for (int decade = 0; decade <= 12; ++decade) {
        const double expectedAngle = 3.0 * std::pow(10.0, -decade);

        double worstAngle = 0.0;
        double worstAxis = 0.0;
        for (int i = 0; i < 2000; ++i) {
            const Base::Vector3d expectedAxis = src.unitVector();

            const double half = expectedAngle / 2.0;
            Base::Rotation rot(
                expectedAxis.x * std::sin(half),
                expectedAxis.y * std::sin(half),
                expectedAxis.z * std::sin(half),
                std::cos(half)
            );

            Base::Vector3d axis;
            double angle {};
            rot.getValue(axis, angle);

            worstAngle = std::max(worstAngle, std::fabs(angle - expectedAngle) / expectedAngle);
            worstAxis = std::max(worstAxis, (axis - expectedAxis).Length());
        }

        EXPECT_LT(worstAngle, 1.0e-9) << "angle is wrong for a rotation of " << expectedAngle;
        EXPECT_LT(worstAxis, 1.0e-9) << "axis is wrong for a rotation of " << expectedAngle;
    }
}

// Composing a rotation with its own inverse leaves a quaternion whose vector part is rounding
// noise rather than zero. The rotation is the identity, so the axis has to stay the conventional
// +Z instead of being read off that noise.
TEST(Rotation, TestComposedIdentityKeepsTheConventionalAxis)
{
    UnitVectorSource src(20250729);
    const Base::Vector3d unitZ(0.0, 0.0, 1.0);

    for (int i = 0; i < 100000; ++i) {
        Base::Rotation rot(src.unitVector(), (src.uniform() + 1.0) * std::numbers::pi);
        Base::Vector3d axis;
        double angle {};
        (rot * rot.inverse()).getValue(axis, angle);

        ASSERT_LT((axis - unitZ).Length(), 1.0e-12);
        ASSERT_DOUBLE_EQ(angle, 0.0);
    }
}

TEST(Rotation, TestAxisAngleRoundTripOverTheWholeRange)
{
    UnitVectorSource src(20250730);

    for (int i = 0; i < 20000; ++i) {
        const Base::Vector3d expectedAxis = src.unitVector();
        // (0, 2*pi)
        const double expectedAngle = (src.uniform() + 1.0) * std::numbers::pi;

        Base::Rotation rot(expectedAxis, expectedAngle);
        Base::Rotation viaQuat(rot.getValue());

        Base::Vector3d axis;
        double angle {};
        viaQuat.getValue(axis, angle);

        ASSERT_NEAR(angle, expectedAngle, 1.0e-9);
        ASSERT_LT((axis - expectedAxis).Length(), 1.0e-9);
    }
}
// NOLINTEND(cppcoreguidelines-avoid-magic-numbers,readability-magic-numbers)
