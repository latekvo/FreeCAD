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

bool isFinite(const Base::Rotation& rot)
{
    const double* quat = rot.getValue();
    return std::isfinite(quat[0]) && std::isfinite(quat[1]) && std::isfinite(quat[2])
        && std::isfinite(quat[3]);
}

}  // namespace

// Base::Rotation(from, to) used to build its axis from u%v and its angle from acos(u*v).
// Rounding regularly pushes u*v marginally outside [-1,1], and acos() then returns NaN.
TEST(Rotation, TestFromToIsNeverNaN)
{
    UnitVectorSource src(20250731);

    for (int i = 0; i < 200000; ++i) {
        Base::Vector3d from = src.unitVector();

        // Nearly parallel, nearly antiparallel and unconstrained pairs.
        const double eps = 1.0e-9;
        Base::Vector3d nearlyParallel(
            from.x + eps * src.uniform(),
            from.y + eps * src.uniform(),
            from.z + eps * src.uniform()
        );
        Base::Vector3d nearlyOpposite(
            -from.x + eps * src.uniform(),
            -from.y + eps * src.uniform(),
            -from.z + eps * src.uniform()
        );

        ASSERT_TRUE(isFinite(Base::Rotation(from, nearlyParallel.Normalized())));
        ASSERT_TRUE(isFinite(Base::Rotation(from, nearlyOpposite.Normalized())));
        ASSERT_TRUE(isFinite(Base::Rotation(from, src.unitVector())));
    }
}

// The whole point of Rotation(from, to) is that it maps 'from' onto 'to'. Forming the axis as
// u%v and the angle as acos(u*v) loses so much accuracy near theta == 0 and theta == pi that the
// resulting rotation could be off by more than a radian.
TEST(Rotation, TestFromToMapsFromOntoTo)
{
    UnitVectorSource src(20250801);
    double worst = 0.0;

    for (int i = 0; i < 200000; ++i) {
        Base::Vector3d from = src.unitVector();

        // Build 'to' by rotating 'from' about a perpendicular axis by a known angle. The angles
        // are spread logarithmically over [1e-18, pi] and over [pi-1e-18, pi] so that both
        // degenerate ends are covered.
        Base::Vector3d axis = src.unitVector();
        axis = axis - from * (from * axis);
        if (axis.Length() < 1.0e-3) {
            continue;
        }
        axis.Normalize();

        const double decades = 18.0 * std::fabs(src.uniform());
        double angle = std::pow(10.0, -decades);
        if (i % 2 == 0) {
            angle = std::numbers::pi - angle;
        }

        Base::Vector3d to = from * std::cos(angle) + axis.Cross(from) * std::sin(angle);
        to.Normalize();

        Base::Vector3d mapped = Base::Rotation(from, to).multVec(from);
        worst = std::max(worst, (mapped - to).Length());
    }

    EXPECT_LT(worst, 1.0e-12);
}

TEST(Rotation, TestFromToExactCases)
{
    const Base::Vector3d unitZ(0.0, 0.0, 1.0);
    const Base::Vector3d unitY(0.0, 1.0, 0.0);
    const Base::Vector3d null(0.0, 0.0, 0.0);

    EXPECT_TRUE(Base::Rotation(unitZ, unitZ).isIdentity());

    // Exactly antiparallel: any perpendicular axis with a rotation of pi will do.
    Base::Rotation flip(unitZ, -unitZ);
    EXPECT_TRUE(isFinite(flip));
    EXPECT_LT((flip.multVec(unitZ) + unitZ).Length(), 1.0e-12);

    // Orthogonal.
    Base::Rotation quarter(unitZ, unitY);
    EXPECT_LT((quarter.multVec(unitZ) - unitY).Length(), 1.0e-12);

    // A null vector has no direction; the result must still be a valid rotation.
    EXPECT_TRUE(Base::Rotation(null, unitZ).isIdentity());
    EXPECT_TRUE(Base::Rotation(unitZ, null).isIdentity());
    EXPECT_TRUE(Base::Rotation(null, null).isIdentity());

    // Input vectors need not be normalised.
    Base::Rotation scaled(Base::Vector3d(0.0, 0.0, 3.0), Base::Vector3d(0.0, 7.0, 0.0));
    EXPECT_LT((scaled.multVec(unitZ) - unitY).Length(), 1.0e-12);
}

// Rotation::fromNormalVector() is Rotation(+Z, normal), so normals pointing downwards land
// exactly in the ill-conditioned antiparallel regime.
TEST(Rotation, TestFromNormalVectorNearNegativeZ)
{
    UnitVectorSource src(20250802);
    const Base::Vector3d unitZ(0.0, 0.0, 1.0);

    for (int i = 0; i < 100000; ++i) {
        const double eps = 1.0e-9;
        Base::Vector3d normal(eps * src.uniform(), eps * src.uniform(), -1.0);
        normal.Normalize();

        Base::Rotation rot = Base::Rotation::fromNormalVector(normal);
        ASSERT_TRUE(isFinite(rot));
        ASSERT_LT((rot.multVec(unitZ) - normal).Length(), 1.0e-12);
    }
}
// NOLINTEND(cppcoreguidelines-avoid-magic-numbers,readability-magic-numbers)
