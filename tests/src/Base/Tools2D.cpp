#include <gtest/gtest.h>

#include <cmath>
#include <random>

#include <Base/Tools2D.h>

class Line2D: public ::testing::Test
{
protected:
    Line2D()
        : pt1(0.0, 0.0)
        , pt2(3.0, 4.0)
    {}
    void SetUp() override
    {}
    void TearDown() override
    {}
    Base::Vector2d GetFirst() const
    {
        return pt1;
    }
    Base::Vector2d GetSecond() const
    {
        return pt2;
    }
    Base::Line2d GetLine() const
    {
        Base::Line2d line(pt1, pt2);
        return line;
    }

private:
    Base::Vector2d pt1, pt2;
};

TEST_F(Line2D, TestLength)
{
    Base::Line2d line(GetLine());
    EXPECT_DOUBLE_EQ(line.Length(), 5.0);
}

TEST_F(Line2D, TestPoints)
{
    Base::Line2d line(GetLine());
    EXPECT_EQ(line.clV1, GetFirst());
    EXPECT_EQ(line.clV2, GetSecond());
}

TEST_F(Line2D, TestFromPos)
{
    Base::Line2d line(GetLine());
    EXPECT_EQ(line.FromPos(2.5), Base::Vector2d(1.5, 2.0));
}

TEST_F(Line2D, TestContains)
{
    Base::Line2d line(GetLine());
    EXPECT_EQ(line.Contains(Base::Vector2d(1.5, 2.0)), true);
}

TEST(Polygon2D, TestDefault)
{
    Base::Polygon2d poly;
    EXPECT_EQ(poly.GetCtVectors(), 0);
}

TEST(Polygon2D, TestAdd)
{
    Base::Polygon2d poly;
    poly.Add(Base::Vector2d());
    EXPECT_EQ(poly.GetCtVectors(), 1);
}

TEST(Polygon2D, TestRemove)
{
    Base::Polygon2d poly;
    poly.Add(Base::Vector2d());
    EXPECT_EQ(poly.GetCtVectors(), 1);
    EXPECT_EQ(poly.Delete(1), false);
    EXPECT_EQ(poly.Delete(0), true);
    EXPECT_EQ(poly.GetCtVectors(), 0);
}

TEST(Polygon2D, TestClear)
{
    Base::Polygon2d poly;
    poly.Add(Base::Vector2d());
    poly.DeleteAll();
    EXPECT_EQ(poly.GetCtVectors(), 0);
}

namespace
{
// Distance from a point to the infinite line, relative to the line's own length.
double relativeResidual(const Base::Vector2d& point, const Base::Line2d& line)
{
    const Base::Vector2d dir = line.clV2 - line.clV1;
    const double length = dir.Length();
    if (length == 0.0) {
        return 0.0;
    }
    const Base::Vector2d rel = point - line.clV1;
    return std::fabs((rel.x * dir.y) - (rel.y * dir.x)) / (length * length);
}
}  // namespace

// Line2d::Intersect used to solve y = m*x + b. That form cannot express a vertical line, so it
// needed DBL_MAX as a sentinel, and it lost most of its precision on lines merely close to
// vertical because b = y - m*x cancels once m is large.
TEST_F(Line2D, TestIntersectNearVerticalLines)
{
    std::mt19937_64 rng(20250728);
    std::uniform_real_distribution<double> coordinate(-100.0, 100.0);

    for (double tilt : {1.0e-1, 1.0e-3, 1.0e-6, 1.0e-9}) {
        double worst = 0.0;
        for (int i = 0; i < 2000; ++i) {
            const double length1 = 1.0 + std::fabs(coordinate(rng));
            const double length2 = 1.0 + std::fabs(coordinate(rng));
            const double tilt1 = tilt * coordinate(rng) / 100.0;
            const double tilt2 = tilt * coordinate(rng) / 100.0;

            const Base::Vector2d start1(coordinate(rng), coordinate(rng));
            const Base::Line2d line1(
                start1,
                Base::Vector2d(
                    start1.x + (length1 * std::sin(tilt1)),
                    start1.y + (length1 * std::cos(tilt1))
                )
            );
            const Base::Vector2d start2(coordinate(rng), coordinate(rng));
            const Base::Line2d line2(
                start2,
                Base::Vector2d(
                    start2.x + (length2 * std::sin(tilt2 + 0.7)),
                    start2.y + (length2 * std::cos(tilt2 + 0.7))
                )
            );

            Base::Vector2d point;
            ASSERT_TRUE(line1.Intersect(line2, point));
            worst = std::max(
                worst,
                std::max(relativeResidual(point, line1), relativeResidual(point, line2))
            );
        }
        EXPECT_LT(worst, 1.0e-11) << "lines tilted " << tilt << " rad away from vertical";
    }
}

TEST_F(Line2D, TestIntersectExactCases)
{
    Base::Vector2d point;

    // two axis aligned lines
    const Base::Line2d horizontal(Base::Vector2d(-5.0, 2.0), Base::Vector2d(5.0, 2.0));
    const Base::Line2d vertical(Base::Vector2d(3.0, -7.0), Base::Vector2d(3.0, 7.0));
    ASSERT_TRUE(horizontal.Intersect(vertical, point));
    EXPECT_NEAR(point.x, 3.0, 1.0e-12);
    EXPECT_NEAR(point.y, 2.0, 1.0e-12);

    // and the other way round
    ASSERT_TRUE(vertical.Intersect(horizontal, point));
    EXPECT_NEAR(point.x, 3.0, 1.0e-12);
    EXPECT_NEAR(point.y, 2.0, 1.0e-12);

    // the intersection lies outside both segments -- Intersect() works on infinite lines
    const Base::Line2d shortA(Base::Vector2d(0.0, 0.0), Base::Vector2d(1.0, 0.0));
    const Base::Line2d shortB(Base::Vector2d(10.0, -1.0), Base::Vector2d(10.0, 1.0));
    ASSERT_TRUE(shortA.Intersect(shortB, point));
    EXPECT_NEAR(point.x, 10.0, 1.0e-12);
    EXPECT_NEAR(point.y, 0.0, 1.0e-12);
}

TEST_F(Line2D, TestIntersectParallelAndDegenerate)
{
    Base::Vector2d point;

    const Base::Line2d line(Base::Vector2d(0.0, 0.0), Base::Vector2d(2.0, 1.0));
    EXPECT_FALSE(
        line.Intersect(Base::Line2d(Base::Vector2d(0.0, 5.0), Base::Vector2d(2.0, 6.0)), point)
    );
    EXPECT_FALSE(
        line.Intersect(Base::Line2d(Base::Vector2d(0.0, 5.0), Base::Vector2d(4.0, 7.0)), point)
    );

    // both vertical
    const Base::Line2d v1(Base::Vector2d(1.0, 0.0), Base::Vector2d(1.0, 3.0));
    const Base::Line2d v2(Base::Vector2d(4.0, -2.0), Base::Vector2d(4.0, 9.0));
    EXPECT_FALSE(v1.Intersect(v2, point));

    // a line of zero length is not a line
    const Base::Line2d degenerate(Base::Vector2d(1.0, 1.0), Base::Vector2d(1.0, 1.0));
    EXPECT_FALSE(line.Intersect(degenerate, point));
    EXPECT_FALSE(degenerate.Intersect(line, point));
}

// Both slopes used to collapse onto the DBL_MAX sentinel once the lines came within 1e-10 of
// vertical, and the exact m1 == m2 test that followed then declared them parallel. Two lines that
// do meet were reported as never meeting at all.
TEST_F(Line2D, TestNearVerticalLinesAreNotReportedParallel)
{
    Base::Vector2d point;

    const Base::Line2d vertical(Base::Vector2d(0.0, 0.0), Base::Vector2d(0.0, 10.0));
    const Base::Line2d tilted(Base::Vector2d(5.0, 0.0), Base::Vector2d(5.0 + 1.0e-11, 10.0));

    // The intersection is a long way off, so pin it down by the property that defines it: the
    // point has to lie on both infinite lines. |AB x AP| / |AB| is its distance from line AB.
    auto distanceFromLine = [](const Base::Line2d& line, const Base::Vector2d& pnt) {
        const Base::Vector2d dir = line.clV2 - line.clV1;
        const Base::Vector2d rel = pnt - line.clV1;
        return std::fabs(dir.x * rel.y - dir.y * rel.x) / dir.Length();
    };

    ASSERT_TRUE(vertical.Intersect(tilted, point));
    EXPECT_LT(distanceFromLine(vertical, point), 1.0e-6);
    EXPECT_LT(distanceFromLine(tilted, point), 1.0e-6);

    // and the same pair the other way round
    ASSERT_TRUE(tilted.Intersect(vertical, point));
    EXPECT_LT(distanceFromLine(vertical, point), 1.0e-6);
    EXPECT_LT(distanceFromLine(tilted, point), 1.0e-6);
}
