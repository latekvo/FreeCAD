// SPDX-License-Identifier: LGPL-2.1-or-later

#include <gtest/gtest.h>

#include <algorithm>
#include <cstdint>
#include <iterator>
#include <set>
#include <vector>

#include "Mod/Part/App/BRepMesh.h"

// NOLINTBEGIN
class BRepMeshTest: public ::testing::Test
{
protected:
    void SetUp() override
    {}

    void TearDown() override
    {}

    std::vector<Part::BRepMesh::Domain> getNoDomains() const
    {
        std::vector<Part::BRepMesh::Domain> domains;
        return domains;
    }

    std::vector<Part::BRepMesh::Domain> getEmptyDomains() const
    {
        Part::BRepMesh::Domain domain;
        std::vector<Part::BRepMesh::Domain> domains;
        domains.push_back(domain);
        domains.push_back(domain);
        return domains;
    }

    std::vector<Part::BRepMesh::Domain> getConnectedDomains() const
    {
        Part::BRepMesh::Domain domain1;
        domain1.points.emplace_back(0, 0, 0);
        domain1.points.emplace_back(10, 0, 0);
        domain1.points.emplace_back(10, 10, 0);
        domain1.points.emplace_back(0, 10, 0);

        {
            Part::BRepMesh::Facet f1;
            f1.I1 = 0;
            f1.I2 = 1;
            f1.I3 = 2;
            domain1.facets.emplace_back(f1);
        }
        {
            Part::BRepMesh::Facet f2;
            f2.I1 = 0;
            f2.I2 = 2;
            f2.I3 = 3;
            domain1.facets.emplace_back(f2);
        }

        Part::BRepMesh::Domain domain2;
        domain2.points.emplace_back(0, 0, 0);
        domain2.points.emplace_back(0, 10, 0);
        domain2.points.emplace_back(0, 10, 10);
        domain2.points.emplace_back(0, 0, 10);

        {
            Part::BRepMesh::Facet f1;
            f1.I1 = 0;
            f1.I2 = 1;
            f1.I3 = 2;
            domain2.facets.emplace_back(f1);
        }
        {
            Part::BRepMesh::Facet f2;
            f2.I1 = 0;
            f2.I2 = 2;
            f2.I3 = 3;
            domain2.facets.emplace_back(f2);
        }

        std::vector<Part::BRepMesh::Domain> domains;
        domains.push_back(domain1);
        domains.push_back(domain2);
        return domains;
    }

    std::vector<Part::BRepMesh::Domain> getUnconnectedDomains() const
    {
        double eps = 1.0e-10;
        Part::BRepMesh::Domain domain1;
        domain1.points.emplace_back(eps, eps, eps);
        domain1.points.emplace_back(10, 0, 0);
        domain1.points.emplace_back(10, 10, 0);
        domain1.points.emplace_back(eps, 10, eps);

        {
            Part::BRepMesh::Facet f1;
            f1.I1 = 0;
            f1.I2 = 1;
            f1.I3 = 2;
            domain1.facets.emplace_back(f1);
        }
        {
            Part::BRepMesh::Facet f2;
            f2.I1 = 0;
            f2.I2 = 2;
            f2.I3 = 3;
            domain1.facets.emplace_back(f2);
        }

        Part::BRepMesh::Domain domain2;
        domain2.points.emplace_back(0, 0, 0);
        domain2.points.emplace_back(0, 10, 0);
        domain2.points.emplace_back(0, 10, 10);
        domain2.points.emplace_back(0, 0, 10);

        {
            Part::BRepMesh::Facet f1;
            f1.I1 = 0;
            f1.I2 = 1;
            f1.I3 = 2;
            domain2.facets.emplace_back(f1);
        }
        {
            Part::BRepMesh::Facet f2;
            f2.I1 = 0;
            f2.I2 = 2;
            f2.I3 = 3;
            domain2.facets.emplace_back(f2);
        }

        std::vector<Part::BRepMesh::Domain> domains;
        domains.push_back(domain1);
        domains.push_back(domain2);
        return domains;
    }

    // Three points A, B, C with tol = Precision::Confusion() = 1e-7 such that A and C are the
    // same point within the tolerance, while the tolerant "less than" that used to drive the
    // merge orders them A < B < C:
    //   A < B  because |A.x - B.x| < tol, so the comparison falls through to y and 0 < 5
    //   B < C  because |B.x - C.x| > tol and B.x < C.x
    //   A == C because every coordinate is within tol
    // Sorting therefore parks B between the two duplicates and std::adjacent_find never sees
    // them next to each other -- the classic symptom of a comparator that is not a strict weak
    // ordering.
    std::vector<Part::BRepMesh::Domain> getDomainWithNonAdjacentDuplicates() const
    {
        Part::BRepMesh::Domain domain;
        domain.points.emplace_back(0.0, 0.0, 0.0);      // A
        domain.points.emplace_back(-0.2e-7, 5.0, 0.0);  // B
        domain.points.emplace_back(0.9e-7, 0.0, 0.0);   // C, same as A within the tolerance
        domain.points.emplace_back(10.0, 0.0, 0.0);
        domain.points.emplace_back(10.0, 5.0, 0.0);

        {
            Part::BRepMesh::Facet f1;
            f1.I1 = 0;  // A
            f1.I2 = 1;  // B
            f1.I3 = 3;
            domain.facets.emplace_back(f1);
        }
        {
            Part::BRepMesh::Facet f2;
            f2.I1 = 2;  // C
            f2.I2 = 3;
            f2.I3 = 4;
            domain.facets.emplace_back(f2);
        }

        std::vector<Part::BRepMesh::Domain> domains;
        domains.push_back(domain);
        return domains;
    }

    // Two points within the tolerance of each other but on opposite sides of a lookup grid
    // border, plus a third point sharing a cell with the first. A guard rather than a
    // reproduction: any scheme that only compares points sharing a cell, or only compares
    // neighbours in a cell-sorted order, misses this.
    std::vector<Part::BRepMesh::Domain> getDomainWithDuplicatesAcrossACellBorder() const
    {
        Part::BRepMesh::Domain domain;
        domain.points.emplace_back(6.36e-6, 0.0, 0.0);  // A
        domain.points.emplace_back(6.44e-6, 0.0, 0.0);  // B, 0.8e-7 from A
        domain.points.emplace_back(6.00e-6, 5.0, 0.0);  // C, shares a cell with A
        domain.points.emplace_back(10.0, 0.0, 0.0);
        domain.points.emplace_back(10.0, 5.0, 0.0);

        {
            Part::BRepMesh::Facet f1;
            f1.I1 = 0;  // A
            f1.I2 = 2;  // C
            f1.I3 = 3;
            domain.facets.emplace_back(f1);
        }
        {
            Part::BRepMesh::Facet f2;
            f2.I1 = 1;  // B
            f2.I2 = 3;
            f2.I3 = 4;
            domain.facets.emplace_back(f2);
        }

        std::vector<Part::BRepMesh::Domain> domains;
        domains.push_back(domain);
        return domains;
    }
};

TEST_F(BRepMeshTest, testNoDomains)
{
    std::vector<Base::Vector3d> points;
    std::vector<Part::BRepMesh::Facet> faces;
    Part::BRepMesh brepMesh;
    brepMesh.getFacesFromDomains(getNoDomains(), points, faces);

    EXPECT_TRUE(points.empty());
    EXPECT_TRUE(faces.empty());
}

TEST_F(BRepMeshTest, testEmptyDomains)
{
    std::vector<Base::Vector3d> points;
    std::vector<Part::BRepMesh::Facet> faces;
    Part::BRepMesh brepMesh;
    brepMesh.getFacesFromDomains(getEmptyDomains(), points, faces);

    EXPECT_TRUE(points.empty());
    EXPECT_TRUE(faces.empty());
}

TEST_F(BRepMeshTest, testConnectedDomains)
{
    std::vector<Base::Vector3d> points;
    std::vector<Part::BRepMesh::Facet> faces;
    Part::BRepMesh brepMesh;
    brepMesh.getFacesFromDomains(getConnectedDomains(), points, faces);

    EXPECT_EQ(points.size(), 6);
    EXPECT_EQ(faces.size(), 4);
}

TEST_F(BRepMeshTest, testUnconnectedDomains)
{
    std::vector<Base::Vector3d> points;
    std::vector<Part::BRepMesh::Facet> faces;
    Part::BRepMesh brepMesh;
    brepMesh.getFacesFromDomains(getUnconnectedDomains(), points, faces);

    EXPECT_EQ(points.size(), 6);
    EXPECT_EQ(faces.size(), 4);
}

namespace
{
std::size_t countSharedPoints(const Part::BRepMesh::Facet& lhs, const Part::BRepMesh::Facet& rhs)
{
    std::set<std::uint32_t> first {lhs.I1, lhs.I2, lhs.I3};
    std::set<std::uint32_t> second {rhs.I1, rhs.I2, rhs.I3};
    std::vector<std::uint32_t> shared;
    std::set_intersection(
        first.begin(),
        first.end(),
        second.begin(),
        second.end(),
        std::back_inserter(shared)
    );
    return shared.size();
}
}  // namespace

TEST_F(BRepMeshTest, testDuplicatesSeparatedInSortOrderAreMerged)
{
    std::vector<Base::Vector3d> points;
    std::vector<Part::BRepMesh::Facet> faces;
    Part::BRepMesh brepMesh;
    brepMesh.getFacesFromDomains(getDomainWithNonAdjacentDuplicates(), points, faces);

    // A and C must be welded into a single point, leaving 4 of the original 5.
    EXPECT_EQ(points.size(), 4);
    ASSERT_EQ(faces.size(), 2);

    // and the two facets must actually share that point now
    EXPECT_EQ(countSharedPoints(faces[0], faces[1]), 2);
}

TEST_F(BRepMeshTest, testDuplicatesAcrossACellBorderAreMerged)
{
    std::vector<Base::Vector3d> points;
    std::vector<Part::BRepMesh::Facet> faces;
    Part::BRepMesh brepMesh;
    brepMesh.getFacesFromDomains(getDomainWithDuplicatesAcrossACellBorder(), points, faces);

    EXPECT_EQ(points.size(), 4);
    ASSERT_EQ(faces.size(), 2);
    EXPECT_EQ(countSharedPoints(faces[0], faces[1]), 2);
}
// NOLINTEND
