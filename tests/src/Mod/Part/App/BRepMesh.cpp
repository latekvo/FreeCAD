// SPDX-License-Identifier: LGPL-2.1-or-later

#include <gtest/gtest.h>
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

    // A domain with a sliver triangle whose first two corners are distinct but closer together
    // than Precision::Confusion() -- the shape a fillet run-out or a cone apex tessellates into.
    // Welding those two corners turns the triangle into a zero-area facet.
    std::vector<Part::BRepMesh::Domain> getDomainWithSliverFacet() const
    {
        const double eps = 1.0e-10;

        Part::BRepMesh::Domain domain;
        domain.points.emplace_back(0, 0, 0);
        domain.points.emplace_back(eps, eps, 0);
        domain.points.emplace_back(10, 0, 0);
        domain.points.emplace_back(10, 10, 0);

        {
            // degenerates once points 0 and 1 are welded
            Part::BRepMesh::Facet f1;
            f1.I1 = 0;
            f1.I2 = 1;
            f1.I3 = 2;
            domain.facets.emplace_back(f1);
        }
        {
            Part::BRepMesh::Facet f2;
            f2.I1 = 0;
            f2.I2 = 2;
            f2.I3 = 3;
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

TEST_F(BRepMeshTest, testSliverFacetIsRemovedWhenItsCornersAreWelded)
{
    std::vector<Base::Vector3d> points;
    std::vector<Part::BRepMesh::Facet> faces;
    Part::BRepMesh brepMesh;
    brepMesh.getFacesFromDomains(getDomainWithSliverFacet(), points, faces);

    // the two near-coincident corners must be welded into one point ...
    EXPECT_EQ(points.size(), 3);
    // ... and the facet they degenerate must not be handed out
    EXPECT_EQ(faces.size(), 1);

    for (const auto& face : faces) {
        EXPECT_NE(face.I1, face.I2);
        EXPECT_NE(face.I2, face.I3);
        EXPECT_NE(face.I3, face.I1);
    }

    // every index must still address a real point
    for (const auto& face : faces) {
        EXPECT_LT(static_cast<std::size_t>(face.I1), points.size());
        EXPECT_LT(static_cast<std::size_t>(face.I2), points.size());
        EXPECT_LT(static_cast<std::size_t>(face.I3), points.size());
    }
}

TEST_F(BRepMeshTest, testSegmentsStayInsideTheFacetArray)
{
    std::vector<Base::Vector3d> points;
    std::vector<Part::BRepMesh::Facet> faces;
    Part::BRepMesh brepMesh;
    brepMesh.getFacesFromDomains(getDomainWithSliverFacet(), points, faces);

    // createSegments() slices the facet array back apart by domain size, so dropping a facet
    // has to shrink the domain it came from as well.
    std::size_t indexed = 0;
    for (const auto& segment : brepMesh.createSegments()) {
        indexed += segment.size();
        for (std::size_t index : segment) {
            EXPECT_LT(index, faces.size()) << "segment index past the end of the facet array";
        }
    }
    EXPECT_EQ(indexed, faces.size()) << "segments cover a different number of facets";
}
// NOLINTEND
