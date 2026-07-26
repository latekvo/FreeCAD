// SPDX-License-Identifier: LGPL-2.1-or-later

#include <gtest/gtest.h>

#include <vector>

#include <Mod/Mesh/App/Core/Degeneration.h>
#include <Mod/Mesh/App/Core/Elements.h>
#include <Mod/Mesh/App/Core/MeshKernel.h>

// NOLINTBEGIN(cppcoreguidelines-*,readability-*)

namespace
{

// Builds a strip of independent triangles. Every entry in 'degenerated' that is true produces a
// triangle whose second corner sits on top of the first one, i.e. a zero-area facet.
MeshCore::MeshKernel makeStrip(const std::vector<bool>& degenerated)
{
    MeshCore::MeshPointArray points;
    MeshCore::MeshFacetArray facets;

    for (std::size_t i = 0; i < degenerated.size(); ++i) {
        const auto base = static_cast<float>(i) * 10.0F;
        points.push_back(MeshCore::MeshPoint(Base::Vector3f(base, 0.0F, 0.0F)));
        points.push_back(
            MeshCore::MeshPoint(
                degenerated[i] ? Base::Vector3f(base, 0.0F, 0.0F)
                               : Base::Vector3f(base + 5.0F, 0.0F, 0.0F)
            )
        );
        points.push_back(MeshCore::MeshPoint(Base::Vector3f(base, 5.0F, 0.0F)));

        MeshCore::MeshFacet facet;
        facet._aulPoints[0] = static_cast<MeshCore::PointIndex>(3 * i);
        facet._aulPoints[1] = static_cast<MeshCore::PointIndex>(3 * i + 1);
        facet._aulPoints[2] = static_cast<MeshCore::PointIndex>(3 * i + 2);
        facets.push_back(facet);
    }

    MeshCore::MeshKernel kernel;
    kernel.Assign(points, facets, true);
    return kernel;
}

std::size_t countDegenerated(MeshCore::MeshKernel& kernel)
{
    MeshCore::MeshEvalDegeneratedFacets eval(kernel, 0.0F);
    return eval.GetIndices().size();
}

}  // namespace

// MeshFixDegeneratedFacets rewound its iterator with Set(index - 1) after removing a facet.
// FacetIndex is unsigned, so removing the facet at index 0 wrapped that to FACET_INDEX_MAX, the
// iterator jumped to end() and the loop gave up -- leaving every later degenerated facet in the
// mesh while still reporting success.
TEST(MeshFixDegeneratedFacets, RemovesAllWhenTheFirstFacetIsDegenerated)
{
    MeshCore::MeshKernel kernel = makeStrip({true, false, true, false, true});
    ASSERT_EQ(kernel.CountFacets(), 5);
    ASSERT_EQ(countDegenerated(kernel), 3);

    MeshCore::MeshFixDegeneratedFacets fix(kernel, 0.0F);
    EXPECT_TRUE(fix.Fixup());

    EXPECT_EQ(countDegenerated(kernel), 0);
    EXPECT_EQ(kernel.CountFacets(), 2);
}

TEST(MeshFixDegeneratedFacets, RemovesAllWhenTheFirstFacetIsFine)
{
    MeshCore::MeshKernel kernel = makeStrip({false, true, false, true});
    ASSERT_EQ(countDegenerated(kernel), 2);

    MeshCore::MeshFixDegeneratedFacets fix(kernel, 0.0F);
    EXPECT_TRUE(fix.Fixup());

    EXPECT_EQ(countDegenerated(kernel), 0);
    EXPECT_EQ(kernel.CountFacets(), 2);
}

TEST(MeshFixDegeneratedFacets, RemovesAllWhenEveryFacetIsDegenerated)
{
    MeshCore::MeshKernel kernel = makeStrip({true, true, true});
    ASSERT_EQ(countDegenerated(kernel), 3);

    MeshCore::MeshFixDegeneratedFacets fix(kernel, 0.0F);
    EXPECT_TRUE(fix.Fixup());

    EXPECT_EQ(countDegenerated(kernel), 0);
    EXPECT_EQ(kernel.CountFacets(), 0);
}

// Same off-by-one in MeshFixCorruptedFacets, which looks at topologically -- rather than
// geometrically -- degenerated facets, i.e. facets that use the same point index twice.
TEST(MeshFixCorruptedFacets, RemovesAllWhenTheFirstFacetIsCorrupted)
{
    MeshCore::MeshPointArray points;
    for (int i = 0; i < 4; ++i) {
        points.push_back(MeshCore::MeshPoint(Base::Vector3f(static_cast<float>(i), 0.0F, 0.0F)));
    }
    points.push_back(MeshCore::MeshPoint(Base::Vector3f(0.0F, 5.0F, 0.0F)));

    auto makeFacet = [](MeshCore::PointIndex p0, MeshCore::PointIndex p1, MeshCore::PointIndex p2) {
        MeshCore::MeshFacet facet;
        facet._aulPoints[0] = p0;
        facet._aulPoints[1] = p1;
        facet._aulPoints[2] = p2;
        return facet;
    };

    MeshCore::MeshFacetArray facets;
    facets.push_back(makeFacet(0, 0, 4));  // corrupted, at index 0
    facets.push_back(makeFacet(1, 2, 4));  // fine
    facets.push_back(makeFacet(3, 3, 4));  // corrupted

    MeshCore::MeshKernel kernel;
    kernel.Assign(points, facets, true);
    ASSERT_EQ(kernel.CountFacets(), 3);

    MeshCore::MeshFixCorruptedFacets fix(kernel);
    EXPECT_TRUE(fix.Fixup());

    MeshCore::MeshEvalCorruptedFacets eval(kernel);
    EXPECT_TRUE(eval.Evaluate());
    EXPECT_EQ(kernel.CountFacets(), 1);
}

// NOLINTEND(cppcoreguidelines-*,readability-*)
