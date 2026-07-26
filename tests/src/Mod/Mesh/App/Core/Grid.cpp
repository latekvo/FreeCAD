// SPDX-License-Identifier: LGPL-2.1-or-later

#include <gtest/gtest.h>

#include <vector>

#include <Mod/Mesh/App/Core/Elements.h>
#include <Mod/Mesh/App/Core/Grid.h>
#include <Mod/Mesh/App/Core/MeshKernel.h>

// NOLINTBEGIN(cppcoreguidelines-*,readability-*)

namespace
{

MeshCore::MeshKernel makeUnitSquareMesh()
{
    MeshCore::MeshKernel kernel;
    kernel.AddFacet(
        MeshCore::MeshGeomFacet(
            Base::Vector3f(0.0F, 0.0F, 0.0F),
            Base::Vector3f(10.0F, 0.0F, 0.0F),
            Base::Vector3f(0.0F, 10.0F, 0.0F)
        )
    );
    kernel.AddFacet(
        MeshCore::MeshGeomFacet(
            Base::Vector3f(10.0F, 0.0F, 0.0F),
            Base::Vector3f(10.0F, 10.0F, 0.0F),
            Base::Vector3f(0.0F, 10.0F, 0.0F)
        )
    );
    return kernel;
}

}  // namespace

// CheckPosition() converted (point - min) / cellLength straight to unsigned long. Truncating a
// ratio in (-1, 0) towards zero gives 0, so a point up to one cell below the grid was reported as
// being inside cell 0 and GetElements() handed back that cell's facets instead of none.
TEST(MeshGrid, CheckPositionRejectsPointsBelowTheGrid)
{
    MeshCore::MeshKernel kernel = makeUnitSquareMesh();
    MeshCore::MeshFacetGrid grid(kernel, 2U, 2U, 2U);

    Base::BoundBox3f bounds = grid.GetBoundBox();

    const float cellX = bounds.LengthX() / 2.0F;
    const float cellY = bounds.LengthY() / 2.0F;
    const float cellZ = bounds.LengthZ() / 2.0F;

    unsigned long posX {};
    unsigned long posY {};
    unsigned long posZ {};

    // just inside the lower corner
    EXPECT_TRUE(grid.CheckPosition(
        Base::Vector3f(
            bounds.MinX + (0.1F * cellX),
            bounds.MinY + (0.1F * cellY),
            bounds.MinZ + (0.1F * cellZ)
        ),
        posX,
        posY,
        posZ
    ));

    // less than one cell below the grid on each axis in turn
    for (int axis = 0; axis < 3; ++axis) {
        Base::Vector3f outside(
            bounds.MinX + (0.5F * cellX),
            bounds.MinY + (0.5F * cellY),
            bounds.MinZ + (0.5F * cellZ)
        );
        if (axis == 0) {
            outside.x = bounds.MinX - (0.5F * cellX);
        }
        else if (axis == 1) {
            outside.y = bounds.MinY - (0.5F * cellY);
        }
        else {
            outside.z = bounds.MinZ - (0.5F * cellZ);
        }

        EXPECT_FALSE(grid.CheckPosition(outside, posX, posY, posZ))
            << "point below the grid on axis " << axis << " was accepted";

        std::vector<MeshCore::ElementIndex> facets;
        EXPECT_EQ(grid.GetElements(outside, facets), 0)
            << "facets returned for a point outside the grid on axis " << axis;
    }

    // far below, and far above
    EXPECT_FALSE(grid.CheckPosition(Base::Vector3f(-1000.0F, -1000.0F, -1000.0F), posX, posY, posZ));
    EXPECT_FALSE(grid.CheckPosition(Base::Vector3f(1000.0F, 1000.0F, 1000.0F), posX, posY, posZ));
}

// NOLINTEND(cppcoreguidelines-*,readability-*)
