// SPDX-License-Identifier: LGPL-2.1-or-later

#include <gtest/gtest.h>
#include "PartTestHelpers.h"
#include <algorithm>

#include <BRepBndLib.hxx>
#include <BRepPrimAPI_MakeBox.hxx>
#include <Bnd_Box.hxx>
#include <TopExp_Explorer.hxx>
#include <TopLoc_Location.hxx>

#include <Mod/Part/App/TopoShape.h>
#include "src/App/InitApplication.h"


class TopoShapeTest: public ::testing::Test
{
protected:
    static void SetUpTestSuite()
    {
        tests::initApplication();
    }

    void SetUp() override
    {
        Base::Interpreter().runString("import Part");
        _docName = App::GetApplication().getUniqueDocumentName("test");
        App::GetApplication().newDocument(_docName.c_str(), "testUser");
        _hasher = Base::Reference<App::StringHasher>(new App::StringHasher);
        ASSERT_EQ(_hasher.getRefCount(), 1);
    }

    void TearDown() override
    {
        App::GetApplication().closeDocument(_docName.c_str());
    }


private:
    std::string _docName;
    Data::ElementIDRefs _sid;
    App::StringHasherRef _hasher;
};

// clang-format off
TEST_F(TopoShapeTest, TestElementTypeFace1)
{
    EXPECT_EQ(Part::TopoShape::getElementTypeAndIndex("Face1"),
              std::make_pair(std::string("Face"), 1UL));
}

TEST_F(TopoShapeTest, TestElementTypeEdge12)
{
    EXPECT_EQ(Part::TopoShape::getElementTypeAndIndex("Edge12"),
              std::make_pair(std::string("Edge"), 12UL));
}

TEST_F(TopoShapeTest, TestElementTypeVertex3)
{
    EXPECT_EQ(Part::TopoShape::getElementTypeAndIndex("Vertex3"),
              std::make_pair(std::string("Vertex"), 3UL));
}

TEST_F(TopoShapeTest, TestElementTypeFacer)
{
    EXPECT_EQ(Part::TopoShape::getElementTypeAndIndex("Facer"),
              std::make_pair(std::string(), 0UL));
}

TEST_F(TopoShapeTest, TestElementTypeVertex)
{
    EXPECT_EQ(Part::TopoShape::getElementTypeAndIndex("Vertex"),
              std::make_pair(std::string(), 0UL));
}

TEST_F(TopoShapeTest, TestElementTypeEmpty)
{
    EXPECT_EQ(Part::TopoShape::getElementTypeAndIndex(""),
              std::make_pair(std::string(), 0UL));
}

TEST_F(TopoShapeTest, TestElementTypeNull)
{
    EXPECT_EQ(Part::TopoShape::getElementTypeAndIndex(nullptr),
              std::make_pair(std::string(), 0UL));
}

TEST_F(TopoShapeTest, TestElementTypeWithHash)
{
    EXPECT_EQ(Part::TopoShape::getElementTypeAndIndex(";#7:1;:G0;XTR;:H11a6:8,F.Face3"),
              std::make_pair(std::string("Face"), 3UL));
}

TEST_F(TopoShapeTest, TestElementTypeWithSubelements)
{
    EXPECT_EQ(Part::TopoShape::getElementTypeAndIndex("Part.Body.Pad.Face3"),
              std::make_pair(std::string(), 0UL));
}

TEST_F(TopoShapeTest, TestElementTypeNonMatching)
{
    for (std::array elements = {"Face0", "Face01", "XFace3", "Face3extra"};
         const auto& element : elements) {
        EXPECT_EQ(Part::TopoShape::getElementTypeAndIndex(element),
                  std::make_pair(std::string(), 0UL));
    }
}

TEST_F(TopoShapeTest, TestTypeFace1)
{
    EXPECT_EQ(Part::TopoShape::getTypeAndIndex("Face1"),
              std::make_pair(std::string("Face"), 1UL));
}

TEST_F(TopoShapeTest, TestTypeEdge12)
{
    EXPECT_EQ(Part::TopoShape::getTypeAndIndex("Edge12"),
              std::make_pair(std::string("Edge"), 12UL));
}

TEST_F(TopoShapeTest, TestTypeVertex3)
{
    EXPECT_EQ(Part::TopoShape::getTypeAndIndex("Vertex3"),
              std::make_pair(std::string("Vertex"), 3UL));
}

TEST_F(TopoShapeTest, TestTypeFacer)
{
    EXPECT_EQ(Part::TopoShape::getTypeAndIndex("Facer"),
              std::make_pair(std::string("Facer"), 0UL));
}

TEST_F(TopoShapeTest, TestTypeVertex)
{
    EXPECT_EQ(Part::TopoShape::getTypeAndIndex("Vertex"),
              std::make_pair(std::string("Vertex"), 0UL));
}

TEST_F(TopoShapeTest, TestTypeEmpty)
{
    EXPECT_EQ(Part::TopoShape::getTypeAndIndex(""),
              std::make_pair(std::string(), 0UL));
}

TEST_F(TopoShapeTest, TestTypeNull)
{
    EXPECT_EQ(Part::TopoShape::getTypeAndIndex(nullptr),
              std::make_pair(std::string(), 0UL));
}

TEST_F(TopoShapeTest, TestGetSubshape)
{
    // Arrange
    auto [cube1, cube2] = PartTestHelpers::CreateTwoTopoShapeCubes();
    // Act
    auto face = cube1.getSubShape("Face2");
    auto vertex = cube2.getSubShape(TopAbs_VERTEX,2);
    auto silentFail = cube1.getSubShape("NotThere", true);
    // Assert
    EXPECT_EQ(face.ShapeType(), TopAbs_FACE);
    EXPECT_EQ(vertex.ShapeType(), TopAbs_VERTEX);
    EXPECT_TRUE(silentFail.IsNull());
    EXPECT_THROW(cube1.getSubShape("Face7"), Base::IndexError);          // Out of range
    EXPECT_THROW(cube1.getSubShape("WOOHOO", false), Base::ValueError);  // Invalid
}

namespace
{
// The location of a sub-shape is the composition of its own location with all of its parents'.
// Applying an identity transform must therefore leave the chain alone -- OCCT does not recognise
// a TopLoc_Location built from an identity gp_Trsf as identity, so a naive implementation adds
// another link every single time.
int locationDepth(const TopoDS_Shape& shape)
{
    int depth = 0;
    TopLoc_Location loc = shape.Location();
    while (!loc.IsIdentity() && depth < 100000) {
        ++depth;
        loc = loc.NextLocation();
    }
    return depth;
}

int maxFaceLocationDepth(const TopoDS_Shape& shape)
{
    int worst = 0;
    for (TopExp_Explorer it(shape, TopAbs_FACE); it.More(); it.Next()) {
        worst = std::max(worst, locationDepth(it.Current()));
    }
    return worst;
}
}  // namespace

TEST_F(TopoShapeTest, TestIdentityTransformDoesNotGrowTheLocationChain)
{
    Part::TopoShape shape(BRepPrimAPI_MakeBox(1.0, 2.0, 3.0).Shape());
    ASSERT_EQ(maxFaceLocationDepth(shape.getShape()), 0);

    for (int i = 0; i < 50; ++i) {
        shape.setTransform(Base::Matrix4D());
        shape.transformShape(Base::Matrix4D(), false, true);
    }

    EXPECT_EQ(locationDepth(shape.getShape()), 0);
    EXPECT_EQ(maxFaceLocationDepth(shape.getShape()), 0);
}

TEST_F(TopoShapeTest, TestSetTransformStillApplies)
{
    Part::TopoShape shape(BRepPrimAPI_MakeBox(1.0, 2.0, 3.0).Shape());

    Base::Matrix4D matrix;
    matrix.move(Base::Vector3d(10.0, 20.0, 30.0));
    shape.setTransform(matrix);

    EXPECT_EQ(shape.getTransform(), matrix);
    // replacing, not composing: the chain stays a single link
    EXPECT_EQ(locationDepth(shape.getShape()), 1);

    shape.setTransform(Base::Matrix4D());
    EXPECT_EQ(shape.getTransform(), Base::Matrix4D());
    EXPECT_EQ(locationDepth(shape.getShape()), 0);
}

TEST_F(TopoShapeTest, TestTransformShapeStillApplies)
{
    Part::TopoShape shape(BRepPrimAPI_MakeBox(1.0, 2.0, 3.0).Shape());

    Base::Matrix4D matrix;
    matrix.move(Base::Vector3d(10.0, 0.0, 0.0));
    shape.transformShape(matrix, false, true);

    Bnd_Box bounds;
    BRepBndLib::Add(shape.getShape(), bounds);
    Standard_Real xmin {};
    Standard_Real ymin {};
    Standard_Real zmin {};
    Standard_Real xmax {};
    Standard_Real ymax {};
    Standard_Real zmax {};
    bounds.Get(xmin, ymin, zmin, xmax, ymax, zmax);
    // Bnd_Box pads its result by Precision::Confusion()
    EXPECT_NEAR(xmin, 10.0, 1.0e-6);
    EXPECT_NEAR(xmax, 11.0, 1.0e-6);
}

// clang-format on
