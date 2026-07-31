// SPDX-License-Identifier: LGPL-2.1-or-later

#include <gtest/gtest.h>

#include <algorithm>
#include <cmath>

#include <BRep_Builder.hxx>
#include <BRep_TEdge.hxx>
#include <BRep_TFace.hxx>
#include <BRepBndLib.hxx>
#include <BRepCheck_Analyzer.hxx>
#include <BRepGProp.hxx>
#include <BRepPrimAPI_MakeBox.hxx>
#include <Bnd_Box.hxx>
#include <GProp_GProps.hxx>
#include <TopExp_Explorer.hxx>
#include <TopLoc_Location.hxx>
#include <TopoDS.hxx>
#include <TopoDS_Compound.hxx>
#include <TopoDS_Iterator.hxx>
#include <gp_Ax1.hxx>
#include <gp_Dir.hxx>
#include <gp_Pnt.hxx>
#include <gp_Trsf.hxx>
#include <gp_Vec.hxx>

#include <Mod/Part/App/TopoShape.h>

namespace
{

int chainDepth(const TopLoc_Location& location)
{
    int links = 0;
    for (TopLoc_Location item = location; !item.IsIdentity(); item = item.NextLocation()) {
        ++links;
    }
    return links;
}

/// The deepest location chain any sub-shape reference carries.
int deepestChain(const TopoDS_Shape& shape)
{
    int deepest = chainDepth(shape.Location());
    for (TopoDS_Iterator it(shape, Standard_False, Standard_False); it.More(); it.Next()) {
        deepest = std::max(deepest, deepestChain(it.Value()));
    }
    return deepest;
}

bool sameTransformation(const TopLoc_Location& one, const TopLoc_Location& other)
{
    const gp_Trsf& a = one.Transformation();
    const gp_Trsf& b = other.Transformation();
    for (int row = 1; row <= 3; ++row) {
        for (int col = 1; col <= 4; ++col) {
            if (std::abs(a.Value(row, col) - b.Value(row, col)) > 1e-9) {
                return false;
            }
        }
    }
    return true;
}

double volumeOf(const TopoDS_Shape& shape)
{
    GProp_GProps props;
    BRepGProp::VolumeProperties(shape, props);
    return props.Mass();
}

/// A location that transforms exactly like `real` but carries `padding` identity links, the way
/// a document accumulates them when a feature composes an identity placement onto its shape.
TopLoc_Location paddedLocation(const gp_Trsf& real, int padding)
{
    TopLoc_Location location(real);
    for (int i = 0; i < padding; ++i) {
        location = location * TopLoc_Location(gp_Trsf());
    }
    return location;
}

TopoDS_Shape inCompound(const TopoDS_Shape& shape)
{
    TopoDS_Compound compound;
    BRep_Builder builder;
    builder.MakeCompound(compound);
    builder.Add(compound, shape);
    return compound;
}

}  // namespace

TEST(StripIdentityLocations, DropsIdentityLinksAndKeepsTheGeometry)
{
    gp_Trsf move;
    move.SetTranslation(gp_Vec(1.0, 2.0, 3.0));
    TopoDS_Shape box = BRepPrimAPI_MakeBox(10.0, 20.0, 30.0).Shape();
    TopoDS_Shape placed = inCompound(box.Moved(paddedLocation(move, 10)));

    ASSERT_EQ(deepestChain(placed), 11);
    Bnd_Box before;
    BRepBndLib::Add(placed, before, Standard_False);

    TopoDS_Shape stripped = Part::stripIdentityLocations(placed);

    EXPECT_EQ(deepestChain(stripped), 1);
    EXPECT_DOUBLE_EQ(volumeOf(stripped), volumeOf(placed));

    Bnd_Box after;
    BRepBndLib::Add(stripped, after, Standard_False);
    EXPECT_FALSE(after.IsVoid());
    EXPECT_NEAR(after.SquareExtent(), before.SquareExtent(), 1e-9);
    double x1 {};
    double y1 {};
    double z1 {};
    double x2 {};
    double y2 {};
    double z2 {};
    double u1 {};
    double v1 {};
    double w1 {};
    double u2 {};
    double v2 {};
    double w2 {};
    before.Get(x1, y1, z1, x2, y2, z2);
    after.Get(u1, v1, w1, u2, v2, w2);
    EXPECT_NEAR(u1, x1, 1e-9);
    EXPECT_NEAR(v1, y1, 1e-9);
    EXPECT_NEAR(w1, z1, 1e-9);
    EXPECT_NEAR(u2, x2, 1e-9);
    EXPECT_NEAR(v2, y2, 1e-9);
    EXPECT_NEAR(w2, z2, 1e-9);
}

TEST(StripIdentityLocations, KeepsTheSubShapeCountAndTheirPlaces)
{
    gp_Trsf move;
    move.SetRotation(gp_Ax1(gp_Pnt(), gp_Dir(0.0, 0.0, 1.0)), 0.7);
    move.SetTranslationPart(gp_Vec(5.0, 0.0, 0.0));
    TopoDS_Shape box = BRepPrimAPI_MakeBox(10.0, 20.0, 30.0).Shape();
    TopoDS_Shape placed = inCompound(box.Moved(paddedLocation(move, 6)));

    TopoDS_Shape stripped = Part::stripIdentityLocations(placed);

    for (auto type : {TopAbs_SOLID, TopAbs_FACE, TopAbs_EDGE, TopAbs_VERTEX}) {
        TopExp_Explorer was(placed, type);
        TopExp_Explorer now(stripped, type);
        for (; was.More() && now.More(); was.Next(), now.Next()) {
            // Same order, same place: only the route the location took there is shorter.
            EXPECT_TRUE(sameTransformation(was.Current().Location(), now.Current().Location()));
            EXPECT_EQ(was.Current().Orientation(), now.Current().Orientation());
        }
        EXPECT_EQ(was.More(), now.More());
    }
}

TEST(StripIdentityLocations, ReducesTheLocationsTheGeometryCarriesToo)
{
    // OCCT pairs a pcurve with its face by comparing location chains item by item, so a face
    // whose own location keeps identity links has to be reduced along with the topology. Left
    // behind, the pcurve is no longer found and the face reports no bounds at all.
    TopoDS_Shape box = BRepPrimAPI_MakeBox(10.0, 20.0, 30.0).Shape();
    for (TopExp_Explorer it(box, TopAbs_FACE); it.More(); it.Next()) {
        auto face = Handle(BRep_TFace)::DownCast(it.Current().TShape());
        face->Location(paddedLocation(face->Location().Transformation(), 8));
    }
    TopoDS_Shape placed = inCompound(box);
    Bnd_Box before;
    BRepBndLib::Add(placed, before, Standard_False);
    ASSERT_FALSE(before.IsVoid());
    ASSERT_LT(before.SquareExtent(), 1e6);

    TopoDS_Shape stripped = Part::stripIdentityLocations(placed);

    Bnd_Box after;
    BRepBndLib::Add(stripped, after, Standard_False);
    EXPECT_FALSE(after.IsVoid());
    EXPECT_NEAR(after.SquareExtent(), before.SquareExtent(), 1e-9);
    EXPECT_TRUE(BRepCheck_Analyzer(stripped).IsValid());

    for (TopExp_Explorer it(stripped, TopAbs_FACE); it.More(); it.Next()) {
        auto face = Handle(BRep_TFace)::DownCast(it.Current().TShape());
        EXPECT_EQ(chainDepth(face->Location()), 0);
    }
}

TEST(StripIdentityLocations, LeavesAShapeWithNothingToDropAlone)
{
    gp_Trsf move;
    move.SetTranslation(gp_Vec(1.0, 2.0, 3.0));
    TopoDS_Shape placed = inCompound(BRepPrimAPI_MakeBox(10.0, 20.0, 30.0).Shape().Moved(
        TopLoc_Location(move)
    ));

    TopoDS_Shape stripped = Part::stripIdentityLocations(placed);

    // Not merely equivalent: the very same shape, so a caller that compares shapes to decide
    // whether anything changed is not told that something did.
    EXPECT_TRUE(stripped.IsEqual(placed));
}

TEST(StripIdentityLocations, AcceptsANullShape)
{
    TopoDS_Shape nothing;
    EXPECT_TRUE(Part::stripIdentityLocations(nothing).IsNull());
}

TEST(IdentityTransform, RecognisesWhatFormDoesNot)
{
    EXPECT_TRUE(Part::isIdentityTransform(gp_Trsf()));

    // gp_Trsf::Form() answers gp_Rotation for this one whatever the angle is.
    gp_Trsf turnedByNothing;
    turnedByNothing.SetRotation(gp_Ax1(gp_Pnt(), gp_Dir(0.0, 0.0, 1.0)), 0.0);
    EXPECT_NE(turnedByNothing.Form(), gp_Identity);
    EXPECT_TRUE(Part::isIdentityTransform(turnedByNothing));

    gp_Trsf shifted;
    shifted.SetTranslation(gp_Vec(0.0, 0.0, 1e-9));
    EXPECT_FALSE(Part::isIdentityTransform(shifted));

    gp_Trsf turned;
    turned.SetRotation(gp_Ax1(gp_Pnt(), gp_Dir(0.0, 0.0, 1.0)), 1e-9);
    EXPECT_FALSE(Part::isIdentityTransform(turned));
}

TEST(IdentityTransform, MovingByItLeavesTheLocationChainAlone)
{
    TopoDS_Shape box = BRepPrimAPI_MakeBox(10.0, 20.0, 30.0).Shape();
    gp_Trsf move;
    move.SetTranslation(gp_Vec(1.0, 2.0, 3.0));
    Part::TopoShape::move(box, move);
    ASSERT_EQ(chainDepth(box.Location()), 1);

    gp_Trsf turnedByNothing;
    turnedByNothing.SetRotation(gp_Ax1(gp_Pnt(), gp_Dir(0.0, 0.0, 1.0)), 0.0);
    for (int i = 0; i < 20; ++i) {
        Part::TopoShape::move(box, turnedByNothing);
    }

    // Twenty transformations that move nothing must leave nothing behind: OCCT cancels two chain
    // items only when they are the same datum object, so anything added here would stay forever.
    EXPECT_EQ(chainDepth(box.Location()), 1);
    EXPECT_TRUE(sameTransformation(box.Location(), TopLoc_Location(move)));
}

TEST(IdentityTransform, MovingByARealTransformStillMoves)
{
    TopoDS_Shape box = BRepPrimAPI_MakeBox(10.0, 20.0, 30.0).Shape();
    gp_Trsf move;
    move.SetTranslation(gp_Vec(1.0, 2.0, 3.0));
    Part::TopoShape::move(box, move);
    Part::TopoShape::move(box, move);

    EXPECT_EQ(chainDepth(box.Location()), 2);
    gp_Trsf twice;
    twice.SetTranslation(gp_Vec(2.0, 4.0, 6.0));
    EXPECT_TRUE(sameTransformation(box.Location(), TopLoc_Location(twice)));
}
