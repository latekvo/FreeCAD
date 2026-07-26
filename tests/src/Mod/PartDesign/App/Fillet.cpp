// SPDX-License-Identifier: LGPL-2.1-or-later

#include <gtest/gtest.h>
#include "src/App/InitApplication.h"

#include <string>
#include <vector>

#include <App/Application.h>
#include <App/Document.h>
#include <Base/Interpreter.h>
#include <Mod/PartDesign/App/Body.h>
#include <Mod/PartDesign/App/FeatureFillet.h>
#include <Mod/PartDesign/App/FeaturePrimitive.h>

#include <BRepGProp.hxx>
#include <GProp_GProps.hxx>

// NOLINTBEGIN(readability-magic-numbers,cppcoreguidelines-avoid-magic-numbers)

class FilletTest: public ::testing::Test
{
protected:
    static void SetUpTestSuite()
    {
        tests::initApplication();
        // The document object types are registered by the module initialiser.
        Base::Interpreter().runString("import _PartDesign");
    }

    void SetUp() override
    {
        _doc = App::GetApplication().newDocument("Fillet_test", "testUser");
        _body = _doc->addObject<PartDesign::Body>();
        _box = _doc->addObject<PartDesign::AdditiveBox>();
        _body->addObject(_box);
        _box->Length.setValue(10.0);
        _box->Width.setValue(10.0);
        _box->Height.setValue(10.0);
        _doc->recompute();
    }

    void TearDown() override
    {
        App::GetApplication().closeDocument(_doc->getName());
    }

    // Fillets every edge of the box with the given radius and returns the feature.
    PartDesign::Fillet* filletAllEdges(double radius)
    {
        std::vector<std::string> edges;
        const int count = _box->Shape.getShape().countSubShapes(TopAbs_EDGE);
        edges.reserve(count);
        for (int i = 1; i <= count; ++i) {
            edges.push_back("Edge" + std::to_string(i));
        }

        auto* fillet = _doc->addObject<PartDesign::Fillet>();
        _body->addObject(fillet);
        fillet->Base.setValue(_box, edges);
        fillet->Radius.setValue(radius);
        _doc->recompute();
        return fillet;
    }

    static double volumeOf(const Part::TopoShape& shape)
    {
        GProp_GProps props;
        BRepGProp::VolumeProperties(shape.getShape(), props);
        return props.Mass();
    }

    App::Document* _doc = nullptr;            // NOLINT
    PartDesign::Body* _body = nullptr;        // NOLINT
    PartDesign::AdditiveBox* _box = nullptr;  // NOLINT
};

TEST_F(FilletTest, TestReasonableRadiiSucceed)
{
    for (double radius : {0.01, 0.5, 1.0, 2.0, 4.0, 4.9}) {
        auto* fillet = filletAllEdges(radius);
        ASSERT_FALSE(fillet->isError()) << "radius " << radius << " was rejected";
        ASSERT_FALSE(fillet->Shape.getShape().isNull()) << "radius " << radius;
        EXPECT_TRUE(fillet->Shape.getShape().isValid()) << "radius " << radius;
        // a fillet can only remove material
        EXPECT_LT(volumeOf(fillet->Shape.getShape()), 1000.0 + 1.0e-9) << "radius " << radius;
        _doc->removeObject(fillet->getNameInDocument());
        _doc->recompute();
    }
}

// A radius larger than half the edge length makes the fillet surfaces run into each other.
// BRepFilletAPI_MakeFillet still reports IsDone() and still returns a shape, but the shape is
// self-intersecting and its shell is open. The feature used to store it and report success.
TEST_F(FilletTest, TestOversizedRadiusIsRejected)
{
    for (double radius : {5.001, 6.0}) {
        auto* fillet = filletAllEdges(radius);
        EXPECT_TRUE(fillet->isError())
            << "radius " << radius << " should have been reported as an error";
        if (!fillet->isError() && !fillet->Shape.getShape().isNull()) {
            // if it is ever accepted, it must at least be a sane solid
            EXPECT_TRUE(fillet->Shape.getShape().isValid()) << "radius " << radius;
            EXPECT_LT(volumeOf(fillet->Shape.getShape()), 1000.0 + 1.0e-9) << "radius " << radius;
        }
        _doc->removeObject(fillet->getNameInDocument());
        _doc->recompute();
    }
}

// NOLINTEND(readability-magic-numbers,cppcoreguidelines-avoid-magic-numbers)
