// SPDX-License-Identifier: LGPL-2.1-or-later

#include <gtest/gtest.h>
#include "src/App/InitApplication.h"

#include <string>
#include <vector>

#include <App/Application.h>
#include <App/Document.h>
#include <Base/Interpreter.h>
#include <Mod/Part/App/FeaturePartBox.h>
#include <Mod/PartDesign/App/FeatureDraft.h>

// NOLINTBEGIN(readability-magic-numbers,cppcoreguidelines-avoid-magic-numbers)

class DressUpTest: public ::testing::Test
{
protected:
    static void SetUpTestSuite()
    {
        tests::initApplication();
        // The PartDesign document object types are registered by the module initialiser, so the
        // extension module has to be imported before Draft can be added to a document.
        Base::Interpreter().runString("import _PartDesign");
    }

    void SetUp() override
    {
        _doc = App::GetApplication().newDocument("DressUp_test", "testUser");
        _box = _doc->addObject<Part::Box>();
        _box->Length.setValue(10.0);
        _box->Width.setValue(10.0);
        _box->Height.setValue(10.0);
        _draft = _doc->addObject<PartDesign::Draft>();
        _doc->recompute();
    }

    void TearDown() override
    {
        App::GetApplication().closeDocument(_doc->getName());
    }

    Part::Box* getBox() const
    {
        return _box;
    }

    PartDesign::Draft* getDraft() const
    {
        return _draft;
    }

private:
    App::Document* _doc = nullptr;
    Part::Box* _box = nullptr;
    PartDesign::Draft* _draft = nullptr;
};

// DressUp::getFaces() walks getSubValues() but indexed getShadowSubs() with a counter that only
// advanced for entries starting with "Face". Any other entry in the link -- an edge left over
// from a fillet, a wire -- shifted the two lists against each other, so the faces behind it
// resolved somebody else's reference and were silently dropped.
TEST_F(DressUpTest, GetFacesStaysInSyncWithNonFaceEntries)
{
    auto* draft = getDraft();
    const Part::TopoShape shape = getBox()->Shape.getShape();

    const std::vector<std::string> subs {"Edge1", "Face2", "Face4"};
    std::vector<App::PropertyLinkBase::ShadowSub> shadows {
        {"Edge1", "Edge1"},
        {"Face2", "Face2"},
        {"Face4", "Face4"},
    };
    draft->Base.setValue(getBox(), subs, std::move(shadows));

    const std::vector<Part::TopoShape> faces = draft->getFaces(shape);

    ASSERT_EQ(faces.size(), 2);
    EXPECT_TRUE(faces[0].getShape().IsSame(shape.getSubShape("Face2")));
    EXPECT_TRUE(faces[1].getShape().IsSame(shape.getSubShape("Face4")));
}

TEST_F(DressUpTest, GetFacesWorksWithoutNonFaceEntries)
{
    auto* draft = getDraft();
    const Part::TopoShape shape = getBox()->Shape.getShape();

    const std::vector<std::string> subs {"Face1", "Face3"};
    std::vector<App::PropertyLinkBase::ShadowSub> shadows {
        {"Face1", "Face1"},
        {"Face3", "Face3"},
    };
    draft->Base.setValue(getBox(), subs, std::move(shadows));

    const std::vector<Part::TopoShape> faces = draft->getFaces(shape);

    ASSERT_EQ(faces.size(), 2);
    EXPECT_TRUE(faces[0].getShape().IsSame(shape.getSubShape("Face1")));
    EXPECT_TRUE(faces[1].getShape().IsSame(shape.getSubShape("Face3")));
}

// NOLINTEND(readability-magic-numbers,cppcoreguidelines-avoid-magic-numbers)
