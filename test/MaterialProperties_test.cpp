#define BOOST_TEST_MODULE "Material_test"
#include <boost/test/unit_test.hpp>

// OpenSceneGraph includes
#include <osg/Geode>
#include <osg/Group>
#include <osg/ShapeDrawable>

// Rock includes
#include <normal_depth_map/Tools.hpp>
#include "TestHelper.hpp"

using namespace normal_depth_map;
using namespace test_helper;

BOOST_AUTO_TEST_SUITE(MaterialProperties)

// insert a sphere in the scene with desired position, radius and reflectance properties
void addSimpleObject(osg::ref_ptr<osg::Group> root, osg::Vec3 position, float radius, float reflectance) {
    // create the drawable
    osg::ref_ptr<osg::Drawable> drawable = new osg::ShapeDrawable(new osg::Sphere(position, radius));

    // create the stateset and add the uniform
    osg::ref_ptr<osg::StateSet> stateset = new osg::StateSet();
    stateset->addUniform(new osg::Uniform("reflectance", reflectance));

    // add the stateset to the drawable
    drawable->setStateSet(stateset);

    if(!root->getNumChildren()) {
        osg::ref_ptr<osg::Geode> geode = new osg::Geode();
        root->addChild(geode);
    }

    root->getChild(0)->asGeode()->addDrawable(drawable);
}

BOOST_AUTO_TEST_CASE(differentMaterials_testCase) {
    float maxRange = 20.0f;
    float fovX = M_PI / 3;          // 60 degrees
    float fovY = M_PI / 3;          // 60 degrees
    float radius = 5;
    // uint height = 500;
    osg::Vec3 position(0, 0, -14);

    // create the scenes
    osg::ref_ptr<osg::Group> root1 = new osg::Group();
    addSimpleObject(root1, position, radius, 1.0);
    cv::Mat scene1 = computeNormalDepthMap(root1, maxRange, fovX, fovY);

    osg::ref_ptr<osg::Group> root2 = new osg::Group();
    addSimpleObject(root2, position, radius, 0.35);
    cv::Mat scene2 = computeNormalDepthMap(root2, maxRange, fovX, fovY);

    osg::ref_ptr<osg::Group> root3 = new osg::Group();
    addSimpleObject(root3, position, radius, 1.40);
    cv::Mat scene3 = computeNormalDepthMap(root3, maxRange, fovX, fovY);

    osg::ref_ptr<osg::Group> root4 = new osg::Group();
    addSimpleObject(root4, position, radius, 2.12);
    cv::Mat scene4 = computeNormalDepthMap(root4, maxRange, fovX, fovY);

    std::vector<cv::Mat> channels1, channels2, channels3, channels4;
    cv::split(scene1, channels1);
    cv::split(scene2, channels2);
    cv::split(scene3, channels3);
    cv::split(scene4, channels4);

    // assert that normal matrixes are differents
    BOOST_CHECK(areEquals(channels1[0], channels2[0]) == false);
    BOOST_CHECK(areEquals(channels1[0], channels3[0]) == false);
    BOOST_CHECK(areEquals(channels1[0], channels4[0]) == false);
    BOOST_CHECK(areEquals(channels2[0], channels3[0]) == false);
    BOOST_CHECK(areEquals(channels2[0], channels4[0]) == false);
    BOOST_CHECK(areEquals(channels3[0], channels4[0]) == false);

    // output
    cv::Mat output1, output2, output;
    cv::hconcat(scene1, scene2, output1);
    cv::hconcat(scene3, scene4, output2);
    cv::vconcat(output1, output2, output);
    cv::resize(output, output, cv::Size(output.rows / 2, output.cols / 2));

    cv::imshow("singleobject material test", output);
    cv::waitKey();
}

BOOST_AUTO_TEST_CASE(objectsDifferentMaterials_testCase) {
    float maxRange = 20.0f;
    float fovX = M_PI / 3;          // 60 degrees
    float fovY = M_PI / 3;          // 60 degrees

    // create the scene
    osg::ref_ptr<osg::Group> root = new osg::Group();
    addSimpleObject(root, osg::Vec3(-3.0,  2.5, -10), 2, 0.5);
    addSimpleObject(root, osg::Vec3( 0.0, -2.5, -10), 2, 1.0);
    addSimpleObject(root, osg::Vec3( 3.0,  2.5, -10), 2, 1.5);

    // normal depth map
    cv::Mat scene = computeNormalDepthMap(root, maxRange, fovX, fovY);

    // output
    cv::imshow("multiobject material test", scene);
    cv::waitKey();
}

BOOST_AUTO_TEST_SUITE_END()
