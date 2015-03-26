#define BOOST_TEST_MODULE "NormalMapping_test"
#include <boost/test/unit_test.hpp>

// OpenSceneGraph includes
#include <osg/Geode>
#include <osg/Group>
#include <osg/Image>
#include <osg/ShapeDrawable>
#include <osg/StateSet>
#include <osg/Texture2D>
#include <osgDB/ReadFile>

// Rock includes
#include <normal_depth_map/NormalDepthMap.hpp>
#include <normal_depth_map/ImageViewerCaptureTool.hpp>
#include "TestHelper.hpp"

// C++ includes
#include <iostream>

using namespace normal_depth_map;
using namespace test_helper;

enum TextureUnitTypes {
    TEXTURE_UNIT_DIFFUSE,
    TEXTURE_UNIT_NORMAL
};

enum TextureImages {
    TEXTURE_CONCRETE,
    TEXTURE_GRAY,
    TEXTURE_ROCKS
};

BOOST_AUTO_TEST_SUITE(NormalMapping)

// add one object to scene (sphere)
void addSimpleObject(osg::ref_ptr<osg::Group> root) {
    osg::ref_ptr<osg::Geode> geode = new osg::Geode();
    geode->addDrawable(new osg::ShapeDrawable(new osg::Sphere(osg::Vec3(0,0,-14), 5)));
    root->addChild(geode);
    root->getChild(0)->asGeode()->addDrawable(geode->getDrawable(0));
}

// add two objects to scene (sphere and box)
void addMultiObject(osg::ref_ptr<osg::Group> root) {
    osg::ref_ptr<osg::Geode> sphere = new osg::Geode();
    sphere->addDrawable(new osg::ShapeDrawable(new osg::Sphere(osg::Vec3(-2, -2, -10), 1.25)));
    root->addChild(sphere);
    root->getChild(0)->asGeode()->addDrawable(sphere->getDrawable(0));

    osg::ref_ptr<osg::Geode> box = new osg::Geode();
    box->addDrawable(new osg::ShapeDrawable(new osg::Box(osg::Vec3(2, 2, -20), 7.5)));
    root->addChild(box);
    root->getChild(1)->asGeode()->addDrawable(box->getDrawable(0));
}

// define texture attributes
osg::ref_ptr<osg::StateSet> insertNormalMapTexture(osg::ref_ptr<osg::Image> diffuseImage, osg::ref_ptr<osg::Image> normalImage) {
    osg::ref_ptr<osg::Texture2D> diffuse = new osg::Texture2D();
    osg::ref_ptr<osg::Texture2D> normal = new osg::Texture2D();

    diffuse->setImage(diffuseImage);
    diffuse->setDataVariance(osg::Object::DYNAMIC);
    diffuse->setFilter(osg::Texture::MIN_FILTER, osg::Texture::LINEAR_MIPMAP_LINEAR);
    diffuse->setFilter(osg::Texture::MAG_FILTER, osg::Texture::LINEAR);
    diffuse->setWrap(osg::Texture::WRAP_S, osg::Texture::REPEAT);
    diffuse->setWrap(osg::Texture::WRAP_T, osg::Texture::REPEAT);
    diffuse->setResizeNonPowerOfTwoHint(false);
    diffuse->setMaxAnisotropy(8.0f);

    normal->setImage(normalImage);
    normal->setDataVariance(osg::Object::DYNAMIC);
    normal->setFilter(osg::Texture::MIN_FILTER, osg::Texture::LINEAR_MIPMAP_LINEAR);
    normal->setFilter(osg::Texture::MAG_FILTER, osg::Texture::LINEAR);
    normal->setWrap(osg::Texture::WRAP_S, osg::Texture::REPEAT);
    normal->setWrap(osg::Texture::WRAP_T, osg::Texture::REPEAT);
    normal->setResizeNonPowerOfTwoHint(false);
    normal->setMaxAnisotropy(8.0f);

    osg::ref_ptr<osg::StateSet> normalState = new osg::StateSet();
    normalState->setTextureAttributeAndModes(TEXTURE_UNIT_DIFFUSE, diffuse, osg::StateAttribute::ON);
    normalState->setTextureAttributeAndModes(TEXTURE_UNIT_NORMAL, normal, osg::StateAttribute::ON);
    return normalState;
}

// get texture files
void loadTextures(osg::ref_ptr<osg::Group> root, TextureImages textureId) {
    std::string current_path(__FILE__);
    current_path = current_path.substr(0, current_path.find_last_of("/"));

    // load texture files
    std::string texture_type;
    switch(textureId) {
        case TEXTURE_CONCRETE:
            texture_type = "concrete_texture";
            break;
        case TEXTURE_GRAY:
            texture_type = "gray_texture";
            break;
        case TEXTURE_ROCKS:
            texture_type = "rocks_texture";
            break;
        default:
            throw std::invalid_argument("Texture image parameter does not match a known enum value");
    }

    osg::ref_ptr<osg::Image> diffuseImage = osgDB::readImageFile(current_path + "/textures/" + texture_type + "_d.png");
    osg::ref_ptr<osg::Image> normalImage = osgDB::readImageFile(current_path + "/textures/" + texture_type + "_n.png");
    BOOST_CHECK( (!diffuseImage || !normalImage) == false );

    // texture properties
    osg::ref_ptr<osg::Geode> geode = new osg::Geode();
    geode->setStateSet(insertNormalMapTexture(diffuseImage, normalImage));
    root->addChild(geode);
}

// create simple scene without texture
osg::ref_ptr<osg::Group> createSimpleScene() {
    osg::ref_ptr<osg::Group> root = new osg::Group();
    addSimpleObject(root);
    return root;
}

// create scene with normal mapping and one object
osg::ref_ptr<osg::Group> createNormalMapSimpleScene() {
    osg::ref_ptr<osg::Group> root = new osg::Group();
    osg::ref_ptr<osg::StateSet> stateset = new osg::StateSet();
    stateset->addUniform(new osg::Uniform("diffuseTexture", TEXTURE_UNIT_DIFFUSE));
    stateset->addUniform(new osg::Uniform("normalTexture", TEXTURE_UNIT_NORMAL));
    stateset->setDataVariance(osg::Object::STATIC);
    root->setStateSet(stateset);

    loadTextures(root, TEXTURE_GRAY);
    addSimpleObject(root);
    return root;
}

// create scene with normal mapping and multiple objects
osg::ref_ptr<osg::Group> createNormalMapMultiScene() {
    osg::ref_ptr<osg::Group> root = new osg::Group();
    osg::ref_ptr<osg::StateSet> stateset = new osg::StateSet();
    stateset->addUniform(new osg::Uniform("diffuseTexture", TEXTURE_UNIT_DIFFUSE));
    stateset->addUniform(new osg::Uniform("normalTexture", TEXTURE_UNIT_NORMAL));
    stateset->setDataVariance(osg::Object::STATIC);
    root->setStateSet(stateset);

    loadTextures(root, TEXTURE_CONCRETE);
    loadTextures(root, TEXTURE_ROCKS);
    addMultiObject(root);
    return root;
}

cv::Mat getNormalGroundTruth() {
    cv::Mat normalGroundTruth = cv::Mat::zeros(cv::Size(5,5), CV_32FC1);

    normalGroundTruth.at<float>(0,0) = 0.78039;
    normalGroundTruth.at<float>(1,0) = 0.85490;
    normalGroundTruth.at<float>(2,0) = 0.85490;
    normalGroundTruth.at<float>(3,0) = 0.89411;
    normalGroundTruth.at<float>(4,0) = 0.92941;
    normalGroundTruth.at<float>(0,1) = 0.87843;
    normalGroundTruth.at<float>(1,1) = 0.92156;
    normalGroundTruth.at<float>(2,1) = 0.92156;
    normalGroundTruth.at<float>(3,1) = 0.93333;
    normalGroundTruth.at<float>(4,1) = 0.92156;
    normalGroundTruth.at<float>(0,2) = 0.96078;
    normalGroundTruth.at<float>(1,2) = 0.97647;
    normalGroundTruth.at<float>(2,2) = 0.96470;
    normalGroundTruth.at<float>(3,2) = 0.88627;
    normalGroundTruth.at<float>(4,2) = 0.89803;
    normalGroundTruth.at<float>(0,3) = 0.98823;
    normalGroundTruth.at<float>(1,3) = 0.98431;
    normalGroundTruth.at<float>(2,3) = 0.93333;
    normalGroundTruth.at<float>(3,3) = 0.86666;
    normalGroundTruth.at<float>(4,3) = 0.90980;
    normalGroundTruth.at<float>(0,4) = 0.98823;
    normalGroundTruth.at<float>(1,4) = 0.96470;
    normalGroundTruth.at<float>(2,4) = 0.87058;
    normalGroundTruth.at<float>(3,4) = 0.83137;
    normalGroundTruth.at<float>(4,4) = 0.95686;

    return normalGroundTruth;
}

BOOST_AUTO_TEST_CASE(differentNormalMaps_TestCase) {
    float maxRange = 20.0f;
    float fovX = M_PI / 3;  // 60 degrees
    float fovY = M_PI / 3;  // 60 degrees

    osg::ref_ptr<osg::Group> rawRoot = createSimpleScene();
    osg::ref_ptr<osg::Group> normalRoot = createNormalMapSimpleScene();

    cv::Mat rawShader    = computeNormalDepthMap(rawRoot, maxRange, fovX, fovY);
    cv::Mat normalShader = computeNormalDepthMap(normalRoot, maxRange, fovX, fovY);

    cv::Mat rawSonar    = drawSonarImage(rawShader, maxRange, fovX * 0.5);
    cv::Mat normalSonar = drawSonarImage(normalShader, maxRange, fovX * 0.5);

    std::vector<cv::Mat> rawChannels, normalChannels;
    cv::split(rawShader, rawChannels);
    cv::split(normalShader, normalChannels);

    // assert that the normal matrixes are different
    BOOST_CHECK(areEquals(rawChannels[0], normalChannels[0]) == false);

    // assert that the depth matrixes are equals
    BOOST_CHECK(areEquals(rawChannels[1], normalChannels[1]) == true);

    // plot sonar sample output
    cv::Mat compShader, compSonar;
    cv::hconcat(rawShader, normalShader, compShader);
    cv::hconcat(rawSonar, normalSonar, compSonar);
    cv::imshow("shader images - single object", compShader);
    cv::imshow("sonar images - single object", compSonar);
    cv::waitKey();
}

BOOST_AUTO_TEST_CASE(multiTextureScene_TestCase) {
    float maxRange = 25.0f;
    float fovX = M_PI / 4;  // 45 degrees
    float fovY = M_PI / 4;  // 45 degrees

    osg::ref_ptr<osg::Group> normalRoot = createNormalMapMultiScene();
    cv::Mat normalShader = computeNormalDepthMap(normalRoot, maxRange, fovX, fovY);
    cv::Mat normalSonar  = drawSonarImage(normalShader, maxRange, fovX * 0.5);

    // plot sonar sample output
    cv::imshow("shader image - multi object", normalShader);
    cv::imshow("sonar image - multi object", normalSonar);
    cv::waitKey();
}

BOOST_AUTO_TEST_CASE(pixelValidation_TestCase) {
    float maxRange = 20.0f;
    float fovX = M_PI / 3;  // 60 degrees
    float fovY = M_PI / 3;  // 60 degrees

    osg::ref_ptr<osg::Group> normalRoot = createNormalMapSimpleScene();
    cv::Mat cvNormal = computeNormalDepthMap(normalRoot, maxRange, fovX, fovY);

    cv::Mat normalRoi;
    cv::extractChannel(cvNormal(cv::Rect(160,175,5,5)), normalRoi, 0);
    roundMat(normalRoi, 5);
    cv::Mat normalGroundTruth = getNormalGroundTruth();

    BOOST_CHECK(areEquals(normalRoi, normalGroundTruth) == true);
}

BOOST_AUTO_TEST_SUITE_END()
