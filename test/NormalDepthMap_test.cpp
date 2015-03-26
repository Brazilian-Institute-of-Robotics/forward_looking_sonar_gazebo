// C++ includes
#include <iostream>

// Rock includes
#include <normal_depth_map/ImageViewerCaptureTool.hpp>
#include <normal_depth_map/NormalDepthMap.hpp>
#include "TestHelper.hpp"

// OSG includes
#include <osg/Geode>
#include <osg/Group>
#include <osg/ShapeDrawable>
#include <osgDB/ReadFile>

#define BOOST_TEST_MODULE "NormalDepthMap_test"
#include <boost/test/unit_test.hpp>

using namespace normal_depth_map;
using namespace test_helper;

BOOST_AUTO_TEST_SUITE(test_NormalDepthMap)

// reference points, and map values for each view in viewPointsFromScene1
void referencePointsFromScene(
        std::vector<std::vector<cv::Point> > *setPoints,
        std::vector<std::vector<cv::Point3f> > *setValues) {

    std::vector<cv::Point> points;
    // image points in view1
    points.push_back(cv::Point(74, 417));
    points.push_back(cv::Point(60, 320));
    points.push_back(cv::Point(267, 130));
    points.push_back(cv::Point(366, 226));
    points.push_back(cv::Point(361, 240));
    points.push_back(cv::Point(424, 314));
    setPoints->push_back(points);
    points.clear();

    // image points in view2
    points.push_back(cv::Point(80, 80));
    points.push_back(cv::Point(130, 475));
    points.push_back(cv::Point(390, 128));
    points.push_back(cv::Point(391, 210));
    points.push_back(cv::Point(280, 187));
    setPoints->push_back(points);
    points.clear();

    // image points in view3
    points.push_back(cv::Point(142, 77));
    points.push_back(cv::Point(254, 309));
    points.push_back(cv::Point(434, 65));
    points.push_back(cv::Point(123, 26));
    points.push_back(cv::Point(200, 100));
    setPoints->push_back(points);
    points.clear();

    // image points in view3
    points.push_back(cv::Point(75, 64));
    points.push_back(cv::Point(250, 251));
    points.push_back(cv::Point(410, 459));
    points.push_back(cv::Point(15, 485));
    points.push_back(cv::Point(461, 36));
    setPoints->push_back(points);

    std::vector<cv::Point3f> values;

    // pixel value from each point in image from view1
    values.push_back(cv::Point3f(0.9921, 0.1853, 0));
    values.push_back(cv::Point3f(0.2705, 0.1987, 0));
    values.push_back(cv::Point3f(0.9058, 0.6374, 0));
    values.push_back(cv::Point3f(0.9529, 0.6047, 0));
    values.push_back(cv::Point3f(0.2666, 0.6164, 0));
    values.push_back(cv::Point3f(0.1686, 0.9812, 0));
    setValues->push_back(values);
    values.clear();

    // pixel value from each point in image from view2
    values.push_back(cv::Point3f(0.0000, 0.0000, 0));
    values.push_back(cv::Point3f(0.9098, 0.7698, 0));
    values.push_back(cv::Point3f(1.0000, 0.1942, 0));
    values.push_back(cv::Point3f(0.4313, 0.2047, 0));
    values.push_back(cv::Point3f(0.1490, 0.8185, 0));
    setValues->push_back(values);
    values.clear();

    // pixel value from each point in image from view3
    values.push_back(cv::Point3f(0.8980, 0.4632, 0));
    values.push_back(cv::Point3f(0.8235, 0.5043, 0));
    values.push_back(cv::Point3f(0.2000, 0.6628, 0));
    values.push_back(cv::Point3f(0.0588, 0.6830, 0));
    values.push_back(cv::Point3f(0.6862, 0.4759, 0));
    setValues->push_back(values);
    values.clear();

    // pixel value from each point in image from view4
    values.push_back(cv::Point3f(0.9647, 0.4474, 0));
    values.push_back(cv::Point3f(1.0000, 0.4316, 0));
    values.push_back(cv::Point3f(0.9607, 0.4486, 0));
    values.push_back(cv::Point3f(0.0000, 0.0000, 0));
    values.push_back(cv::Point3f(0.9529, 0.4535, 0));
    setValues->push_back(values);
    values.clear();
}

BOOST_AUTO_TEST_CASE(applyShaderNormalDepthMap_TestCase) {
    std::vector<std::vector<cv::Point> > setPoints;
    std::vector<std::vector<cv::Point3f> > setValues;
    referencePointsFromScene(&setPoints, &setValues);

    // sonar parameters
    float maxRange = 50;
    float fovX = M_PI * 1.0 / 6; // 30 degrees
    float fovY = M_PI * 1.0 / 6; // 30 degrees

    // define the different camera point of views
    std::vector<osg::Vec3d> eyes, centers, ups;
    viewPointsFromDemoScene(&eyes, &centers, &ups);

    // create a simple scene with multiple objects
    osg::ref_ptr<osg::Group> root = new osg::Group();
    makeDemoScene(root);

    // uint precision = 1000;
    for (uint i = 0; i < eyes.size(); i++) {
        // compute and display the final shader and sonar images
        cv::Mat rawShader = computeNormalDepthMap(root, maxRange, fovX, fovY, 0, eyes[i], centers[i], ups[i]);
        cv::Mat rawSonar  = drawSonarImage(rawShader, maxRange, fovX * 0.5);
        cv::imshow("shader image", rawShader);
        cv::imshow("sonar image", rawSonar);
        cv::waitKey();

        // start check process
        cv::Mat normalMap, depthMap;
        cv::extractChannel(rawShader, normalMap, 0);
        cv::extractChannel(rawShader, depthMap,  1);

        for (uint j = 0; j < setPoints[i].size(); ++j) {
            cv::Point p = setPoints[i][j];

            // check normal values
            BOOST_CHECK_CLOSE(normalMap.at<float>(p.y,p.x), setValues[i][j].x, 2);

            // check depth values
            BOOST_CHECK_CLOSE(depthMap.at<float>(p.y,p.x), setValues[i][j].y, 2);
        }
    }
}

void checkDepthValueRadialVariation(cv::Mat3f image, uint id) {
    static const int groudTruth0[] = {
        4941, 4941, 4901, 4901, 4901, 4901, 4901, 4901, 4862, 4862, 4862, 4862,
        4862, 4862, 4862, 4901, 4901, 4901, 4901, 4901, 4901, 4941, 4941};

    static const int groudTruth1[] = {
        9725, 9019, 8313, 7647, 7019, 6431, 5921, 5490, 5137, 4941, 4862,
        4941, 5176, 5490, 5921, 6470, 7058, 7686, 8352, 9058};

    static const int groudTruth2[] = {
        0, 0, 0, 0, 0, 0, 8745, 7333, 6078, 5215, 4862, 5215, 6078, 7333,
        8745, 0, 0, 0, 0, 0, 0};

    std::vector<std::vector<int> > groundTruthVector(3);
    groundTruthVector[0] = std::vector<int>(groudTruth0, groudTruth0 + sizeof(groudTruth0) / sizeof(int));
    groundTruthVector[1] = std::vector<int>(groudTruth1, groudTruth1 + sizeof(groudTruth1) / sizeof(int));
    groundTruthVector[2] = std::vector<int>(groudTruth2, groudTruth2 + sizeof(groudTruth2) / sizeof(int));

    cv::Point centerPoint(image.size().width / 2, image.size().height / 2);
    std::vector<int> histSizes;
    for (int i = 0; i < image.size().width; i = i + image.size().width * 0.05)
        histSizes.push_back((uint)(image[centerPoint.y][i][1] * 10000));

    BOOST_CHECK_EQUAL_COLLECTIONS(  histSizes.begin(),
                                    histSizes.end(),
                                    groundTruthVector[id].begin(),
                                    groundTruthVector[id].end());
}

BOOST_AUTO_TEST_CASE(depthValueRadialVariation_testCase) {
    osg::ref_ptr<osg::Geode> scene = new osg::Geode();
    osg::ref_ptr<osg::Shape> box;
    uint numberSphere = 100;
    double multi = 2;
    double boxSize = 5;
    double distance = 100;
    double maxRange = 200;

    for (uint i = 0; i < numberSphere; ++i) {
        box = new osg::Box(osg::Vec3(i * multi, 0, -distance), boxSize);
        scene->addDrawable(new osg::ShapeDrawable(box));
        box = new osg::Box(osg::Vec3(i * -multi, 0, -distance), boxSize);
        scene->addDrawable(new osg::ShapeDrawable(box));
        box = new osg::Box(osg::Vec3(0, i * -multi, -distance), boxSize);
        scene->addDrawable(new osg::ShapeDrawable(box));
        box = new osg::Box(osg::Vec3(0, i * multi, -distance), boxSize);
        scene->addDrawable(new osg::ShapeDrawable(box));
    }

    NormalDepthMap normalDepthMap(maxRange, M_PI / 6, M_PI / 6);
    normalDepthMap.setDrawNormal(false);
    normalDepthMap.addNodeChild(scene);

    uint sizeVector = 3;
    double fovys[] = {150, 120, 20};
    double fovxs[] = {20, 120, 150};
    uint heightSize[] = {500, 500, 500};

    for (uint j = 0; j < sizeVector; ++j) {
        ImageViewerCaptureTool capture(fovys[j] * M_PI / 180.0, fovxs[j] * M_PI / 180.0, heightSize[j]);
        capture.setBackgroundColor(osg::Vec4d(0, 0, 0, 0));
        osg::ref_ptr<osg::Image> osgImage = capture.grabImage(normalDepthMap.getNormalDepthMapNode());

        cv::Mat3f cvImage(osgImage->t(), osgImage->s());
        cvImage.data = osgImage->data();
        cvImage = cvImage.clone();
        cv::cvtColor(cvImage, cvImage, cv::COLOR_RGB2BGR, CV_32FC3);
        cv::flip(cvImage, cvImage, 0);
        checkDepthValueRadialVariation(cvImage, j);
    }
}

BOOST_AUTO_TEST_SUITE_END();
