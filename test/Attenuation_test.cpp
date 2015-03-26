// C++ includes
#include <iostream>

// Rock includes
#include <normal_depth_map/Tools.hpp>
#include "TestHelper.hpp"

#define BOOST_TEST_MODULE "Attenuation_test"
#include <boost/test/unit_test.hpp>

using namespace normal_depth_map;
using namespace test_helper;

BOOST_AUTO_TEST_SUITE(AcousticAttenuation)

BOOST_AUTO_TEST_CASE(attenuationCalculation_testCase){
    double frequency = 700.0;   // kHz
    double temperature = 20.0;  // celsius degrees
    double depth = 1;           // meters
    double salinity = 35;       // ppt
    double acidity = 8.1;       // pH

    double attenuationCoeff = underwaterSignalAttenuation(frequency, temperature, depth, salinity, acidity);
    BOOST_CHECK_CLOSE(attenuationCoeff, 0.0247, 3);
}

void getReferencePoints(std::vector<cv::Mat>& referencePoints) {
    cv::Mat view1 = cv::Mat::zeros(cv::Size(4,4), CV_32FC1);
    view1.at<float>(0,0) = 0.1019;
    view1.at<float>(0,1) = 0.0666;
    view1.at<float>(0,2) = 0.0745;
    view1.at<float>(0,3) = 0.0823;
    view1.at<float>(1,0) = 0.1098;
    view1.at<float>(1,1) = 0.0627;
    view1.at<float>(1,2) = 0.0705;
    view1.at<float>(1,3) = 0.0745;
    view1.at<float>(2,0) = 0.1176;
    view1.at<float>(2,1) = 0.0588;
    view1.at<float>(2,2) = 0.0627;
    view1.at<float>(2,3) = 0.0705;
    view1.at<float>(3,0) = 0.1176;
    view1.at<float>(3,1) = 0.0549;
    view1.at<float>(3,2) = 0.0588;
    view1.at<float>(3,3) = 0.0666;

    cv::Mat view2 = cv::Mat::zeros(cv::Size(4,4), CV_32FC1);
    view2.at<float>(0,0) = 0.3490;
    view2.at<float>(0,1) = 0.3490;
    view2.at<float>(0,2) = 0.3490;
    view2.at<float>(0,3) = 0.3490;
    view2.at<float>(1,0) = 0.3490;
    view2.at<float>(1,1) = 0.3490;
    view2.at<float>(1,2) = 0.3490;
    view2.at<float>(1,3) = 0.3490;
    view2.at<float>(2,0) = 0.3490;
    view2.at<float>(2,1) = 0.3490;
    view2.at<float>(2,2) = 0.3490;
    view2.at<float>(2,3) = 0.3490;
    view2.at<float>(3,0) = 0.3490;
    view2.at<float>(3,1) = 0.3490;
    view2.at<float>(3,2) = 0.3490;
    view2.at<float>(3,3) = 0.3450;

    cv::Mat view3 = cv::Mat::zeros(cv::Size(4,4), CV_32FC1);
    view3.at<float>(0,0) = 0.3490;
    view3.at<float>(0,1) = 0.3490;
    view3.at<float>(0,2) = 0.3529;
    view3.at<float>(0,3) = 0.3529;
    view3.at<float>(1,0) = 0.3490;
    view3.at<float>(1,1) = 0.3490;
    view3.at<float>(1,2) = 0.3490;
    view3.at<float>(1,3) = 0.3529;
    view3.at<float>(2,0) = 0.3490;
    view3.at<float>(2,1) = 0.3490;
    view3.at<float>(2,2) = 0.3490;
    view3.at<float>(2,3) = 0.3490;
    view3.at<float>(3,0) = 0.3490;
    view3.at<float>(3,1) = 0.3490;
    view3.at<float>(3,2) = 0.3490;
    view3.at<float>(3,3) = 0.3490;

    cv::Mat view4 = cv::Mat::zeros(cv::Size(4,4), CV_32FC1);
    view4.at<float>(0,0) = 0.5529;
    view4.at<float>(0,1) = 0.5529;
    view4.at<float>(0,2) = 0.5529;
    view4.at<float>(0,3) = 0.5529;
    view4.at<float>(1,0) = 0.5529;
    view4.at<float>(1,1) = 0.5529;
    view4.at<float>(1,2) = 0.5529;
    view4.at<float>(1,3) = 0.5529;
    view4.at<float>(2,0) = 0.5529;
    view4.at<float>(2,1) = 0.5529;
    view4.at<float>(2,2) = 0.5529;
    view4.at<float>(2,3) = 0.5529;
    view4.at<float>(3,0) = 0.5529;
    view4.at<float>(3,1) = 0.5529;
    view4.at<float>(3,2) = 0.5529;
    view4.at<float>(3,3) = 0.5529;

    referencePoints.push_back(view1);
    referencePoints.push_back(view2);
    referencePoints.push_back(view3);
    referencePoints.push_back(view4);
}

BOOST_AUTO_TEST_CASE(attenuationDemo_testCase) {
    // sonar parameters
    float maxRange = 50;            // 50 meters
    float fovX = M_PI / 6;    // 30 degrees
    float fovY = M_PI / 6;    // 30 degrees

    // attenuation coefficient
    double frequency = 700.0;       // kHz
    double temperature = 20.0;      // celsius degrees
    double depth = 1;               // meters
    double salinity = 0;            // ppt
    double acidity = 8;             // pH
    double attenuationCoeff = underwaterSignalAttenuation(frequency, temperature, depth, salinity, acidity);

     // define the different camera point of views
    std::vector<osg::Vec3d> eyes, centers, ups;
    viewPointsFromDemoScene(&eyes, &centers, &ups);

    // create a simple scene with multiple objects
    osg::ref_ptr<osg::Group> root = new osg::Group();
    makeDemoScene(root);

    // get reference points in a defined position
    cv::Rect roi(175,400,4,4);
    std::vector<cv::Mat> referencePoints;
    getReferencePoints(referencePoints);

    // display the same scene with and without underwater acoustic attenuation
    for (uint i = 0; i < eyes.size(); ++i) {
        cv::Mat rawShader = computeNormalDepthMap(root, maxRange, fovX, fovY, 0, eyes[i], centers[i], ups[i]);
        cv::Mat rawSonar  = drawSonarImage(rawShader, maxRange, fovX * 0.5);

        cv::Mat attShader = computeNormalDepthMap(root, maxRange, fovX, fovY, attenuationCoeff, eyes[i], centers[i], ups[i]);
        cv::Mat attSonar  = drawSonarImage(attShader, maxRange, fovX * 0.5);

        // check with reference points
        cv::Mat localPoints;
        cv::extractChannel(attShader(roi), localPoints, 0);
        roundMat(localPoints, 4);
        BOOST_CHECK(areEquals(localPoints, referencePoints[i]) == true);

        // output
        cv::Mat compShader, compSonar;
        cv::hconcat(rawShader, attShader, compShader);
        cv::hconcat(rawSonar, attSonar, compSonar);
        cv::imshow("shader images", compShader);
        cv::imshow("sonar images", compSonar);
        cv::waitKey();
    }
}

BOOST_AUTO_TEST_SUITE_END();
