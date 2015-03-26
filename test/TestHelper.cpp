#include "TestHelper.hpp"
#include <osg/Geode>
#include <osg/Group>
#include <osg/ShapeDrawable>
#include <osgDB/ReadFile>

#include <normal_depth_map/ImageViewerCaptureTool.hpp>
#include <normal_depth_map/NormalDepthMap.hpp>

using namespace normal_depth_map;

// simple sonar plot
cv::Mat test_helper::drawSonarImage(cv::Mat3f image, double maxRange, double maxAngleX) {
    cv::Mat1b imagePlot = cv::Mat1b::zeros(500, 500);
    cv::Point2f centerPlot(imagePlot.cols / 2, 0);
    double factor = imagePlot.rows / maxRange;
    double slope = 2 * maxAngleX * (1.0 / (image.cols - 1));

    for (int j = 0; j < image.rows; ++j) {
        for (int i = 0; i < image.cols; ++i) {
            double distance = image[j][i][1] * maxRange;
            double alpha = slope * i - maxAngleX;

            cv::Point2f tempPoint(distance * sin(alpha), distance * cos(alpha));
            tempPoint = tempPoint * factor + centerPlot;
            imagePlot[(uint) tempPoint.y][(uint) tempPoint.x] = 255 * image[j][i][0];
        }
    }

    cv::Mat imagePlotMap;
    cv::applyColorMap(imagePlot, imagePlotMap, cv::COLORMAP_HOT);

    cv::line( imagePlotMap, centerPlot, cv::Point2f(maxRange * sin(maxAngleX) * factor,
    maxRange * cos(maxAngleX) * factor) + centerPlot, cv::Scalar(255), 1, CV_AA);

    cv::line( imagePlotMap, centerPlot, cv::Point2f(maxRange * sin(-maxAngleX) * factor,
    maxRange * cos(maxAngleX) * factor) + centerPlot, cv::Scalar(255), 1, CV_AA);
    return imagePlotMap;
}

// compute the normal depth map for a osg scene
cv::Mat test_helper::computeNormalDepthMap(  osg::ref_ptr<osg::Group> root,
                                        float maxRange,
                                        float fovX,
                                        float fovY,
                                        double attenuationCoeff,
                                        osg::Vec3d eye,
                                        osg::Vec3d center,
                                        osg::Vec3d up,
                                        uint height
                                    ) {
    // normal depth map
    NormalDepthMap normalDepthMap(maxRange, fovX * 0.5, fovY * 0.5, attenuationCoeff);
    ImageViewerCaptureTool capture(fovY, fovX, height);
    capture.setBackgroundColor(osg::Vec4d(0, 0, 0, 0));
    capture.setCameraPosition(eye, center, up);
    normalDepthMap.addNodeChild(root);

    // grab scene
    osg::ref_ptr<osg::Image> osgImage = capture.grabImage(normalDepthMap.getNormalDepthMapNode());
    osg::ref_ptr<osg::Image> osgDepth = capture.getDepthBuffer();
    cv::Mat cvImage = cv::Mat(osgImage->t(), osgImage->s(), CV_32FC3, osgImage->data());
    cv::Mat cvDepth = cv::Mat(osgDepth->t(), osgDepth->s(), CV_32FC1, osgDepth->data());
    cvDepth = cvDepth.mul( cv::Mat1f(cvDepth < 1) / 255);

    std::vector<cv::Mat> channels;
    cv::split(cvImage, channels);
    channels[1] = cvDepth;
    cv::merge(channels, cvImage);
    cv::cvtColor(cvImage, cvImage, cv::COLOR_RGB2BGR);
    cv::flip(cvImage, cvImage, 0);

    return cvImage.clone();
}

void test_helper::roundMat(cv::Mat& roi, int precision) {
    if(precision < 0) return;
    for (int x = 0; x < roi.cols; x++) {
        for (int y = 0; y < roi.rows; y++) {
            float value = (float) ((int) (roi.at<float>(x,y) * pow(10, precision)) / pow(10,precision));
            roi.at<float>(x,y) = value;
        }
    }
}

// check if two matrixes are equals
bool test_helper::areEquals (const cv::Mat& image1, const cv::Mat& image2) {
    cv::Mat diff = image1 != image2;
    return (cv::countNonZero(diff) == 0);
}

// draw the scene with a small ball in the center with a big cube, cylinder and cone in back
void test_helper::makeDemoScene(osg::ref_ptr<osg::Group> root) {
    osg::Geode *sphere = new osg::Geode();
    sphere->addDrawable(new osg::ShapeDrawable(new osg::Sphere(osg::Vec3(), 1)));
    root->addChild(sphere);

    osg::Geode *cylinder = new osg::Geode();
    cylinder->addDrawable(new osg::ShapeDrawable(new osg::Cylinder(osg::Vec3(30, 0, 10), 10, 10)));
    root->addChild(cylinder);

    osg::Geode *cone = new osg::Geode();
    cylinder->addDrawable(new osg::ShapeDrawable(new osg::Cone(osg::Vec3(0, 30, 0), 10, 10)));
    root->addChild(cone);

    osg::Geode *box = new osg::Geode();
    cylinder->addDrawable(new osg::ShapeDrawable(new osg::Box(osg::Vec3(0, -30, -10), 10)));
    root->addChild(box);
}

// define different point of views of the same scene
void test_helper::viewPointsFromDemoScene(std::vector<osg::Vec3d> *eyes,
                          std::vector<osg::Vec3d> *centers,
                          std::vector<osg::Vec3d> *ups) {

    // point of view 1 - near from the ball with the cylinder in back
    eyes->push_back(osg::Vec3d(-8.77105, -4.20531, -3.24954));
    centers->push_back(osg::Vec3d(-7.84659, -4.02528, -2.91345));
    ups->push_back(osg::Vec3d(-0.123867, -0.691871, 0.711317));

    // point of view 2 - near from the ball with the cube in back
    eyes->push_back(osg::Vec3d(3.38523, 10.093, 1.12854));
    centers->push_back(osg::Vec3d(3.22816, 9.12808, 0.918259));
    ups->push_back(osg::Vec3d(-0.177264, -0.181915, 0.967204));

    // point of view 3 - near the cone in up side
    eyes->push_back(osg::Vec3d(-10.6743, 38.3461, 26.2601));
    centers->push_back(osg::Vec3d(-10.3734, 38.086, 25.3426));
    ups->push_back(osg::Vec3d(0.370619, -0.854575, 0.36379));

    // point of view 4 - Faced the cube plane
    eyes->push_back(osg::Vec3d(0.0176255, -56.5841, -10.0666));
    centers->push_back(osg::Vec3d(0.0176255, -55.5841, -10.0666));
    ups->push_back(osg::Vec3d(0, 0, 1));
}
