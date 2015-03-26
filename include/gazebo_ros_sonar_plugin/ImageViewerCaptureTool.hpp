#ifndef SIMULATION_NORMAL_DEPTH_MAP_SRC_IMAGECAPTURETOOL_HPP_
#define SIMULATION_NORMAL_DEPTH_MAP_SRC_IMAGECAPTURETOOL_HPP_

#include <osgViewer/Viewer>

namespace normal_depth_map {

/**
 * @brief Capture the osg::Image from a node scene without show the render window
 *
 *  Makes the osg::image from the osg::node scene without to show a GUI.
 */

class WindowCaptureScreen: public osg::Camera::DrawCallback {
public:

    /**
     * @brief This class should only be constructed by ImageViewerCaptureTool.
     *
     * This class allow access raw data image from the viewer, like float value, with call back function;
     *
     *  @param gc: it is a pointer to viewer GraphicsContext
     */
    WindowCaptureScreen(osg::ref_ptr<osg::GraphicsContext> gc);
    ~WindowCaptureScreen();

    /**
     * @brief Gets the osg::image from the call back operator
     *
     * This method gets the osg::image from the call back, and synchronizes the OSG threats before return the image.
     *
     *  @return osg::Image: it is return a image from the scene with defined camera and view parameters.
     */
    osg::ref_ptr<osg::Image> captureImage();
    osg::ref_ptr<osg::Image> getDepthBuffer();

private:

    /**
     * @brief Call back operator to capture the image raw data in float resolution;
     *
     * This operator overrides osg::Camera::DrawCallback to get the image in float resolution;
     */
    void operator ()(osg::RenderInfo& renderInfo) const;

    OpenThreads::Mutex *_mutex;
    OpenThreads::Condition *_condition;
    osg::ref_ptr<osg::Image> _image;
    osg::ref_ptr<osg::Image> _depth_buffer;
};

class ImageViewerCaptureTool {
public:

    /**
     * @brief This class generate a hide viewer to get the osg::image without
     *  GUI.
     *
     *  @param width: Width to generate the image
     *  @param height: height to generate the image
     */
    ImageViewerCaptureTool(uint width = 640, uint height = 480);

    /**
     * @brief This constructor class generate a image according fovy, fovx and
     *  height resolution.
     *
     *  @param fovy: vertical field of view (in radians)
     *  @param fovx: horizontal field of view (in radians)
     *  @param height: height to generate the image
     */

    ImageViewerCaptureTool( double fovY, double fovX, uint value,
                            bool isHeight = true);

    /**
     * @brief This function gets the main node scene and generate a image with
     * float values
     *
     *  @param node: node with the main scene
     */

    osg::ref_ptr<osg::Image> grabImage(osg::ref_ptr<osg::Node> node);

    /**
     * @brief This function gets the image create by depth buffer
     *
     */

    osg::ref_ptr<osg::Image> getDepthBuffer();

    void setCameraPosition( const osg::Vec3d& eye, const osg::Vec3d& center,
                            const osg::Vec3d& up);
    void getCameraPosition(osg::Vec3d& eye, osg::Vec3d& center, osg::Vec3d& up);
    void setBackgroundColor(osg::Vec4d color);

    void setViewMatrix (osg::Matrix matrix)
      { _viewer->getCamera()->setViewMatrix(matrix); };

    osg::Matrix getViewMatrix()
      { return _viewer->getCamera()->getViewMatrix(); };

protected:

    void initializeProperties(uint width, uint height);

    osg::ref_ptr<WindowCaptureScreen> _capture;
    osg::ref_ptr<osgViewer::Viewer> _viewer;
};

} /* namespace normal_depth_map */

#endif /* SIMULATION_NORMAL_DEPTH_MAP_SRC_IMAGECAPTURETOOL_HPP_ */
