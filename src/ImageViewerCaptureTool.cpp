#include "ImageViewerCaptureTool.hpp"
#include <iostream>
#include <unistd.h>

namespace normal_depth_map {

ImageViewerCaptureTool::ImageViewerCaptureTool(uint width, uint height) {
    // initialize the hide viewer;
    initializeProperties(width, height);
}

ImageViewerCaptureTool::ImageViewerCaptureTool( double fovY, double fovX,
                                                uint value, bool isHeight) {
    uint width, height;

    if (isHeight) {
        height = value;
        width = height * tan(fovX * 0.5) / tan(fovY * 0.5);
    } else {
        width = value;
        height = width * tan(fovY * 0.5) / tan(fovX * 0.5);
    }

    double aspectRatio = width * 1.0 / height;

    initializeProperties(width, height);
    _viewer->getCamera()->setComputeNearFarMode(osg::CullSettings::DO_NOT_COMPUTE_NEAR_FAR);
    _viewer->getCamera()->setProjectionMatrixAsPerspective(fovY * 180.0 / M_PI, aspectRatio, 0.1, 1000);
}

void ImageViewerCaptureTool::initializeProperties(uint width, uint height) {
    _viewer = new osgViewer::Viewer;

    osg::ref_ptr<osg::GraphicsContext::Traits> traits = new osg::GraphicsContext::Traits;
    traits->width = width;
    traits->height = height;
    traits->pbuffer = true;
    traits->readDISPLAY();

    osg::ref_ptr<osg::Camera> camera = this->_viewer->getCamera();
    osg::ref_ptr<osg::GraphicsContext> gfxc = osg::GraphicsContext::createGraphicsContext(traits.get());
    camera->setGraphicsContext(gfxc);
    camera->setDrawBuffer(GL_FRONT);
    camera->setViewport(new osg::Viewport(0, 0, width, height));

    // initialize the class to get the image in float data resolution
    _capture = new WindowCaptureScreen(gfxc);
    camera->setFinalDrawCallback(_capture);
}

osg::ref_ptr<osg::Image> ImageViewerCaptureTool::grabImage(osg::ref_ptr<osg::Node> node) {
    // set the current root node
    _viewer->setSceneData(node);

    // if the view matrix is invalid (NaN), use the identity
    osg::ref_ptr<osg::Camera> camera = _viewer->getCamera();
    if (camera->getViewMatrix().isNaN())
        camera->setViewMatrix(osg::Matrix::identity());

    // grab the current frame
    _viewer->frame();
    return _capture->captureImage();
}

osg::ref_ptr<osg::Image> ImageViewerCaptureTool::getDepthBuffer() {
    return _capture->getDepthBuffer();
}


void ImageViewerCaptureTool::setCameraPosition( const osg::Vec3d& eye,
                                                const osg::Vec3d& center,
                                                const osg::Vec3d& up) {

    _viewer->getCamera()->setViewMatrixAsLookAt(eye, center, up);
}

void ImageViewerCaptureTool::getCameraPosition( osg::Vec3d& eye,
                                                osg::Vec3d& center,
                                                osg::Vec3d& up) {

    _viewer->getCamera()->getViewMatrixAsLookAt(eye, center, up);
}

void ImageViewerCaptureTool::setBackgroundColor(osg::Vec4d color) {
    _viewer->getCamera()->setClearColor(color);
}

////////////////////////////////
////WindowCaptureScreen METHODS
////////////////////////////////

WindowCaptureScreen::WindowCaptureScreen(osg::ref_ptr<osg::GraphicsContext> gc) {
    _mutex = new OpenThreads::Mutex();
    _condition = new OpenThreads::Condition();
    _image = new osg::Image();
    _depth_buffer = new osg::Image();

    // checks the GraficContext from the camera viewer
    if (gc->getTraits()) {
        GLenum pixelFormat;
        if (gc->getTraits()->alpha)
            pixelFormat = GL_RGBA;
        else
            pixelFormat = GL_RGB;

        int width = gc->getTraits()->width;
        int height = gc->getTraits()->height;

        // allocates the image memory space
        _image->allocateImage(width, height, 1, pixelFormat, GL_FLOAT);
        _depth_buffer->allocateImage(width, height, 1,  GL_DEPTH_COMPONENT, GL_FLOAT);
    }
}

WindowCaptureScreen::~WindowCaptureScreen() {
    delete (_condition);
    delete (_mutex);
}

osg::ref_ptr<osg::Image> WindowCaptureScreen::captureImage() {
    //wait to finish the capture image in call back
    _condition->wait(_mutex);
    return _image;
}

osg::ref_ptr<osg::Image> WindowCaptureScreen::getDepthBuffer() {
    return _depth_buffer;
}


void WindowCaptureScreen::operator ()(osg::RenderInfo& renderInfo) const {
    osg::ref_ptr<osg::GraphicsContext> gc = renderInfo.getState()->getGraphicsContext();
    if (gc->getTraits()) {
        _mutex->lock();
        _image->readPixels( 0, 0, _image->s(), _image->t(), _image->getPixelFormat(), GL_FLOAT);
        _depth_buffer->readPixels(0, 0, _image->s(), _image->t(), _depth_buffer->getPixelFormat(), GL_FLOAT);

        //grants the access to image
        _condition->signal();
        _mutex->unlock();
    }
}

}
