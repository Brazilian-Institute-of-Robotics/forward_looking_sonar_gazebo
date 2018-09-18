// MCCR Authors 2018

#include <sstream>

#include <ignition/math/Helpers.hh>
#include <ignition/math/Pose3.hh>
#include <ignition/math/Vector3.hh>

#ifndef _WIN32
  #include <dirent.h>
#else
  // Ensure that Winsock2.h is included before Windows.h, which can get
  // pulled in by anybody (e.g., Boost).
  #include <Winsock2.h>
  #include "gazebo/common/win_dirent.h"
#endif

#include "gazebo/rendering/ogre_gazebo.h"

#include "gazebo/common/Assert.hh"
#include "gazebo/common/Events.hh"
#include "gazebo/common/Console.hh"
#include "gazebo/common/Exception.hh"
#include "gazebo/common/Mesh.hh"
#include "gazebo/common/MeshManager.hh"
#include "gazebo/common/Timer.hh"

#include "gazebo/rendering/Visual.hh"
#include "gazebo/rendering/Conversions.hh"
#include "gazebo/rendering/Scene.hh"
#include "gazebo_ros_sonar_plugin/FLSonar.hh"

using namespace gazebo;
using namespace rendering;


//////////////////////////////////////////////////
FLSonar::FLSonar(const std::string &_namePrefix, ScenePtr _scene,
                   const bool _autoRender) 
  : Camera(_namePrefix, _scene, false),
    imageWidth(0),
    imageHeight(0),
    binCount(0),
    beamCount(0)
{
  
}

//////////////////////////////////////////////////
FLSonar::~FLSonar()
{

  Ogre::TextureManager::getSingleton().remove(
        this->camTexture->getName());

}

//////////////////////////////////////////////////
void FLSonar::Load(sdf::ElementPtr _sdf)
{
  Camera::Load(_sdf);

  GZ_ASSERT(_sdf->Get<double>("vfov"),"Vertical FOV is not set");

  this->SetVertFOV(_sdf->Get<double>("vfov"));

  Ogre::Radian fov_now(this->vfov);
  this->camera->setFOVy( fov_now	); 
  this->camera->setAspectRatio(1.0);
  this->camera->setAutoAspectRatio (0);

  {sdf::ElementPtr iSdf = _sdf->GetElement("clip");
  GZ_ASSERT(iSdf!=nullptr,"clip is not set");
  GZ_ASSERT(iSdf->Get<double>("far"),"far element of clip is not set");	
  this->SetFarClip(iSdf->Get<double>("far"));}

  {sdf::ElementPtr iSdf = _sdf->GetElement("clip");
  GZ_ASSERT(iSdf!=nullptr,"clip is not set");
  GZ_ASSERT(iSdf->Get<double>("near"),"near element of clip is not set");	
  this->SetNearClip(iSdf->Get<double>("near"));}

  {sdf::ElementPtr iSdf = _sdf->GetElement("image");
  GZ_ASSERT(iSdf!=nullptr,"Image is not set");
  GZ_ASSERT(iSdf->Get<double>("width"),"Width element of image is not set");	
  this->SetImageWidth(iSdf->Get<double>("width"));}

  {sdf::ElementPtr iSdf = _sdf->GetElement("image");
  GZ_ASSERT(iSdf!=nullptr,"Image is not set");
  GZ_ASSERT(iSdf->Get<double>("height"),"Height element of image is not set");	
  this->SetImageHeight(iSdf->Get<double>("height"));}

  GZ_ASSERT(_sdf->Get<double>("bin_count"),"bin_count is not set");	
  this->SetBinCount(_sdf->Get<double>("bin_count"));

  GZ_ASSERT(_sdf->Get<double>("beam_count"),"beam_count is not set");	
  this->SetBeamCount(_sdf->Get<double>("beam_count"));

}

//////////////////////////////////////////////////
void FLSonar::Load()
{
  Camera::Load();
}

//////////////////////////////////////////////////
void FLSonar::Init()
{
  Camera::Init();
}

//////////////////////////////////////////////////
void FLSonar::Fini()
{
  Camera::Fini();
}

//////////////////////////////////////////////////
void FLSonar::CreateTexture(const std::string &_textureName)
{
  

  camTexture = Ogre::TextureManager::getSingleton().createManual(
    "RttTex", 
    Ogre::ResourceGroupManager::DEFAULT_RESOURCE_GROUP_NAME, 
    Ogre::TEX_TYPE_2D, 
    this->imageWidth, this->imageHeight, 
    0, 
    Ogre::PF_FLOAT32_RGB  , 
    Ogre::TU_RENDERTARGET).getPointer();
  this->camTarget = camTexture->getBuffer()->getRenderTarget();
  this->camTarget->addViewport(this->camera);
  this->camTarget->getViewport(0)->setClearEveryFrame(true);
  this->camTarget->getViewport(0)->setBackgroundColour(Ogre::ColourValue::Black);
  this->camTarget->getViewport(0)->setOverlaysEnabled(false);
  this->camTarget->getViewport(0)->setShadowsEnabled(false);
  this->camTarget->getViewport(0)->setSkiesEnabled(false);
  this->camTarget->getViewport(0)->setVisibilityMask(GZ_VISIBILITY_ALL  & ~(GZ_VISIBILITY_GUI | GZ_VISIBILITY_SELECTABLE));

  this->camMaterial = (Ogre::Material*)(
    Ogre::MaterialManager::getSingleton().getByName("GazeboRosSonar/NormalDepthMap").get());
  this->camMaterial->load();

  {Ogre::Technique *technique = this->camMaterial->getTechnique(0);
  GZ_ASSERT(technique, "Sonar material script error: technique not found");

  Ogre::Pass *pass = technique->getPass(0);
  GZ_ASSERT(pass, "Sonar material script error: pass not found");
  GZ_ASSERT(pass->hasVertexProgram(),"Must have vertex program");
  GZ_ASSERT(pass->hasFragmentProgram(),"Must have vertex program");}

}

//////////////////////////////////////////////////
void FLSonar::ImageTextureToCV(float _width,int _height, Ogre::Texture* _inTex)
{
  common::Timer firstPassTimer, secondPassTimer;

  firstPassTimer.Start();

  this->rawImage = cv::Mat::zeros(this->imageWidth, this->imageHeight, CV_32FC3);

  _inTex->convertToImage(this->imgSonar);
  this->rawImage.data = this->imgSonar.getData(); 


  double firstPassDur = firstPassTimer.GetElapsed().Double();


  // gzwarn << "Time to blit: "<< firstPassDur << std::endl;


  cv::cvtColor(this->rawImage, this->rawImage, cv::COLOR_RGB2BGR);



  // cv::Mat B = cv::Mat::zeros(this->imageWidth, this->imageHeight, CV_8UC3);
  // this->rawImage.convertTo(B,CV_8UC3,255);
  // cv::imwrite("teste.png",B);

}

//////////////////////////////////////////////////
void FLSonar::PostRender()
{
 
  ignition::math::Pose3d myPose = this->WorldPose();
  gzwarn << "Pose x " << myPose.Pos().X() << "Pose y " << myPose.Pos().Y() << "Pose z " << myPose.Pos().Z() << std::endl;
  gzwarn << " Aspect Ratio: " << this->camera->getAspectRatio() << ", auto aspcet: " << this->camera->getAutoAspectRatio() << std::endl;

  // gzwarn << "--------------------Starte-----------------" << std::endl;
  //this->GetScene()->PrintSceneGraph();
  // gzwarn << "---------------------------ENDED--------------" << std::endl;
}

/////////////////////////////////////////////////
void FLSonar::UpdateRenderTarget(Ogre::RenderTarget *_target,
                   Ogre::Material *_material, Ogre::Camera *_cam,
                   const bool _updateTex)
{
  Ogre::RenderSystem *renderSys;
  Ogre::Viewport *vp = NULL;
  Ogre::SceneManager *sceneMgr = this->scene->OgreSceneManager();
  Ogre::Pass *pass;

  renderSys = this->scene->OgreSceneManager()->getDestinationRenderSystem();
  // Get pointer to the material pass
  pass = _material->getBestTechnique()->getPass(0);

  // Render the depth texture
  _cam->setFarClipDistance(this->FarClip());

  Ogre::AutoParamDataSource autoParamDataSource;

  vp = _target->getViewport(0);

  // Need this line to render the ground plane. No idea why it's necessary.
  sceneMgr->_setPass(pass, true, false);
  autoParamDataSource.setCurrentPass(pass);
  autoParamDataSource.setCurrentViewport(vp);
  autoParamDataSource.setCurrentRenderTarget(_target);
  autoParamDataSource.setCurrentSceneManager(sceneMgr);
  autoParamDataSource.setCurrentCamera(_cam, true);

  renderSys->setLightingEnabled(false);
  renderSys->_setFog(Ogre::FOG_NONE);

#if OGRE_VERSION_MAJOR == 1 && OGRE_VERSION_MINOR == 6
  pass->_updateAutoParamsNoLights(&autoParamDataSource);
#else
  pass->_updateAutoParams(&autoParamDataSource, 1);
#endif

  // NOTE: We MUST bind parameters AFTER updating the autos
  if (pass->hasVertexProgram())
  {
    renderSys->bindGpuProgram(
        pass->getVertexProgram()->_getBindingDelegate());

#if OGRE_VERSION_MAJOR == 1 && OGRE_VERSION_MINOR == 6
    renderSys->bindGpuProgramParameters(Ogre::GPT_VERTEX_PROGRAM,
    pass->getVertexProgramParameters());
#else
    renderSys->bindGpuProgramParameters(Ogre::GPT_VERTEX_PROGRAM,
      pass->getVertexProgramParameters(), 1);
#endif
  }

  if (pass->hasFragmentProgram())
  {
    renderSys->bindGpuProgram(
    pass->getFragmentProgram()->_getBindingDelegate());

#if OGRE_VERSION_MAJOR == 1 && OGRE_VERSION_MINOR == 6
    renderSys->bindGpuProgramParameters(Ogre::GPT_FRAGMENT_PROGRAM,
    pass->getFragmentProgramParameters());
#else
      renderSys->bindGpuProgramParameters(Ogre::GPT_FRAGMENT_PROGRAM,
      pass->getFragmentProgramParameters(), 1);
#endif
  }
}

/////////////////////////////////////////////////
void FLSonar::notifyRenderSingleObject(Ogre::Renderable *_rend,
      const Ogre::Pass* /*pass*/, const Ogre::AutoParamDataSource* /*source*/,
      const Ogre::LightList* /*lights*/, bool /*supp*/)
{
  Ogre::Vector4 retro = Ogre::Vector4(1.0f, 0, 0, 0);
  try
  {
    retro = _rend->getCustomParameter(1);
  }
  catch(Ogre::ItemIdentityException& e)
  {
    _rend->setCustomParameter(1, Ogre::Vector4(1.0f, 0, 0, 0));
  }

  Ogre::Pass *pass = this->camMaterial->getBestTechnique()->getPass(0);

  Ogre::RenderSystem *renderSys =
                  this->scene->OgreSceneManager()->getDestinationRenderSystem();

  Ogre::AutoParamDataSource autoParamDataSource;

  Ogre::Viewport *vp = this->camTarget->getViewport(0);

  renderSys->_setViewport(0);
  renderSys->_setViewport(vp);
  autoParamDataSource.setCurrentRenderable(_rend);
  autoParamDataSource.setCurrentPass(pass);
  autoParamDataSource.setCurrentViewport(vp);
  autoParamDataSource.setCurrentRenderTarget(this->camTarget);
  autoParamDataSource.setCurrentSceneManager(this->scene->OgreSceneManager());
  autoParamDataSource.setCurrentCamera(this->camera, true);

  pass->_updateAutoParams(&autoParamDataSource,
      Ogre::GPV_GLOBAL || Ogre::GPV_PER_OBJECT);

  pass->getFragmentProgramParameters()->setNamedConstant("farPlane", float(this->FarClip()));
  pass->getFragmentProgramParameters()->setNamedConstant("drawNormal", int(1));
  pass->getFragmentProgramParameters()->setNamedConstant("drawDepth", int(1));
  pass->getFragmentProgramParameters()->setNamedConstant("reflectance", retro[0]);
  pass->getFragmentProgramParameters()->setNamedConstant("attenuationCoeff", 0.0f);

  renderSys->bindGpuProgram(
      pass->getVertexProgram()->_getBindingDelegate());

  renderSys->bindGpuProgramParameters(Ogre::GPT_VERTEX_PROGRAM,
      pass->getVertexProgramParameters(),
      Ogre::GPV_GLOBAL || Ogre::GPV_PER_OBJECT);

  renderSys->bindGpuProgram(
      pass->getFragmentProgram()->_getBindingDelegate());

  renderSys->bindGpuProgramParameters(Ogre::GPT_FRAGMENT_PROGRAM,
      pass->getFragmentProgramParameters(),
      Ogre::GPV_GLOBAL || Ogre::GPV_PER_OBJECT);
}

//////////////////////////////////////////////////
void FLSonar::RenderImpl()
{
  common::Timer firstPassTimer, secondPassTimer;

  firstPassTimer.Start();

  Ogre::SceneManager *sceneMgr = this->scene->OgreSceneManager();


  sceneMgr->_suppressRenderStateChanges(true);
  sceneMgr->addRenderObjectListener(this);
  this->UpdateRenderTarget(this->camTarget,
                  this->camMaterial, this->camera,false);
  this->camTarget->update();
  sceneMgr->removeRenderObjectListener(this);
  sceneMgr->_suppressRenderStateChanges(false);

  double firstPassDur = firstPassTimer.GetElapsed().Double();

  gzwarn << firstPassDur << std::endl;

}

//////////////////////////////////////////////////
double FLSonar::GetVertFOV() const
{
  return this->VertFOV();
}

//////////////////////////////////////////////////
double FLSonar::VertFOV() const
{
  return this->vfov;
}

//////////////////////////////////////////////////
void FLSonar::SetVertFOV(const double _vfov)
{
  this->vfov = _vfov;
}


//////////////////////////////////////////////////
double FLSonar::GetNearClip() const
{
  return this->NearClip();
}

//////////////////////////////////////////////////
double FLSonar::NearClip() const
{
  return this->nearClip;
}

//////////////////////////////////////////////////
double FLSonar::GetFarClip() const
{
  return this->FarClip();
}

//////////////////////////////////////////////////
double FLSonar::FarClip() const
{
  return this->farClip;
}

//////////////////////////////////////////////////
void FLSonar::SetNearClip(const double _near)
{
  this->nearClip = _near;
}

//////////////////////////////////////////////////
void FLSonar::SetFarClip(const double _far)
{
  this->farClip = _far;
}

//////////////////////////////////////////////////
int FLSonar::ImageWidth()
{
  return this->imageWidth;
}

//////////////////////////////////////////////////
int FLSonar::ImageHeight()
{
  return this->imageHeight;
}

//////////////////////////////////////////////////
int FLSonar::BinCount()
{
  return this->binCount;
}

//////////////////////////////////////////////////
int FLSonar::BeamCount()
{
  return this->beamCount;
}

//////////////////////////////////////////////////
cv::Mat FLSonar::ShaderImage() const
{
  return this->rawImage;
}

//////////////////////////////////////////////////
void FLSonar::SetImageWidth(const int &_value)
{
  this->imageWidth = _value;
}

//////////////////////////////////////////////////
void FLSonar::SetImageHeight(const int &_value)
{
  this->imageHeight = _value;
}

//////////////////////////////////////////////////
void FLSonar::SetBinCount(const int &_value)
{
  this->binCount = _value;
}

//////////////////////////////////////////////////
void FLSonar::SetBeamCount(const int &_value)
{
  this->beamCount = _value;
}

//////////////////////////////////////////////////
bool FLSonar::SetProjectionType(const std::string &_type)
{
  bool result = true;


  this->camera->setProjectionType(Ogre::PT_PERSPECTIVE);
  this->camera->setCustomProjectionMatrix(false);
  this->scene->SetShadowsEnabled(false);


  return result;
}

//////////////////////////////////////////////////
void FLSonar::PreRender(const math::Pose &_pose)
{
  ignition::math::Pose3d poseIgnition(_pose.pos.x,_pose.pos.y,_pose.pos.z,_pose.rot.w,_pose.rot.x,_pose.rot.y,_pose.rot.z);
  this->SetWorldPose(poseIgnition);
}

//////////////////////////////////////////////////
void FLSonar::GetSonarImage()
{


  int sonarImageWidth = this->imageWidth;
  int sonarImageHeight = this->imageHeight;

  this->sonarImage = cv::Mat::zeros(sonarImageWidth, sonarImageHeight, CV_32F);
  this->sonarImageMask = cv::Mat::zeros(sonarImageWidth, sonarImageHeight, CV_8UC1);


  this->ImageTextureToCV(this->imageWidth,this->imageHeight,this->camTexture);

  // this->DebugPrintImageChannelToFile("TesteBlue.dat", this->rawImage,0);
  // this->DebugPrintImageChannelToFile("TesteGreen.dat", this->rawImage,1);

  std::vector<float> accumData;
  accumData.assign(this->binCount*this->beamCount, 0.0);
  this->CvToSonarBin(accumData);

  // this->DebugPrintMatrixToFile<float>("Teste2.dat", accumData);
  

  std::vector<int> transferTable;
  transferTable.clear();
  this->sonarImage.setTo(0);
  this->sonarImageMask.setTo(0);
  this->GenerateTransferTable(transferTable);

  // this->DebugPrintMatrixToFile<int>("Teste3.dat", transferTable);

  this->TransferTableToSonar(accumData, transferTable);

  cv::Mat B = cv::Mat::zeros(sonarImageWidth, sonarImageHeight, CV_8UC1);
  this->sonarImage.convertTo(B,CV_8UC1,255);

  cv::applyColorMap(B, B, cv::COLORMAP_WINTER);
  //cv::bitwise_and(B,this->sonarImageMask,B) ;
  //B.copyTo(C , this->sonarImageMask );
  B.setTo(0,~this->sonarImageMask);

  cv::imwrite("Sera.png",B);


}

//////////////////////////////////////////////////
void FLSonar::CvToSonarBin(std::vector<float> &_accumData)
{
    
    for (int i_beam = 0; i_beam < this->beamCount-1; i_beam++)
    {

      cv::Mat roi = this->rawImage(cv::Rect(i_beam*ceil(this->imageWidth/this->beamCount),0,ceil(this->imageWidth/this->beamCount),this->rawImage.rows));


      this->sonarBinsDepth.assign( this->binCount, 0 );

      unsigned int width = roi.rows;
      unsigned int height = roi.cols;

      

      // calculate depth histogram
      float* ptr = reinterpret_cast<float*>(roi.data);
      for (int i = 0; i < width * height; i++) {
        int xIndex= i / height;
        int yIndex= (i % height)/height*width;
        int bin_idx = roi.at<cv::Vec3f>(xIndex,yIndex)[1] * (this->binCount - 1);
        this->sonarBinsDepth[bin_idx]++;
      }

      // // calculate bins intesity using normal values, depth histogram and sigmoid function
      this->bins.assign(this->binCount, 0.0);
      for (int i = 0; i < width * height; i++) {
          int xIndex= i / height;
          int yIndex= (i % height)/height*width;
          int bin_idx = roi.at<cv::Vec3f>(xIndex,yIndex)[1] * (this->binCount - 1);
          float intensity = (1.0 / this->sonarBinsDepth[bin_idx]) * this->Sigmoid( roi.at<cv::Vec3f>(xIndex,yIndex)[0]);
          this->bins[bin_idx] += intensity;
      }

      int id_beam = ((-this->vfov/2+ i_beam*this->vfov/this->beamCount + M_PI) / (2*M_PI)) * (this->beamCount-1);

      for (size_t i = 0; i < this->binCount; ++i)
        _accumData[id_beam*this->binCount + i] = this->bins[i];

    }
}

//////////////////////////////////////////////////
void FLSonar::TransferTableToSonar(const std::vector<float> &_accumData, const std::vector<int> &_transfer)
{
    for (size_t i = 0; i < _transfer.size(); ++i) 
  {
    if (_transfer[i] != -1) 
      if(_transfer[i]<_accumData.size())
        this->sonarImage.at<float>(i / this->sonarImage.cols, i % this->sonarImage.cols) = _accumData.at(_transfer[i]);
  }
}

//////////////////////////////////////////////////
void FLSonar::GenerateTransferTable(std::vector<int> &_transfer)
{
  // set the origin
  cv::Point2f origin(this->sonarImage.cols / 2, this->sonarImage.rows / 2);


  for (size_t j = 0; j < this->sonarImage.rows; j++) 
  {
      for (size_t i = 0; i < this->sonarImage.cols; i++) 
      {
          // current point
          cv::Point2f point(i - origin.x, j - origin.y);
          point.x = point.x * (float) this->binCount / (this->sonarImage.cols * 0.5);
          point.y = point.y * (float) this->binCount / (this->sonarImage.rows * 0.5);

          double radius = sqrt(point.x * point.x + point.y * point.y);
          double theta = atan2(-point.x, -point.y);

          // pixels out the sonar image
          if( radius > this->binCount || !radius || theta < -this->vfov/2 || theta > this->vfov/2)
              _transfer.push_back(-1);

          // pixels in the sonar image
          else 
          {
              this->sonarImageMask.at<uchar>(j,i) = 255;
              int idBeam = int(((theta + M_PI) / (2*M_PI)) * (this->beamCount-1));
              _transfer.push_back(idBeam * this->binCount + (int)radius);
          }
      }
  }

}

//////////////////////////////////////////////////
template <typename T>
void FLSonar::DebugPrintMatrixToFile(const std::string &_filename, const std::vector<T> &_matrix)
{
  FILE* imageFile;
  imageFile = fopen(_filename.c_str(),"w");
  for (int i = 0 ; i < this->binCount +1; i++)
  {
      if( i==0 )
      {
        for(int j = 0; j < this->beamCount; j++)
          fprintf(imageFile,"%9u ",j);
      }
      else
      {
        fprintf(imageFile,"%u: ",i-1);
        for(int j = 0; j < this->beamCount; j++)
        {
          if(std::is_same<T, float>::value)
            fprintf(imageFile,"%1.7f ",(float) _matrix[j*this->binCount + i-1]);
          if(std::is_same<T, int>::value)
            fprintf(imageFile,"%9d ",(int) _matrix[j*this->binCount + i-1]);
        }
      }

      fprintf(imageFile,"\n");
  }
  fclose(imageFile);
}

//////////////////////////////////////////////////
void FLSonar::DebugPrintImageToFile(const std::string &_filename, const cv::Mat &_image)
{
    FILE* imageFile;
    imageFile = fopen(_filename.c_str(),"w");
    for (int i = 0 ; i < _image.cols*_image.rows ; i++)
    {
        fprintf(imageFile,"%u : %f %f %f\n",i,_image.at<cv::Vec3f>(i / _image.cols, i % _image.cols)[0],
          _image.at<cv::Vec3f>(i / _image.cols, i % _image.cols)[1],_image.at<cv::Vec3f>(i / _image.cols, i % _image.cols)[2]);
    }
    fclose(imageFile);
}

//////////////////////////////////////////////////
void FLSonar::DebugPrintImageChannelToFile(const std::string &_filename, const cv::Mat &_image, const int &_channel)
{
    
  FILE* imageFile;
  imageFile = fopen(_filename.c_str(),"w");
  for (int i = 0 ; i < _image.rows +1; i++)
  {
      if( i==0 )
      {
        for(int j = 0; j < _image.cols; j++)
          fprintf(imageFile,"%9u ",j);
      }
      else
      {
        fprintf(imageFile,"%u: ",i-1);
        for(int j = 0; j < _image.cols; j++)
          fprintf(imageFile,"%1.7f ",_image.at<cv::Vec3f>(i, j)[_channel]);
      }

      fprintf(imageFile,"\n");
  }

  fclose(imageFile);
}

//////////////////////////////////////////////////
void FLSonar::DebugPrintTexture(Ogre::Texture *_texture)
{
    Ogre::Image img;
    _texture->convertToImage(img);  
    img.save("MyCamtest.png");
}

//////////////////////////////////////////////////
void FLSonar::PixelBoxTextureToCV(Ogre::Texture *_texture,cv::Mat &_image,int _width, int _height)
{
  Ogre::HardwarePixelBufferSharedPtr pixelBuffer;

  pixelBuffer = _texture->getBuffer();
  size_t size = Ogre::PixelUtil::getMemorySize(
                    _width, _height, 1, Ogre::PF_FLOAT32_RGB);


  Ogre::PixelBox dstBox(_width, _height,
    1, Ogre::PF_FLOAT32_RGB , _image.data);

  pixelBuffer->blitToMemory(dstBox);
}

//////////////////////////////////////////////////
float FLSonar::Sigmoid(float x) 
{
    float beta = 18, x0 = 0.666666667;
    float t = (x - x0) * beta;
    return (0.5 * tanh(0.5 * t) + 0.5);
}