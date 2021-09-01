// Copyright 2018 Brazilian Intitute of Robotics"

#include <gtest/gtest.h>
#include "gazebo/rendering/Camera.hh"
#include "gazebo/rendering/RenderingIface.hh"
#include "gazebo/rendering/RenderTypes.hh"
#include "gazebo/rendering/Scene.hh"
#include "gazebo/rendering/ogre_gazebo.h"
#include "gazebo/test/ServerFixture.hh"
#include "gazebo/common/MeshManager.hh"
#include "ignition/math/Vector3.hh"

#include <forward_looking_sonar_gazebo/FLSonar.hh>

// OpenCV includes
#include <opencv2/opencv.hpp>

using namespace gazebo;
class Sonar_TEST: public RenderingFixture
{
protected:

  void SpawnOgreSphere(gazebo::rendering::ScenePtr _scene, ignition::math::Vector3d _pose)
  {
    Ogre::SceneManager *sceneManager= _scene->OgreSceneManager();

    Ogre::SceneNode *rootSceneNode = sceneManager->getRootSceneNode();

    Ogre::SceneNode *sceneNode = rootSceneNode->createChildSceneNode("ballNode");

    // common::MeshManager::Instance()->CreateBox("contact_sphere",
    // ignition::math::Vector3d(1.5,1.5,1),
    // ignition::math::Vector2d(0,0));

    // common::MeshManager::Instance()->CreateSphere("contact_sphere", 1, 100, 100);

    //Add the mesh into OGRE
    if (!sceneNode->getCreator()->hasEntity("unit_sphere") &&
        common::MeshManager::Instance()->HasMesh("unit_sphere"))
    {
      const common::Mesh *mesh =
        common::MeshManager::Instance()->GetMesh("unit_sphere");
      _scene->WorldVisual()->InsertMesh(mesh);
    }

    Ogre::Entity *obj = _scene->OgreSceneManager()->createEntity(
                      "VISUAL__bal", "unit_sphere");

    obj->setMaterialName("Gazebo/Blue");
    obj->setVisibilityFlags(GZ_VISIBILITY_ALL);
    sceneNode->attachObject(obj);
    sceneNode->setVisible(true);
    sceneNode->setPosition(_pose.X(),_pose.Y(),_pose.Z());
  }

  void SpawnOgreCone(gazebo::rendering::ScenePtr _scene, ignition::math::Vector3d _pose)
  {
    Ogre::SceneManager *sceneManager= _scene->OgreSceneManager();

    Ogre::SceneNode *rootSceneNode = sceneManager->getRootSceneNode();

    Ogre::SceneNode *sceneNode = rootSceneNode->createChildSceneNode("coneNode");


    //Add the mesh into OGRE
    if (!sceneNode->getCreator()->hasEntity("unit_cone") &&
        common::MeshManager::Instance()->HasMesh("unit_cone"))
    {
      const common::Mesh *mesh =
        common::MeshManager::Instance()->GetMesh("unit_cone");
      _scene->WorldVisual()->InsertMesh(mesh);
    }

    Ogre::Entity *obj = _scene->OgreSceneManager()->createEntity(
                      "VISUAL__cone", "unit_cone");

    obj->setMaterialName("Gazebo/Blue");
    obj->setVisibilityFlags(GZ_VISIBILITY_ALL);
    sceneNode->attachObject(obj);
    sceneNode->setVisible(true);
    sceneNode->setPosition(_pose.X(),_pose.Y(),_pose.Z());
  }

  void SpawnOgreCube(gazebo::rendering::ScenePtr _scene, ignition::math::Vector3d _pose)
  {
    Ogre::SceneManager *sceneManager= _scene->OgreSceneManager();

    Ogre::SceneNode *rootSceneNode = sceneManager->getRootSceneNode();

    Ogre::SceneNode *sceneNode = rootSceneNode->createChildSceneNode("cubeNode");


    //Add the mesh into OGRE
    if (!sceneNode->getCreator()->hasEntity("unit_cube") &&
        common::MeshManager::Instance()->HasMesh("unit_cube"))
    {
      const common::Mesh *mesh =
        common::MeshManager::Instance()->GetMesh("unit_cube");
      _scene->WorldVisual()->InsertMesh(mesh);
    }

    Ogre::Entity *obj = _scene->OgreSceneManager()->createEntity(
                      "VISUAL__cube", "unit_cube");

    obj->setMaterialName("Gazebo/Blue");
    obj->setVisibilityFlags(GZ_VISIBILITY_ALL);
    sceneNode->attachObject(obj);
    sceneNode->setVisible(true);
    sceneNode->setPosition(_pose.X(),_pose.Y(),_pose.Z());
  }

  void ApplyMask(cv::Mat &_image,cv::Mat &_mask)
  {
    _image.setTo(cv::Vec3f(0,0,0),~_mask);
  }

  void GetCVValuesSphere(cv::Mat &_image,cv::Mat &_mask)
  {
    _image.at<cv::Vec3f>(371,490)[0] = 0.564056;
    _image.at<cv::Vec3f>(371,490)[1] = 0.746238;
    _image.at<cv::Vec3f>(371,490)[2] = 0;
    _mask.at<uchar>(371,490) = 255;

    _image.at<cv::Vec3f>(400,400)[0] = 0.946981;
    _image.at<cv::Vec3f>(400,400)[1] = 0.821607;
    _image.at<cv::Vec3f>(400,400)[2] = 0;
    _mask.at<uchar>(400,400) = 255;

    _image.at<cv::Vec3f>(360,360)[0] = 1;
    _image.at<cv::Vec3f>(360,360)[1] = 0.833263;
    _image.at<cv::Vec3f>(360,360)[2] = 0;
    _mask.at<uchar>(360,360) = 255;

  }

  void GetCVValuesCone(cv::Mat &_image,cv::Mat &_mask)
  {
    _image.at<cv::Vec3f>(371,490)[0] = 0;
    _image.at<cv::Vec3f>(371,490)[1] = 0;
    _image.at<cv::Vec3f>(371,490)[2] = 0;
    _mask.at<uchar>(371,490) = 255;

    _image.at<cv::Vec3f>(400,400)[0] = 0.908717;
    _image.at<cv::Vec3f>(400,400)[1] = 0.836009;
    _image.at<cv::Vec3f>(400,400)[2] = 0;
    _mask.at<uchar>(400,400) = 255;

    _image.at<cv::Vec3f>(360,360)[0] = 1;
    _image.at<cv::Vec3f>(360,360)[1] = 0.833263;
    _image.at<cv::Vec3f>(360,360)[2] = 0;
    _mask.at<uchar>(360,360) = 255;

  }

  bool CompareImages(const cv::Mat &_image1,const cv::Mat &_image2,float _tol)
  {
    for(int i = 0; i<_image1.rows;i++)
    {
      for(int j = 0; j<_image1.cols;j++)
      {
        float diff0 = std::abs( _image1.at<cv::Vec3f>(i,j)[0] - _image2.at<cv::Vec3f>(i,j)[0]);
        float diff1 = std::abs( _image1.at<cv::Vec3f>(i,j)[1] - _image2.at<cv::Vec3f>(i,j)[1]);
        float diff2 = std::abs( _image1.at<cv::Vec3f>(i,j)[2] - _image2.at<cv::Vec3f>(i,j)[2]);

        if( diff0 > _tol || diff1 > _tol || diff2 > _tol){
          gzwarn << _image1.at<cv::Vec3f>(i,j)[0] << " " << _image2.at<cv::Vec3f>(i,j)[0] << std::endl;
          gzwarn << _image1.at<cv::Vec3f>(i,j)[1] << " " << _image2.at<cv::Vec3f>(i,j)[1] << std::endl;
          gzwarn << _image1.at<cv::Vec3f>(i,j)[2] << " " << _image2.at<cv::Vec3f>(i,j)[2] << std::endl;
          return false;}
      }
    }

    return true;
  }
};


/////////////////////////////////////////////////
TEST_F(Sonar_TEST, Create)
{
  std::string programsFolder = std::string(OGRE_MEDIA_PATH) + "/materials/programs";
  gazebo::common::SystemPaths::Instance()->AddGazeboPaths(programsFolder.c_str());

  std::string materialsFolder = std::string(OGRE_MEDIA_PATH) + "/materials/scripts";
  Ogre::ResourceGroupManager::getSingleton().addResourceLocation(
          materialsFolder.c_str(), "FileSystem", "General", true);

  Ogre::ResourceGroupManager::getSingleton().addResourceLocation(
          programsFolder.c_str(), "FileSystem", "General", true);

  Ogre::ResourceGroupManager::getSingleton().initialiseResourceGroup(
          "General");


  Load("worlds/empty.world",false);

  gazebo::rendering::ScenePtr scene = gazebo::rendering::get_scene("default");

  if (!scene)
      scene = gazebo::rendering::create_scene("default", true);

  SetUp();
  ASSERT_TRUE(scene != nullptr);


  std::stringstream newSonarSS;
  newSonarSS <<"<sdf version='1.6'>"
      << "<plugin name='SonarVisual' filename='libfl_sonar_ros.so' >"
      << "<horizontal_fov>1.57079632679</horizontal_fov>"
      << "<vfov>1.3</vfov>"
      << "<bin_count>720</bin_count>"
      << "<beam_count>800</beam_count>"
      << "<image>"
      << "  <width>256</width>"
      << "  <height>512</height>"
      << "  <format>R8G8B8</format>"
      << "</image>"
      << "<clip>"
      << "  <near>0.1</near>"
      << "  <far>0.3</far>"
      << "</clip>"
      << "</plugin>"
      << "</sdf>";

  sdf::ElementPtr FLSonarSDF(new sdf::Element);
  sdf::initFile("plugin.sdf", FLSonarSDF);
  sdf::readString(newSonarSS.str(), FLSonarSDF);

  gzwarn << PROJECT_SOURCE_PATH << std::endl;

  rendering::FLSonar *flSonar = new rendering::FLSonar("test_sonar",scene,false);
  flSonar->Init();
  flSonar->Load(FLSonarSDF);

  ASSERT_EQ(0.3,flSonar->GetFarClip());
  
  ASSERT_EQ(0.1,flSonar->NearClip());

  ASSERT_EQ(256,flSonar->ImageWidth());

  ASSERT_EQ(512,flSonar->ImageHeight());

  ASSERT_EQ(720,flSonar->BinCount());

  ASSERT_EQ(800,flSonar->BeamCount());

  ASSERT_EQ(1.3,flSonar->VertFOV());

}

/////////////////////////////////////////////////
TEST_F(Sonar_TEST, SphereDraw)
{
  std::string programsFolder = std::string(OGRE_MEDIA_PATH) + "/materials/programs";
  gazebo::common::SystemPaths::Instance()->AddGazeboPaths(programsFolder.c_str());

  std::string materialsFolder = std::string(OGRE_MEDIA_PATH) + "/materials/scripts";
  Ogre::ResourceGroupManager::getSingleton().addResourceLocation(
          materialsFolder.c_str(), "FileSystem", "General", true);

  Ogre::ResourceGroupManager::getSingleton().addResourceLocation(
          programsFolder.c_str(), "FileSystem", "General", true);

  Ogre::ResourceGroupManager::getSingleton().initialiseResourceGroup(
          "General");


  Load("worlds/heightmap.world",false);

  gazebo::rendering::ScenePtr scene = gazebo::rendering::get_scene("default");

  if (!scene)
      scene = gazebo::rendering::create_scene("default", true);

  SetUp();
  ASSERT_TRUE(scene != nullptr);


  std::stringstream newSonarSS;
  newSonarSS <<"<sdf version='1.6'>"
      << "<plugin name='SonarVisual' filename='libfl_sonar_ros.so' >"
      << "<horizontal_fov>1.1</horizontal_fov>"
      << "<vfov>0.78539816339</vfov>"
      << "<bin_count>720</bin_count>"
      << "<beam_count>720</beam_count>"
      << "<image>"
      << "  <width>720</width>"
      << "  <height>720</height>"
      << "  <format>R32G32B32</format>"
      << "</image>"
      << "<clip>"
      << "  <near>0.1</near>"
      << "  <far>3</far>"
      << "</clip>"
      << "</plugin>"
      << "</sdf>";

  sdf::ElementPtr FLSonarSDF(new sdf::Element);
  sdf::initFile("plugin.sdf", FLSonarSDF);
  sdf::readString(newSonarSS.str(), FLSonarSDF);

  rendering::FLSonar *flSonar = new rendering::FLSonar("test_sonar",scene,false);
  flSonar->Init();
  flSonar->Load(FLSonarSDF);
  gzwarn << "Image Width " << flSonar->ImageWidth() << std::endl;
  flSonar->CreateTexture("GPUTexture");

  ignition::math::Pose3d sonarPose(0,0,3,0,M_PI/2,M_PI/2);

  // SpawnSphere("ball",
  //   math::Vector3(0,0,0), math::Vector3(0,0,0),
  //   true, true);
  
  SpawnOgreSphere(scene,ignition::math::Vector3d(0,0,1));

  // common::Time::MSleep(3000);


  flSonar->PreRender(sonarPose);
  flSonar->RenderImpl();
  flSonar->GetSonarImage();
  flSonar->PostRender();

  cv::Mat shaderOutput = flSonar->ShaderImage();
  cv::Mat shaderMask = cv::Mat::zeros(shaderOutput.rows,shaderOutput.cols, CV_8UC1);
  cv::Mat shaderRef = cv::Mat::zeros(shaderOutput.rows,shaderOutput.cols, CV_32FC3);

  GetCVValuesSphere(shaderRef,shaderMask);

  ApplyMask(shaderOutput,shaderMask);

  cv::Mat B = cv::Mat::zeros(shaderOutput.rows, shaderOutput.cols, CV_8UC3);
  shaderOutput.convertTo(B,CV_8UC3,255);
  cv::imwrite("teste2.png",B);

  ASSERT_TRUE(CompareImages(shaderOutput,shaderRef,5e-3));
}

/////////////////////////////////////////////////
TEST_F(Sonar_TEST, ConeDraw)
{
  std::string programsFolder = std::string(OGRE_MEDIA_PATH) + "/materials/programs";
  gazebo::common::SystemPaths::Instance()->AddGazeboPaths(programsFolder.c_str());

  std::string materialsFolder = std::string(OGRE_MEDIA_PATH) + "/materials/scripts";
  Ogre::ResourceGroupManager::getSingleton().addResourceLocation(
          materialsFolder.c_str(), "FileSystem", "General", true);

  Ogre::ResourceGroupManager::getSingleton().addResourceLocation(
          programsFolder.c_str(), "FileSystem", "General", true);

  Ogre::ResourceGroupManager::getSingleton().initialiseResourceGroup(
          "General");


  Load("worlds/heightmap.world",false);

  gazebo::rendering::ScenePtr scene = gazebo::rendering::get_scene("default");

  if (!scene)
      scene = gazebo::rendering::create_scene("default", true);

  SetUp();
  ASSERT_TRUE(scene != nullptr);


  std::stringstream newSonarSS;
  newSonarSS <<"<sdf version='1.6'>"
      << "<plugin name='SonarVisual' filename='libfl_sonar_ros.so' >"
      << "<horizontal_fov>1.1</horizontal_fov>"
      << "<vfov>0.78539816339</vfov>"
      << "<bin_count>720</bin_count>"
      << "<beam_count>720</beam_count>"
      << "<image>"
      << "  <width>720</width>"
      << "  <height>720</height>"
      << "  <format>R32G32B32</format>"
      << "</image>"
      << "<clip>"
      << "  <near>0.1</near>"
      << "  <far>3</far>"
      << "</clip>"
      << "</plugin>"
      << "</sdf>";

  sdf::ElementPtr FLSonarSDF(new sdf::Element);
  sdf::initFile("plugin.sdf", FLSonarSDF);
  sdf::readString(newSonarSS.str(), FLSonarSDF);

  rendering::FLSonar *flSonar = new rendering::FLSonar("test_sonar",scene,false);
  flSonar->Init();
  flSonar->Load(FLSonarSDF);
  gzwarn << "Image Width " << flSonar->ImageWidth() << std::endl;
  flSonar->CreateTexture("GPUTexture");

  ignition::math::Pose3d sonarPose(0,0,3,0,M_PI/2,M_PI/2);

  // SpawnSphere("ball",
  //   math::Vector3(0,0,0), math::Vector3(0,0,0),
  //   true, true);
  
  SpawnOgreCone(scene,ignition::math::Vector3d(0,0,1));

  // common::Time::MSleep(3000);


  flSonar->PreRender(sonarPose);
  flSonar->RenderImpl();
  flSonar->GetSonarImage();
  flSonar->PostRender();

  cv::Mat shaderOutput = flSonar->ShaderImage();
  cv::Mat shaderMask = cv::Mat::zeros(shaderOutput.rows,shaderOutput.cols, CV_8UC1);
  cv::Mat shaderRef = cv::Mat::zeros(shaderOutput.rows,shaderOutput.cols, CV_32FC3);

  GetCVValuesCone(shaderRef,shaderMask);

  ApplyMask(shaderOutput,shaderMask);

  cv::Mat B = cv::Mat::zeros(shaderOutput.rows, shaderOutput.cols, CV_8UC3);
  shaderOutput.convertTo(B,CV_8UC3,255);
  cv::imwrite("teste2.png",B);

  ASSERT_TRUE(CompareImages(shaderOutput,shaderRef,5e-3));
}

/////////////////////////////////////////////////
int main(int argc, char **argv)
{
  ::testing::InitGoogleTest(&argc, argv);
  return RUN_ALL_TESTS();
}
