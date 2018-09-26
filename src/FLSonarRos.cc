#include "gazebo_ros_sonar_plugin/FLSonarRos.hh"

#include <sensor_msgs/Range.h>


namespace gazebo
{

  void FLSonarRos::Load(sensors::SensorPtr _parent, sdf::ElementPtr _sdf)
  {
    // Store the pointer to the model
    this->sensor = _parent;

    this->updateConnection =  event::Events::ConnectRender(
      std::bind(&FLSonarRos::OnUpdate, this));
    this->updatePostRender =  event::Events::ConnectPostRender(
      std::bind(&FLSonarRos::OnPostRender, this));
    this->updatePreRender =  event::Events::ConnectPreRender(
      std::bind(&FLSonarRos::OnPreRender, this));


    if (rendering::RenderEngine::Instance()->GetRenderPathType() ==
    rendering::RenderEngine::NONE)
    {
      gzerr << "Unable to create CameraSensor. Rendering is disabled.\n";
      return;
    }

    std::string worldName = this->sensor->WorldName();

    this->world = physics::get_world("default");
    parent = this->world->GetEntity(this->sensor->ParentName());

    GZ_ASSERT(parent,"This parent does not exisst");

    gzwarn << "link name :" << _sdf->Get<std::string>("link_reference") << std::endl;
    current = parent->GetChildLink(_sdf->Get<std::string>("link_reference"));
    GZ_ASSERT(current,"It must have this link");

    this->scene = rendering::get_scene(worldName);

    // Get scene pointer
    gzwarn << rendering::RenderEngine::Instance()->SceneCount() << " Num Scenes" << std::endl;

    if(!this->scene)
    {
      this->scene = rendering::RenderEngine::Instance()->CreateScene(worldName,false,true);
      gzwarn << "Que budega" << std::endl;
    }

    if(scene != nullptr)
    { 
      
      gzwarn << "Got Scene" << std::endl;
      double hfov = M_PI/2;
      this->sonar=boost::shared_ptr<rendering::FLSonar>(new rendering::FLSonar(this->sensor->Name(),this->scene,false)); 
      this->sonar->SetFarClip(100.0);
      this->sonar->Init();
      this->sonar->Load(_sdf);
      this->sonar->CreateTexture("GPUTexture");
    }

    if (!ros::isInitialized())
    {
      gzerr << "Not loading plugin since ROS has not been "
            << "properly initialized.  Try starting gazebo with ros plugin:\n"
            << "  gazebo -s libgazebo_ros_api_plugin.so\n";
      return;
    }
    this->rosNode.reset(new ros::NodeHandle(""));

    this->sonarImageTransport.reset(new image_transport::ImageTransport(*this->rosNode));

    GZ_ASSERT(std::strcmp(_sdf->Get<std::string>("topic").c_str(),""),"Topic name is not set");

    this->sonarImagePub = this->sonarImageTransport->advertise(_sdf->Get<std::string>("topic"), 1);

    this->bDebug = false;
    if (_sdf->HasElement("debug"))
    {
      this->bDebug = _sdf->Get<bool>("debug");
      if(this->bDebug)
      {
        this->shaderImageTransport.reset(new image_transport::ImageTransport(*this->rosNode));
        this->shaderImagePub = this->shaderImageTransport->advertise(_sdf->Get<std::string>("topic") + "/shader", 1);
      }
    }


  }



  void FLSonarRos::OnPreRender()
  {

    this->sonar->PreRender(current->GetWorldCoGPose());
    this->sonar->GetSonarImage();

  }

  void FLSonarRos::OnUpdate()
  {
    gzwarn << this->sensor->ParentName() << std::endl;
    this->sonar->RenderImpl();

  }


  void FLSonarRos::OnPostRender()
  {
    this->sonar->PostRender();

    // Publish sonar image
    {
      cv::Mat sonarImage = this->sonar->SonarImage();
      cv::Mat sonarMask = this->sonar->SonarMask();

      cv::Mat B = cv::Mat::zeros(sonarImage.rows, sonarImage.cols, CV_8UC1);
      sonarImage.convertTo(B,CV_8UC1,255);

      cv::applyColorMap(B, B, cv::COLORMAP_WINTER);
      B.setTo(0,~sonarMask);

      sensor_msgs::ImagePtr msg = cv_bridge::CvImage(std_msgs::Header(), "bgr8", B).toImageMsg();
      this->sonarImagePub.publish(msg);
    }

    // Publish shader image
    if(this->bDebug)
    {
      cv::Mat shaderImage = this->sonar->ShaderImage();
      cv::Mat B = cv::Mat::zeros(shaderImage.rows, shaderImage.cols, CV_8UC3);
      shaderImage.convertTo(B,CV_8UC3,255);

      sensor_msgs::ImagePtr msg = cv_bridge::CvImage(std_msgs::Header(), "bgr8", B).toImageMsg();
      this->shaderImagePub.publish(msg);
    }

  }
}



  