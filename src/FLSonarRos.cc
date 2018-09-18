#include "gazebo_ros_sonar_plugin/FLSonarRos.hh"


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
    gzwarn << rendering::RenderEngine::Instance()->GetSceneCount() << " Num Scenes" << std::endl;

    if(!this->scene)
    {
      this->scene = rendering::RenderEngine::Instance()->CreateScene(worldName,false,true);
      gzwarn << "Que budega" << std::endl;
    }

    if(scene != nullptr)
    { 
      
      gzwarn << "Got Scene" << std::endl;
      double hfov = M_PI/2;
      this->sonar=boost::shared_ptr<rendering::FLSonar>(new rendering::FLSonar(this->sensor->GetName(),this->scene,false)); 
      this->sonar->SetFarClip(100.0);
      this->sonar->Init();
      this->sonar->Load(_sdf);
      this->sonar->CreateTexture("GPUTexture");
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

  }
}



  