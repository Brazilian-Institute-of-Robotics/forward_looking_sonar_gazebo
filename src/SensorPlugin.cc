#include <functional>
#include <boost/scoped_ptr.hpp>
#include <math.h>
#include "gazebo/rendering/ogre_gazebo.h"
#include <gazebo/gazebo.hh>
#include <gazebo/physics/physics.hh>
#include <gazebo/common/common.hh>
#include <ignition/math/Vector3.hh>
#include <gazebo/rendering/rendering.hh>
#include "gazebo_ros_sonar_plugin/Sonar.hh"
#include <gazebo/sensors/sensors.hh>


namespace gazebo
{
  class SonarVisual : public SensorPlugin
  {
    public: rendering::ScenePtr scene;
    public: boost::shared_ptr<gazebo::rendering::Sonar> sonar;
    public: void Load(sensors::SensorPtr _parent, sdf::ElementPtr _sdf)
    {
      // Store the pointer to the model
      this->sensor = _parent;


      // Listen to the update event. This event is broadcast every
      // simulation iteration.
      // this->updateConnection = event::Events::ConnectPause(
      //     std::bind(&SonarVisual::OnUpdate, this));

      this->updateConnection =  event::Events::ConnectRender(
        std::bind(&SonarVisual::OnUpdate, this));
      this->updatePostRender =  event::Events::ConnectPostRender(
        std::bind(&SonarVisual::OnPostRender, this));


      if (rendering::RenderEngine::Instance()->GetRenderPathType() ==
      rendering::RenderEngine::NONE)
      {
        gzerr << "Unable to create CameraSensor. Rendering is disabled.\n";
        return;
      }

      std::string worldName = this->sensor->WorldName();
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
        this->sonar=boost::shared_ptr<rendering::Sonar>(new rendering::Sonar("FirstSonar",this->scene,false)); 
        this->sonar->SetFarClip(100.0);
        this->sonar->Init();
        this->sonar->Load(_sdf);
        this->sonar->SetCameraCount(1);
        this->sonar->SetRangeCount(100, 100);
        this->sonar->SetHorzFOV(hfov);
        this->sonar->SetCosHorzFOV(hfov);
        this->sonar->SetRayCountRatio(2);
        this->sonar->CreateMyCam();
        this->sonar->CreateLaserTexture("GPUTexture");
      }

    }

    // Called by the world update start event
    public: void OnUpdate()
    {
      this->sonar->RenderImpl();
      // Apply a small linear velocity to the model.
      //this->model->SetLinearVel(ignition::math::Vector3d(.3, 0, 0));
    }

    // Called by the world update start event
    public: void OnPostRender()
    {
      this->sonar->PostRender();
      // gzwarn << rendering::RenderEngine::Instance()->GetSceneCount() << " Num Scenes - " << this->sensor->WorldName() << std::endl;
      //rendering::RenderEngine::Instance()->RemoveScene(const std::string &_name);
      // Apply a small linear velocity to the model.
      //this->model->SetLinearVel(ignition::math::Vector3d(.3, 0, 0));
    }

    // Pointer to the model
    private: sensors::SensorPtr sensor;

    // Pointer to the update event connection
    private: event::ConnectionPtr updateConnection;

    // Pointer to the update event connection
    private: event::ConnectionPtr updatePostRender;
  };

  // Register this plugin with the simulator
  GZ_REGISTER_SENSOR_PLUGIN(SonarVisual)
}