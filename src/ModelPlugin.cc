#include <functional>
#include <boost/scoped_ptr.hpp>
<<<<<<< HEAD
<<<<<<< HEAD
=======
#include "gazebo/rendering/ogre_gazebo.h"
>>>>>>> 3db1d56... Modification of main class to sonar
=======
>>>>>>> 7c8862b... 2
#include <gazebo/gazebo.hh>
#include <gazebo/physics/physics.hh>
#include <gazebo/common/common.hh>
#include <ignition/math/Vector3.hh>
#include <gazebo/rendering/rendering.hh>
#include "gazebo_ros_sonar_plugin/Sonar.hh"

namespace gazebo
{
  class SonarVisual : public ModelPlugin
  {
    public: rendering::ScenePtr scene;
    public: boost::shared_ptr<gazebo::rendering::Sonar> sonar;
    public: void Load(physics::ModelPtr _parent, sdf::ElementPtr _sdf)
    {
      // Store the pointer to the model
      this->model = _parent;

<<<<<<< HEAD
<<<<<<< HEAD
=======

>>>>>>> 3db1d56... Modification of main class to sonar
=======
>>>>>>> 7c8862b... 2
      // Listen to the update event. This event is broadcast every
      // simulation iteration.
      this->updateConnection = event::Events::ConnectPause(
          std::bind(&SonarVisual::OnUpdate, this));


      if (rendering::RenderEngine::Instance()->GetRenderPathType() ==
      rendering::RenderEngine::NONE)
      {
        gzerr << "Unable to create CameraSensor. Rendering is disabled.\n";
        return;
      }

      std::string worldName = this->model->GetWorld()->GetName();
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
        this->sonar=boost::shared_ptr<rendering::Sonar>(new rendering::Sonar("FirstSonar",this->scene,false)); 
        this->sonar->SetFarClip(100.0);
        this->sonar->Init();
        this->sonar->Load(_sdf);
        this->sonar->SetCameraCount(1);
        this->sonar->CreateLaserTexture("GPUTexture");
      }

    }

    // Called by the world update start event
    public: void OnUpdate()
    {
      // Apply a small linear velocity to the model.
      this->model->SetLinearVel(ignition::math::Vector3d(.3, 0, 0));
    }

    // Pointer to the model
    private: physics::ModelPtr model;

    // Pointer to the update event connection
    private: event::ConnectionPtr updateConnection;
  };

  // Register this plugin with the simulator
  GZ_REGISTER_MODEL_PLUGIN(SonarVisual)
}