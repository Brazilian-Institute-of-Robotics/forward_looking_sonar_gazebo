#include <functional>
#include <boost/scoped_ptr.hpp>
#include <math.h>
#include "gazebo/rendering/ogre_gazebo.h"
#include <gazebo/gazebo.hh>
#include <gazebo/physics/physics.hh>
#include <gazebo/common/common.hh>
#include <ignition/math/Vector3.hh>
#include <gazebo/rendering/rendering.hh>
#include <gazebo/sensors/sensors.hh>

#include "gazebo_ros_sonar_plugin/FLSonar.hh"

namespace gazebo
{
  class FLSonarRos : public SensorPlugin
  {

public:
    /**
     * @brief Documentation Iherited
     * 
     * @param _parent 
     * @param _sdf 
     */
    void Load(sensors::SensorPtr _parent, sdf::ElementPtr _sdf);

    /**
     * @brief Called for pre render
     * 
     */
    void OnPreRender();

    /**
     * @brief Called For Render
     * 
     */
    void OnUpdate();

    /**
     * @brief Called for post render
     * 
     */
    void OnPostRender();

public:
    //// \brief Scene parent containing sensor
    rendering::ScenePtr scene;

    //// \brief World that contatins the sensor
    physics::WorldPtr world;

    //// \brief Parent Entity of the sensor
    physics::EntityPtr parent;

    //// \brief Link that the sensor is attached to
    physics::LinkPtr current;

    //// \brief sonar sensor where the link link is attached to
    boost::shared_ptr<gazebo::rendering::FLSonar> sonar;

private:
    // Pointer to the model
    sensors::SensorPtr sensor;

    // Pointer to the update event connection
    event::ConnectionPtr updateConnection;

    // Pointer to the update event connection
    event::ConnectionPtr updatePostRender;
    event::ConnectionPtr updatePreRender;


  };

  // Register this plugin with the simulator
  GZ_REGISTER_SENSOR_PLUGIN(FLSonarRos)
}