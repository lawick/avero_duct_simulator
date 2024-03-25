#include <stdio.h>
#include <eigen3/Eigen/Eigen>
#include <boost/bind.hpp>

#include <gazebo/common/Plugin.hh>
#include <gazebo/gazebo.hh>
#include <gazebo/physics/physics.hh>

#include <mav_msgs/default_topics.h>  // This comes from the mav_comm repo
#include "rotors_gazebo_plugins/common.h"
#include "rotors_gazebo_plugins/motor_model_duct.hpp"
#include "rotors_gazebo_plugins/motor_model_rotor.hpp"
#include "rotors_gazebo_plugins/motor_model_servo.hpp"
#include "Actuators.pb.h"
#include "ConnectGazeboToRosTopic.pb.h"
#include "ConnectRosToGazeboTopic.pb.h"


namespace gazebo {

static const std::string kDefaultMotorStateTopic = "gazebo/motor_states/";

typedef const boost::shared_ptr<const gz_sensor_msgs::Actuators> GzActuatorsMsgPtr;

class GazeboMultiDuctPlugin : public ModelPlugin { // Inheritate from the Model Pluin 
 public:
  GazeboMultiDuctPlugin()
      : ModelPlugin(),
        received_first_reference_(false),
        pubs_and_subs_created_(false),
        namespace_(kDefaultNamespace),
        motor_state_pub_topic_(kDefaultMotorStateTopic),
        command_actuator_sub_topic_(
            mav_msgs::default_topics::COMMAND_ACTUATORS),
        //---------------
        node_handle_(NULL) {}

  ~GazeboMultiDuctPlugin() {}

  protected:
  void Load(physics::ModelPtr _model, sdf::ElementPtr _sdf); // Wird immer wenn das Plugin aufergufen wird abgerufen!

  void OnUpdate(const common::UpdateInfo& /*_info*/); // Ist die Callback funktion wenn die Simulation läuft und alles aktualisiert wird 
  
 private:
  bool received_first_reference_;
  bool pubs_and_subs_created_;
  void CreatePubsAndSubs();
  void CommandMotorCallback(GzActuatorsMsgPtr& actuators_msg); // Callback für die Motor States -> es werden die neuen Reference gesetzt für die States, da alle von MotorModel inherited sind haben alle die gleichen Variabeln 

  bool IsValidLink(const sdf::ElementPtr motor);
  bool IsValidJoint(const sdf::ElementPtr motor);

  //===== VARIABLES READ FROM SDF FILE =====//
  std::string namespace_;
  std::string motor_state_pub_topic_;
  std::string command_actuator_sub_topic_;

  gazebo::transport::NodePtr node_handle_;
  gazebo::transport::PublisherPtr motor_state_pub_;
  gazebo::transport::SubscriberPtr cmd_motor_sub_;

  physics::ModelPtr model_;
  physics::WorldPtr world_;

  /// \brief Pointer to the update event connection.
  event::ConnectionPtr updateConnection_;

  // Vector of motors
  std::vector<std::unique_ptr<MotorModel>> motors_;
};
} 