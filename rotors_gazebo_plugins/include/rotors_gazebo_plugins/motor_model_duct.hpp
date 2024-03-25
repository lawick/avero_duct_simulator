// 3RD PARTY
#include <Eigen/Core>
#include <boost/bind.hpp>
#include <gazebo/physics/physics.hh>

// USER
#include "rotors_gazebo_plugins/common.h"
#include "rotors_gazebo_plugins/motor_model.hpp"

namespace gazebo{
  // Default values from KDE885 performance data chart
// static constexpr double kDefaultMotorTorqueConst0 = 0.00218;
// static constexpr double kDefaultMotorTorqueConst1 = 0.00318;
// static constexpr double kDefaultMotorTorqueConst2 = 0.00047;

class MotorModelDuct : public MotorModel{
    public: 
    MotorModelDuct(const physics::ModelPtr _model, const sdf::ElementPtr _motor)
        : MotorModel(),
            turning_direction_(spin::CCW),
            time_constant_up_(kDefaultTimeConstantUp),
            time_constant_down_(kDefaultTimeConstantDown),
            max_rot_velocity_(kDefaultMaxRotVelocity),
            min_rot_velocity_(kDefaultMinRotVelocity),
            thrust_constant_(kDefaultThrustConstant),
            moment_constant_(kDefaultMomentConstant),
            // motor_torque_constant0_(kDefaultMotorTorqueConst0),
            // motor_torque_constant1_(kDefaultMotorTorqueConst1),
            // motor_torque_constant2_(kDefaultMotorTorqueConst2),
            rotor_drag_coefficient_(kDefaultRotorDragCoefficient),
            rolling_moment_coefficient_(kDefaultRollingMomentCoefficient),
            rotor_velocity_slowdown_sim_(kDefaultRotorVelocitySlowdownSim) {
        motor_ = _motor;
        joint_ = _model->GetJoint(motor_->GetElement("jointName")->Get<std::string>());
        link_ = _model->GetLink(motor_->GetElement("linkName")->Get<std::string>());
        link_dynamixel_base_name = motor_->GetElement("linkBase")->Get<std::string>();
        joint_dynamixel_base_ = _model->GetJoint(motor_->GetElement("linkBase")->Get<std::string>());
        joint_dynamixel_top_ = _model->GetJoint(motor_->GetElement("linkTop")->Get<std::string>());
        InitializeParams();
    }

    virtual ~MotorModelDuct(){}

    protected:
      // Parameters
    std::string joint_name_;
    std::string link_name_;
    int turning_direction_;
    double time_constant_up_;
    double time_constant_down_;
    double max_rot_velocity_;
    double min_rot_velocity_;
    double thrust_constant_;
    double moment_constant_;
    // Motor torque to thrust relation (2nd order polynomial)
    double motor_torque_constant0_;
    double motor_torque_constant1_;
    double motor_torque_constant2_;
    double rotor_drag_coefficient_;
    double rolling_moment_coefficient_;
    double rotor_velocity_slowdown_sim_;

    std::unique_ptr<FirstOrderFilter<double>> rotor_velocity_filter_;

    sdf::ElementPtr motor_;
    physics::JointPtr joint_;
    physics::LinkPtr link_;
    physics::JointPtr joint_dynamixel_base_; 
    physics::JointPtr joint_dynamixel_top_; 
    std::string link_dynamixel_base_name; 
    physics::LinkPtr link_dynamixel_top_;

    void InitializeParams(){
    //std::cout << "InizializeParams not Implemented!";
    
    if (motor_->HasElement("spinDirection")) {
      std::string turning_direction = motor_->GetElement("spinDirection")->Get<std::string>();
      if (turning_direction == "cw")
        turning_direction_ = spin::CW;
      else if (turning_direction == "ccw")
        turning_direction = spin::CCW;
      else
        gzerr << "[motor_model_rotor] Spin not valid, using 'ccw.'\n";
    } else {
      gzwarn << "[motor_model_rotor] spinDirection not specified, using ccw.\n";
    }
    getSdfParam<double>(motor_, "rotorDragCoefficient", rotor_drag_coefficient_,
                        rotor_drag_coefficient_);
    getSdfParam<double>(motor_, "rollingMomentCoefficient", rolling_moment_coefficient_,
                        rolling_moment_coefficient_);
    getSdfParam<double>(motor_, "maxRotVelocity", max_rot_velocity_, max_rot_velocity_);
    getSdfParam<double>(motor_, "minRotVelocity", min_rot_velocity_, min_rot_velocity_);
    getSdfParam<double>(motor_, "thrustConstant", thrust_constant_, thrust_constant_);
    getSdfParam<double>(motor_, "momentConstant", moment_constant_, moment_constant_);
    getSdfParam<double>(motor_, "motorTorqueConstant0", motor_torque_constant0_,motor_torque_constant0_);
    getSdfParam<double>(motor_, "motorTorqueConstant1", motor_torque_constant1_,motor_torque_constant1_);
    getSdfParam<double>(motor_, "motorTorqueConstant2", motor_torque_constant2_,motor_torque_constant2_);
    getSdfParam<double>(motor_, "timeConstantUp", time_constant_up_, time_constant_up_);
    getSdfParam<double>(motor_, "timeConstantDown", time_constant_down_, time_constant_down_);
    getSdfParam<double>(motor_, "rotorVelocitySlowdownSim", rotor_velocity_slowdown_sim_, 10);
    
    std::cout << "Inizialized Params as followed: " << std::endl;
    std::cout << "momentConstant: " << moment_constant_ << std::endl;
    std::cout << "linkBaseName: " << link_dynamixel_base_name << std::endl; 
    std::cout << "linkBase: " << joint_dynamixel_base_ << std::endl; 

    std::cout << "link_: " << link_ << std::endl; 

    // // Create the first order filter.
    // rotor_velocity_filter_.reset(
    //     new FirstOrderFilter<double>(time_constant_up_, time_constant_down_, ref_motor_rot_vel_));
    }

    void Publish() {} // Do we need ist?

    void UpdateForcesAndMoments() {

      double pos_base = joint_dynamixel_base_->Position(0); // Returns the Position in radians!
      double pos_top = joint_dynamixel_top_->Position(0);
      // std::cout << "Position Base is at: " << pos_base << std::endl;
      // std::cout << "Position Top is at: " << pos_top << std::endl;
       // std::cout << "UpdateForcesAndTorques not implemented!";
        // Do we set the velocity of the Motor to the PWM signal as mock speed 
        // And then here in the plugin convert this given PWM signal to a 

        // Wie erhalte ich die States der Dynamixel?

        // Macht es sinn alles in einem Plugin zusammen zu fassen?

        // Wo lassen wir die Kräfte angreiffen?
        /*
        double sim_motor_pwm = duct_joint_->GetVelocity(0); // Get the PWM Value from the duct_joint_ around axis=0; 
                double sim_dynamixel1_vel = dynamixel1_joint_->GetVelocity(0); // Get the Velocity Value from the first dynamixel around axis=0; // base
                double sim_dynamixel2_vel = dynamixel2_joint_->GetVelocity(0); // Get the Velocity Value from the second dynamixel around axis=0; // top 

        */
      
    }





};


}