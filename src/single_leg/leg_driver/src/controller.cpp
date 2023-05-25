// #include "leg_driver/controller.hpp"
#include "controller.hpp"


namespace SingleLeg{
    Controller::Controller(std::vector<double> kp, std::vector<double> kd){
        kp_ = kp;
        kd_ = kd;
    }

    Controller::Controller(){}

    void Controller::updata_torque(int leg_DOF, leg_msgs::LegTorqueControl &leg_torque_control_msg){
        // Initialize leg command message
        leg_torque_control_msg.motor_torques.resize(leg_DOF);
        
        auto tmp = leg_torque_control_msg.header.stamp;
        leg_torque_control_msg.motor_torques.at(0).header.stamp = tmp;
        leg_torque_control_msg.motor_torques.at(0).torque = 100;
        leg_torque_control_msg.motor_torques.at(1).header.stamp = tmp;
        leg_torque_control_msg.motor_torques.at(1).torque = 100;
        leg_torque_control_msg.motor_torques.at(2).header.stamp = tmp;
        leg_torque_control_msg.motor_torques.at(2).torque = 100;
    }
}