// #include "leg_driver/leg_driver.hpp"
#include "leg_driver.hpp"


namespace SingleLeg{
    LegDriver::LegDriver(ros::NodeHandle nh){
        nh_ = nh;
        loadROSParam("leg_driver/kp", kp_);
        loadROSParam("leg_driver/kd", kd_);

        leg_torque_control_command_topic_ = "torque_command";
        leg_torque_control_pub_ = nh_.advertise<leg_msgs::LegTorqueControl>(leg_torque_control_command_topic_, 1);

        controller_ = Controller(kp_, kd_);
        leg_DOF_ = 3;
    }

    template <class ParamType>
    bool LegDriver::loadROSParam(std::string paramName, ParamType &varName){
        if (!nh_.getParam(paramName, varName)){  // Func `getParam` for getting a string value from the parameter server and store the value in `varName`. 
            ROS_ERROR("Can't find param %s from parameter server", paramName.c_str());
            return false;
        }
        return true;
    }

    void LegDriver::publishControl(){
        leg_torque_control_msg_.header.stamp = ros::Time::now();
        controller_.updata_torque(leg_DOF_, leg_torque_control_msg_);
        leg_torque_control_pub_.publish(leg_torque_control_msg_);
    }

    void LegDriver::spin(){
        ros::Rate loop_rate(10);

        while(ros::ok()){
            publishControl();

            ros::spinOnce();
            loop_rate.sleep();
        }
    }
}

