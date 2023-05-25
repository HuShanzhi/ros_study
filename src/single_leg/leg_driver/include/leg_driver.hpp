#include <leg_msgs/LegTorqueControl.h>
#include <leg_msgs/MotorTorque.h>
#include <ros/ros.h>
// #include "leg_driver/controller.hpp"
#include "controller.hpp"

namespace SingleLeg{

    class LegDriver{
    public:
        /**
         * @brief Constructor for LegDriver
         * @param[in] nh ROS NodeHandle to publish and subscribe from
        */
        LegDriver(ros::NodeHandle nh);

        void spin();

    private:
        /**
         * 
        */
        template <class ParamType>
        bool loadROSParam(std::string paramName, ParamType &varName);

        /**
         * 
        */
        void publishControl();

        Controller controller_;

        int leg_DOF_;

        // ros node handler
        ros::NodeHandle nh_;

        // ros publisher for publishing leg torque command
        ros::Publisher leg_torque_control_pub_;

        std::string leg_torque_control_command_topic_;

        leg_msgs::LegTorqueControl leg_torque_control_msg_;
        
        // PD gain
        std::vector<double> kp_;
        std::vector<double> kd_;
    };
}

