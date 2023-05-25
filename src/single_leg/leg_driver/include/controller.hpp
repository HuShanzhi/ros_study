#include <leg_msgs/LegTorqueControl.h>
#include <leg_msgs/MotorTorque.h>


namespace SingleLeg{
    class Controller{
    public:
        // Constructor for Controller.
        Controller(std::vector<double> kp, std::vector<double> kd);
        
        // 
        Controller();

        void updata_torque(int leg_DOF, leg_msgs::LegTorqueControl &leg_torque_control_msg);
    private:
        // leg_msgs::LegTorqueControl leg_torque_control_msg_;

        // PD gain
        std::vector<double> kp_;
        std::vector<double> kd_;
    };
}

