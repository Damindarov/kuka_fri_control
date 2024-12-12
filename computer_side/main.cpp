#include "kukafricontroller.hpp"
#include <cmath>
#include <thread>
#include <iostream>
#include <iomanip>

#include "logger/jarraylogger.hpp"
#include "control/control.hpp"

using namespace KUKA_CONTROL;
using namespace kuka_control;

int main(int argc, char **argv)
{   
    KukaFRIController kuka(KUKA_CONTROL::TORQUE);
    jarray torque = {0, 0, 0, 0, 0, 0, 0};
    kuka.setTargetJointTorque(torque);

    bool flag = true;
    kuka.start();
    jarray current_position;
    jarray current_torque;
    double last_joint_torque = 0;
    constexpr double Kp = 4.9;
    constexpr double Kd = std::sqrt(Kp) * 2;
    const double q_desired = 0.5;
    double q_desired_ = q_desired;
    double q0 = kuka.getJointPosition()[5];
    double prev_pos = 0;
    double q_dot = 0;
    bool flag1 = true;

    Control contrololo(0, 0, 1.5, 0.005);

    jarray initial_position = kuka.getJointPosition();
    LOGGER::JArrayLogger pos_logger("actual_position");
    LOGGER::JArrayLogger torq_logger("actual_torque");

    LOGGER::JArrayLogger commanded_pos_logger("commanded_position");
    LOGGER::JArrayLogger commanded_torq_logger("commanded_torque");

    contrololo.setPreviousPos(initial_position[6]);

    while (true)
    {
        current_position = kuka.getJointPosition();
        current_torque = kuka.getTorque();

        pos_logger.log(current_position);
        torq_logger.log(current_torque);

        initial_position[6] = current_position[6];

        last_joint_torque = contrololo.calcTorque(current_position[6], 15*M_PI/180);
        std::cout << last_joint_torque << std::endl;
        
        commanded_pos_logger.log(initial_position);
        commanded_torq_logger.log({0, 0, 0, 0, 0, 0, last_joint_torque});
        kuka.setTargetJointPosition(initial_position);
        kuka.setTargetJointTorque({0, 0, 0, 0, 0, 0, 0});
        // kuka.setTarget(torque);
        std::this_thread::sleep_for(std::chrono::microseconds(900));
    }

    return 0;
}
