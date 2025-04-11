#include "med7client.hpp"
#include <cstring>
#include <iostream>
#include <math.h>

namespace KUKA::FRI
{
    Med7Client::Med7Client() {}

    Med7Client::~Med7Client() = default;

    void Med7Client::onStateChange(ESessionState oldState, ESessionState newState)
    {
        LBRClient::onStateChange(oldState, newState);
        // State change handling remains the same
        switch (newState)
        {
        case IDLE:
        case MONITORING_WAIT:
        case MONITORING_READY:
        case COMMANDING_WAIT:
        case COMMANDING_ACTIVE:
        default:
            break;
        }
    }

    void Med7Client::setCommand(hardware_interface::CommandInterface &command)
    {
        Med7Client::command();
        auto &cmd = command.get<double>("position");
        double jointPos[LBRState::NUMBER_OF_JOINTS];
        memcpy(jointPos, cmd.data(), LBRState::NUMBER_OF_JOINTS * sizeof(double));
        robotCommand().setJointPosition(jointPos);
    }

    void Med7Client::getState(hardware_interface::StateInterface &state)
    {
        auto &&mjp = robotState().getMeasuredJointPosition();
        std::copy(mjp, mjp + LBRState::NUMBER_OF_JOINTS,
                  state.get<double>("position").begin());
        if (robotState().getSessionState() == COMMANDING_ACTIVE)
        {
            auto &&ipop = robotState().getIpoJointPosition();
            std::copy(ipop, ipop + LBRState::NUMBER_OF_JOINTS,
                      state.get<double>("ipo_position").begin());
        }
    }
} // namespace KUKA::FRI