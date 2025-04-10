#include "med7client.hpp"
#include <cstring>
#include <iostream>
#include <math.h>

namespace KUKA::FRI
{
    Med7Client::Med7Client()
        : _jointMask(0), _freqHz(0), _amplRad(0),
          _filterCoeff(0), _offset(0), _phi(0), _stepWidth(0)
    {
    }

    Med7Client::~Med7Client() = default;

    void Med7Client::onStateChange(ESessionState oldState, ESessionState newState)
    {
        LBRClient::onStateChange(oldState, newState);
        // State change handling remains the same
        switch (newState)
        {
        case MONITORING_READY:
        case IDLE:
        case MONITORING_WAIT:
        case COMMANDING_WAIT:
        case COMMANDING_ACTIVE:
        default:
            break;
        }
    }

    void Med7Client::setCommand(hardware_interface::CommandInterface &command)
    {
        this->command();
        auto &cmd = command.get<double>("position");
        double jointPos[LBRState::NUMBER_OF_JOINTS];
        memcpy(jointPos, cmd.data(), LBRState::NUMBER_OF_JOINTS * sizeof(double));
        robotCommand().setJointPosition(jointPos);
    }
    
    void Med7Client::getState(hardware_interface::StateInterface& state)
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

    // ClientData *Med7Client::createData()
    // {
    //     ClientData *const data = new ClientData(robotState_.NUMBER_OF_JOINTS);

    //     robotState_._message = &data->monitoringMsg;
    //     robotCommand_._cmdMessage = &data->commandMsg;
    //     robotCommand_._monMessage = &data->monitoringMsg;

    //     data->expectedMonitorMsgID = static_cast<uint32_t>(LBRState::LBRMONITORMESSAGEID);
    //     data->commandMsg.header.messageIdentifier = static_cast<uint32_t>(LBRCommand::LBRCOMMANDMESSAGEID);

    //     return data;
    // }
} // namespace KUKA::FRI