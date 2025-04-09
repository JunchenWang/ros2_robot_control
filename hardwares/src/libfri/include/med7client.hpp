
#include "friLBRClient.h"
#include "hardware_interface/command_interface.hpp"
#include "hardware_interface/state_interface.hpp"
#include "iostream"
#include <cstring>
#include <math.h>
#include <vector>

namespace KUKA::FRI
{
    class Med7Client : public KUKA::FRI::LBRClient
    {
    public:
        Med7Client()
        {
        }

        ~Med7Client() {}

        virtual void onStateChange(KUKA::FRI::ESessionState oldState, KUKA::FRI::ESessionState newState)
        {
            LBRClient::onStateChange(oldState, newState);
            // (re)initialize sine parameters when entering Monitoring
            switch (newState)
            {
            case KUKA::FRI::MONITORING_READY:
            case KUKA::FRI::IDLE:
            case KUKA::FRI::MONITORING_WAIT:
            case KUKA::FRI::COMMANDING_WAIT:
            case KUKA::FRI::COMMANDING_ACTIVE:
            default:
            {
                break;
            }
            }
        }

        virtual void command()
        {
            // calculate new offset
            // const double newOffset = _amplRad * sin(_phi);
            // _offset = (_offset * _filterCoeff) + (newOffset * (1.0 - _filterCoeff));
            // _phi += _stepWidth;
            // if (_phi >= (2 * M_PI))
            // {
            //     _phi -= (2 * M_PI);
            // }
            // // add offset to ipo joint position for all masked joints
            // double jointPos[KUKA::FRI::LBRState::NUMBER_OF_JOINTS];
            // memcpy(jointPos, robotState().getIpoJointPosition(), KUKA::FRI::LBRState::NUMBER_OF_JOINTS * sizeof(double));
            // std::cout << "ipo: " << jointPos[0] << " " << jointPos[1] << " " << jointPos[2] << " " << jointPos[3] << " " << jointPos[4] << " " << jointPos[5] << " " << jointPos[6] << std::endl;
            // double measuredPos[KUKA::FRI::LBRState::NUMBER_OF_JOINTS];
            // memcpy(measuredPos, robotState().getMeasuredJointPosition(), KUKA::FRI::LBRState::NUMBER_OF_JOINTS * sizeof(double));
            // std::cout << "measured: " << measuredPos[0] << " " << measuredPos[1] << " " << measuredPos[2] << " " << measuredPos[3] << " " << measuredPos[4] << " " << measuredPos[5] << " " << measuredPos[6] << std::endl;
            // for (unsigned int i = 0; i < KUKA::FRI::LBRState::NUMBER_OF_JOINTS; i++)
            // {
            //     if (_jointMask & (1u << i))
            //     {
            //         jointPos[i] += _offset;
            //     }
            // }
            auto &cmd = command_copy_.get<double>("position");
            double jointPos[LBRState::NUMBER_OF_JOINTS];
            memcpy(jointPos, robotState().getIpoJointPosition(), LBRState::NUMBER_OF_JOINTS * sizeof(double));
            robotCommand().setJointPosition(jointPos);
        }

        void setCommand(hardware_interface::CommandInterface &command)
        {
            command_copy_ = command;
        }

        hardware_interface::StateInterface getState()
        {

            auto &&robot_state = robotState();
            auto &&jointPosition_pdouble = robot_state.getMeasuredJointPosition();
            std::copy(jointPosition_pdouble, jointPosition_pdouble + LBRState::NUMBER_OF_JOINTS, state_.get<double>("position").begin());

            if (robot_state.getSessionState() == KUKA::FRI::COMMANDING_ACTIVE)
            {
                auto &&jointIpoPosition_pdouble = robot_state.getIpoJointPosition();
                std::copy(jointIpoPosition_pdouble, jointIpoPosition_pdouble + LBRState::NUMBER_OF_JOINTS, state_.get<double>("ipo_position").begin());
            }

            return state_;
        }

        ClientData *createData()
        {
            ClientData *const data = new ClientData(robotState_.NUMBER_OF_JOINTS);

            // link monitoring and command message to wrappers
            robotState_._message = &data->monitoringMsg;
            robotCommand_._cmdMessage = &data->commandMsg;
            robotCommand_._monMessage = &data->monitoringMsg;

            // set specific message IDs
            data->expectedMonitorMsgID = uint32_t(LBRState::LBRMONITORMESSAGEID);
            data->commandMsg.header.messageIdentifier = uint32_t(LBRCommand::LBRCOMMANDMESSAGEID);

            return data;
        }

    private:
        unsigned int _jointMask; //!< Bitmask encoding of overlay joints
        double _freqHz;          //!< sine frequency (Hertz)
        double _amplRad;         //!< sine amplitude (radians)
        double _filterCoeff;     //!< filter coefficient
        double _offset;          //!< offset for current interpolation step
        double _phi;             //!< phase of sine wave
        double _stepWidth;       //!< stepwidth for sine

        // LBRState _robotState;
        hardware_interface::CommandInterface command_copy_;
        hardware_interface::StateInterface state_;
        // ESessionState _currentState;

        LBRState robotState_;      //!< wrapper class for the FRI monitoring message
        LBRCommand robotCommand_;  //!< wrapper class for the FRI command message
    };

}
