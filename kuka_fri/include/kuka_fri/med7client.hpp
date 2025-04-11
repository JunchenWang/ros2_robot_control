#ifndef MED7_CLIENT_HPP
#define MED7_CLIENT_HPP

#include "FRIMessages.pb.h"
#include "friClientApplication.h"
#include "friClientData.h"
#include "friClientIf.h"
#include "friConnectionIf.h"
#include "friLBRClient.h"
#include "friTransformationClient.h"
#include "friUdpConnection.h"
#include "hardware_interface/command_interface.hpp"
#include "hardware_interface/state_interface.hpp"
#include <vector>

namespace KUKA::FRI
{
    class Med7Client : public LBRClient
    {
    public:
        Med7Client();
        ~Med7Client();

        void StateChange(ESessionState oldState, ESessionState newState, hardware_interface::StateInterface &state);

        void setCommand(hardware_interface::CommandInterface &command);

        void waitCommand(hardware_interface::StateInterface &state);

        void getState(hardware_interface::StateInterface &state);

    private:
    };
}

#endif // MED7_CLIENT_HPP