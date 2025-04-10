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

        virtual void onStateChange(ESessionState oldState, ESessionState newState) override;
        // virtual void command() override;

        // void setCommand(hardware_interface::CommandInterface& command);
        void setCommand(hardware_interface::CommandInterface &command);

        void getState(hardware_interface::StateInterface &state);

    private:
        unsigned int _jointMask;
        double _freqHz;
        double _amplRad;
        double _filterCoeff;
        double _offset;
        double _phi;
        double _stepWidth;
    };
} // namespace KUKA::FRI

#endif // MED7_CLIENT_HPP