#ifndef MED7_CLIENT_HPP
#define MED7_CLIENT_HPP

#include "friLBRClient.h"
#include "friClientData.h"
#include "hardware_interface/command_interface.hpp"
#include "hardware_interface/state_interface.hpp"
#include "friClientApplication.h"
#include "friUdpConnection.h"
#include "FRIMessages.pb.h"
#include "friClientData.h"
#include "friClientIf.h"
#include "friConnectionIf.h"
#include "friTransformationClient.h"
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

        void getState(hardware_interface::StateInterface& state);
        ClientData* createData();

    private:
        unsigned int _jointMask;
        double _freqHz;
        double _amplRad;
        double _filterCoeff;
        double _offset;
        double _phi;
        double _stepWidth;

        hardware_interface::CommandInterface command_copy_;
        // hardware_interface::StateInterface state_;
        
        LBRState robotState_;
        LBRCommand robotCommand_;
    };
} // namespace KUKA::FRI

#endif // MED7_CLIENT_HPP