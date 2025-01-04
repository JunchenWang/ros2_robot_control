#include "controller_interface/controller_interface.hpp"
#include <iostream>
namespace controllers
{
    class DummyController : public controller_interface::ControllerInterface
    {
    public:
        DummyController() 
        {
        }
       
    protected:
      
    };

} // namespace controllers

#include <pluginlib/class_list_macros.hpp>

PLUGINLIB_EXPORT_CLASS(controllers::DummyController, controller_interface::ControllerInterface)