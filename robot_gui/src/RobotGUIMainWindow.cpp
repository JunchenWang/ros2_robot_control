#include "RobotGUIMainWindow.h"
#include "ui_RobotGUIMainWindow.h"
#include <limits>
#include <QInputDialog>
#include <QLabel>
#include <fstream>
#include "rclcpp/time.hpp"

// Subscriber::Subscriber(MainWindow *wnd) : Node("robot_monitor"), mainWnd(wnd), start_time(std::chrono::nanoseconds(this->now().nanoseconds()))
// {
//     auto topic_callback =
//         [this](sensor_msgs::msg::JointState::SharedPtr msg) -> void
//     {
//         // RCLCPP_INFO(this->get_logger(), "I heard: '%f'", 1);
//         rclcpp::Time time = msg->header.stamp;
//         std::chrono::time_point<std::chrono::system_clock, std::chrono::nanoseconds> msg_time(std::chrono::nanoseconds(time.nanoseconds()));
//         if(msg_time < start_time)
//             mainWnd->pushMessage(time.seconds(), msg);
//         else
//             mainWnd->pushMessage(std::chrono::duration<double>(msg_time - start_time).count(), msg);
//     };
//     subscription_ =
//         this->create_subscription<sensor_msgs::msg::JointState>("joint_states", rclcpp::SensorDataQoS(), topic_callback);


//     auto service_callback =
//         [this](const std::shared_ptr<std_srvs::srv::Empty::Request> /*request*/,
//                                              std::shared_ptr<std_srvs::srv::Empty::Response> /*response*/) -> void
//     {
//         start_time = std::chrono::time_point<std::chrono::system_clock, std::chrono::nanoseconds>(std::chrono::nanoseconds(this->now().nanoseconds()));
//         mainWnd->clear();
//     };

//     service_ =
//         this->create_service<std_srvs::srv::Empty>("~/clear", service_callback);
// }

RobotGUIMainWindow::RobotGUIMainWindow(QWidget *parent)
    : QMainWindow(parent), ui(new Ui::MainWindow)
{
    ui->setupUi(this);
   
    thread_ = std::make_shared<std::thread>([this]()
                                       { 
                                        auto executor = std::make_shared<rclcpp::executors::StaticSingleThreadedExecutor>();
                                        executor->add_node(node_);
                                        executor->spin(); 
                                        this->close();
                                        });
    thread_->detach();

}


RobotGUIMainWindow::~RobotGUIMainWindow()
{
    delete ui;
}
