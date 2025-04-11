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

    rclcpp::NodeOptions node_options;
    node_options.allow_undeclared_parameters(true);
    node_options.automatically_declare_parameters_from_overrides(true);
    node_ = std::make_shared<rclcpp::Node>("robot_gui", "", node_options);
    auto robot_description = node_->get_parameter_or<std::string>("robot_description", "");
    // std::cerr << robot_description << std::endl;
    // std::vector<std::string> controller_class = node_->get_parameter_or<std::vector<std::string>>("controllers", std::vector<std::string>());
    // for (auto name : controller_class)
    // {

    //     // auto pos = name.rfind(":");
    //     // name = name.substr(pos + 1);
    //     std::cerr << name << std::endl;
    //     ui->listWidget->addItem(QString::fromStdString(name));
    // }
    
    thread_ = std::make_shared<std::thread>([this]()
                                            { 
                                        auto executor = std::make_shared<rclcpp::executors::StaticSingleThreadedExecutor>();
                                        executor->add_node(node_);
                                        executor->spin(); 
                                        this->close(); });
    thread_->detach();
}

RobotGUIMainWindow::~RobotGUIMainWindow()
{
    delete ui;
}
