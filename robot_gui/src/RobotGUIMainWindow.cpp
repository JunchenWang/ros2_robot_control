#include "RobotGUIMainWindow.h"
#include "ui_RobotGUIMainWindow.h"
#include <limits>
#include <QInputDialog>
#include <QLabel>
#include <fstream>
#include <QDoubleSpinBox>
#include "urdf/model.h"
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
    urdf::Model urdf_model;
    urdf_model.initString(robot_description);
    robot_ = robot_math::urdf_to_robot(robot_description, joint_names_, end_effector_, base_link_);
    ui->label->setText(QString("%1 with end-effector %2").arg(QString::fromStdString(urdf_model.getName())).arg(QString::fromStdString(end_effector_)));
    client_ = node_->create_client<std_srvs::srv::Empty>("control_node/stop");
    cmd_client_ = node_->create_client<robot_control_msgs::srv::ControlCommand>("control_node/control_command");
    
    // ForwardController UI
    for(auto && jt : joint_names_)
    {
        auto item = new QDoubleSpinBox;
        item->setDecimals(3);
        item->setSingleStep(0.1);
        auto j = urdf_model.getJoint(jt);
        item->setRange(j->limits->lower, j->limits->upper);
        ui->formLayout_3->addRow(QString::fromStdString(jt), item);
    }
    // Robot state UI
    for(auto && jt : joint_names_)
    {

        auto item = new QLineEdit;
        item->setReadOnly(true);
        auto j = urdf_model.getJoint(jt);
        ui->formLayout->addRow(QString::fromStdString(jt), item);
        joint_display_.push_back(item);
    }
    auto item = new QLineEdit;
    item->setReadOnly(true);
    ui->formLayout->addRow("x", item);
    pose_display_.push_back(item);
    item = new QLineEdit;
    item->setReadOnly(true);
    ui->formLayout->addRow("y", item);
    pose_display_.push_back(item);
    item = new QLineEdit;
    item->setReadOnly(true);
    ui->formLayout->addRow("z", item);
    pose_display_.push_back(item);

    item = new QLineEdit;
    item->setReadOnly(true);
    ui->formLayout->addRow("rx", item);
    pose_display_.push_back(item);
    item = new QLineEdit;
    item->setReadOnly(true);
    ui->formLayout->addRow("ry", item);
    pose_display_.push_back(item);
    item = new QLineEdit;
    item->setReadOnly(true);
    ui->formLayout->addRow("rz", item);
    pose_display_.push_back(item);

    connect(ui->pushButton, &QPushButton::clicked, [this] {

        int ind = ui->comboBox->currentIndex();
        if(ind == 0)
        {
            
            auto request = std::make_shared<std_srvs::srv::Empty::Request>();
            auto result = client_->async_send_request(request, [this](rclcpp::Client<std_srvs::srv::Empty>::SharedFuture future) {
                
                future;
                RCLCPP_INFO(node_->get_logger(), "stop controllers");
            });
        }
        else if(ind > 0)
        {
            auto request = std::make_shared<robot_control_msgs::srv::ControlCommand::Request>();
            request->cmd_name = "activate";
            request->cmd_params = ui->comboBox->currentText().toStdString();
            auto result = cmd_client_->async_send_request(request, [this](rclcpp::Client<robot_control_msgs::srv::ControlCommand>::SharedFuture future) {
                if (future.get()->result)
                {
                    RCLCPP_INFO(node_->get_logger(), "activate success!");
                }
                else
                {
                    RCLCPP_ERROR(node_->get_logger(), "activate failed!");
                }
            });
        }
    });

    state_receiver_ = node_->create_subscription<sensor_msgs::msg::JointState>(
        "joint_states", rclcpp::SystemDefaultsQoS(),
        [this](sensor_msgs::msg::JointState::SharedPtr msg)
        {
            if (msg->name.size() != joint_names_.size())
            {
                RCLCPP_ERROR(node_->get_logger(), "Size of command does not match size of joint state");
                return;
            }
            for (std::size_t i = 0; i < msg->name.size(); ++i)
            {
                auto item = joint_display_[i];
                item->setText(QString("%1").arg(msg->position[i], 5, 'f', 3));
            }
            Eigen::Matrix4d T;
            robot_math::forward_kinematics(&robot_, msg->position, T);
            auto pose = robot_math::tform_to_pose(T);
            pose_display_[0]->setText(QString("%1").arg(pose[0], 5, 'f', 4));
            pose_display_[1]->setText(QString("%1").arg(pose[1], 5, 'f', 4));
            pose_display_[2]->setText(QString("%1").arg(pose[2], 5, 'f', 4));
            pose_display_[3]->setText(QString("%1").arg(pose[3], 5, 'f', 4));
            pose_display_[4]->setText(QString("%1").arg(pose[4], 5, 'f', 4));
            pose_display_[5]->setText(QString("%1").arg(pose[5], 5, 'f', 4));
        });

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
