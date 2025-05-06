#include "RobotGUIMainWindow.h"
#include "ui_RobotGUIMainWindow.h"
#include <limits>
#include <QInputDialog>
#include <QLabel>
#include <fstream>
#include <QDoubleSpinBox>
#include "urdf/model.h"
#include "rclcpp/time.hpp"

RobotGUIMainWindow::RobotGUIMainWindow(QWidget *parent)
    : QMainWindow(parent), ui(new Ui::MainWindow), keep_running_(true)
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
    command_publisher_ = node_->create_publisher<std_msgs::msg::Float64MultiArray>("ForwardController/commands", rclcpp::SystemDefaultsQoS());
    // ForwardController UI
    for (auto &&jt : joint_names_)
    {
        auto item = new QDoubleSpinBox;
        item->setDecimals(3);
        item->setSingleStep(0.1);
        auto j = urdf_model.getJoint(jt);
        item->setRange(j->limits->lower, j->limits->upper);
        ui->formLayout_3->addRow(QString::fromStdString(jt), item);
        joint_command_spinbox_.push_back(item);
        connect(item, &QDoubleSpinBox::valueChanged, [this](double)
                { send_forward_command(); });
    }
    // Robot state UI
    for (auto &&jt : joint_names_)
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

    connect(ui->comboBox, &QComboBox::currentIndexChanged, [this](int index)
            { ui->tabWidget_2->setCurrentIndex(index); });
    connect(this, &RobotGUIMainWindow::joint_state_changed, this, &RobotGUIMainWindow::update_joint_state);
    connect(ui->pushButton, &QPushButton::clicked, [this]
            {

        int ind = ui->comboBox->currentIndex();
        
        if(ind == 0)
        {
            
            auto request = std::make_shared<std_srvs::srv::Empty::Request>();
            auto result = client_->async_send_request(request, [this](rclcpp::Client<std_srvs::srv::Empty>::SharedFuture /*future*/) {
                
                RCLCPP_INFO(node_->get_logger(), "stop controllers");
            });
            controller_mode_ = 0;
            this->statusBar()->showMessage(QString("active controller: %1").arg(ui->comboBox->currentText()));
        }
        else if(ind > 0)
        {
            auto request = std::make_shared<robot_control_msgs::srv::ControlCommand::Request>();
            request->cmd_name = "activate";
            request->cmd_params = ui->comboBox->currentText().toStdString();
            if(ind == 1) // forward controller
            {
                for(int i = 0; i < 6; i++)
                    joint_command_spinbox_[i]->setValue(joint_display_[i]->text().toDouble());
            }
            auto result = cmd_client_->async_send_request(request, [this, ind](rclcpp::Client<robot_control_msgs::srv::ControlCommand>::SharedFuture future) {
                if (future.get()->result)
                {
                    controller_mode_ = ind;
                    this->statusBar()->showMessage(QString("active controller: %1").arg(ui->comboBox->currentText()));
                    RCLCPP_INFO(node_->get_logger(), "activate success!");
                }
                else
                {
                    RCLCPP_ERROR(node_->get_logger(), "activate failed!");
                }
            });
            
        } });

    state_receiver_ = node_->create_subscription<sensor_msgs::msg::JointState>(
        "joint_states", rclcpp::SystemDefaultsQoS(),
        [this](sensor_msgs::msg::JointState::SharedPtr msg)
        {
            if (msg->name.size() != joint_names_.size())
            {
                RCLCPP_ERROR(node_->get_logger(), "Size of command does not match size of joint state");
                return;
            }
            emit joint_state_changed(msg);
        });

    thread_ = std::make_shared<std::thread>([this]()
                                            { 
                                        auto executor = std::make_shared<rclcpp::executors::StaticSingleThreadedExecutor>();
                                        executor->add_node(node_);
                                        executor->spin(); 
                                        this->close(); });
    thread_->detach();

    com_thread_ = std::make_shared<std::thread>(std::bind(&RobotGUIMainWindow::receive_data, this));
}

RobotGUIMainWindow::~RobotGUIMainWindow()
{
    keep_running_ = false;
    com_thread_->join();
    delete ui;
}

void RobotGUIMainWindow::send_forward_command()
{
    if (controller_mode_ == 1)
    {
        std_msgs::msg::Float64MultiArray msg;
        msg.data.resize(6);
        for (int i = 0; i < 6; i++)
            msg.data[i] = joint_command_spinbox_[i]->value();
        command_publisher_->publish(msg);
    }
}

void RobotGUIMainWindow::receive_data()
{
    int port = 7755;
    int socket_handle = socket(AF_INET, SOCK_DGRAM, 0);

    if (socket_handle == -1)
    {
        throw std::runtime_error("create socket failed");
    }
    struct sockaddr_in addr;
    memset(&addr, 0, sizeof(addr));
    addr.sin_family = AF_INET;
    addr.sin_port = htons(port);
    addr.sin_addr.s_addr = INADDR_ANY;
    if (bind(socket_handle, (struct sockaddr *)&addr, sizeof(addr)))
    {
        throw std::runtime_error("bind socket in service failed");
    }
    struct timeval read_timeout;
    read_timeout.tv_sec = 0;
    read_timeout.tv_usec = 10;
    // non-blocking
    if (setsockopt(socket_handle, SOL_SOCKET, SO_RCVTIMEO, &read_timeout, sizeof(read_timeout)))
    {
        throw std::runtime_error("set socket time out failed");
    }
    RCLCPP_INFO(node_->get_logger(), "receive thread started, port: %d", port);
    rclcpp::Rate rate(50);
    while (rclcpp::ok() && keep_running_)
    {
        auto t_start = std::chrono::high_resolution_clock::now();
        int resv_num = recvfrom(socket_handle, &buffer_, sizeof(buffer_), 0, nullptr, nullptr);
        if (resv_num > 0)
        {
            switch (buffer_.type)
            {
            case 0:

                break;

            default:
                break;
            }
        }
        rate.sleep();
    }

    ::close(socket_handle);
}

void RobotGUIMainWindow::update_joint_state(const sensor_msgs::msg::JointState::SharedPtr msg)
{
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
}
