#ifndef MAINWINDOW_H
#define MAINWINDOW_H

#include <QMainWindow>
#include <QTimer>
#include <deque>
#include <thread>
#include <chrono>
#include "rclcpp/rclcpp.hpp"
#include "robot_math/robot_math.hpp"
#include "sensor_msgs/msg/joint_state.hpp"
#include <QLineEdit>
#include <QDoubleSpinBox>
#include "robot_control_msgs/srv/control_command.hpp"
#include "std_srvs/srv/empty.hpp"
#include "std_msgs/msg/float64_multi_array.hpp"
#include <netinet/in.h>
#include <sys/socket.h>
#include <arpa/inet.h>
QT_BEGIN_NAMESPACE
namespace Ui { class MainWindow; }
QT_END_NAMESPACE

class RobotGUIMainWindow : public QMainWindow
{
    Q_OBJECT

public:
    RobotGUIMainWindow(QWidget *parent = nullptr);
    ~RobotGUIMainWindow();
    void send_forward_command();
    
protected:
    void receive_data();
private:
    Ui::MainWindow *ui;
    std::shared_ptr<std::thread> thread_;
    std::shared_ptr<std::thread> com_thread_;
    rclcpp::Node::SharedPtr node_;
    robot_math::Robot robot_;
    std::vector<std::string> joint_names_;
    std::string end_effector_;
    std::string base_link_;
    rclcpp::Subscription<sensor_msgs::msg::JointState>::SharedPtr state_receiver_;
    std::vector<QLineEdit *> joint_display_;
    std::vector<QLineEdit *> pose_display_;
    std::vector<QDoubleSpinBox *> joint_command_spinbox_;
    rclcpp::Client<std_srvs::srv::Empty>::SharedPtr client_;
    rclcpp::Client<robot_control_msgs::srv::ControlCommand>::SharedPtr cmd_client_;
    rclcpp::Publisher<std_msgs::msg::Float64MultiArray>::SharedPtr command_publisher_;
    int controller_mode_;
    volatile bool keep_running_;
    union 
    {
        int type;
        char data_[8192];
    } buffer_;
    
};
#endif // MAINWINDOW_H
