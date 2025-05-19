#ifndef MAINWINDOW_H
#define MAINWINDOW_H

#include <QMainWindow>
#include "widget.h"
#include "robot_control_msgs/msg/robot_state.hpp"
#include <QTimer>
#include <deque>
#include <thread>
#include <mutex>
#include "rclcpp/rclcpp.hpp"
#include <QLabel>
#include "std_srvs/srv/empty.hpp"
#include <chrono>
QT_BEGIN_NAMESPACE
namespace Ui { class MainWindow; }
QT_END_NAMESPACE

class MainWindow : public QMainWindow
{
    Q_OBJECT

public:
    MainWindow(QWidget *parent = nullptr);
    ~MainWindow();
    void pushMessage(double t, robot_control_msgs::msg::RobotState::SharedPtr msg);
    double getDataDuration();
    void clear();
    void log2file();
    void onTimer();
    void setJointDisplay();
    void setScaling(bool );
    void setWindowWidth();
    void setLineWidth();
    void zoomIn();
    void zoomOut();
    void zoomReset();
    void setLogging(bool checked);
private:
    Ui::MainWindow *ui;
    Widget *widgets[4];
    QLabel * message;
    unsigned int sampleCount = 50000;
    QList<QPointF> m_buffer[4][7];
    double time_width = 10;
    QTimer timer;
    std::vector<std::vector<double>> frames;
    std::vector<double> frames_time;
    bool isScaling;
    bool isLogging;
    std::shared_ptr<std::thread> tt;
    std::mutex mtx;
    rclcpp::Time start_time_;
    rclcpp::Node::SharedPtr node_;
    rclcpp::Subscription<robot_control_msgs::msg::RobotState>::SharedPtr subscription_;
    bool first_time_;
};
#endif // MAINWINDOW_H
