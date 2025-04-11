#ifndef MAINWINDOW_H
#define MAINWINDOW_H

#include <QMainWindow>
#include <QTimer>
#include <deque>
#include <thread>
#include <chrono>
#include "rclcpp/rclcpp.hpp"

QT_BEGIN_NAMESPACE
namespace Ui { class MainWindow; }
QT_END_NAMESPACE

class RobotGUIMainWindow : public QMainWindow
{
    Q_OBJECT

public:
    RobotGUIMainWindow(QWidget *parent = nullptr);
    ~RobotGUIMainWindow();
private:
    Ui::MainWindow *ui;
    std::shared_ptr<std::thread> thread_;
    rclcpp::Node::SharedPtr node_;
};
#endif // MAINWINDOW_H
