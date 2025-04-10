#include <QMainWindow>
#include <QApplication>
#include "rclcpp/rclcpp.hpp"
#include "RobotGUIMainWindow.h"
int main(int argc, char *argv[])
{
    rclcpp::init(argc, argv);
    QApplication a(argc, argv);
    RobotGUIMainWindow w;
    w.show();
    int ret = a.exec();
    rclcpp::shutdown();
    return ret;
}
