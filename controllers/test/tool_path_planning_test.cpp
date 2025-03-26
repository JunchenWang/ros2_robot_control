#include <chrono>
#include <iostream>
#include <fstream>
#include <opencamlib/adaptivepathdropcutter.hpp>
#include <opencamlib/adaptivewaterline.hpp>
#include <opencamlib/ballcutter.hpp>
#include <opencamlib/bullcutter.hpp>
#include <opencamlib/conecutter.hpp>
#include <opencamlib/cylcutter.hpp>
#include <opencamlib/line.hpp>
#include <opencamlib/ocl.hpp>
#include <opencamlib/path.hpp>
#include <opencamlib/pathdropcutter.hpp>
#include <opencamlib/point.hpp>
#include <opencamlib/stlreader.hpp>
#include <opencamlib/stlsurf.hpp>
#include <opencamlib/waterline.hpp>
#include <rclcpp/rclcpp.hpp>
#include <string>

template <typename T>
void printPoint(T point)
{
    std::cout << "X: " << point.x << " Y: " << point.y << " Z: " << point.z << std::endl;
}

template <typename T>
void printPoints(std::vector<T> points)
{
    for (auto &point : points)
        std::cout << "X: " << point.x << " Y: " << point.y << " Z: " << point.z << std::endl;
}

template <typename T>
void printLoops(std::vector<std::vector<T>> loops)
{
    for (auto &points : loops)
    {
        printPoints(points);
        std::cout << "-------------------" << std::endl;
    }
}

template <typename T>
void savePoints(std::vector<T> points, std::string path)
{
    std::ofstream file(path, std::ios::app);
    for (auto &point : points)
        file << point.x << " " << point.y << " " << point.z << std::endl;
    file.close();
}

template <typename T>
void saveLoops(std::vector<std::vector<T>> loops, std::string path)
{
    std::ofstream file(path, std::ios::app);
    for (auto &points : loops)
    {
        for (auto &point : points)
            file << point.x << " " << point.y << " " << point.z << std::endl;
        file << "-------------------" << std::endl;
    }
    file << "===================" << std::endl;
    file.close();
}

void waterline(ocl::STLSurf surface, ocl::MillingCutter *cutter, double z, double sampling, std::string cutterName = "")
{
    ocl::Waterline wl = ocl::Waterline();
    wl.setSTL(surface);
    wl.setCutter(cutter);
    wl.setSampling(sampling);
    for (double h = 0; h < z; h = h + 0.1)
    {
        wl.reset();
        wl.setZ(h);
        wl.run();
        auto loops = wl.getLoops();
        if (loops.size() == 0)
            break;
        printLoops(loops);
        if (cutterName != "")
            saveLoops(loops, "/home/eric/shared_data/stl/" + cutterName + ".txt");
    }
}

void adaptiveWaterline(ocl::STLSurf surface, ocl::MillingCutter *cutter, double z, double sampling, double minSampling, std::string cutterName = "")
{
    ocl::AdaptiveWaterline awl = ocl::AdaptiveWaterline();
    awl.setSTL(surface);
    awl.setCutter(cutter);
    awl.setSampling(sampling);
    awl.setMinSampling(minSampling);
    for (double h = 0; h < z; h = h + 0.1)
    {
        awl.reset();
        awl.setZ(h);
        awl.run();
        auto loops = awl.getLoops();
        printLoops(loops);
        if (cutterName != "")
            saveLoops(loops, "/home/eric/shared_data/stl/" + cutterName + ".txt");
    }
}

void pathDropCutter(ocl::STLSurf surface, ocl::MillingCutter *cutter, double sampling, ocl::Path *path)
{
    ocl::PathDropCutter pdc = ocl::PathDropCutter();
    pdc.setSTL(surface);
    pdc.setCutter(cutter);
    pdc.setPath(path);
    pdc.setSampling(sampling);
    pdc.reset();
    pdc.setZ(0);
    pdc.run();
    auto points = pdc.getPoints();
    printPoints(points);
}

void adaptivePathDropCutter(ocl::STLSurf surface, ocl::MillingCutter *cutter, double sampling, double minSampling, ocl::Path *path)
{
    ocl::AdaptivePathDropCutter apdc = ocl::AdaptivePathDropCutter();
    apdc.setSTL(surface);
    apdc.setCutter(cutter);
    apdc.setPath(path);
    apdc.setSampling(sampling);
    apdc.setMinSampling(minSampling);
    apdc.reset();
    apdc.setZ(0);
    apdc.run();
    auto points = apdc.getPoints();
    printPoints(points);
}

class ToolPathPlanningTest : public rclcpp::Node
{
public:
    ToolPathPlanningTest() : Node("tool_path_planning_test")
    {
        test();
    }
    void test()
    {
        ocl::STLSurf surface = ocl::STLSurf();
        std::wstring stlPath = L"~/shared_data/models/stl/after.stl";
        ocl::STLReader(stlPath, surface);
        std::cout << "surface size: " << surface.size() << "\n";

        ocl::CylCutter cylCutter = ocl::CylCutter(0.4, 10);        // 圆柱立铣刀 0.4mm 直径 10mm 长度
        ocl::ConeCutter coneCutter = ocl::ConeCutter(4, 0.05, 20); // 圆锥立铣刀 4mm 锥角 0.05rad 长度 20mm
        std::vector<ocl::MillingCutter *> cutters;
        cutters.push_back(&cylCutter);
        // cutters.push_back(&coneCutter);
        double z = 15;
        double sampling = 0.3;
        for (auto cutter : cutters)
        {
            std::cout << "WL + Cutter: " << cutter->str() << "\n";
            waterline(surface, cutter, z, sampling, cutter->str());
        }
        // double minSampling = 0.01;
        // for (auto cutter : cutters)
        // {
        //     std::cout << "AWL + Cutter: " << cutter->str() << "\n";
        //     adaptiveWaterline(surface, cutter, z, sampling, minSampling);
        // }
        // ocl::Path path = ocl::Path();
        // int i = 0;
        // for (double y = 0; y <= 0.2; y = y + 0.1)
        // {
        //     bool ltr = ((int)i % 2) == 0;
        //     ocl::Point p1 = ocl::Point(ltr ? -2 : 11, y, 0);
        //     ocl::Point p2 = ocl::Point(ltr ? 11 : -2, y, 0);
        //     ocl::Line l = ocl::Line(p1, p2);
        //     path.append(l);
        //     ocl::Point p3 = ocl::Point(ltr ? 11 : -2, y + 1, 0);
        //     ocl::Line l2 = ocl::Line(p2, p3);
        //     path.append(l2);
        //     i++;
        // }
        // for (auto cutter : cutters)
        // {
        //     std::cout << "PDC + Cutter: " << cutter->str() << "\n";
        //     pathDropCutter(surface, cutter, sampling, &path);
        // }
        // for (auto cutter : cutters)
        // {
        //     std::cout << "APDC: " << cutter->str() << "\n";
        //     adaptivePathDropCutter(surface, cutter, sampling, minSampling, &path);
        // }
        std::cout << "Done!" << std::endl;
    }
};

int main(int argc, char *argv[])
{
    // 初始化 ROS2
    rclcpp::init(argc, argv);

    // 创建节点并运行
    rclcpp::spin(std::make_shared<ToolPathPlanningTest>());

    // 关闭 ROS2
    rclcpp::shutdown();
    return 0;
}
