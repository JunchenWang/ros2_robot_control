#ifndef FILE_UTILS
#define FILE_UTILS

#include "ament_index_cpp/get_package_share_directory.hpp"
#include <string>
#include <vector>
#include <map>
#include <fstream>
#include <ctime>
#include <iostream>
#include <sstream>
#include <algorithm>
#include <iterator>
#include <cstddef> 

class FileUtils
{
public:
    // 获取当前时间的后缀字符串
    static std::string getTimeSuffix();

    // 读取文件并按指定的每次读取数量，将数据存入向量
    static bool readToVector(const std::string &file_path, std::vector<std::vector<double>> &data, size_t read_count);

    // 读取传感器标定文件并提取参数
    static bool readForceSensorCalibration(const std::string &file_path, float &mass, float cog[3], float offset[6]);

    // 解析文件，将数据存入map
    static std::map<std::string, std::vector<double>> parseFile(const std::string& filename);

    // 获取用户home路径
    static std::string getHomeDirectory();

    // 获取工作空间指定功能包目录
    static std::string getPackageDirectory(const std::string &packageName, const std::string &folderName = "ros2_robot_control");

    // 修改 YAML 文件键值
    static void modifyYamlValue(const std::string &filePath, const std::string &key, const std::vector<double> &newValues);
};

#endif // FILE_UTILS
