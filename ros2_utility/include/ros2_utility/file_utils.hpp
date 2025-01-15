#ifndef FILE_UTILS
#define FILE_UTILS

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

using namespace std;

class FileUtils
{
public:
    // 获取当前时间的后缀字符串
    static string getTimeSuffix();

    // 读取文件并按指定的每次读取数量，将数据存入向量
    static bool readToVector(const string &file_path, vector<vector<double>> &data, size_t read_count);

    // 读取传感器标定文件并提取参数
    static bool readForceSensorCalibration(const string &file_path, float &mass, float cog[3], float offset[6]);

    // 解析文件，将数据存入map
    static map<string, vector<double>> parseFile(const string& filename);

    // 获取用户home路径
    static string getHomeDirectory();
};

#endif // FILE_UTILS
