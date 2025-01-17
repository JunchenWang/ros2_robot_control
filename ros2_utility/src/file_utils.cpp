#include "ros2_utility/file_utils.hpp"

using namespace std;

// 获取当前时间的后缀字符串
string FileUtils::getTimeSuffix()
{
    time_t t = time(nullptr);
    tm *now = localtime(&t);
    char time_suffix[32];
    strftime(time_suffix, sizeof(time_suffix), "_%Y_%m_%d_%H_%M_%S", now); // 格式化时间为字符串
    return string(time_suffix);
}

// 读取文件并按指定的每次读取数量，将数据存入向量
bool FileUtils::readToVector(const string &file_path, vector<vector<double>> &data, size_t read_count)
{
    ifstream fin(file_path);
    if (!fin.is_open())
    {
        cerr << "Failed to open file: " << file_path << endl;
        return false; // 文件打开失败
    }

    double v;
    vector<double> temp;
    while (fin >> v)
    {
        temp.push_back(v);             // 逐个读取数据并存入 temp
        if (temp.size() == read_count) // 每次读取指定数量的数据
        {
            data.push_back(temp); // 将当前的数据组加入到 data 中
            temp.clear();         // 清空当前组，准备读取下一组
        }
    }

    fin.close();
    return true;
}

// 读取传感器标定文件并提取参数
bool FileUtils::readForceSensorCalibration(const string &file_path, float &mass, float cog[3], float offset[6])
{
    ifstream fin(file_path);
    if (!fin.is_open())
    {
        cerr << "Failed to open sensor calibration file: " << file_path << endl;
        return false; // 文件打开失败
    }

    // 从文件中读取质心位置和偏移量
    fin >> mass >> cog[0] >> cog[1] >> cog[2];                                         // 读取质心位置
    fin >> offset[0] >> offset[1] >> offset[2] >> offset[3] >> offset[4] >> offset[5]; // 读取传感器偏移量

    fin.close();
    return true;
}

// 解析文件，将数据存入map
map<string, vector<double>> FileUtils::parseFile(const string &filePath)
{
    ifstream file(filePath);
    if (!file.is_open())
    {
        throw runtime_error("Could not open file: " + filePath);
    }

    map<string, vector<double>> data;
    string line;

    // 跳过第一行非数据行（如 "T=0.002;"）
    while (getline(file, line))
    {
        line.erase(remove(line.begin(), line.end(), ' '), line.end()); // 移除空格
        if (line.empty() || line.find('=') != string::npos)
        {
            continue; // 如果行中包含 `=`，跳过该行
        }
        break; // 找到表头行，退出循环
    }

    // 解析表头
    istringstream headerStream(line);
    vector<string> columnNames;
    while (getline(headerStream, line, ','))
    {
        columnNames.push_back(line); // 保存列名
        data[line] = {};             // 初始化对应列的 vector
    }

    // 解析数据行
    while (getline(file, line))
    {
        line.erase(remove(line.begin(), line.end(), ' '), line.end()); // 移除空格
        if (line.empty())
        {
            continue; // 跳过空行
        }

        istringstream lineStream(line);
        string value;
        size_t columnIndex = 0;

        while (getline(lineStream, value, ','))
        {
            try
            {
                if (columnIndex < columnNames.size())
                {
                    data[columnNames[columnIndex]].push_back(stod(value)); // 严格按列名顺序存储数据
                }
                columnIndex++;
            }
            catch (const invalid_argument &e)
            {
                cerr << "Invalid value: '" << value << "' in file: " << filePath << endl;
                data[columnNames[columnIndex]].push_back(0.0); // 遇到无效数据时填充默认值
            }
        }
    }

    file.close();
    return data;
}

// 获取用户home路径
string FileUtils::getHomeDirectory()
{
    const char *home_dir = getenv("HOME");
    if (!home_dir)
    {
        throw runtime_error("Error: HOME environment variable not set");
    }
    return string(home_dir);
}

// 获取工作空间指定功能包目录
string FileUtils::getPackageDirectory(const std::string &packageName, const std::string &folderName)
{
    // 获取ROS2包的共享目录路径
    string package_share_directory = ament_index_cpp::get_package_share_directory(packageName);
    // 推导出工作空间的根目录
    string workspace_directory = package_share_directory + "/../../../../";
    return workspace_directory + "src/" + folderName + "/" + packageName;
}

// 修改 YAML 文件指定节点的值
void FileUtils::modifyYamlValue(const std::string &filePath, const std::string &key, const std::vector<double> &newValues)
{
    std::ifstream inputFile(filePath);
    if (!inputFile)
    {
        std::cerr << "Error: Unable to open file for reading - " << filePath << std::endl;
        return;
    }

    std::vector<std::string> fileContent;
    std::string line;
    bool keyFound = false;

    while (std::getline(inputFile, line))
    {
        // 去除前后的空格，判断是否为目标key
        size_t keyPos = line.find(key + ":");
        if (keyPos != std::string::npos)
        {
            keyFound = true;

            // 保留缩进
            std::string indentation = line.substr(0, keyPos);

            // 构造新的值
            std::ostringstream newLine;
            newLine << indentation << key << ": ";
            if (newValues.size() == 1)
            {
                newLine << newValues[0]; // 单个值，不加方括号
            }
            else
            {
                newLine << "[";
                for (size_t i = 0; i < newValues.size(); ++i)
                {
                    newLine << newValues[i];
                    if (i < newValues.size() - 1)
                    {
                        newLine << ", ";
                    }
                }
                newLine << "]";
            }
            fileContent.push_back(newLine.str());

            // 跳过当前行的原始值
            continue;
        }

        fileContent.push_back(line); // 保留其他行
    }

    inputFile.close();

    if (!keyFound)
    {
        std::cerr << "Error: Key not found in file - " << key << std::endl;
        return;
    }

    // 写回文件
    std::ofstream outputFile(filePath);
    if (!outputFile)
    {
        std::cerr << "Error: Unable to open file for writing - " << filePath << std::endl;
        return;
    }

    for (const auto &contentLine : fileContent)
    {
        outputFile << contentLine << "\n";
    }

    outputFile.close();
    std::cout << "Successfully updated the YAML file." << std::endl;
}