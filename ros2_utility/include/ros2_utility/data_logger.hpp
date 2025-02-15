#pragma once

#include <eigen3/Eigen/Dense>

#include <chrono>
#include <fstream>
#include <functional>
#include <iomanip>
#include <iostream>
#include <variant>
#include <vector>

#define NAME(var) #var
#define DATA_WRAPPER(var)                                               \
    DataInfo                                                            \
    {                                                                   \
        NAME(var), [&, this]() -> const decltype(var) & { return var; } \
    }
#define DATA_WRAPPER_SIZE(var, size)               \
    DataInfo                                       \
    {                                              \
        NAME(var), [&, this] { return var; }, size \
    }
#define CONFIG_WRAPPER(var) {NAME(var), var}
#define CONFIG_WRAPPER_SIZE(var, size) {NAME(var), var, size}

/**
 * @brief 实验配置类
 *
 * @details 用于记录实验配置，如控制器参数，滤波器参数等
 *
 * @param name 配置名称
 * @param config 配置值
 * @param cols 配置值的列数
 *
 * @example
 * ExperimentContext("a", 1.0)
 * ExperimentContext("b", array, 3)  // array为double[3]
 * ExperimentContext("c", {1.0, 2.0, 3.0})
 * ExperimentContext("d", MatrixXd(3, 3))
 * 宏定义简化写法
 * CONFIG_WRAPPER(a)
 * CONFIG_WRAPPER_SIZE(b, 3)
 * CONFIG_WRAPPER(c)
 * CONFIG_WRAPPER(d)
 * @note 用于DataLogger类
 */
class ExperimentContext
{
    std::string name;
    std::vector<double> config;
    size_t cols;

public:
    ExperimentContext(std::string name, double config) : name(name), config({config}), cols(1) {}
    ExperimentContext(std::string name, double *config, size_t size) : name(name), config(config, config + size), cols(1) {}
    ExperimentContext(std::string name, std::vector<double> config) : name(name), config(config), cols(1) {}
    ExperimentContext(std::string name, Eigen::MatrixXd config) : name(name), config(config.data(), config.data() + config.size()), cols(config.cols()) {}
    /**
     * @brief 生成实验配置字符串
     *
     * @return string 实验配置字符串
     */
    std::string recordConfig() const
    {
        std::string str = name + "=[";
        for (size_t i = 0; i < config.size(); i++)
        {
            str += std::to_string(config[i]);
            // 每cols个元素用;分隔
            if ((i + 1) % cols == 0)
                str += ";";
            else
                str += ",";
        }
        str += "];";
        if (cols > 1)
            str += name + "_cols=" + std::to_string(cols) + ";";
        return str;
    }
    friend class DataLogger;
};

/**
 * @brief 数据记录类
 *
 * @details 用于记录实验数据
 *
 * @param name 数据名称
 * @param getData 获取数据的函数
 * @param size 数据大小
 *
 * @example []中的内容取决于实际情况
 * DataInfo("a", [this]() -> const double & { return a; })          // a为double
 * DataInfo("b", [this]() -> const double * { return b; }, 1)       // b为指向a的double*
 * DataInfo("c", [this]() -> const double * { return c; }, 3)       // c为double[3]
 * DataInfo("d", [this]() -> const vector<double> & { return d; })  // d为vector<double>
 * DataInfo("e", [this]() -> const Vector2d & { return e; })        // e为Vector2d
 * DataInfo("f", [this]() -> const VectorXd & { return f; })        // f为VectorXd
 * DataInfo("g", [this]() -> const Matrix2d & { return g; })        // g为Matrix2d
 * DataInfo("h", [this]() -> const MatrixXd & { return h; })        // h为MatrixXd
 *
 * 宏定义简化写法
 * DATA_WRAPPER(a)
 * DATA_WRAPPER_SIZE(b, 1)
 * DATA_WRAPPER_SIZE(c, 3)
 * DATA_WRAPPER(d)
 * DATA_WRAPPER(e)
 * DATA_WRAPPER(f)
 * DATA_WRAPPER(g)
 * DATA_WRAPPER(h)
 */
class DataInfo
{
    std::string name;
    std::function<void(std::vector<double> &)> recordData;
    size_t size;

public:
    DataInfo(std::string name, std::function<const double()> getData) : name(name), size(1)
    {
        recordData = [getData](std::vector<double> &dataLog)
        {
            dataLog.push_back(getData());
        };
    }
    DataInfo(std::string name, std::function<const double *()> getData, size_t size) : name(name), size(size)
    {
        recordData = [getData, size](std::vector<double> &dataLog)
        {
            const double *d = getData();
            dataLog.insert(dataLog.end(), d, d + size);
        };
    }
    DataInfo(std::string name, std::function<std::vector<double>()> getData) : name(name), size(getData().size())
    {
        recordData = [getData](std::vector<double> &dataLog)
        {
            const std::vector<double> &d = getData();
            dataLog.insert(dataLog.end(), d.begin(), d.end());
        };
    }
    DataInfo(std::string name, std::function<Eigen::MatrixXd()> getData) : name(name), size(getData().size())
    {
        recordData = [getData](std::vector<double> &dataLog)
        {
            const Eigen::MatrixXd &d = getData();
            dataLog.insert(dataLog.end(), d.data(), d.data() + d.size());
        };
    }
    friend class DataLogger;
};

/**
 * @brief 数据记录类
 *
 * @details 用于记录实验数据
 *
 * @param allDataInfo_initializer 所有数据信息
 * @param allConfig 实验配置
 * @param rows 估计数据行数 通常用总时间除以采样时间得到，请适当高估！
 *
 * @example
 * DataLogger logger(
 *    { // 此处是initializer_list<DataInfo>， 具体用法见DataInfo
 *      DataInfo("a", [this]() -> const double & { return a; }),
 *      ...
 *      DataInfo("h", [this]() -> const MatrixXd & { return h; }),
 *    },
 *    { // 此处是initializer_list<ExperimentContext>， 具体用法见ExperimentContext
 *      ExperimentContext("a", 1.0),
 *      ...
 *      ExperimentContext("h", MatrixXd(3, 3)),
 *    },
 *    size_t(400 / dt)
 * );
 * @see ExperimentContext
 * @see DataInfo
 */
class DataLogger
{
private:
    std::vector<double> dataLog;
    std::vector<DataInfo> allDataInfo;
    std::vector<ExperimentContext> allConfig;
    size_t length;

public:
    DataLogger(std::initializer_list<DataInfo> allDataInfo_initializer, std::initializer_list<ExperimentContext> allConfig, size_t rows)
    {
        allDataInfo = allDataInfo_initializer;

        length = 0;
        for (const auto &dataInfo : allDataInfo)
        {
            length += dataInfo.size;
        }
        this->allConfig = allConfig;

        dataLog.reserve(length * rows); // 提前分配内存，减少内存重新分配开销
    };
    ~DataLogger() {};
    /**
     * @brief 记录数据
     * @details 将所有数据记录到dataLog中，每次调用都会记录一行数据
     */
    void record()
    {
        for (const auto &dataInfo : allDataInfo)
        {
            dataInfo.recordData(dataLog);
        }
    };
    /**
     * @brief 打印数据，用于调试
     */
    void print()
    {
        for (size_t i = 0; i != dataLog.size(); i++)
        {
            std::cout << dataLog[i] << " ";
            if ((i + 1) % length == 0)
                std::cout << std::endl;
        }
    }

    /**
     * @brief 保存数据到文件
     * @param prefixPath 文件路径前缀
     */
    void save(const std::string prefixPath = "", std::string prefixName = "test", std::string otherComment = "") const
    {
        // 生成文件名
        std::string filename = prefixPath + prefixName + "_";
        auto now = std::chrono::system_clock::now();
        auto now_c = std::chrono::system_clock::to_time_t(now);
        std::stringstream ss;
        ss << std::put_time(localtime(&now_c), "%Y-%m-%d %H-%M-%S");
        filename += ss.str();
        filename += ".txt";

        std::ofstream file(filename);
        if (!file.is_open())
        {
            std::cerr << "Error: cannot open file " << filename << std::endl;
            return;
        }

        if (!otherComment.empty())
            file << otherComment << "; ";
        // 写入实验配置
        for (const auto &config : allConfig)
        {
            file << config.recordConfig();
        }
        file << std::endl;

        // file << setprecision(10); // 设置输出精度(暂时不需要)

        // 写入标题栏
        for (const auto &dataInfo : allDataInfo)
        {
            if (dataInfo.size > 1)
                for (size_t i = 0; i < dataInfo.size; i++)
                {
                    file << dataInfo.name + std::to_string(i + 1) << ", ";
                }
            else
                file << dataInfo.name << ", ";
        }
        file << std::endl;

        // 写入数据
        for (size_t i = 0; i < dataLog.size(); i += length)
        {
            for (size_t j = 0; j < length; j++)
            {
                file << dataLog[i + j] << ", ";
            }
            file << std::endl;
        }

        file.close();
    }

    /**
     * @brief 将数据追加到文件末尾
     * @param file_path 文件路径
     * @param data 数据
     */
    static void appendToFile(const std::string &file_path, const Eigen::VectorXd &data)
    {
        std::ofstream fout(file_path, std::ios::app);
        if (!fout)
        {
            std::cout << "Failed to open file for appending: " << file_path << std::endl;
            return;
        }
        fout << data.transpose() << std::endl;
        fout.close();
    }
};