#include "path_planner/tpp_utils.hpp"

pcl::PolygonMesh TPPUtils::loadMesh(const std::string &path)
{
    pcl::PolygonMesh mesh;
    if (pcl::io::loadPLYFile(path, mesh) < 0)
        throw std::runtime_error("Failed to load mesh");
    return mesh;
}

// 创建规划器
std::unique_ptr<const noether::ToolPathPlanner> TPPUtils::createToolPathPlanner(const YAML::Node &config)
{
    YAML::Node tpp_config = config["tool_path_planner"];

    if (tpp_config["name"].as<std::string>() == "BoundaryEdgePlanner")
    {
        // 创建边界规划器
        return std::make_unique<noether::BoundaryEdgePlanner>();
    }

    // 解析方向生成器（FixedDirectionGenerator）
    YAML::Node dir_gen_config = tpp_config["direction_generator"];
    std::unique_ptr<noether::FixedDirectionGenerator> dir_gen = std::make_unique<noether::FixedDirectionGenerator>(
        Eigen::Vector3d(dir_gen_config["x"].as<double>(),
                        dir_gen_config["y"].as<double>(),
                        dir_gen_config["z"].as<double>()));

    // 解析原点生成器（FixedOriginGenerator）
    YAML::Node origin_gen_config = tpp_config["origin_generator"];
    std::unique_ptr<noether::FixedOriginGenerator> origin_gen = std::make_unique<noether::FixedOriginGenerator>(
        Eigen::Vector3d(origin_gen_config["x"].as<double>(),
                        origin_gen_config["y"].as<double>(),
                        origin_gen_config["z"].as<double>()));

    // 创建PlaneSlicerRasterPlanner
    auto planner = std::make_unique<noether::PlaneSlicerRasterPlanner>(std::move(dir_gen), // 将所有权转移给构造函数
                                                                       std::move(origin_gen));

    // 设置其他参数
    planner->setLineSpacing(tpp_config["line_spacing"].as<double>());
    planner->setPointSpacing(tpp_config["point_spacing"].as<double>());
    planner->setMinHoleSize(tpp_config["min_hole_size"].as<double>());
    planner->setSearchRadius(tpp_config["search_radius"].as<double>());
    planner->setMinSegmentSize(tpp_config["min_segment_size"].as<double>());
    planner->generateRastersBidirectionally(tpp_config["bidirectional"].as<bool>());

    return planner;
}

// 创建路径修饰器
std::vector<std::unique_ptr<const noether::ToolPathModifier>> TPPUtils::createToolPathModifiers(const YAML::Node &config)
{
    // 读取tool_path_modifiers配置段
    YAML::Node tpm_configs = config["tool_path_modifiers"];

    std::vector<std::unique_ptr<const noether::ToolPathModifier>> modifiers;
    for (const YAML::Node &node : tpm_configs)
    {
        const std::string name = node["name"].as<std::string>();

        if (name == "SnakeOrganizationModifier")
        {
            modifiers.push_back(std::make_unique<noether::SnakeOrganizationModifier>());
        }
    }
    return modifiers;
}

// 创建复合路径修饰器
std::unique_ptr<noether::CompoundModifier> TPPUtils::createCompoundModifier(
    std::vector<std::unique_ptr<const noether::ToolPathModifier>> modifiers)
{
    std::vector<noether::ToolPathModifier::ConstPtr> mods;
    for (auto &mod : modifiers)
        mods.push_back(std::move(mod)); // 转移所有权到基类指针容器

    return std::make_unique<noether::CompoundModifier>(std::move(mods));
}

// 导出刀具路径
void TPPUtils::saveToolPaths(const std::vector<noether::ToolPaths> &paths, const std::string &filename, const char &sep)
{
    std::ofstream file(filename);
    if (!file.is_open())
    {
        RCLCPP_ERROR(rclcpp::get_logger("path_generator"), "Failed to open file %s", filename.c_str());
        return;
    }
    // 遍历所有路径层次
    for (const noether::ToolPaths &tool_paths : paths)                // tool_paths 表示路径集合，往往单个物体只有一个 tool_paths
        for (const noether::ToolPath &tool_path : tool_paths)         // tool_path 表示一道路径
            for (const noether::ToolPathSegment &segment : tool_path) // segment 表示一个线段集合，往往一个 tool_path 只包含一个 segment
            {
                for (const Eigen::Isometry3d &waypoint : segment) // 单个路径点
                {
                    // 提取平移向量和四元数
                    Eigen::Vector3d translation = waypoint.translation();
                    Eigen::Matrix3d rotation = waypoint.rotation();
                    Eigen::Quaterniond quaternion(rotation);

                    // 格式化输出 x; y; z; qx; qy; qz; qw
                    file << translation.x() << sep
                         << translation.y() << sep
                         << translation.z() << sep
                         << quaternion.x() << sep
                         << quaternion.y() << sep
                         << quaternion.z() << sep
                         << quaternion.w() << "\n";

                    // 格式化输出 x; y; z; 法向量 nx; ny; nz
                    // Eigen::AngleAxisd angle_axis(rotation);
                    // file << translation.x() << sep
                    //      << translation.y() << sep
                    //      << translation.z() << sep
                    //      << rotation(0, 2) << sep
                    //      << rotation(1, 2) << sep
                    //      << rotation(2, 2) << "\n";
                }
                // 用空行分隔不同路径段
                file << "\n";
            }
    file.close();
    RCLCPP_INFO(rclcpp::get_logger("path_generator"), "Tool paths saved to %s", filename.c_str());
}

// 读取刀具路径，存成二维 vector
std::vector<std::vector<Eigen::Matrix4d>> TPPUtils::readToolPaths(const std::string& filename)
{
    std::vector<std::vector<Eigen::Matrix4d>> allPaths;
    std::vector<Eigen::Matrix4d> currentPath;
    
    std::ifstream file(filename);
    std::string line;

    while (std::getline(file, line)) {
        // 处理空行分隔符
        if (line.empty()) {
            if (!currentPath.empty()) {
                allPaths.push_back(currentPath);
                currentPath.clear();
            }
            continue;
        }

        // 解析每行数据
        std::istringstream iss(line);
        Eigen::Vector3d translation;
        double qx, qy, qz, qw; // 四元数顺序为 x,y,z,w

        if (iss >> translation[0] >> translation[1] >> translation[2]
                >> qx >> qy >> qz >> qw) {
            
            // 构造四元数（参数顺序：w,x,y,z）
            Eigen::Quaterniond quat(qw, qx, qy, qz);
            quat.normalize();
            
            // 构建齐次变换矩阵
            Eigen::Matrix4d mat = Eigen::Matrix4d::Identity();
            mat.block(0, 0, 3, 3) = quat.toRotationMatrix();
            mat.block(0, 3, 3, 1) = translation;
            
            currentPath.push_back(mat);
        }
    }

    // 处理最后一个路径段
    if (!currentPath.empty()) {
        allPaths.push_back(currentPath);
    }

    return allPaths;
}