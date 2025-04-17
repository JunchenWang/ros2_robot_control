#ifndef TPP_UTILS_HPP
#define TPP_UTILS_HPP

#include "rclcpp/rclcpp.hpp"
#include "noether_tpp/core/tool_path_planner_pipeline.h"
#include "noether_tpp/tool_path_planners/raster/plane_slicer_raster_planner.h"
#include "noether_tpp/tool_path_planners/edge/boundary_edge_planner.h"
#include "noether_tpp/tool_path_planners/raster/direction_generators/fixed_direction_generator.h"
#include "noether_tpp/tool_path_planners/raster/origin_generators/fixed_origin_generator.h"
#include "noether_tpp/tool_path_modifiers/snake_organization_modifier.h"
#include "noether_tpp/tool_path_modifiers/compound_modifier.h"

#include <pcl/io/ply_io.h>
#include <yaml-cpp/yaml.h>
#include <fstream>
#include <ctime>
#include <iostream>

// 空 MeshModifier
class IdentityMeshModifier : public noether::MeshModifier
{
public:
    std::vector<pcl::PolygonMesh> modify(pcl::PolygonMesh mesh)
    {
        return {std::move(mesh)}; // 直接返回原始网格
    }
};

class TPPUtils
{
public:
    // 读取PLY文件
    static pcl::PolygonMesh loadMesh(const std::string &path);

    // 创建规划器
    static std::unique_ptr<const noether::ToolPathPlanner> createToolPathPlanner(const YAML::Node &config);

    // 创建路径修饰器
    static std::vector<std::unique_ptr<const noether::ToolPathModifier>> createToolPathModifiers(const YAML::Node &config);

    // 创建复合路径修饰器
    static std::unique_ptr<noether::CompoundModifier> createCompoundModifier(std::vector<std::unique_ptr<const noether::ToolPathModifier>> modifiers);

    // 导出刀具路径
    static void saveToolPaths(const std::vector<noether::ToolPaths> &tool_paths, const std::string &filename, const char &sep = ' ');

    // 读取刀具路径，存成二维 vector
    static std::vector<std::vector<Eigen::Matrix4d>> readToolPaths(const std::string& filename);
};

#endif // TPP_UTILS_HPP