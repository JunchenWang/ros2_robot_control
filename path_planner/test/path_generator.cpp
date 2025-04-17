#include "path_planner/tpp_utils.hpp"
#include "ros2_utility/transform_interpolator.hpp"
#include "ros2_utility/file_utils.hpp"
class PathGenerator : public rclcpp::Node
{
public:
    PathGenerator() : Node("path_generator")
    {
        RCLCPP_INFO(this->get_logger(), "Noether TPP initialized!");
        test();
    }
    void test()
    {
        // 1. 读取模型文件和配置文件
        pcl::PolygonMesh mesh = TPPUtils::loadMesh("/home/eric/shared_data/models/ply/after_empty.ply");
        std::string packagePath = FileUtils::getPackageDirectory("path_planner");
        std::string planMode = "plane_slicer";
        // std::string planMode = "boundary_edge";

        std::string yamlPath = packagePath + "/config/tooth_after_" + planMode + ".yaml";
        std::string txtPath = packagePath + "/config/tool_paths_" + planMode + ".txt";

        YAML::Node config = YAML::LoadFile(yamlPath);
        RCLCPP_INFO(this->get_logger(), "Mesh file and config file loaded successfully!");

        // 2. 创建模型修饰器(空)
        std::unique_ptr<noether::MeshModifier> mesh_mod = std::make_unique<IdentityMeshModifier>();

        // 3. 创建规划器
        std::unique_ptr<const noether::ToolPathPlanner> planner = TPPUtils::createToolPathPlanner(config);

        // 4. 创建路径修饰器和复合路径修饰器
        std::vector<std::unique_ptr<const noether::ToolPathModifier>> modifiers = TPPUtils::createToolPathModifiers(config);
        std::unique_ptr<noether::CompoundModifier> composite_mod = TPPUtils::createCompoundModifier(std::move(modifiers));

        // 5. 创建路径规划 pipeline 并执行规划
        noether::ToolPathPlannerPipeline pipeline(
            std::move(mesh_mod),
            std::move(planner),      // 自动向上转型为ToolPathPlanner::ConstPtr
            std::move(composite_mod) // 转换为ToolPathModifier::ConstPtr
        );
        std::vector<noether::ToolPaths> tool_paths = pipeline.plan(mesh);

        // 6. 打印刀具路径信息
        RCLCPP_INFO(this->get_logger(), "Number of tool paths: %zu", tool_paths.size());
        RCLCPP_INFO(this->get_logger(), "Number of tool path (tool paths[0]): %zu", tool_paths[0].size());
        RCLCPP_INFO(this->get_logger(), "Number of segments (tool paths[0][0]): %zu", tool_paths[0][0].size());
        RCLCPP_INFO(this->get_logger(), "Number of waypoints (tool paths[0][0][0]): %zu", tool_paths[0][0][0].size());
        RCLCPP_INFO(this->get_logger(), "Tool path planning completed!");

        // 7. 导出刀具路径
        TPPUtils::saveToolPaths(tool_paths, txtPath, ' ');

        // 8. 读取刀具路径
        std::vector<std::vector<Eigen::Matrix4d>> tool_paths_2d = TPPUtils::readToolPaths(txtPath);
        RCLCPP_INFO(this->get_logger(), "Number of tool paths (2D): %zu", tool_paths_2d.size());
        RCLCPP_INFO(this->get_logger(), "Number of waypoints (tool paths[0]): %zu", tool_paths_2d[0].size());
        for (size_t i = 0; i < tool_paths_2d[0].size(); ++i)
            std::cout << "" << i << ": " << tool_paths_2d[0][i].block(0, 3, 3, 1).transpose() << std::endl;

        // 9. 将路径单位改为米
        for (auto &tool_path : tool_paths_2d)
            for (auto &T : tool_path)
                T.block(0, 3, 3, 1) *= 0.001; // 将单位从毫米转换为米

        // 10. 将路径导入 TransformInterpolator
        interpolator_ = std::make_unique<TransformInterpolator>(tool_paths_2d, 0.1, 0.002, true);
        RCLCPP_INFO(this->get_logger(), "TransformInterpolator has %zu transforms.", interpolator_->T_traj_.size());

        // 11. 循环测试
        rclcpp::Rate rate(500); // 设置循环频率为500Hz
        int count = 0;
        // 循环执行插值计算，直到完成
        while (rclcpp::ok() && !interpolator_->isInterpolationComplete())
        {
            Eigen::Matrix4d T = interpolator_->step(); // 使用当前速度
            std::cout << "t: " << interpolator_->t_ << "  ";
            std::cout << interpolator_->getCurrentIndex() << std::endl;

            // 增加计数器
            count++;

            // 处理 ROS2 回调
            rclcpp::spin_some(this->get_node_base_interface());
            rate.sleep(); // 控制循环速率
        }
        // 输出插值完成后的计数结果
        std::cout << "Interpolation completed in " << count << " steps." << std::endl;
    }

public:
    std::unique_ptr<TransformInterpolator> interpolator_;
};

int main(int argc, char **argv)
{
    rclcpp::init(argc, argv);
    rclcpp::spin(std::make_shared<PathGenerator>());
    rclcpp::shutdown();
    return 0;
}