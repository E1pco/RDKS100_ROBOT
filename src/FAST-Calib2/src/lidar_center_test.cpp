/*
Developer: Chunran Zheng <zhengcr@connect.hku.hk>

LiDAR-only batch test entry for target annulus center extraction.
*/

#include "data_preprocess.hpp"
#include "lidar_detect.hpp"

#include <algorithm>
#include <array>
#include <cctype>
#include <fstream>
#include <pcl/io/pcd_io.h>
#include <pcl/filters/impl/extract_indices.hpp>
#include <pcl/filters/impl/filter.hpp>
#include <pcl/filters/impl/filter_indices.hpp>
#include <pcl/filters/impl/passthrough.hpp>
#include <pcl/filters/impl/voxel_grid.hpp>
#include <pcl/impl/pcl_base.hpp>
#include <pcl/segmentation/impl/extract_clusters.hpp>
#include <pcl/segmentation/impl/sac_segmentation.hpp>
#include <sensor_msgs/msg/point_cloud2.hpp>
#include <sensor_msgs/point_cloud2_iterator.hpp>
#include <rosbag2_storage/storage_options.hpp>
#include <sys/stat.h>

namespace
{
// 判断 PointCloud2 消息中是否包含指定字段
bool hasField(const sensor_msgs::msg::PointCloud2& msg, const std::string& name)
{
    for (const auto& field : msg.fields)
    {
        if (field.name == name) return true;
    }
    return false;
}

// 从 rosbag2 中读取指定 LiDAR topic，并统一转换为 Common::Point 点云
bool loadCloudFromBag(const std::string& bag_path,
                      const std::string& lidar_topic,
                      pcl::PointCloud<Common::Point>::Ptr cloud,
                      LiDARType& detected_type)
{
    cloud->clear();
    detected_type = LiDARType::Unknown;

    rosbag2_cpp::readers::SequentialReader reader;
    rosbag2_storage::StorageOptions storage_options;
    storage_options.uri = bag_path;
    storage_options.storage_id = "sqlite3";

    rosbag2_cpp::ConverterOptions converter_options;
    converter_options.input_serialization_format = "cdr";
    converter_options.output_serialization_format = "cdr";

    try
    {
        reader.open(storage_options, converter_options);
    }
    catch (const std::exception& e)
    {
        RCLCPP_ERROR_STREAM(rclcpp::get_logger("LiDAR Test"), "Failed to open bag " << bag_path << ": " << e.what());
        return false;
    }

    size_t message_count = 0;

    while (reader.has_next())
    {
        auto serialized_message = reader.read_next();
        std::string topic_name = serialized_message->topic_name;

        // Filter by topic
        if (topic_name != lidar_topic) {
            continue;
        }

        // Convert rosbag2 serialized message to rclcpp serialized message
        rclcpp::SerializedMessage rcl_serialized_msg;
        rcl_serialized_msg.reserve(serialized_message->serialized_data->buffer_length);
        memcpy(rcl_serialized_msg.get_rcl_serialized_message().buffer,
               serialized_message->serialized_data->buffer,
               serialized_message->serialized_data->buffer_length);
        rcl_serialized_msg.get_rcl_serialized_message().buffer_length =
            serialized_message->serialized_data->buffer_length;

        // Try to deserialize as livox CustomMsg
        try {
            auto livox_msg = std::make_shared<livox_ros_driver2::msg::CustomMsg>();
            rclcpp::Serialization<livox_ros_driver2::msg::CustomMsg> serializer;
            serializer.deserialize_message(&rcl_serialized_msg, livox_msg.get());

            detected_type = LiDARType::Solid;
            cloud->reserve(cloud->size() + livox_msg->point_num);
            for (uint32_t i = 0; i < livox_msg->point_num; ++i)
            {
                Common::Point p;
                p.x = livox_msg->points[i].x;
                p.y = livox_msg->points[i].y;
                p.z = livox_msg->points[i].z;
                p.intensity = static_cast<float>(livox_msg->points[i].reflectivity);
                p.ring = static_cast<std::uint16_t>(livox_msg->points[i].line);
                cloud->push_back(p);
            }
            ++message_count;
            continue;
        } catch (...) {
            // Not a livox message, try PointCloud2
        }

        // Try to deserialize as PointCloud2
        try {
            auto pcl_msg = std::make_shared<sensor_msgs::msg::PointCloud2>();
            rclcpp::Serialization<sensor_msgs::msg::PointCloud2> serializer;
            serializer.deserialize_message(&rcl_serialized_msg, pcl_msg.get());

            const bool has_ring = hasField(*pcl_msg, "ring");
            const bool has_intensity = hasField(*pcl_msg, "intensity");
            const bool has_reflectivity = hasField(*pcl_msg, "reflectivity");

            if (detected_type == LiDARType::Unknown)
            {
                detected_type = has_ring ? LiDARType::Mech : LiDARType::Solid;
            }

            sensor_msgs::PointCloud2ConstIterator<float> it_x(*pcl_msg, "x");
            sensor_msgs::PointCloud2ConstIterator<float> it_y(*pcl_msg, "y");
            sensor_msgs::PointCloud2ConstIterator<float> it_z(*pcl_msg, "z");

            std::unique_ptr<sensor_msgs::PointCloud2ConstIterator<std::uint16_t>> it_ring_ptr;
            if (has_ring)
            {
                it_ring_ptr.reset(new sensor_msgs::PointCloud2ConstIterator<std::uint16_t>(*pcl_msg, "ring"));
            }

            std::unique_ptr<sensor_msgs::PointCloud2ConstIterator<float>> it_intensity_ptr;
            if (has_intensity)
            {
                it_intensity_ptr.reset(new sensor_msgs::PointCloud2ConstIterator<float>(*pcl_msg, "intensity"));
            }
            else if (has_reflectivity)
            {
                it_intensity_ptr.reset(new sensor_msgs::PointCloud2ConstIterator<float>(*pcl_msg, "reflectivity"));
            }

            const size_t n = static_cast<size_t>(pcl_msg->width) * pcl_msg->height;
            cloud->reserve(cloud->size() + n);
            for (size_t i = 0; i < n; ++i, ++it_x, ++it_y, ++it_z)
            {
                Common::Point p;
                p.x = *it_x;
                p.y = *it_y;
                p.z = *it_z;
                p.ring = 0xFFFF;
                p.intensity = 0.0f;

                if (it_ring_ptr)
                {
                    p.ring = **it_ring_ptr;
                    ++(*it_ring_ptr);
                }
                if (it_intensity_ptr)
                {
                    p.intensity = **it_intensity_ptr;
                    ++(*it_intensity_ptr);
                }

                cloud->push_back(p);
            }
            ++message_count;
        } catch (...) {
            // Not a PointCloud2 either
        }
    }

    RCLCPP_INFO(rclcpp::get_logger("LiDAR Test"), "Loaded %zu messages, %zu points from %s",
             message_count, cloud->size(), bag_path.c_str());
    return message_count > 0 && !cloud->empty();
}

// 将 LiDAR 类型枚举转换为日志可读字符串
std::string lidarTypeName(LiDARType type)
{
    switch (type)
    {
        case LiDARType::Solid: return "solid";
        case LiDARType::Mech: return "mech";
        default: return "unknown";
    }
}

// 去掉路径末尾多余的斜杠
std::string trimTrailingSlash(std::string path)
{
    while (!path.empty() && path.back() == '/')
    {
        path.pop_back();
    }
    return path;
}

// 获取路径最后一级文件名或目录名
std::string pathBaseName(const std::string& path)
{
    const std::string clean_path = trimTrailingSlash(path);
    const size_t pos = clean_path.find_last_of('/');
    return pos == std::string::npos ? clean_path : clean_path.substr(pos + 1);
}

// 获取路径父目录的最后一级名称
std::string pathParentName(const std::string& path)
{
    const std::string clean_path = trimTrailingSlash(path);
    const size_t last_slash = clean_path.find_last_of('/');
    if (last_slash == std::string::npos) return "";
    return pathBaseName(clean_path.substr(0, last_slash));
}

// 去掉文件名扩展名
std::string stripExtension(const std::string& filename)
{
    const size_t pos = filename.find_last_of('.');
    return pos == std::string::npos ? filename : filename.substr(0, pos);
}

// 将任意字符串转换为安全的文件名前缀片段
std::string sanitizeFilePart(std::string value)
{
    for (char& c : value)
    {
        if (!(std::isalnum(static_cast<unsigned char>(c)) || c == '-' || c == '_'))
        {
            c = '_';
        }
    }
    return value;
}

// 确保输出目录存在
void ensureDirectory(const std::string& path)
{
    if (path.empty()) return;
    mkdir(path.c_str(), 0755);
}

// 解析测试输出目录，兼容未展开的 ROS launch 变量
std::string resolveOutputDirectory(const Params& params)
{
    std::string output_dir = params.output_path;
    if (output_dir.empty() || output_dir.find("$(") != std::string::npos)
    {
        output_dir = "/home/chunran/02_calib_ws/src/FAST-Calib/output";
    }
    output_dir = trimTrailingSlash(output_dir);
    ensureDirectory(output_dir);
    return output_dir;
}

// 根据 bag 所在目录和文件名生成输出文件前缀
std::string outputPrefixForBag(const std::string& bag_path)
{
    return sanitizeFilePart(pathParentName(bag_path) + "_" +
                            stripExtension(pathBaseName(bag_path)));
}

// 构造带 RGB 颜色的 PCL 点
pcl::PointXYZRGB makeRgbPoint(float x, float y, float z, std::uint8_t r, std::uint8_t g, std::uint8_t b)
{
    pcl::PointXYZRGB p;
    p.x = x;
    p.y = y;
    p.z = z;
    p.r = r;
    p.g = g;
    p.b = b;
    return p;
}

// 在线性颜色表中按比例插值
std::array<std::uint8_t, 3> lerpColor(const std::array<std::uint8_t, 3>& a,
                                      const std::array<std::uint8_t, 3>& b,
                                      float t)
{
    std::array<std::uint8_t, 3> out;
    for (int i = 0; i < 3; ++i)
    {
        out[i] = static_cast<std::uint8_t>(std::round(a[i] + t * (b[i] - a[i])));
    }
    return out;
}

// 按索引生成均匀分布的 RGB 颜色
std::array<std::uint8_t, 3> colorForIndex(int index, int total)
{
    static const std::vector<std::array<std::uint8_t, 3>> palette = {{
        {255, 0, 0}, {0, 255, 0}, {0, 0, 255}, {255, 255, 0},
        {255, 0, 255}, {0, 255, 255}, {128, 0, 0}, {0, 128, 0},
        {0, 0, 128}, {128, 128, 0}, {128, 0, 128}, {0, 128, 128}
    }};
    if (total <= 0) return palette[0];
    return palette[static_cast<size_t>(index) % palette.size()];
}

// 保存调试用彩色点云，不同点类别用不同颜色标注
void saveDebugCloud(const pcl::PointCloud<Common::Point>::Ptr& plane_cloud,
                    const pcl::PointCloud<Common::Point>::Ptr& annulus_cloud,
                    const pcl::PointCloud<pcl::PointXYZ>::Ptr& boundary_cloud,
                    const pcl::PointCloud<pcl::PointXYZ>::Ptr& centers,
                    const Params& params,
                    const std::string& bag_path)
{
    pcl::PointCloud<pcl::PointXYZRGB>::Ptr debug_cloud(new pcl::PointCloud<pcl::PointXYZRGB>);
    debug_cloud->reserve(plane_cloud->size() + annulus_cloud->size() +
                         boundary_cloud->size() + centers->size());

    for (const auto& p : *plane_cloud)
    {
        debug_cloud->push_back(makeRgbPoint(p.x, p.y, p.z, 128, 128, 128));
    }

    for (const auto& p : *annulus_cloud)
    {
        debug_cloud->push_back(makeRgbPoint(p.x, p.y, p.z, 255, 0, 0));
    }

    if (boundary_cloud)
    {
        for (const auto& p : *boundary_cloud)
        {
            debug_cloud->push_back(makeRgbPoint(p.x, p.y, p.z, 0, 255, 255));
        }
    }

    for (size_t i = 0; i < centers->size(); ++i)
    {
        const auto& c = centers->points[i];
        auto color = colorForIndex(static_cast<int>(i), static_cast<int>(centers->size()));
        for (int s = 0; s < 50; ++s)
        {
            const float eps = 0.002f * static_cast<float>(s);
            debug_cloud->push_back(makeRgbPoint(c.x + eps, c.y, c.z, color[0], color[1], color[2]));
            debug_cloud->push_back(makeRgbPoint(c.x - eps, c.y, c.z, color[0], color[1], color[2]));
            debug_cloud->push_back(makeRgbPoint(c.x, c.y + eps, c.z, color[0], color[1], color[2]));
            debug_cloud->push_back(makeRgbPoint(c.x, c.y - eps, c.z, color[0], color[1], color[2]));
        }
    }

    const std::string output_dir = resolveOutputDirectory(params);
    const std::string prefix = output_dir + "/" + outputPrefixForBag(bag_path);
    const std::string output_path = prefix + "_debug_cloud.pcd";
    if (pcl::io::savePCDFileASCII(output_path, *debug_cloud) == 0)
    {
        std::cout << BOLDYELLOW << "[LiDAR Test] Saved debug cloud to " << BOLDWHITE << output_path << RESET << std::endl;
    }
    else
    {
        RCLCPP_ERROR_STREAM(rclcpp::get_logger("LiDAR Test"), "Failed to save debug cloud to " << output_path);
    }
}

// 保存圆心坐标到文本文件
void saveCenterCoordinates(const pcl::PointCloud<pcl::PointXYZ>::Ptr& centers,
                           const Params& params,
                           const std::string& bag_path)
{
    const std::string output_dir = resolveOutputDirectory(params);
    const std::string prefix = output_dir + "/" + outputPrefixForBag(bag_path);
    const std::string output_path = prefix + "_centers.txt";

    std::ofstream ofs(output_path);
    if (!ofs.is_open())
    {
        RCLCPP_ERROR_STREAM(rclcpp::get_logger("LiDAR Test"), "Failed to save center coordinates to " << output_path);
        return;
    }

    ofs << std::fixed << std::setprecision(6);
    for (size_t i = 0; i < centers->size(); ++i)
    {
        const auto& p = centers->points[i];
        ofs << "center_" << i << ": " << p.x << ", " << p.y << ", " << p.z << "\n";
    }
    ofs.close();
    std::cout << BOLDYELLOW << "[LiDAR Test] Saved center coordinates to " << BOLDWHITE << output_path << RESET << std::endl;
}

// 计算并打印拟合圆的半径质量统计
void printRadiusQuality(const std::vector<std::vector<double>>& radii_by_center,
                        double target_radius)
{
    for (size_t i = 0; i < radii_by_center.size(); ++i)
    {
        const auto& radii = radii_by_center[i];
        if (radii.empty())
        {
            std::cout << "[Radius Quality] Center " << i << ": no points" << std::endl;
            continue;
        }

        double sum = 0.0;
        double min_r = std::numeric_limits<double>::max();
        double max_r = std::numeric_limits<double>::lowest();
        for (double r : radii)
        {
            sum += r;
            min_r = std::min(min_r, r);
            max_r = std::max(max_r, r);
        }
        double mean = sum / radii.size();

        double sq_sum = 0.0;
        for (double r : radii)
        {
            sq_sum += (r - mean) * (r - mean);
        }
        double stddev = std::sqrt(sq_sum / radii.size());

        std::cout << "[Radius Quality] Center " << i << ": "
                  << "count=" << radii.size()
                  << ", mean=" << std::fixed << std::setprecision(4) << mean
                  << ", std=" << stddev
                  << ", min=" << min_r
                  << ", max=" << max_r
                  << ", target=" << target_radius
                  << ", error=" << (mean - target_radius) << std::endl;
    }
}

// 计算并打印固态 LiDAR 的半径质量
void validateRadiusQuality(const pcl::PointCloud<pcl::PointXYZ>::Ptr& edge_cloud,
                           const pcl::PointCloud<pcl::PointXYZ>::Ptr& centers_z0,
                           const Params& params,
                           LiDARType type)
{
    if (!edge_cloud || edge_cloud->empty() || !centers_z0 || centers_z0->size() != TARGET_NUM_CIRCLES)
    {
        return;
    }

    const double target_radius = params.circle_radius;
    const double gate = target_radius * 0.3;

    std::vector<std::vector<double>> radii_by_center(TARGET_NUM_CIRCLES);
    for (const auto& p : *edge_cloud)
    {
        if (!std::isfinite(p.x) || !std::isfinite(p.y)) continue;

        int best_center = -1;
        double best_residual = std::numeric_limits<double>::max();
        double best_radius = 0.0;
        for (int i = 0; i < TARGET_NUM_CIRCLES; ++i)
        {
            const auto& center = centers_z0->points[i];
            const double dx = static_cast<double>(p.x) - static_cast<double>(center.x);
            const double dy = static_cast<double>(p.y) - static_cast<double>(center.y);
            const double radius = std::sqrt(dx * dx + dy * dy);
            const double residual = std::fabs(radius - target_radius);
            if (residual < best_residual)
            {
                best_center = i;
                best_residual = residual;
                best_radius = radius;
            }
        }
        if (best_center >= 0 && best_residual <= gate)
        {
            radii_by_center[best_center].push_back(best_radius);
        }
    }
    printRadiusQuality(radii_by_center, target_radius);
}

}  // namespace

// LiDAR 圆心提取批量测试入口
int main(int argc, char** argv)
{
    rclcpp::init(argc, argv);
    auto node = rclcpp::Node::make_shared("lidar_center_test");

    if (argc < 3)
    {
        std::cerr << "Usage: ros2 run fast_calib lidar_center_test <bag_path> <lidar_topic> [auto|solid|mech]" << std::endl;
        return 2;
    }

    Params params = loadParameters(node);
    params.bag_path = argv[1];
    params.lidar_topic = argv[2];

    const std::string mode = argc >= 4 ? argv[3] : "auto";

    pcl::PointCloud<Common::Point>::Ptr cloud(new pcl::PointCloud<Common::Point>);
    LiDARType detected_type = LiDARType::Unknown;
    if (!loadCloudFromBag(params.bag_path, params.lidar_topic, cloud, detected_type))
    {
        return 1;
    }

    LiDARType run_type = detected_type;
    if (mode == "solid") run_type = LiDARType::Solid;
    if (mode == "mech") run_type = LiDARType::Mech;

    std::cout << "[LiDAR Test] Bag: " << params.bag_path << std::endl;
    std::cout << "[LiDAR Test] Topic: " << params.lidar_topic << std::endl;
    std::cout << "[LiDAR Test] Detected type: " << lidarTypeName(detected_type)
              << ", run type: " << lidarTypeName(run_type) << std::endl;

    LidarDetect lidar_detect(node, params);
    pcl::PointCloud<pcl::PointXYZ>::Ptr raw_centers(new pcl::PointCloud<pcl::PointXYZ>);

    if (run_type == LiDARType::Solid)
    {
        lidar_detect.detect_solid_lidar(cloud, raw_centers);
    }
    else if (run_type == LiDARType::Mech)
    {
        lidar_detect.detect_mech_lidar(cloud, raw_centers);
    }
    else
    {
        RCLCPP_ERROR(node->get_logger(), "[LiDAR Test] Unknown LiDAR type.");
        return 1;
    }

    pcl::PointCloud<pcl::PointXYZ>::Ptr centers(new pcl::PointCloud<pcl::PointXYZ>);
    sortPatternCenters(raw_centers, centers, "lidar");

    std::cout << "[LiDAR Test] Raw center count: " << raw_centers->size() << std::endl;
    std::cout << "[LiDAR Test] Sorted center count: " << centers->size() << std::endl;
    for (size_t i = 0; i < centers->size(); ++i)
    {
        const auto& p = centers->points[i];
        std::cout << "[LiDAR Test] Center " << i << ": "
                  << std::fixed << std::setprecision(6)
                  << p.x << ", " << p.y << ", " << p.z << std::endl;
    }
    validateTargetGeometry(centers, params.delta_width_circles, params.delta_height_circles, "LiDAR");
    validateRadiusQuality(lidar_detect.getEdgeCloud(), lidar_detect.getCenterZ0Cloud(), params, run_type);
    saveCenterCoordinates(centers, params, params.bag_path);
    saveDebugCloud(lidar_detect.getPlaneCloud(), lidar_detect.getAnnulusOriginalCloud(),
                   lidar_detect.getBoundaryOriginalCloud(),
                   centers, params, params.bag_path);

    rclcpp::shutdown();
    return 0;
}
