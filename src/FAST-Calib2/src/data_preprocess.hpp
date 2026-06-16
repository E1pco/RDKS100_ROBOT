/*
Developer: Chunran Zheng <zhengcr@connect.hku.hk>

This file is subject to the terms and conditions outlined in the 'LICENSE' file,
which is included as part of this source code package.
*/

#ifndef DATA_PREPROCESS_HPP
#define DATA_PREPROCESS_HPP

#include <Eigen/Core>
#include <pcl/io/pcd_io.h>
#include <pcl/point_cloud.h>
#include <pcl/point_types.h>
#include <pcl_conversions/pcl_conversions.h>
#include <rclcpp/rclcpp.hpp>
#include <rclcpp/serialization.hpp>
#include <sensor_msgs/msg/point_cloud2.hpp>
#include <sensor_msgs/point_cloud2_iterator.hpp>
#include <rosbag2_cpp/readers/sequential_reader.hpp>
#include <rosbag2_storage/storage_options.hpp>
#include <rosbag2_cpp/converter_options.hpp>
#include <fstream>
#include <unordered_map>
#include "common_lib.h"

// Include livox_ros_driver2 message headers
#include <livox_ros_driver2/msg/custom_msg.hpp>
#include <livox_ros_driver2/msg/custom_point.hpp>

using namespace std;

enum class LiDARType : int {
    Unknown = 0,
    Solid   = 1,   // 固态（如 Livox）
    Mech    = 2    // 机械式多线
};

class DataPreprocess
{
public:
    // 改成带线号的点云
    pcl::PointCloud<Common::Point>::Ptr cloud_input_;
    cv::Mat img_input_;
    LiDARType lidar_type_{LiDARType::Unknown};
    LiDARType lidarType() const { return lidar_type_; }

    DataPreprocess(Params &params)
        : cloud_input_(new pcl::PointCloud<Common::Point>)
    {
        string bag_path   = params.bag_path;
        string image_path = params.image_path;
        string lidar_topic = params.lidar_topic;

        // 读图像
        img_input_ = cv::imread(image_path, cv::IMREAD_UNCHANGED);
        if (img_input_.empty())
        {
            std::string msg = "Loading the image " + image_path + " failed";
            RCLCPP_ERROR_STREAM(rclcpp::get_logger("DataPreprocess"), msg.c_str());
            return;
        }

        // 先检查包是否存在
        std::fstream file_;
        file_.open(bag_path, ios::in);
        if (!file_)
        {
            std::string msg = "Loading the rosbag " + bag_path + " failed";
            RCLCPP_ERROR_STREAM(rclcpp::get_logger("DataPreprocess"), msg.c_str());
            return;
        }
        RCLCPP_INFO(rclcpp::get_logger("DataPreprocess"), "Loading the rosbag %s", bag_path.c_str());

        // Open rosbag2
        rosbag2_cpp::readers::SequentialReader reader;
        rosbag2_storage::StorageOptions storage_options;
        storage_options.uri = bag_path;
        storage_options.storage_id = "sqlite3";

        rosbag2_cpp::ConverterOptions converter_options;
        converter_options.input_serialization_format = "cdr";
        converter_options.output_serialization_format = "cdr";

        try {
            reader.open(storage_options, converter_options);
        } catch (const std::exception& e) {
            RCLCPP_ERROR_STREAM(rclcpp::get_logger("DataPreprocess"), "LOADING BAG FAILED: " << e.what());
            return;
        }

        std::unordered_map<std::string, std::string> topic_types;
        for (const auto& topic_metadata : reader.get_all_topics_and_types()) {
            topic_types[topic_metadata.name] = topic_metadata.type;
        }

        auto topic_type_it = topic_types.find(lidar_topic);
        if (topic_type_it == topic_types.end()) {
            RCLCPP_ERROR_STREAM(rclcpp::get_logger("DataPreprocess"),
                                "LiDAR topic " << lidar_topic << " was not found in the rosbag.");
            return;
        }
        const std::string lidar_topic_type = topic_type_it->second;
        RCLCPP_INFO(rclcpp::get_logger("DataPreprocess"),
                    "LiDAR topic %s type: %s", lidar_topic.c_str(), lidar_topic_type.c_str());

        // Read messages
        while (reader.has_next()) {
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

            if (lidar_topic_type == "livox_ros_driver2/msg/CustomMsg") {
                auto livox_msg = std::make_shared<livox_ros_driver2::msg::CustomMsg>();
                rclcpp::Serialization<livox_ros_driver2::msg::CustomMsg> serializer;
                serializer.deserialize_message(&rcl_serialized_msg, livox_msg.get());

                lidar_type_ = LiDARType::Solid;
                cloud_input_->reserve(cloud_input_->size() + livox_msg->point_num);
                for (uint32_t i = 0; i < livox_msg->point_num; ++i)
                {
                    Common::Point p;
                    p.x = livox_msg->points[i].x;
                    p.y = livox_msg->points[i].y;
                    p.z = livox_msg->points[i].z;
                    p.intensity = static_cast<float>(livox_msg->points[i].reflectivity);
                    p.ring = static_cast<std::uint16_t>(livox_msg->points[i].line);
                    cloud_input_->push_back(p);
                }
                continue;
            }

            if (lidar_topic_type == "sensor_msgs/msg/PointCloud2") {
                auto pcl_msg = std::make_shared<sensor_msgs::msg::PointCloud2>();
                rclcpp::Serialization<sensor_msgs::msg::PointCloud2> serializer;
                serializer.deserialize_message(&rcl_serialized_msg, pcl_msg.get());

                // 优先判断是否有 ring/line 字段
                bool has_ring = false;
                bool has_line = false;
                bool has_intensity = false;
                bool has_reflectivity = false;
                for (const auto &f : pcl_msg->fields)
                {
                    if (f.name == "ring") has_ring = true;
                    if (f.name == "line") has_line = true;
                    if (f.name == "intensity") has_intensity = true;
                    if (f.name == "reflectivity") has_reflectivity = true;
                }

                // 使用 iterator 安全读取
                sensor_msgs::PointCloud2ConstIterator<float> it_x(*pcl_msg, "x");
                sensor_msgs::PointCloud2ConstIterator<float> it_y(*pcl_msg, "y");
                sensor_msgs::PointCloud2ConstIterator<float> it_z(*pcl_msg, "z");

                // ring/line 可能不存在：不存在时用 0xFFFF 表示未知
                std::unique_ptr<sensor_msgs::PointCloud2ConstIterator<std::uint16_t>> it_ring_ptr;
                std::unique_ptr<sensor_msgs::PointCloud2ConstIterator<float>> it_intensity_ptr;
                if (has_ring)
                {
                    it_ring_ptr.reset(new sensor_msgs::PointCloud2ConstIterator<std::uint16_t>(*pcl_msg, "ring"));
                    lidar_type_ = LiDARType::Mech;
                }
                else if (has_line)
                {
                    it_ring_ptr.reset(new sensor_msgs::PointCloud2ConstIterator<std::uint16_t>(*pcl_msg, "line"));
                    lidar_type_ = LiDARType::Mech;
                }
                else
                {
                    lidar_type_ = LiDARType::Solid;
                }
                if (has_intensity)
                {
                    it_intensity_ptr.reset(new sensor_msgs::PointCloud2ConstIterator<float>(*pcl_msg, "intensity"));
                }
                else if (has_reflectivity)
                {
                    it_intensity_ptr.reset(new sensor_msgs::PointCloud2ConstIterator<float>(*pcl_msg, "reflectivity"));
                }

                const size_t n = static_cast<size_t>(pcl_msg->width) * pcl_msg->height;
                cloud_input_->reserve(cloud_input_->size() + n);

                for (size_t i = 0; i < n; ++i, ++it_x, ++it_y, ++it_z)
                {
                    Common::Point p;
                    p.x = *it_x;
                    p.y = *it_y;
                    p.z = *it_z;

                    if (it_ring_ptr)
                    {
                        p.ring = **it_ring_ptr;
                        ++(*it_ring_ptr);
                    }
                    else
                    {
                        p.ring = 0xFFFF; // 未知线号
                    }
                    if (it_intensity_ptr)
                    {
                        p.intensity = **it_intensity_ptr;
                        ++(*it_intensity_ptr);
                    }
                    else
                    {
                        p.intensity = 0.0f;
                    }

                    cloud_input_->push_back(p);
                }
                continue;
            }

            RCLCPP_ERROR_STREAM(rclcpp::get_logger("DataPreprocess"),
                                "Unsupported LiDAR topic type: " << lidar_topic_type);
            return;
        }

        RCLCPP_INFO(rclcpp::get_logger("DataPreprocess"), "Loaded %zu points from the rosbag.", cloud_input_->size());
    }
};

typedef std::shared_ptr<DataPreprocess> DataPreprocessPtr;

#endif // DATA_PREPROCESS_HPP
