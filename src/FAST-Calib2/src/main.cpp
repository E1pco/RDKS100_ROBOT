/*
Developer: Chunran Zheng <zhengcr@connect.hku.hk>

This file is subject to the terms and conditions outlined in the 'LICENSE' file,
which is included as part of this source code package.
*/

#include "qr_detect.hpp"
#include "lidar_detect.hpp"
#include "data_preprocess.hpp"

int main(int argc, char **argv)
{
    rclcpp::init(argc, argv);
    auto node = rclcpp::Node::make_shared("fast_calib");

    // 读取参数
    Params params = loadParameters(node);

    // 初始化 QR 检测和 LiDAR 检测
    QRDetectPtr qrDetectPtr;
    qrDetectPtr.reset(new QRDetect(node, params));

    LidarDetectPtr lidarDetectPtr;
    lidarDetectPtr.reset(new LidarDetect(node, params));

    DataPreprocessPtr dataPreprocessPtr;
    dataPreprocessPtr.reset(new DataPreprocess(params));

    // 读取图像和点云
    cv::Mat img_input = dataPreprocessPtr->img_input_;
    pcl::PointCloud<Common::Point>::Ptr cloud_input = dataPreprocessPtr->cloud_input_;

    auto input_cloud_pub = node->create_publisher<sensor_msgs::msg::PointCloud2>("input_cloud", 1);
    auto colored_cloud_pub = node->create_publisher<sensor_msgs::msg::PointCloud2>("colored_cloud", 1);
    auto aligned_lidar_centers_pub = node->create_publisher<sensor_msgs::msg::PointCloud2>("aligned_lidar_centers", 1);

    auto publish_debug_clouds = [&]() {
        sensor_msgs::msg::PointCloud2 input_cloud_msg;
        pcl::toROSMsg(*cloud_input, input_cloud_msg);
        input_cloud_msg.header.stamp = node->now();
        input_cloud_msg.header.frame_id = "map";
        input_cloud_pub->publish(input_cloud_msg);

        sensor_msgs::msg::PointCloud2 filtered_cloud_msg;
        pcl::toROSMsg(*lidarDetectPtr->getFilteredCloud(), filtered_cloud_msg);
        filtered_cloud_msg.header = input_cloud_msg.header;
        lidarDetectPtr->filtered_pub_->publish(filtered_cloud_msg);

        sensor_msgs::msg::PointCloud2 plane_cloud_msg;
        pcl::toROSMsg(*lidarDetectPtr->getPlaneCloud(), plane_cloud_msg);
        plane_cloud_msg.header = input_cloud_msg.header;
        lidarDetectPtr->plane_pub_->publish(plane_cloud_msg);

        sensor_msgs::msg::PointCloud2 annulus_cloud_msg;
        pcl::toROSMsg(*lidarDetectPtr->getAnnulusOriginalCloud(), annulus_cloud_msg);
        annulus_cloud_msg.header = input_cloud_msg.header;
        lidarDetectPtr->annulus_pub_->publish(annulus_cloud_msg);

        sensor_msgs::msg::PointCloud2 boundary_cloud_msg;
        pcl::toROSMsg(*lidarDetectPtr->getBoundaryOriginalCloud(), boundary_cloud_msg);
        boundary_cloud_msg.header = input_cloud_msg.header;
        lidarDetectPtr->boundary_pub_->publish(boundary_cloud_msg);

        sensor_msgs::msg::PointCloud2 aligned_cloud_msg;
        pcl::toROSMsg(*lidarDetectPtr->getAlignedCloud(), aligned_cloud_msg);
        aligned_cloud_msg.header = input_cloud_msg.header;
        lidarDetectPtr->aligned_pub_->publish(aligned_cloud_msg);

        sensor_msgs::msg::PointCloud2 edge_cloud_msg;
        pcl::toROSMsg(*lidarDetectPtr->getEdgeCloud(), edge_cloud_msg);
        edge_cloud_msg.header = input_cloud_msg.header;
        lidarDetectPtr->edge_pub_->publish(edge_cloud_msg);

        sensor_msgs::msg::PointCloud2 center_z0_msg;
        pcl::toROSMsg(*lidarDetectPtr->getCenterZ0Cloud(), center_z0_msg);
        center_z0_msg.header = input_cloud_msg.header;
        lidarDetectPtr->center_z0_pub_->publish(center_z0_msg);
    };

    // 检测 QR 码
    PointCloud<PointXYZ>::Ptr qr_center_cloud(new PointCloud<PointXYZ>);
    qr_center_cloud->reserve(4);
    qrDetectPtr->detect_qr(img_input, qr_center_cloud);

    // 检测 LiDAR 数据
    PointCloud<PointXYZ>::Ptr lidar_center_cloud(new PointCloud<PointXYZ>);
    lidar_center_cloud->reserve(4);

    switch (dataPreprocessPtr->lidar_type_)
    {
        case LiDARType::Solid:
            lidarDetectPtr->detect_solid_lidar(cloud_input, lidar_center_cloud);
            break;

        case LiDARType::Mech:
            lidarDetectPtr->detect_mech_lidar(cloud_input, lidar_center_cloud);
            break;

        default:
            std::cerr << BOLDYELLOW
                    << "[Main] Unknown LiDAR type."
                    << RESET << std::endl;
            break;
    }

    // 对 QR 和 LiDAR 检测到的圆心进行排序
    PointCloud<PointXYZ>::Ptr qr_centers(new PointCloud<PointXYZ>);
    PointCloud<PointXYZ>::Ptr lidar_centers(new PointCloud<PointXYZ>);
    sortPatternCenters(qr_center_cloud, qr_centers, "camera");
    sortPatternCenters(lidar_center_cloud, lidar_centers, "lidar");

    validateTargetGeometry(qr_centers, params.delta_width_circles, params.delta_height_circles, "QR");
    validateTargetGeometry(lidar_centers, params.delta_width_circles, params.delta_height_circles, "LiDAR");

    // 保存中间结果：排序后的 LiDAR 圆心和 QR 圆心
    saveTargetHoleCenters(lidar_centers, qr_centers, params);

    if (qr_centers->size() != TARGET_NUM_CIRCLES ||
        lidar_centers->size() != TARGET_NUM_CIRCLES)
    {
        RCLCPP_ERROR(node->get_logger(),
                     "Calibration aborted: need 4 QR centers and 4 LiDAR centers, got %zu QR and %zu LiDAR.",
                     qr_centers->size(), lidar_centers->size());
        RCLCPP_ERROR(node->get_logger(),
                     "Keeping debug publishers alive. Check /input_cloud first, then /filtered_cloud.");

        rclcpp::Rate debug_rate(1);
        while (rclcpp::ok())
        {
            if (DEBUG)
            {
                publish_debug_clouds();

                sensor_msgs::msg::PointCloud2 qr_centers_msg;
                pcl::toROSMsg(*qr_centers, qr_centers_msg);
                qr_centers_msg.header.stamp = node->now();
                qr_centers_msg.header.frame_id = "map";
                qrDetectPtr->qr_pub_->publish(qr_centers_msg);

                sensor_msgs::msg::PointCloud2 lidar_centers_msg;
                pcl::toROSMsg(*lidar_centers, lidar_centers_msg);
                lidar_centers_msg.header = qr_centers_msg.header;
                lidarDetectPtr->center_pub_->publish(lidar_centers_msg);
            }
            rclcpp::spin_some(node);
            debug_rate.sleep();
        }

        rclcpp::shutdown();
        return 1;
    }

    // 计算外参
    Eigen::Matrix4f transformation = Eigen::Matrix4f::Identity();
    pcl::registration::TransformationEstimationSVD<pcl::PointXYZ, pcl::PointXYZ> svd;
    svd.estimateRigidTransformation(*lidar_centers, *qr_centers, transformation);

    // 将 LiDAR 点云转换到 QR 码坐标系
    pcl::PointCloud<pcl::PointXYZ>::Ptr aligned_lidar_centers(new pcl::PointCloud<pcl::PointXYZ>);
    aligned_lidar_centers->reserve(lidar_centers->size());
    alignPointCloud(lidar_centers, aligned_lidar_centers, transformation);

    double rmse = computeRMSE(qr_centers, aligned_lidar_centers);
    if (rmse > 0)
    {
      std::cout << BOLDYELLOW << "[Result] RMSE: " << BOLDRED << std::fixed << std::setprecision(4)
      << rmse << " m" << RESET << std::endl;
    }

    std::cout << BOLDYELLOW << "[Result] Single-scene calibration: extrinsic parameters T_cam_lidar = " << RESET << std::endl;
    std::cout << BOLDCYAN << std::fixed << std::setprecision(6) << transformation << RESET << std::endl;

    pcl::PointCloud<pcl::PointXYZRGB>::Ptr colored_cloud(new pcl::PointCloud<pcl::PointXYZRGB>);
    projectPointCloudToImage(cloud_input, transformation, qrDetectPtr->cameraMatrix_, qrDetectPtr->distCoeffs_, img_input, colored_cloud);

    saveCalibrationResults(params, transformation, colored_cloud, qrDetectPtr->imageCopy_);

    // 主循环
    rclcpp::Rate rate(1);
    while (rclcpp::ok())
    {
      if (DEBUG)
      {
        publish_debug_clouds();

        // 发布 QR 检测结果
        sensor_msgs::msg::PointCloud2 qr_centers_msg;
        pcl::toROSMsg(*qr_centers, qr_centers_msg);
        qr_centers_msg.header.stamp = node->now();
        qr_centers_msg.header.frame_id = "map";
        qrDetectPtr->qr_pub_->publish(qr_centers_msg);

        // 发布 LiDAR 检测结果
        sensor_msgs::msg::PointCloud2 lidar_centers_msg;
        pcl::toROSMsg(*lidar_centers, lidar_centers_msg);
        lidar_centers_msg.header = qr_centers_msg.header;
        lidarDetectPtr->center_pub_->publish(lidar_centers_msg);

        // 发布外参变换后的LiDAR点云
        sensor_msgs::msg::PointCloud2 aligned_lidar_centers_msg;
        pcl::toROSMsg(*aligned_lidar_centers, aligned_lidar_centers_msg);
        aligned_lidar_centers_msg.header = qr_centers_msg.header;
        aligned_lidar_centers_pub->publish(aligned_lidar_centers_msg);

        // 发布彩色点云
        sensor_msgs::msg::PointCloud2 colored_cloud_msg;
        pcl::toROSMsg(*colored_cloud, colored_cloud_msg);
        colored_cloud_msg.header = qr_centers_msg.header;
        colored_cloud_pub->publish(colored_cloud_msg);
      }
      rclcpp::spin_some(node);
      rate.sleep();
    }

    rclcpp::shutdown();
    return 0;
}
