/**
 * @file
 * @brief Example node to publish point clouds and imu topics
 */

#include <ros/console.h>
#include <ros/ros.h>
#include <ros/service.h>
#include <sensor_msgs/Imu.h>
#include <sensor_msgs/PointCloud2.h>
#include <tf2_ros/static_transform_broadcaster.h>

#include <algorithm>
#include <chrono>
#include <memory>

#include "ouster/lidar_scan.h"
#include "ouster/types.h"
#include "ouster_ros/OSConfigSrv.h"
#include "ouster_ros/PacketMsg.h"
#include "ouster_ros/ros.h"

using PacketMsg = ouster_ros::PacketMsg;
using Cloud = ouster_ros::Cloud;
using Point = ouster_ros::Point;
namespace sensor = ouster::sensor;
bool g_flag_lidar = 1;
bool g_flag_imu = 1;
uint64_t g_init_lidar = 0;
uint64_t g_init_imu = 0;
double timestamp_lidar;
double timestamp_imu;

int main(int argc, char** argv) {
    ros::init(argc, argv, "os_cloud_node");
    ros::NodeHandle nh("~");

    auto tf_prefix = nh.param("tf_prefix", std::string{});
    if (!tf_prefix.empty() && tf_prefix.back() != '/') tf_prefix.append("/");
    auto sensor_frame = tf_prefix + "os_sensor";
    auto imu_frame = tf_prefix + "os_imu";
    auto lidar_frame = tf_prefix + "os_lidar";

    ouster_ros::OSConfigSrv cfg{};
    auto client = nh.serviceClient<ouster_ros::OSConfigSrv>("os_config");
    client.waitForExistence();
    if (!client.call(cfg)) {
        ROS_ERROR("Calling config service failed");
        return EXIT_FAILURE;
    }

    auto info = sensor::parse_metadata(cfg.response.metadata);
    uint32_t H = info.format.pixels_per_column;
    uint32_t W = info.format.columns_per_frame;

    auto pf = sensor::get_format(info);

    auto lidar_pub = nh.advertise<sensor_msgs::PointCloud2>("points", 10);
    auto imu_pub = nh.advertise<sensor_msgs::Imu>("imu", 100);

    auto xyz_lut = ouster::make_xyz_lut(info);

    Cloud cloud{W, H};
    ouster::LidarScan ls{W, H};

    ouster::ScanBatcher batch(W, pf);

    auto lidar_handler = [&](const PacketMsg& pm) mutable {
        if (batch(pm.buf.data(), ls)) {
            auto h = std::find_if(
                ls.headers.begin(), ls.headers.end(), [](const auto& h) {
                    return h.timestamp != std::chrono::nanoseconds{0};
                });
            if (h != ls.headers.end()) 
            {
                scan_to_cloud(xyz_lut, h->timestamp, ls, cloud);
                // if(g_flag_lidar == 1)
                // {
                //     g_init_lidar = ros::Time::now().toNSec()- h->timestamp.count();
                //     g_flag_lidar = 0;
                // }
                timestamp_lidar = (g_init_imu + h->timestamp.count())/1e9;

                lidar_pub.publish(ouster_ros::cloud_to_cloud_msg(
                    cloud, timestamp_lidar, sensor_frame));
            }
        }
    };

    auto imu_handler = [&](const PacketMsg& p) {
        
        const uint8_t* buf_temp = p.buf.data();
        //m.header.stamp.fromNSec(pf.imu_gyro_ts(buf_temp));
        if(g_flag_imu == 1)
        {
                
            g_init_imu = ros::Time::now().toNSec()- pf.imu_gyro_ts(buf_temp);
            g_flag_imu = 0;
        }
        timestamp_imu = (g_init_imu + pf.imu_gyro_ts(buf_temp))/1e9;
        imu_pub.publish(ouster_ros::packet_to_imu_msg(p, imu_frame, pf, timestamp_imu));
    };

    auto lidar_packet_sub = nh.subscribe<PacketMsg, const PacketMsg&>(
        "lidar_packets", 2048, lidar_handler);
    auto imu_packet_sub = nh.subscribe<PacketMsg, const PacketMsg&>(
        "imu_packets", 100, imu_handler);

    // publish transforms
    tf2_ros::StaticTransformBroadcaster tf_bcast{};

    tf_bcast.sendTransform(ouster_ros::transform_to_tf_msg(
        info.imu_to_sensor_transform, sensor_frame, imu_frame));

    tf_bcast.sendTransform(ouster_ros::transform_to_tf_msg(
        info.lidar_to_sensor_transform, sensor_frame, lidar_frame));

    ros::spin();

    return EXIT_SUCCESS;
}
