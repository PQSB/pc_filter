#ifndef PC_FILTER__PC_FILTER_LIB_HPP_
#define PC_FILTER__PC_FILTER_LIB_HPP_

#include <string>
#include <vector>
#include <tuple> 
#include <unordered_map>
#include <unordered_set>
#include <cstdint>
#include <filesystem>
#include <cmath>
#include <memory>

#include <Eigen/Dense>
#include <pcl/point_cloud.h>
#include <pcl/point_types.h>
#include <sensor_msgs/msg/point_cloud2.hpp>
#include <rosbag2_cpp/writer.hpp>

namespace fs = std::filesystem;

constexpr double MAX_DIST_DEFAULT = -1.0;

struct Detection {
    std::string category;
    float x, y, z;
    float w, l, h;
    float ry;
    float score;
};

struct FovCalibration
{
    Eigen::Matrix<float, 3, 4> P2;
    Eigen::Matrix4f Tr_velo_to_cam;

    int image_width{};
    int image_height{};
};

struct Config
{
    // Neccesary
    fs::path pc_dir;
    fs::path rosbag_out;
    fs::path ts_file;

    // Input cloud topic
    std::string pc_topic;
    bool export_input_pc = false;

    // fov filter
    fs::path fov_file;
    bool use_fov_filter = false;
    FovCalibration fov_params;

    // Images
    bool use_images = false;
    fs::path img_dir;
    std::string img_topic;

    // 3D-MOOD detections
    bool use_detections = false;
    fs::path det_dir;
    double det_score{};
    double det_max_dist = MAX_DIST_DEFAULT;
    std::string det_topic;
    std::unordered_set<std::string> det_classes;

    // Semantic segmentation
    bool use_seg = false;
    fs::path seg_dir;
    std::string seg_topic;
    std::unordered_set<uint16_t> seg_classes;
    double seg_max_dist = MAX_DIST_DEFAULT;

    bool no_confirm;

    // To export just the filtered point clouds directly to a directory
    bool only_clouds = false;
    fs::path clouds_out_dir;
};

namespace pc_filter
{

/*First online filtering functions (serve as a starting point for future work)*/
void
filter_cloud_x(
    const sensor_msgs::msg::PointCloud2::SharedPtr msg,
    sensor_msgs::msg::PointCloud2 &filtered_msg);

void
filter_cloud_fov_2d(
    const sensor_msgs::msg::PointCloud2::SharedPtr msg,
    sensor_msgs::msg::PointCloud2 &filtered_msg,
    float min_angle = -M_PI/2.0f, float max_angle = M_PI/2.0f);

/*Input, output, and filtering functions*/
pcl::PointCloud<pcl::PointXYZI>::Ptr
semantic_seg_filter(
  const std::unordered_set<uint16_t>& classes2filter,
  double seg_max_dist, 
  pcl::PointCloud<pcl::PointXYZI>::Ptr& input_cloud,
  fs::path lbl_path,
  const std::vector<int64_t> & original_indices);

FovCalibration
getFovFromFile(const fs::path& fov_file_path);

std::vector<Detection>
loadDetections(
    const fs::path & det_file,
    double det_score_threshold,
    double det_max_dist,
    const std::unordered_set<std::string>& allowed_classes);

pcl::PointCloud<pcl::PointXYZI>::Ptr
filterPointCloud(
    const pcl::PointCloud<pcl::PointXYZI>::Ptr& cloud,
    const std::vector<Detection>& detections);

std::vector<int64_t>
loadTimestamps(const fs::path& ts_file);

std::tuple<pcl::PointCloud<pcl::PointXYZI>::Ptr, std::vector<int64_t>>
loadPointCloudXYZI(
    const fs::path& filepath, const bool use_fov, const FovCalibration& calib);

std::vector<fs::path>
getPointCloudFiles(const fs::path& pc_dir);

std::unordered_map<std::string, fs::path>
buildFileIndex(const fs::path& directory);

void
savePointCloudXYZI(
    const fs::path& filepath,
    const pcl::PointCloud<pcl::PointXYZI>::Ptr& cloud);

/*ROS 2 functions*/
void
createTopics(
    rosbag2_cpp::Writer& writer, const Config& cfg);

void
writeImage(
    const fs::path& img_path,
    const std::string& topic,
    rosbag2_cpp::Writer& writer,
    int64_t timestamp_ns);

void
writeCloud(
    const pcl::PointCloud<pcl::PointXYZI>::Ptr cloud,
    const std::string& topic,
    rosbag2_cpp::Writer& writer,
    int64_t timestamp_ns);

void
writeRosbag(
    rosbag2_cpp::Writer& writer,
    const Config& cfg,
    const std::vector<fs::path>& files,
    const std::vector<int64_t>& timestamps_ns);
}

#endif  // PC_FILTER__PC_FILTER_LIB_HPP_
