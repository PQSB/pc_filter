#include "pc_filter/pc_filter_lib.hpp"

#include <algorithm>
#include <fstream>
#include <iostream>
#include <sstream>

#include <pcl_conversions/pcl_conversions.h>
#include <pcl/filters/crop_box.h>
#include <pcl/filters/extract_indices.h>

#include <opencv2/opencv.hpp>
#include <cv_bridge/cv_bridge.hpp>

#include <rclcpp/serialization.hpp>
#include <rclcpp/time.hpp>

#include <std_msgs/msg/header.hpp>

#include <rosbag2_storage/topic_metadata.hpp>

namespace pc_filter
{

void
filter_cloud_x(
  const sensor_msgs::msg::PointCloud2::SharedPtr msg, sensor_msgs::msg::PointCloud2 &filtered_msg)
{
  std_msgs::msg::Header original_header = msg->header;

  pcl::PointCloud<pcl::PointXYZI>::Ptr cloud(new pcl::PointCloud<pcl::PointXYZI>);
  pcl::fromROSMsg(*msg, *cloud);
 
  pcl::PointCloud<pcl::PointXYZI>::Ptr filtered_cloud(new pcl::PointCloud<pcl::PointXYZI>);
  for (const auto &point : *cloud)
  {
    if (point.x >= 0.0)
    {
      filtered_cloud->points.push_back(point);
    }
  }

  pcl::toROSMsg(*filtered_cloud, filtered_msg);

  filtered_msg.header = original_header;
}

void
filter_cloud_fov_2d(
  const sensor_msgs::msg::PointCloud2::SharedPtr msg,
  sensor_msgs::msg::PointCloud2 &filtered_msg,
  float min_angle, float max_angle)
{
  std_msgs::msg::Header original_header = msg->header;

  pcl::PointCloud<pcl::PointXYZI>::Ptr cloud(new pcl::PointCloud<pcl::PointXYZI>);

  pcl::fromROSMsg(*msg, *cloud);
  pcl::PointCloud<pcl::PointXYZI>::Ptr filtered_cloud(new pcl::PointCloud<pcl::PointXYZI>);
  for (const auto &point : *cloud)
  {

    float angle = std::atan2(point.y, point.x);

    if (angle > min_angle && angle < max_angle)
    {
      filtered_cloud->points.push_back(point);
    }
  }

  pcl::toROSMsg(*filtered_cloud, filtered_msg);

  filtered_msg.header = original_header;
}

pcl::PointCloud<pcl::PointXYZI>::Ptr
semantic_seg_filter(
  const std::unordered_set<uint16_t>& classes2filter,
  double seg_max_dist, 
  pcl::PointCloud<pcl::PointXYZI>::Ptr& input_cloud,
  fs::path lbl_path,
  const std::vector<int64_t> & original_indices)
{
    pcl::PointCloud<pcl::PointXYZI>::Ptr filtered_cloud(new pcl::PointCloud<pcl::PointXYZI>());

    // Open lbl file (.label)
    std::ifstream label_file(lbl_path, std::ios::binary);
    if (!label_file.is_open()) {
        throw std::runtime_error("[ERROR] Cannot open semantic segmentation lbl file: " + lbl_path.string());
    }

    std::vector<uint16_t> all_labels;
    uint32_t temp_label; // lbl file size

    while (label_file.read(reinterpret_cast<char*>(&temp_label), sizeof(uint32_t))) {
        // Extract just the class id bits
        all_labels.push_back(temp_label & 0xFFFF);
    }

    filtered_cloud->reserve(input_cloud->size());

    double max_dist_sq = seg_max_dist * seg_max_dist;
    double dist_sq;

    for (size_t i = 0; i < input_cloud->size(); ++i) {
        // Recover the original index the point had before the FOV filter
        int orig_idx = original_indices[i];

        if (seg_max_dist != MAX_DIST_DEFAULT){
            const auto & point = input_cloud->points[i];
            // To avoid using sqrt
            dist_sq = (point.x * point.x) + (point.y * point.y);
            if (dist_sq > max_dist_sq) {
                filtered_cloud->push_back(input_cloud->points[i]);
                continue;
            }
        }

        if (orig_idx < 0 || orig_idx >= static_cast<int>(all_labels.size())) {
            throw std::runtime_error("[ERROR] Index " + std::to_string(orig_idx) + "out of lbl labels file range");
        }

        uint16_t semantic_class = all_labels[orig_idx];

        // Check if the point is valid and add it in case it is
        if (classes2filter.find(semantic_class) == classes2filter.end()) {
            filtered_cloud->push_back(input_cloud->points[i]);
        }
    }

    return filtered_cloud;
}

/*3D Detector functions*/
std::vector<Detection>
loadDetections(
    const fs::path & det_file,
    double det_score_threshold,
    double det_max_dist,
    const std::unordered_set<std::string>& allowed_classes)
{
    std::ifstream file(det_file);

    if (!file.is_open()) {
        throw std::runtime_error("Cannot open detections file: " + det_file.string());
    }

    std::vector<Detection> detections;

    std::string line;

    double det_max_dist_sq = det_max_dist * det_max_dist;

    while (std::getline(file, line))
    {
        if (line.empty()) {continue;}

        std::stringstream ss(line);

        Detection det;

        if (!(ss >> det.category >> det.x >> det.y >> det.z >> det.w
            >> det.l >> det.h >> det.ry >> det.score)) {continue;}

        if (det.score < det_score_threshold) {continue;}

        // Check if the filter is active
        if (det_max_dist != MAX_DIST_DEFAULT){
            // To avoid using sqrt
            double dist_sq = (det.x * det.x) + (det.y * det.y);
            if (dist_sq > det_max_dist_sq) {continue;}
        }

        if (!allowed_classes.empty() &&
            allowed_classes.find(det.category) == allowed_classes.end()) {continue;}

        detections.push_back(std::move(det));
    }

    return detections;
}

pcl::PointCloud<pcl::PointXYZI>::Ptr
filterPointCloud(
    const pcl::PointCloud<pcl::PointXYZI>::Ptr& cloud,
    const std::vector<Detection>& detections)
{
    auto filtered_cloud = std::make_shared<pcl::PointCloud<pcl::PointXYZI>>();

    if (!cloud || cloud->empty())
    {
        return filtered_cloud;
    }

    if (detections.empty())
    {
        return cloud;
    }

    pcl::IndicesPtr indices_to_keep(new std::vector<int>);

    pcl::CropBox<pcl::PointXYZI> crop;
    crop.setInputCloud(cloud);

    for (const auto& det : detections)
    {
        if (det.l <= 0.0f || det.w <= 0.0f || det.h <= 0.0f){continue;}

        Eigen::Vector4f min_pt(
            -det.l / 2.0f, -det.w / 2.0f, -det.h / 2.0f, 1.0f);

        Eigen::Vector4f max_pt(
            det.l / 2.0f, det.w / 2.0f, det.h / 2.0f,1.0f);

        Eigen::Vector3f translation(det.x, det.y, det.z);
        Eigen::Vector3f rotation(0.0f, 0.0f, det.ry);

        crop.setMin(min_pt);
        crop.setMax(max_pt);

        crop.setTranslation(translation);
        crop.setRotation(rotation);

        std::vector<int> inside_indices;

        crop.filter(inside_indices);

        indices_to_keep->insert(
            indices_to_keep->end(), inside_indices.begin(), inside_indices.end());
    }

    // If there are no points that fall within the detections, return the entire cloud
    if (indices_to_keep->empty())
    {
        return cloud;
    }

    std::sort(indices_to_keep->begin(), indices_to_keep->end());

    indices_to_keep->erase(
        std::unique(
            indices_to_keep->begin(),
            indices_to_keep->end()),
        indices_to_keep->end());

    pcl::ExtractIndices<pcl::PointXYZI> extract;

    extract.setInputCloud(cloud);
    extract.setIndices(indices_to_keep);

    extract.setNegative(true);

    extract.filter(*filtered_cloud);

    return filtered_cloud;
}

FovCalibration
getFovFromFile(const fs::path& fov_file_path)
{
    std::ifstream fov_file(fov_file_path);
    if (!fov_file.is_open()) {
        throw std::runtime_error(
            "[ERROR] Can't open fov filter calib file: " + fov_file_path.string());
    }

    FovCalibration calib;

    // Read images width and height
    if (!(fov_file >> calib.image_width >> calib.image_height)) {
        throw std::runtime_error(
            "[ERROR] File doesn't contains image dimensions: " + fov_file_path.string());
    }

    // Read P2 (3x4)
    for (int r = 0; r < 3; ++r) {
        for (int c = 0; c < 4; ++c) {
            fov_file >> calib.P2(r, c);
        }
    }

    // Read Tr (3x4 - Formato KITTI omitiendo la última fila)
    for (int r = 0; r < 3; ++r) {
        for (int c = 0; c < 4; ++c) {
            fov_file >> calib.Tr_velo_to_cam(r, c);
        }
    }

    if (!fov_file) {
        throw std::runtime_error(
            "[ERROR] The file was terminated prematurely: " + fov_file_path.string());
    }

    // Fill Tr last row since it's 4x4
    calib.Tr_velo_to_cam(3, 0) = 0.0f;
    calib.Tr_velo_to_cam(3, 1) = 0.0f;
    calib.Tr_velo_to_cam(3, 2) = 0.0f;
    calib.Tr_velo_to_cam(3, 3) = 1.0f;

    return calib;
}

std::vector<int64_t>
loadTimestamps(const fs::path& ts_file)
{

    std::ifstream file(ts_file);
    if (!file.is_open())
    {
        throw std::runtime_error("Cannot open timestamp file: " + ts_file.string());
    }

    std::vector<int64_t> timestamps_ns;

    std::string line;
    double timestamp_sec;

    while (std::getline(file, line))
    {
        // Ignore emepty lines
        if (line.empty()) continue; 

        // Transform text into double
        timestamp_sec = std::stod(line);

        // Get the time in nanoseconds
        timestamps_ns.push_back(static_cast<int64_t>(std::llround(timestamp_sec * 1e9)));
    }

    // In case the timestamps file is empty
    if (timestamps_ns.empty())
    {
        throw std::runtime_error("Timestamp file is empty: " + ts_file.string());
    }

    return timestamps_ns;
}

std::tuple<pcl::PointCloud<pcl::PointXYZI>::Ptr, std::vector<int64_t>>
loadPointCloudXYZI(
    const fs::path& filepath, const bool use_fov, const FovCalibration& calib)
{
    auto cloud = std::make_shared<pcl::PointCloud<pcl::PointXYZI>>();
    
    // Stores the index of the original position of each point within the binary file (.bin) before applying any filters.
    std::vector<int64_t> correspondence;

    // Check if the file exists
    if (!fs::exists(filepath)) {
        std::cerr << "[WARN] File does not exist: " << filepath << "\n";
        return {cloud, correspondence};
    }

    // Get the total size of the file
    const size_t file_size = fs::file_size(filepath);
    const size_t point_size = 4 * sizeof(float); // XYZI = 16 bytes

    if (file_size == 0) {
        std::cerr << "[ERROR] Empty file: " << filepath << "\n";
        return {cloud, correspondence};
    }

    if (file_size % point_size != 0) {
        std::cerr << "[ERROR] Invalid .bin file size for XYZI format: "
                  << file_size << " bytes\n";
        return {cloud, correspondence};
    }

    // Calculate the number of points of the point cloud
    const size_t num_points = file_size / point_size;

    std::vector<float> raw(num_points * 4);

    // Open the file
    std::ifstream file(filepath, std::ios::binary);
    if (!file.is_open()) {
        std::cerr << "[ERROR] Cannot open file: " << filepath << "\n";
        return {cloud, correspondence};
    }

    // comprobar si puedo usar fread
    if (!file.read(reinterpret_cast<char*>(raw.data()), file_size)) {
        std::cerr << "[ERROR] Could not read full file\n";
        return {cloud, correspondence};;
    }

    cloud->points.reserve(num_points);
    correspondence.reserve(num_points);

    Eigen::Vector4f X_velo;
    Eigen::Vector4f X_cam;
    Eigen::Vector3f uvw;

    pcl::PointXYZI pt;

    float x, y, z, intensity, u, v;
    for (size_t i = 0; i < num_points; ++i) {
        x = raw[i * 4 + 0];
        y = raw[i * 4 + 1];
        z = raw[i * 4 + 2];
        intensity = raw[i * 4 + 3];

        if (use_fov) {
            X_velo << x, y, z, 1.0f;

            X_cam = calib.Tr_velo_to_cam * X_velo;

            // If point is behind the camera, ignore it
            if (X_cam.z() <= 0.0f) {
                continue;
            }   
        
            // Proyection
            uvw = calib.P2 * X_cam;

            u = uvw.x() / uvw.z();
            v = uvw.y() / uvw.z();

            if (u < 0 || u >= calib.image_width ||
                v < 0 || v >= calib.image_height)
            {
                continue;
            }

        }

        pt.x = x;
        pt.y = y;
        pt.z = z;
        pt.intensity = intensity;
    
        cloud->points.push_back(pt);
        correspondence.push_back(static_cast<int64_t>(i));
    }

    cloud->width  = cloud->points.size();
    cloud->height = 1;
    cloud->is_dense = false;

    return {cloud, correspondence};
}

std::vector<fs::path>
getPointCloudFiles(const fs::path& pc_dir)
{
    std::vector<fs::path> files;

    for (const auto& entry : fs::directory_iterator(pc_dir))
    {
        if (!entry.is_regular_file())
            continue;

        if (entry.path().extension() != ".bin")
            continue;

        files.push_back(entry.path());
    }

    // Since bin files have sequential names with the same number of digits
    std::sort(
        files.begin(),
        files.end());

    return files;
}

std::unordered_map<std::string, fs::path>
buildFileIndex(const fs::path& directory)
{
    std::unordered_map<std::string, fs::path> index;

    for (const auto& entry : fs::directory_iterator(directory))
    {
        if (!entry.is_regular_file())
        {
            continue;
        }

        index.emplace(
            entry.path().stem().string(),
            entry.path());
    }

    return index;
}

void
savePointCloudXYZI(const fs::path& filepath, const pcl::PointCloud<pcl::PointXYZI>::Ptr& cloud)
{
    std::ofstream file(filepath, std::ios::binary);
    if (!file.is_open()) {
        throw std::runtime_error("Cannot open file for writing: " + filepath.string());
    }

    // Create the vector with the correct size
    std::vector<float> buffer(cloud->points.size() * 4);

    // Fill the vector by index
    size_t i = 0;
    for (const auto& pt : cloud->points) {
        buffer[i++] = pt.x;
        buffer[i++] = pt.y;
        buffer[i++] = pt.z;
        buffer[i++] = pt.intensity;
    }

    // Write the reult in file
    file.write(reinterpret_cast<const char*>(buffer.data()), buffer.size() * sizeof(float));

    file.close();
}

/*ROS 2 functions*/
void
createTopics(rosbag2_cpp::Writer& writer, const Config& cfg)
{
    rosbag2_storage::TopicMetadata topic_info;
    topic_info.serialization_format = "cdr";

    if (cfg.export_input_pc) {
        topic_info.name = cfg.pc_topic;
        topic_info.type = "sensor_msgs/msg/PointCloud2";
        topic_info.offered_qos_profiles = {};

        writer.create_topic(topic_info);
    }

    if (cfg.use_detections) {
        topic_info.name = cfg.det_topic;
        topic_info.type = "sensor_msgs/msg/PointCloud2";
        topic_info.offered_qos_profiles = {};

        writer.create_topic(topic_info);

        topic_info.name = "/det_boxes";
        topic_info.type = "visualization_msgs/msg/MarkerArray";
        topic_info.offered_qos_profiles = {};

    }

    if (cfg.use_images) {
        topic_info.name = cfg.img_topic;
        topic_info.type = "sensor_msgs/msg/Image";
        topic_info.offered_qos_profiles = {};

        writer.create_topic(topic_info);
    }

    std::cout << std::endl;
}

void
writeImage(
    const fs::path& img_path,
    const std::string& topic,
    rosbag2_cpp::Writer& writer,
    int64_t timestamp_ns)
{
    cv::Mat image = cv::imread(img_path, cv::IMREAD_COLOR);

    if (image.empty()) {
        throw std::runtime_error("Cannot load image: " + img_path.string());
    }

    auto image_msg = cv_bridge::CvImage(std_msgs::msg::Header(), "bgr8", image).toImageMsg();

    rclcpp::Time img_time(timestamp_ns);
    image_msg->header.frame_id = "cam_2";
    image_msg->header.stamp = img_time;

    writer.write<sensor_msgs::msg::Image>(*image_msg, topic, img_time);
}

void
writeCloud(
    const pcl::PointCloud<pcl::PointXYZI>::Ptr cloud,
    const std::string& topic,
    rosbag2_cpp::Writer& writer,
    int64_t timestamp_ns)
{
    sensor_msgs::msg::PointCloud2 msg;

    pcl::toROSMsg(*cloud,msg);

    rclcpp::Time cloud_time(timestamp_ns);

    msg.header.frame_id = "base_lidar";
    msg.header.stamp = cloud_time;

    writer.write<sensor_msgs::msg::PointCloud2>(msg, topic, cloud_time);
}

void
writeRosbag(
    rosbag2_cpp::Writer& writer,
    const Config& cfg,
    const std::vector<fs::path>& files,
    const std::vector<int64_t>& timestamps_ns)
{
    rclcpp::Serialization<sensor_msgs::msg::PointCloud2> serializer;


    std::unordered_map<std::string, fs::path> image_index;
    std::unordered_map<std::string, fs::path> detection_index;
    std::unordered_map<std::string, fs::path> seg_index;

    if (cfg.use_images)
    {
        image_index = buildFileIndex(cfg.img_dir);
        if (image_index.size() != files.size()) {
            throw std::runtime_error("Number of point clouds (" + std::to_string(files.size()) + 
                ") does not match number of images (" + std::to_string(image_index.size()) + ")");
        }
    }

    if (cfg.use_detections)
    {
        detection_index = buildFileIndex(cfg.det_dir);
        if (detection_index.size() != files.size()) {
            throw std::runtime_error("Number of point clouds (" + std::to_string(files.size()) + 
                ") does not match number of detections (" + std::to_string(detection_index.size()) + ")");
        }
    }

    if (cfg.use_seg)
    {
        seg_index = buildFileIndex(cfg.seg_dir);
        if (seg_index.size() != files.size()) {
            throw std::runtime_error("Number of point clouds (" + std::to_string(files.size()) + 
                ") does not match number of semantic segmentation labels (" + std::to_string(seg_index.size()) + ")");
        }
    }

    std::string frame_id;

    for (size_t i = 0; i < files.size(); ++i)
    {

        frame_id = files[i].stem().string();

        std::cout
            << "\rWritting rosbag messages ["
            << (i + 1)
            << "/"
            << files.size()
            << "]"
            << std::flush;

        if (cfg.use_images)
        {
            auto img_it = image_index.find(frame_id);

            if (img_it != image_index.end()) {
                writeImage(img_it->second.string(), cfg.img_topic, writer, timestamps_ns[i]);

            } else {
                throw std::runtime_error("Missing image for frame: " + frame_id);
            }
        }

        auto [input_cloud, corr] = loadPointCloudXYZI(
            files[i], cfg.use_fov_filter, cfg.fov_params);

        if (!input_cloud || input_cloud->empty())
        {
            std::cerr
                << "\n[WARN] Empty cloud: "
                << files[i]
                << '\n';

            continue;
        }

        if (cfg.export_input_pc) {
            writeCloud(input_cloud, cfg.pc_topic, writer, timestamps_ns[i]);
        }

        if (cfg.use_seg) {
            auto seg_it = seg_index.find(frame_id);

            if (seg_it != seg_index.end()) {
                auto sk_filtered_cloud = semantic_seg_filter(cfg.seg_classes, cfg.seg_max_dist, input_cloud, seg_it->second.string(), corr);
                writeCloud(sk_filtered_cloud, cfg.seg_topic, writer, timestamps_ns[i]);

            } else {
                throw std::runtime_error("Missing semantic segmentation lbl file for frame: " + frame_id);
            }
        }

        if (cfg.use_detections) {
            auto det_it = detection_index.find(frame_id);

            if (det_it != detection_index.end()) {
                auto detections = loadDetections(det_it->second.string(), cfg.det_score, cfg.det_max_dist, cfg.det_classes);
                auto filtered_cloud = filterPointCloud(input_cloud, detections);
                // Filter the cloud first
                writeCloud(filtered_cloud, cfg.det_topic, writer, timestamps_ns[i]);

            } else {
                throw std::runtime_error("Missing detections for frame: " + frame_id);
            }
        }

    }

    std::cout << '\n';
}

}
